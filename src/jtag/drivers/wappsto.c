/***************************************************************************
 *   Copyright (C) 2021 by Andreas Bomholtz <andreas@seluxit.com>          *                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <jtag/swd.h>
#include <jtag/commands.h>
#include <helper/command.h>

#include <curl/curl.h>

#define WAPPSTO_QUEUE_SIZE 64
#define WAPPSTO_MAX_PACKET_LENGTH 512

enum WAPPSTO_CMDS {
    WAPPSTO_INVALID = 0,   /* Invalid */
    WAPPSTO_WRITE_BITS = 1, /* Host wants us to write bits */
    WAPPSTO_READ_BITS = 2,  /* Host wants us to read bits */
};

struct wappsto {
    CURL *curl;
    uint8_t *packet_buffer;
};

static struct wappsto_queue_entry {
    uint8_t cmd;
    uint32_t *dst;
    uint32_t data;
} *wappsto_queue;

static char wappsto_token[129];
static char wappsto_url[300];
static size_t wappsto_queue_length;
static int queue_read = 0;
static int queue_write = 0;
static struct wappsto *wappsto_handle;
static int wappsto_read_count = 0;
static char wappsto_read_buffer[8000] = {0,};
static uint8_t wappsto_timeout = 5;

static int wappsto_init(void);
static int wappsto_quit(void);

static size_t wappsto_write_callback(char *ptr, size_t size, size_t nmemb, void *userdata) {
    size_t realsize = size * nmemb;
    memcpy(&wappsto_read_buffer[wappsto_read_count], ptr, realsize);
    wappsto_read_count += realsize;
    return realsize;
}

static int wappsto_request(const char* data) {
    CURLcode res;

    curl_easy_setopt(wappsto_handle->curl, CURLOPT_URL, wappsto_url);
    curl_easy_setopt(wappsto_handle->curl, CURLOPT_POSTFIELDS, data);
    curl_easy_setopt(wappsto_handle->curl, CURLOPT_WRITEFUNCTION, wappsto_write_callback);

    wappsto_read_count = 0;
    memset(wappsto_read_buffer, 0, 8000);

    /* Perform the request, res will get the return code */
    res = curl_easy_perform(wappsto_handle->curl);
    /* Check for errors */
    if(res != CURLE_OK) {
        fprintf(stderr, "curl_easy_perform() failed: %s\n",
                curl_easy_strerror(res));
    }

    long http_code = 0;
    curl_easy_getinfo (wappsto_handle->curl, CURLINFO_RESPONSE_CODE, &http_code);

    if (http_code == 200) {
        if(strcmp(wappsto_read_buffer, "\"NAK\"") == 0) {
            LOG_ERROR("Failed to send command to wappsto: %s", wappsto_read_buffer);
            return ERROR_FAIL;
        }

        return ERROR_OK;
    }

    if(strstr(wappsto_read_buffer, "timeout") != NULL) {
        LOG_ERROR("Failed to send command to wappsto: TIMEOUT");
        return ERROR_TIMEOUT_REACHED;
    }

    LOG_ERROR("Failed to send command to wappsto: %s", wappsto_read_buffer);
    return ERROR_FAIL;
}

static int wappsto_send_command(const char* cmd, int data) {
    char command_data[100];
    sprintf(command_data, "{\"command\":\"%s\", \"data\":%d}", cmd, data);
    return wappsto_request(command_data);
}

static int wappsto_data_request(const char* mode, uint8_t* data, uint16_t len) {
    char *request = malloc(len*2 + 100);
    sprintf(request, "{\"command\":\"%s\",\"data\":\"", mode);
    for(int i=0; i<len; i++) {
        sprintf(&request[strlen(request)], "%02X", data[i]);
    }
    sprintf(&request[strlen(request)], "\"}");
    int res = wappsto_request(request);
    free(request);
    return res;
}

static int wappsto_send_init(void)
{
    int res = wappsto_send_command("init", 0);

    if(res != ERROR_OK) {
        return ERROR_FAIL;
    }

    LOG_INFO("Wappsto init: %s", wappsto_read_buffer);
    return ERROR_OK;
}

static int wappsto_get_info(void)
{
    int res = wappsto_send_command("info", 0);

    if(res != ERROR_OK) {
        return ERROR_FAIL;
    }

    LOG_INFO("Wappsto connected to WAPP: %s", wappsto_read_buffer);
    return ERROR_OK;
}

static int wappsto_swd_queue_run(void)
{
    int ret = ERROR_OK;
    if(queue_read == 0 && wappsto_queue_length < (WAPPSTO_QUEUE_SIZE - 10)) {
        return ret;
    }

    LOG_DEBUG_IO("Queue count %zu - Read %d Write %d", wappsto_queue_length, queue_read, queue_write);
    queue_read = 0;
    queue_write = 0;

    int packet_size = 0;
    for(size_t i=0; i<wappsto_queue_length; i++) {
        struct wappsto_queue_entry *e = &wappsto_queue[i];
        char *pkt = (char*)&wappsto_handle->packet_buffer[packet_size];
        LOG_DEBUG_IO("0x%02X (%s) - 0x%08X",
            e->cmd,
            e->cmd & SWD_CMD_RnW ? "R" : "W",
            e->data
        );
        pkt[0] = e->cmd | SWD_CMD_START | SWD_CMD_PARK;
        packet_size++;
        if(!(e->cmd & SWD_CMD_RnW)) {
            packet_size+=4;
            pkt[4] = (e->data >> 0)  & 0xFF;
            pkt[3] = (e->data >> 8)  & 0xFF;
            pkt[2] = (e->data >> 16) & 0xFF;
            pkt[1] = (e->data >> 24) & 0xFF;
        }
    }

    int res = wappsto_data_request("data", wappsto_handle->packet_buffer, packet_size);

    packet_size = 1;
    if(res == ERROR_OK) {
        for(size_t i=0; i<wappsto_queue_length; i++) {
            struct wappsto_queue_entry *e = &wappsto_queue[i];

            if(e->cmd & SWD_CMD_RnW) {
                if(e->dst) {
                    sscanf(&wappsto_read_buffer[packet_size],"%08X", e->dst);
                }
                packet_size+=8;
            } else {
                if(wappsto_read_buffer[packet_size+1] == '0') {
                    ret = ERROR_FAIL;
                }
                packet_size+=2;
            }
            e->dst = NULL;
        }
    }

    wappsto_queue_length = 0;

    return ret;
}

static void wappsto_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data)
{
    if (wappsto_queue_length == WAPPSTO_QUEUE_SIZE) {
        if(wappsto_swd_queue_run() != ERROR_OK) {
            LOG_ERROR("Queue failed to send all messages");
            return;
        };
    }

    struct wappsto_queue_entry *e = &wappsto_queue[wappsto_queue_length++];

    e->cmd = cmd;
    e->data = data;
    if(e->cmd & SWD_CMD_RnW) {
        e->dst = dst;
    } else {
        e->dst = NULL;
    }
}

static void wappsto_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
    queue_read++;
    assert(cmd & SWD_CMD_RnW);
    wappsto_swd_queue_cmd(cmd, value, 0);
}

static void wappsto_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
    queue_write++;
    assert(!(cmd & SWD_CMD_RnW));
    wappsto_swd_queue_cmd(cmd, NULL, value);
}

static int wappsto_swd_init(void)
{
    return ERROR_OK;
}

static int wappsto_swd_switch_seq(enum swd_special_seq seq)
{
    return wappsto_send_command("cmd", seq);
}

static int wappsto_reset(int trst, int srst)
{
    return ERROR_OK;
}

COMMAND_HANDLER(wappsto_handle_info_command)
{
    int retval = wappsto_get_info();

    return retval;
}

static void update_wappsto_url(void) {
    sprintf(wappsto_url, "https://wappsto.com/services/extsync/request/%s?timeout=%d", wappsto_token, wappsto_timeout);
}

COMMAND_HANDLER(wappsto_handle_token_command)
{
    if (CMD_ARGC != 1 || CMD_ARGV[0] == NULL || strlen(CMD_ARGV[0]) != 128) {
        LOG_ERROR("Invalid token format - Token must be 128 char long");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }

    strcpy(wappsto_token, CMD_ARGV[0]);
    LOG_INFO("Using wappsto token: %s", wappsto_token);
    update_wappsto_url();

    return ERROR_OK;
}

COMMAND_HANDLER(wappsto_handle_timeout_command)
{
    if (CMD_ARGC != 1)
        return ERROR_COMMAND_SYNTAX_ERROR;

    COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], wappsto_timeout);
    LOG_INFO("Using wappsto timeout: %d", wappsto_timeout);
    update_wappsto_url();

    return ERROR_OK;
}

static const struct command_registration wappsto_subcommand_handlers[] = {
    {
        .name = "info",
        .handler = &wappsto_handle_info_command,
        .mode = COMMAND_EXEC,
        .usage = "",
        .help = "show wappsto info",
    },
    {
        .name = "token",
        .handler = &wappsto_handle_token_command,
        .mode = COMMAND_CONFIG,
        .usage = "<token>",
        .help = "configure the wappsto token",
    },
    {
        .name = "timeout",
        .handler = &wappsto_handle_timeout_command,
        .mode = COMMAND_CONFIG,
        .usage = "<timeout> seconds",
        .help = "configure the timeout waiting for response from wappsto",
    },
    COMMAND_REGISTRATION_DONE
};

static const struct command_registration wappsto_command_handlers[] = {
    {
        .name = "wappsto",
        .mode = COMMAND_ANY,
        .help = "perform wappsto management",
        .usage = "<cmd>",
        .chain = wappsto_subcommand_handlers,
    },
    COMMAND_REGISTRATION_DONE
};

static const char * const wappsto_transports[] = { "swd", NULL };

static const struct swd_driver wappsto_swd = {
    .init = wappsto_swd_init,
    .switch_seq = wappsto_swd_switch_seq,
    .read_reg = wappsto_swd_read_reg,
    .write_reg = wappsto_swd_write_reg,
    .run = wappsto_swd_queue_run,
};

struct adapter_driver wappsto_adapter_driver = {
    .name = "wappsto",
    .commands = wappsto_command_handlers,
    .transports = wappsto_transports,
    .swd_ops = &wappsto_swd,
    .init = wappsto_init,
    .quit = wappsto_quit,
    .reset = wappsto_reset,
};

static int wappsto_init(void)
{
    wappsto_handle = malloc(sizeof(struct wappsto));
    if (wappsto_handle == NULL) {
        LOG_ERROR("Failed to allocate memory");
        return ERROR_FAIL;
    }

    if(strlen(wappsto_url) == 0) {
        LOG_ERROR("Can't run wappsto driver, without token");
        return ERROR_FAIL;
    }

    /* In windows, this will init the winsock stuff */
    curl_global_init(CURL_GLOBAL_ALL);

    wappsto_handle->curl = curl_easy_init();
    if (!wappsto_handle->curl) {
        LOG_ERROR("Failed to allocate memory for cURL");
        return ERROR_FAIL;
    }

    if(wappsto_send_init() != ERROR_OK) {
        return ERROR_FAIL;
    }

    if (wappsto_get_info() != ERROR_OK) {
        return ERROR_FAIL;
    }

    /* Allocate packet buffers */
    wappsto_handle->packet_buffer = malloc(WAPPSTO_MAX_PACKET_LENGTH);
    if (wappsto_handle->packet_buffer == NULL) {
        LOG_ERROR("Failed to allocate memory for the packet buffer");
        return ERROR_FAIL;
    }

    /* Allocate queue */
    wappsto_queue_length = 0;
    wappsto_queue = malloc(WAPPSTO_QUEUE_SIZE * sizeof(*wappsto_queue));
    if (wappsto_queue == NULL) {
        LOG_ERROR("Failed to allocate memory for the wappsto queue");
        return ERROR_FAIL;
    }

    return ERROR_OK;
}

static int wappsto_quit(void)
{
    curl_easy_cleanup(wappsto_handle->curl);
    wappsto_handle->curl = NULL;
    curl_global_cleanup();
    return ERROR_OK;
}
