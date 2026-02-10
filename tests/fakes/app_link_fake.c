#include "app_link.h"

#include <string.h>

uint32_t s_last_cmd_ms = 0U;

static app_link_send_err_t s_last_err = APP_LINK_SEND_OK;
static uint32_t s_last_status = 0U;
static uint32_t s_last_hal_state = 0U;
static uint32_t s_last_hal_err = 0U;

uint8_t g_app_link_last_type = 0U;
uint16_t g_app_link_last_flags = 0U;
uint16_t g_app_link_last_len = 0U;
uint8_t g_app_link_last_payload[ROBOT_FRAME_MAX_PAYLOAD] = {0};
bool g_app_link_force_fail = false;

void app_link_get_last_send_error(app_link_send_err_t *err, uint32_t *status,
                                  uint32_t *hal_state, uint32_t *hal_err)
{
    if (err) {
        *err = s_last_err;
    }
    if (status) {
        *status = s_last_status;
    }
    if (hal_state) {
        *hal_state = s_last_hal_state;
    }
    if (hal_err) {
        *hal_err = s_last_hal_err;
    }
}

void app_link_start(void)
{
}

bool app_link_send(uint8_t type, uint16_t flags, const uint8_t *payload,
                   uint16_t len, uint16_t seq_override)
{
    (void)seq_override;
    g_app_link_last_type = type;
    g_app_link_last_flags = flags;
    g_app_link_last_len = len;
    if (len > 0U && payload != NULL) {
        if (len > sizeof(g_app_link_last_payload)) {
            len = sizeof(g_app_link_last_payload);
        }
        memcpy(g_app_link_last_payload, payload, len);
    }
    if (g_app_link_force_fail) {
        s_last_err = APP_LINK_SEND_ERR_UART_BUSY;
        return false;
    }
    s_last_err = APP_LINK_SEND_OK;
    return true;
}

void app_link_poll(void)
{
}

void app_log_link_errors(void)
{
}

bool app_link_feed_cdc(const uint8_t *data, uint32_t len)
{
    (void)data;
    (void)len;
    return false;
}

void app_link_debug_frame(const uint8_t *frame, size_t len)
{
    (void)frame;
    (void)len;
}

void app_link_dispatch(const robot_frame_t *frame)
{
    (void)frame;
}

const char *app_link_send_err_str(app_link_send_err_t err)
{
    switch (err) {
    case APP_LINK_SEND_OK:
        return "ok";
    case APP_LINK_SEND_ERR_UART_BUSY:
        return "uart_busy";
    default:
        return "err";
    }
}
