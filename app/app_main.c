#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "app_main.h"
#include "app_config.h"
#include "shared_protocol/robot_protocol.h"
#include "mux_channels.h"
#include "usbd_cdc_if.h"

#define APP_LINK_UART                APP_LOG_UART
#define APP_LINK_RX_BUFFER_BYTES     512U
#define APP_LINK_FRAME_BUFFER_BYTES  ROBOT_FRAME_MAX_ENCODED
#define APP_LINK_TX_BUFFER_BYTES     320U
#define APP_TELEM_PERIOD_MS          20U
#define APP_HEARTBEAT_TIMEOUT_MS     250U

static void app_init(void);
static void app_idle_tick(void);
static void app_log_uart_write(const uint8_t *data, size_t len);
static void app_link_start(void);
static void app_link_process_chunk(const uint8_t *data, size_t len);
static void app_link_flush_frame(void);
static void app_link_handle_encoded_frame(const uint8_t *frame, size_t len);
static void app_link_dispatch(const robot_frame_t *frame);
static void app_cmd_handler(uint8_t msg_type, const uint8_t *payload, size_t len, void *ctx);
static void app_link_restart_rx(void);
static bool app_link_send(uint8_t type, uint16_t flags, const uint8_t *payload, uint16_t len, uint16_t seq_override);
static void app_send_telem(void);

void app_main(void)
{
    app_init();

    while (1)
    {
        app_idle_tick();
    }
}

void app_log_printf(const char *fmt, ...)
{
    if (APP_LOG_UART == NULL)
    {
        return;
    }

    char buffer[APP_LOG_BUFFER_BYTES];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len < 0)
    {
        return;
    }

    size_t bounded_len = (size_t)len;
    if (bounded_len >= sizeof(buffer))
    {
        bounded_len = sizeof(buffer) - 1U;
    }

    app_log_uart_write((const uint8_t *)buffer, bounded_len);
}

static robot_mux_t s_mux;
static uint8_t s_uart_rx_buffer[APP_LINK_RX_BUFFER_BYTES];
static uint8_t s_cobs_frame_buffer[APP_LINK_FRAME_BUFFER_BYTES];
static size_t  s_cobs_frame_len = 0U;
static uint8_t s_tx_buffer[APP_LINK_TX_BUFFER_BYTES];
static uint16_t s_seq_counters[ROBOT_CHANNEL_MAX + 1U] = {0};
static uint32_t s_last_telem_ms = 0U;
static uint32_t s_last_cmd_ms = 0U;

static void app_init(void)
{
    robot_mux_init(&s_mux);
    robot_mux_register(&s_mux, ROBOT_CHANNEL_CMD, app_cmd_handler, NULL);

    APP_LOG_INFO("Booting robot firmware (frame v%u)", ROBOT_FRAME_VERSION);
    APP_LOG_INFO("CMD channel id: %u", ROBOT_CHANNEL_CMD);

    app_link_start();
    s_last_cmd_ms = HAL_GetTick();
}

static void app_idle_tick(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_last_telem_ms) >= APP_TELEM_PERIOD_MS)
    {
        app_send_telem();
        s_last_telem_ms = now;
    }

    if ((now - s_last_cmd_ms) > APP_HEARTBEAT_TIMEOUT_MS)
    {
        APP_LOG_ERROR("Link heartbeat timeout");
        s_last_cmd_ms = now; // rate-limit log spam
    }

    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    HAL_Delay(APP_IDLE_TICK_MS);
}

static void app_log_uart_write(const uint8_t *data, size_t len)
{
    if (len == 0U)
    {
        return;
    }

    /* Send over USB CDC (HS instance in FS PHY); fall back to USART2 on failure. */
    if (USBD_OK != CDC_Transmit_HS((uint8_t *)data, (uint16_t)len))
    {
#ifdef APP_LOG_UART
        HAL_UART_Transmit(APP_LOG_UART, (uint8_t *)data, (uint16_t)len, 10U);
#endif
    }
}

static void app_link_start(void)
{
    if (APP_LINK_UART == NULL)
    {
        APP_LOG_ERROR("Link UART not initialized");
        return;
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(APP_LINK_UART, s_uart_rx_buffer, sizeof(s_uart_rx_buffer)) != HAL_OK)
    {
        APP_LOG_ERROR("UART RX start failed");
        return;
    }
    __HAL_DMA_DISABLE_IT(APP_LINK_UART->hdmarx, DMA_IT_HT);
}

static void app_link_restart_rx(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(APP_LINK_UART, s_uart_rx_buffer, sizeof(s_uart_rx_buffer)) != HAL_OK)
    {
        APP_LOG_ERROR("UART RX restart failed");
        return;
    }
    __HAL_DMA_DISABLE_IT(APP_LINK_UART->hdmarx, DMA_IT_HT);
}

static void app_link_process_chunk(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i)
    {
        uint8_t byte = data[i];
        if (byte == 0x00U)
        {
            app_link_flush_frame();
        }
        else if (s_cobs_frame_len < sizeof(s_cobs_frame_buffer))
        {
            s_cobs_frame_buffer[s_cobs_frame_len++] = byte;
        }
        else
        {
            APP_LOG_ERROR("UART frame overflow, dropping data");
            s_cobs_frame_len = 0U;
        }
    }
}

static void app_link_flush_frame(void)
{
    if (s_cobs_frame_len == 0U)
    {
        return;
    }

    app_link_handle_encoded_frame(s_cobs_frame_buffer, s_cobs_frame_len);
    s_cobs_frame_len = 0U;
}

static void app_link_handle_encoded_frame(const uint8_t *frame, size_t len)
{
    robot_frame_t decoded;
    robot_frame_t ack;
    uint8_t encoded_ack[ROBOT_FRAME_MAX_ENCODED];
    size_t encoded_ack_len = 0U;

    if (len + 1U > ROBOT_FRAME_MAX_ENCODED)
    {
        APP_LOG_ERROR("Encoded frame too long (%u)", (unsigned int)len);
        return;
    }

    uint8_t encoded_buf[ROBOT_FRAME_MAX_ENCODED];
    memcpy(encoded_buf, frame, len);
    encoded_buf[len] = 0x00U;

    if (!robot_frame_decode(encoded_buf, len + 1U, &decoded))
    {
        APP_LOG_ERROR("Frame decode failed (len=%u)", (unsigned int)len);
        return;
    }

    /* Auto-ACK if requested */
    if ((decoded.hdr.flags & ROBOT_FLAG_ACK_REQ) != 0U)
    {
        if (robot_frame_init(&ack,
                             ROBOT_MSG_ACK,
                             decoded.hdr.seq,
                             ROBOT_FLAG_IS_ACK,
                             NULL,
                             0U) &&
            robot_frame_encode(&ack, encoded_ack, sizeof(encoded_ack), &encoded_ack_len))
        {
            HAL_UART_Transmit(APP_LINK_UART, encoded_ack, (uint16_t)encoded_ack_len, 10U);
        }
    }

    app_link_dispatch(&decoded);
}

static void app_link_dispatch(const robot_frame_t *frame)
{
    if (frame == NULL)
    {
        return;
    }

    /* Track link liveness on CMD heartbeats */
    if (frame->hdr.type == ROBOT_MSG_CMD_HEARTBEAT)
    {
        s_last_cmd_ms = HAL_GetTick();
    }

    robot_mux_dispatch(&s_mux, frame->hdr.type, frame->payload, frame->hdr.len);
}

static void app_cmd_handler(uint8_t msg_type, const uint8_t *payload, size_t len, void *ctx)
{
    (void)ctx;
    if (msg_type == ROBOT_MSG_CMD_TELEOP)
    {
        if (len < sizeof(robot_cmd_teleop_t))
        {
            APP_LOG_ERROR("CMD teleop size mismatch (%u)", (unsigned int)len);
            return;
        }
        const robot_cmd_teleop_t *cmd = (const robot_cmd_teleop_t *)payload;
        APP_LOG_INFO("Teleop fwd=%.2f turn=%.2f flags=0x%04x", (double)cmd->vx_mps, (double)cmd->wz_radps, cmd->flags);
    }
    else if (msg_type == ROBOT_MSG_CMD_HEARTBEAT)
    {
        APP_LOG_INFO("Heartbeat received");
    }
    else
    {
        APP_LOG_INFO("CMD handler invoked, msg_type=0x%02x", msg_type);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != APP_LINK_UART)
    {
        return;
    }
    app_link_process_chunk(s_uart_rx_buffer, Size);
    app_link_restart_rx();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != APP_LINK_UART)
    {
        return;
    }
    APP_LOG_ERROR("UART error 0x%lx", (unsigned long)huart->ErrorCode);
    app_link_restart_rx();
}

static bool app_link_send(uint8_t type, uint16_t flags, const uint8_t *payload, uint16_t len, uint16_t seq_override)
{
    robot_frame_t frame;
    uint8_t encoded[ROBOT_FRAME_MAX_ENCODED];
    size_t encoded_len = 0U;

    uint16_t seq = (seq_override != 0U) ? seq_override : ++s_seq_counters[robot_channel_from_type(type)];
    if (!robot_frame_init(&frame, type, seq, flags, payload, len))
    {
        return false;
    }
    if (!robot_frame_encode(&frame, encoded, sizeof(encoded), &encoded_len))
    {
        return false;
    }

    return (HAL_UART_Transmit(APP_LINK_UART, encoded, (uint16_t)encoded_len, 10U) == HAL_OK);
}

static void app_send_telem(void)
{
    robot_telem_v1_t telem;
    memset(&telem, 0, sizeof(telem));
    telem.version = 1U;
    telem.status = 0U;
    telem.timestamp_ms = HAL_GetTick();
    telem.batt_v = 0.0f;
    telem.batt_pct = 0.0f;
    telem.temp_c = 0.0f;

    if (!app_link_send(ROBOT_MSG_TELEM_FRAME, 0U, (const uint8_t *)&telem, sizeof(telem), 0U))
    {
        APP_LOG_ERROR("Failed to send telem frame");
    }
}
