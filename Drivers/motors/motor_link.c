#include "motor_link.h"

#include <math.h>
#include <string.h>

#include "app_config.h"
#include "main.h"
#include "stm32h7xx_hal.h"

#ifndef MOTOR_LINK_TELEM_WARN_MS
#define MOTOR_LINK_TELEM_WARN_MS 5U
#endif

#ifndef MOTOR_LINK_TELEM_STALE_MS
#define MOTOR_LINK_TELEM_STALE_MS 10U
#endif

#ifndef MOTOR_LINK_KT_NM_PER_A
#define MOTOR_LINK_KT_NM_PER_A 0.0f
#endif

#ifndef MOTOR_LINK_KV_RPM_PER_V
#define MOTOR_LINK_KV_RPM_PER_V 0.0f
#endif

#ifndef MOTOR_LINK_ACK_TIMEOUT_MS
#define MOTOR_LINK_ACK_TIMEOUT_MS 200U
#endif

#ifndef MOTOR_LINK_TELEM_BASE_HZ
#define MOTOR_LINK_TELEM_BASE_HZ 1000.0f
#endif

#ifndef MOTOR_LINK_TELEM_RATE_HZ
#define MOTOR_LINK_TELEM_RATE_HZ 500.0f
#endif

#ifndef MOTOR_LINK_CONFIG_DOWNSAMPLE
#define MOTOR_LINK_CONFIG_DOWNSAMPLE 10000U
#endif

#ifndef MOTOR_LINK_FLOAT_EPS
#define MOTOR_LINK_FLOAT_EPS 1.0e-4f
#endif

#ifndef MOTOR_LINK_RX_BUFFER_BYTES
#define MOTOR_LINK_RX_BUFFER_BYTES 1024U
#endif

#ifndef MOTOR_LINK_DEBUG
#define MOTOR_LINK_DEBUG 1U
#endif

#ifndef MOTOR_LINK_DEBUG_MAX_BYTES
#define MOTOR_LINK_DEBUG_MAX_BYTES 16U
#endif

#ifndef MOTOR_LINK_TX_BUFFER_BYTES
#define MOTOR_LINK_TX_BUFFER_BYTES 32U
#endif

#ifndef MOTOR_LINK_PARSER_BUFFER_BYTES
#define MOTOR_LINK_PARSER_BUFFER_BYTES 1024U
#endif

#define MOTOR_LINK_PI 3.14159265358979323846f

enum
{
    MOTOR_LINK_BIN_MARKER = 0xA5,
    MOTOR_LINK_TYPE_REG_REQ = 0x52,
    MOTOR_LINK_TYPE_REG_RESP = 0x72,
    MOTOR_LINK_TYPE_TELEM_DATA = 0x54
};

enum
{
    MOTOR_LINK_REG_STATUS = 0x00,
    MOTOR_LINK_REG_TARGET = 0x01,
    MOTOR_LINK_REG_ENABLE = 0x04,
    MOTOR_LINK_REG_CONTROL_MODE = 0x05,
    MOTOR_LINK_REG_TORQUE_MODE = 0x06,
    MOTOR_LINK_REG_MODULATION_MODE = 0x07,
    MOTOR_LINK_REG_POSITION = 0x10,
    MOTOR_LINK_REG_VELOCITY = 0x11,
    MOTOR_LINK_REG_TELEMETRY_REG = 0x1A,
    MOTOR_LINK_REG_TELEMETRY_DOWNSAMPLE = 0x1C
};

typedef enum
{
    MOTOR_LINK_RT_U8,
    MOTOR_LINK_RT_U32,
    MOTOR_LINK_RT_I32_F32,
    MOTOR_LINK_RT_F32,
    MOTOR_LINK_RT_BYTES
} motor_link_reg_type_t;

typedef struct
{
    uint8_t buf[MOTOR_LINK_PARSER_BUFFER_BYTES];
    size_t len;
    uint32_t drops;
    uint32_t drops_total;
} motor_link_parser_t;

typedef struct
{
    uint8_t active;
    uint8_t reg;
    motor_link_reg_type_t type;
    uint8_t len;
    uint8_t expected[16];
    uint32_t sent_ms;
} motor_link_pending_t;

typedef struct
{
    uint32_t t_ms;
    uint8_t reg;
    motor_link_reg_type_t type;
    uint8_t ok;
    uint8_t raw_len;
    uint8_t raw[16];
    uint8_t valid;
} motor_link_ack_t;

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t *rx_buf;
    uint16_t rx_len;
    uint16_t rx_last_pos;
    uint8_t motor_id;
    uint8_t *tx_buf;
    uint16_t tx_len;
    motor_link_parser_t parser;
    motor_link_pending_t pending;
    motor_link_ack_t last_ack;
    uint32_t ack_seq;
    uint32_t last_ack_seq;
    uint32_t ack_timeouts;
    float telemetry_base_hz;
} motor_link_port_t;

static uint8_t s_left_rx_buffer[MOTOR_LINK_RX_BUFFER_BYTES]
    __attribute__((section(".dma_buffer"), aligned(32)));
static uint8_t s_right_rx_buffer[MOTOR_LINK_RX_BUFFER_BYTES]
    __attribute__((section(".dma_buffer"), aligned(32)));
static uint8_t s_left_tx_buffer[MOTOR_LINK_TX_BUFFER_BYTES]
    __attribute__((section(".dma_buffer"), aligned(32)));
static uint8_t s_right_tx_buffer[MOTOR_LINK_TX_BUFFER_BYTES]
    __attribute__((section(".dma_buffer"), aligned(32)));

static motor_link_port_t s_left_port;
static motor_link_port_t s_right_port;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

typedef struct
{
    float velocity;
    uint8_t status;
    uint32_t last_vel_ms;
    uint32_t last_status_ms;
    uint32_t stale_count;
    uint32_t late_count;
    uint8_t vel_valid;
    uint8_t status_valid;
} motor_telem_t;

static volatile uint8_t s_initialized = 0U;
static volatile uint8_t s_enable_applied = 0U;
static volatile uint8_t s_config_in_progress = 0U;
static motor_control_mode_t s_control_mode = MOTOR_CONTROL_TORQUE;
static float s_left_dir = 1.0f;
static float s_right_dir = 1.0f;
static float s_left_cmd = 0.0f;
static float s_right_cmd = 0.0f;
static uint32_t s_left_parser_drops = 0U;
static uint32_t s_right_parser_drops = 0U;
static motor_telem_t s_left_telem;
static motor_telem_t s_right_telem;

static const char *motor_link_port_name(const motor_link_port_t *port)
{
    return (port == &s_left_port) ? "left" : "right";
}

static void motor_link_log_bytes(const char *label,
                                 const uint8_t *data,
                                 size_t len)
{
#if MOTOR_LINK_DEBUG
    if (label == NULL || data == NULL || len == 0U)
    {
        return;
    }
    size_t max_len = len;
    if (max_len > MOTOR_LINK_DEBUG_MAX_BYTES)
    {
        max_len = MOTOR_LINK_DEBUG_MAX_BYTES;
    }

    char line[APP_LOG_BUFFER_BYTES];
    int written = snprintf(line, sizeof(line), "%s (%u):", label,
                           (unsigned int)len);
    if (written < 0)
    {
        return;
    }

    for (size_t i = 0U; i < max_len; ++i)
    {
        if ((size_t)written >= sizeof(line))
        {
            break;
        }
        int ret = snprintf(line + written, sizeof(line) - (size_t)written,
                           " %02X", data[i]);
        if (ret < 0)
        {
            break;
        }
        written += ret;
    }

    if (max_len < len && (size_t)written < sizeof(line))
    {
        (void)snprintf(line + written, sizeof(line) - (size_t)written, " ...");
    }

    APP_LOG_INFO("%s", line);
#else
    (void)label;
    (void)data;
    (void)len;
#endif
}

static void motor_link_cache_clean(const void *data, size_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if ((SCB->CCR & SCB_CCR_DC_Msk) == 0U)
    {
        return;
    }
    if (data == NULL || len == 0U)
    {
        return;
    }
    uintptr_t start = (uintptr_t)data;
    uintptr_t end = start + len;
    uintptr_t aligned_start = start & ~(uintptr_t)(32U - 1U);
    uintptr_t aligned_end = (end + (32U - 1U)) & ~(uintptr_t)(32U - 1U);
    SCB_CleanDCache_by_Addr((uint32_t *)aligned_start,
                            (int32_t)(aligned_end - aligned_start));
#else
    (void)data;
    (void)len;
#endif
}

static void motor_link_cache_invalidate(const void *data, size_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if ((SCB->CCR & SCB_CCR_DC_Msk) == 0U)
    {
        return;
    }
    if (data == NULL || len == 0U)
    {
        return;
    }
    uintptr_t start = (uintptr_t)data;
    uintptr_t end = start + len;
    uintptr_t aligned_start = start & ~(uintptr_t)(32U - 1U);
    uintptr_t aligned_end = (end + (32U - 1U)) & ~(uintptr_t)(32U - 1U);
    SCB_InvalidateDCache_by_Addr((uint32_t *)aligned_start,
                                 (int32_t)(aligned_end - aligned_start));
#else
    (void)data;
    (void)len;
#endif
}

static motor_link_reg_type_t motor_link_reg_type(uint8_t reg)
{
    switch (reg)
    {
    case MOTOR_LINK_REG_STATUS:
    case MOTOR_LINK_REG_ENABLE:
    case MOTOR_LINK_REG_CONTROL_MODE:
    case MOTOR_LINK_REG_TORQUE_MODE:
    case MOTOR_LINK_REG_MODULATION_MODE:
        return MOTOR_LINK_RT_U8;
    case MOTOR_LINK_REG_TELEMETRY_DOWNSAMPLE:
        return MOTOR_LINK_RT_U32;
    case MOTOR_LINK_REG_POSITION:
        return MOTOR_LINK_RT_I32_F32;
    case MOTOR_LINK_REG_TELEMETRY_REG:
        return MOTOR_LINK_RT_BYTES;
    default:
        return MOTOR_LINK_RT_F32;
    }
}

static void motor_link_parser_feed(motor_link_parser_t *parser,
                                   const uint8_t *data,
                                   size_t len)
{
    if (parser == NULL || data == NULL || len == 0U)
    {
        return;
    }

    if (len > MOTOR_LINK_PARSER_BUFFER_BYTES)
    {
        data += (len - MOTOR_LINK_PARSER_BUFFER_BYTES);
        len = MOTOR_LINK_PARSER_BUFFER_BYTES;
    }

    if (parser->len + len > MOTOR_LINK_PARSER_BUFFER_BYTES)
    {
        size_t drop = (parser->len + len) - MOTOR_LINK_PARSER_BUFFER_BYTES;
        parser->drops += drop;
        memmove(parser->buf, parser->buf + drop, parser->len - drop);
        parser->len -= drop;
    }

    memcpy(parser->buf + parser->len, data, len);
    parser->len += len;
}

static bool motor_link_parser_try_parse(motor_link_parser_t *parser,
                                        uint8_t *out_type,
                                        const uint8_t **out_payload,
                                        uint16_t *out_len)
{
    if (parser == NULL || parser->len < 3U)
    {
        return false;
    }

    size_t idx = 0U;
    while (idx < parser->len && parser->buf[idx] != MOTOR_LINK_BIN_MARKER)
    {
        idx++;
    }
    if (idx > 0U)
    {
        memmove(parser->buf, parser->buf + idx, parser->len - idx);
        parser->len -= idx;
        if (parser->len < 3U)
        {
            return false;
        }
    }

    uint8_t size_field = parser->buf[1];
    if (size_field < 1U)
    {
        memmove(parser->buf, parser->buf + 1U, parser->len - 1U);
        parser->len -= 1U;
        return false;
    }

    size_t total = 2U + size_field;
    if (parser->len < total)
    {
        return false;
    }

    if (out_type != NULL)
    {
        *out_type = parser->buf[2];
    }
    if (out_payload != NULL)
    {
        *out_payload = (size_field > 1U) ? (parser->buf + 3U) : NULL;
    }
    if (out_len != NULL)
    {
        *out_len = (uint16_t)(size_field - 1U);
    }

    memmove(parser->buf, parser->buf + total, parser->len - total);
    parser->len -= total;
    return true;
}

static bool motor_link_ack_matches_expected(const motor_link_ack_t *ack,
                                            const motor_link_pending_t *pending)
{
    if (ack == NULL || pending == NULL)
    {
        return false;
    }
    if (!pending->active || pending->reg != ack->reg)
    {
        return false;
    }
    if (pending->reg == MOTOR_LINK_REG_TELEMETRY_REG)
    {
        return true;
    }
    if (ack->raw_len != pending->len)
    {
        return false;
    }

    if (pending->type == MOTOR_LINK_RT_F32 && ack->raw_len >= 4U)
    {
        float a = 0.0f;
        float e = 0.0f;
        memcpy(&a, ack->raw, 4U);
        memcpy(&e, pending->expected, 4U);
        return fabsf(a - e) <= MOTOR_LINK_FLOAT_EPS;
    }

    return (memcmp(ack->raw, pending->expected, ack->raw_len) == 0);
}

static void motor_link_clear_uart_errors(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_FLUSH_DRREGISTER(huart);
}

static void motor_link_port_init(motor_link_port_t *port,
                                 UART_HandleTypeDef *huart,
                                 uint8_t *rx_buf,
                                 uint16_t rx_len,
                                 uint8_t *tx_buf,
                                 uint16_t tx_len,
                                 uint8_t motor_id)
{
    memset(port, 0, sizeof(*port));
    port->huart = huart;
    port->rx_buf = rx_buf;
    port->rx_len = rx_len;
    port->tx_buf = tx_buf;
    port->tx_len = tx_len;
    port->rx_last_pos = 0U;
    port->motor_id = motor_id;
    port->telemetry_base_hz = MOTOR_LINK_TELEM_BASE_HZ;

    if (port->huart != NULL && port->rx_buf != NULL && port->rx_len > 0U)
    {
        motor_link_clear_uart_errors(port->huart);
        if (HAL_UART_Receive_DMA(port->huart, port->rx_buf, port->rx_len) == HAL_OK)
        {
            __HAL_DMA_DISABLE_IT(port->huart->hdmarx, DMA_IT_HT);
        }
    }
}

static bool motor_link_port_send_frame(motor_link_port_t *port,
                                       const uint8_t *frame,
                                       uint16_t len)
{
    if (port == NULL || port->huart == NULL || frame == NULL || len == 0U)
    {
        return false;
    }
    if (port->tx_buf == NULL || port->tx_len == 0U)
    {
        return false;
    }
    if (len > port->tx_len)
    {
        return false;
    }

    HAL_UART_StateTypeDef state = port->huart->gState;
    if (state == HAL_UART_STATE_BUSY_TX ||
        state == HAL_UART_STATE_BUSY_TX_RX ||
        state == HAL_UART_STATE_BUSY)
    {
#if MOTOR_LINK_DEBUG
        APP_LOG_ERROR("Motor link: %s TX busy, state=0x%lx",
                      motor_link_port_name(port), (unsigned long)state);
#endif
        return false;
    }

    memcpy(port->tx_buf, frame, len);
    motor_link_cache_clean(port->tx_buf, len);
    if (HAL_UART_Transmit_DMA(port->huart, port->tx_buf, len) != HAL_OK)
    {
#if MOTOR_LINK_DEBUG
        APP_LOG_ERROR("Motor link: %s TX DMA failed err=0x%lx",
                      motor_link_port_name(port),
                      (unsigned long)port->huart->ErrorCode);
#endif
        return false;
    }
    return true;
}

static bool motor_link_port_write_reg_raw(motor_link_port_t *port,
                                          uint8_t reg,
                                          const uint8_t *value,
                                          uint8_t len,
                                          bool track_pending)
{
    if (port == NULL || port->huart == NULL)
    {
        return false;
    }
    if (len > 16U)
    {
        return false;
    }

    uint8_t frame[2U + 1U + 1U + 16U];
    uint8_t size_field = (uint8_t)(1U + 1U + len);
    frame[0] = MOTOR_LINK_BIN_MARKER;
    frame[1] = size_field;
    frame[2] = MOTOR_LINK_TYPE_REG_REQ;
    frame[3] = reg;
    if (len > 0U && value != NULL)
    {
        memcpy(&frame[4], value, len);
    }

    if (track_pending)
    {
        port->pending.active = 1U;
        port->pending.reg = reg;
        port->pending.type = motor_link_reg_type(reg);
        port->pending.len = len;
        if (len > 0U && value != NULL)
        {
            memcpy(port->pending.expected, value, len);
        }
        port->pending.sent_ms = HAL_GetTick();
    }

    if (!motor_link_port_send_frame(port, frame, (uint16_t)(2U + size_field)))
    {
        if (track_pending)
        {
            port->pending.active = 0U;
        }
        return false;
    }

    return true;
}

static bool motor_link_port_write_reg_u8(motor_link_port_t *port,
                                         uint8_t reg,
                                         uint8_t value,
                                         bool track_pending)
{
    return motor_link_port_write_reg_raw(port, reg, &value, 1U, track_pending);
}

static bool motor_link_port_write_reg_u32(motor_link_port_t *port,
                                          uint8_t reg,
                                          uint32_t value,
                                          bool track_pending)
{
    uint8_t buf[4] = {(uint8_t)(value & 0xFFU),
                      (uint8_t)((value >> 8) & 0xFFU),
                      (uint8_t)((value >> 16) & 0xFFU),
                      (uint8_t)((value >> 24) & 0xFFU)};
    return motor_link_port_write_reg_raw(port, reg, buf, 4U, track_pending);
}

static bool motor_link_port_write_reg_f32(motor_link_port_t *port,
                                          uint8_t reg,
                                          float value,
                                          bool track_pending)
{
    uint8_t buf[4];
    memcpy(buf, &value, 4U);
    return motor_link_port_write_reg_raw(port, reg, buf, 4U, track_pending);
}

static void motor_link_port_handle_ack(motor_link_port_t *port,
                                       uint8_t reg,
                                       const uint8_t *payload,
                                       uint16_t len)
{
    if (port == NULL)
    {
        return;
    }

    motor_link_ack_t ack;
    memset(&ack, 0, sizeof(ack));
    ack.t_ms = HAL_GetTick();
    ack.reg = reg;
    ack.type = motor_link_reg_type(reg);
    ack.raw_len = (len > 16U) ? 16U : (uint8_t)len;
    if (ack.raw_len > 0U && payload != NULL)
    {
        memcpy(ack.raw, payload, ack.raw_len);
    }
    ack.valid = 1U;
    ack.ok = motor_link_ack_matches_expected(&ack, &port->pending) ? 1U : 0U;

#if MOTOR_LINK_DEBUG
    if (!ack.ok)
    {
        APP_LOG_ERROR("Motor link: %s ACK mismatch reg=0x%02X len=%u",
                      motor_link_port_name(port), (unsigned)reg,
                      (unsigned)ack.raw_len);
        if (ack.raw_len > 0U)
        {
            motor_link_log_bytes("ACK", ack.raw, ack.raw_len);
        }
    }
#endif

    if (ack.ok && port->pending.active && port->pending.reg == reg)
    {
        port->pending.active = 0U;
    }

    port->last_ack = ack;
    port->ack_seq++;
}

static void motor_link_port_handle_telemetry(motor_link_port_t *port,
                                             const uint8_t *payload,
                                             uint16_t len)
{
    if (port == NULL || payload == NULL || len < 1U)
    {
        return;
    }

    const uint8_t *data = payload + 1U;
    uint16_t data_len = (len > 0U) ? (uint16_t)(len - 1U) : 0U;

    if (data_len < 5U)
    {
        return;
    }

    float velocity = 0.0f;
    memcpy(&velocity, data, 4U);
    uint8_t status = data[4];

    motor_telem_t *telem = (port == &s_left_port) ? &s_left_telem : &s_right_telem;
    uint32_t now = HAL_GetTick();
    telem->velocity = velocity;
    telem->status = status;
    telem->last_vel_ms = now;
    telem->last_status_ms = now;
    telem->vel_valid = 1U;
    telem->status_valid = 1U;
}

static void motor_link_port_parse_frames(motor_link_port_t *port)
{
    if (port == NULL)
    {
        return;
    }

    uint8_t type = 0U;
    const uint8_t *payload = NULL;
    uint16_t plen = 0U;

    while (motor_link_parser_try_parse(&port->parser, &type, &payload, &plen))
    {
        if (type == MOTOR_LINK_TYPE_TELEM_DATA)
        {
            motor_link_port_handle_telemetry(port, payload, plen);
        }
        else if (type == MOTOR_LINK_TYPE_REG_RESP)
        {
            if (plen >= 1U)
            {
                uint8_t reg = payload[0];
                motor_link_port_handle_ack(port, reg, payload + 1U,
                                           (uint16_t)(plen - 1U));
#if MOTOR_LINK_DEBUG
                APP_LOG_INFO("Motor link: %s RX reg=0x%02X len=%u",
                             motor_link_port_name(port), (unsigned)reg,
                             (unsigned)(plen - 1U));
                if ((plen - 1U) > 0U)
                {
                    motor_link_log_bytes("RESP", payload + 1U,
                                         (size_t)(plen - 1U));
                }
#endif
            }
        }
    }

    if (port->parser.drops > 0U)
    {
        port->parser.drops_total += port->parser.drops;
        port->parser.drops = 0U;
    }
}

static void motor_link_port_pump_rx(motor_link_port_t *port)
{
    if (port == NULL || port->huart == NULL || port->huart->hdmarx == NULL)
    {
        return;
    }

    size_t buf_size = port->rx_len;
    if (buf_size == 0U)
    {
        return;
    }

    motor_link_cache_invalidate(port->rx_buf, buf_size);
    uint16_t remaining = __HAL_DMA_GET_COUNTER(port->huart->hdmarx);
    size_t pos = buf_size - (size_t)remaining;
    if (pos >= buf_size)
    {
        pos = 0U;
    }

    if (pos != port->rx_last_pos)
    {
        if (pos > port->rx_last_pos)
        {
            motor_link_parser_feed(&port->parser,
                                   &port->rx_buf[port->rx_last_pos],
                                   pos - port->rx_last_pos);
        }
        else
        {
            motor_link_parser_feed(&port->parser,
                                   &port->rx_buf[port->rx_last_pos],
                                   buf_size - port->rx_last_pos);
            if (pos > 0U)
            {
                motor_link_parser_feed(&port->parser, &port->rx_buf[0], pos);
            }
        }
        port->rx_last_pos = (uint16_t)pos;
        motor_link_port_parse_frames(port);
    }
}

static bool motor_link_port_wait_ack(motor_link_port_t *port,
                                     uint8_t reg,
                                     uint32_t timeout_ms,
                                     motor_link_ack_t *out_ack)
{
    uint32_t start = HAL_GetTick();
    uint32_t last_seq = port->last_ack_seq;

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        motor_link_port_pump_rx(port);
        if (port->ack_seq != last_seq)
        {
            last_seq = port->ack_seq;
            port->last_ack_seq = last_seq;
            if (port->last_ack.valid && port->last_ack.reg == reg)
            {
                if (out_ack != NULL)
                {
                    *out_ack = port->last_ack;
                }
                return port->last_ack.ok != 0U;
            }
        }
        HAL_Delay(1);
    }

    port->ack_timeouts++;
    if (out_ack != NULL)
    {
        memset(out_ack, 0, sizeof(*out_ack));
    }
    return false;
}

static bool motor_link_port_set_telemetry_registers(motor_link_port_t *port,
                                                    const uint8_t *regs,
                                                    size_t count,
                                                    uint8_t motor_id,
                                                    uint32_t timeout_ms)
{
    if (regs == NULL || count == 0U)
    {
        return false;
    }

    size_t value_len = 1U + 2U * count;
    if (value_len > 16U)
    {
        return false;
    }

    uint8_t value[16];
    value[0] = (uint8_t)count;
    for (size_t i = 0U; i < count; ++i)
    {
        value[1U + 2U * i] = motor_id;
        value[1U + 2U * i + 1U] = regs[i];
    }

    if (!motor_link_port_write_reg_raw(port, MOTOR_LINK_REG_TELEMETRY_REG,
                                       value, (uint8_t)value_len, true))
    {
        return false;
    }

    return motor_link_port_wait_ack(port, MOTOR_LINK_REG_TELEMETRY_REG,
                                    timeout_ms, NULL);
}

static bool motor_link_port_set_telemetry_downsample(motor_link_port_t *port,
                                                     uint32_t downsample,
                                                     uint32_t timeout_ms,
                                                     bool wait_ack)
{
    if (downsample == 0U)
    {
        downsample = 1U;
    }

    if (!motor_link_port_write_reg_u32(port, MOTOR_LINK_REG_TELEMETRY_DOWNSAMPLE,
                                       downsample, wait_ack))
    {
        return false;
    }

    if (!wait_ack)
    {
        return true;
    }

    return motor_link_port_wait_ack(port, MOTOR_LINK_REG_TELEMETRY_DOWNSAMPLE,
                                    timeout_ms, NULL);
}

static bool motor_link_port_set_telemetry_rate(motor_link_port_t *port,
                                               float target_hz,
                                               uint32_t timeout_ms)
{
    if (target_hz <= 0.0f || port->telemetry_base_hz <= 0.0f)
    {
        return false;
    }

    float ds_f = port->telemetry_base_hz / target_hz;
    uint32_t ds = (uint32_t)lroundf(ds_f);
    if (ds < 1U)
    {
        ds = 1U;
    }

    return motor_link_port_set_telemetry_downsample(port, ds, timeout_ms, true);
}

static bool motor_link_port_wait_for_traffic(motor_link_port_t *port,
                                             uint32_t timeout_ms)
{
    if (port == NULL)
    {
        return false;
    }

    uint32_t start = HAL_GetTick();
    uint16_t last_pos = port->rx_last_pos;

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        motor_link_port_pump_rx(port);
        if (port->rx_last_pos != last_pos)
        {
            return true;
        }
        HAL_Delay(10);
    }
    return false;
}

static float motor_link_clamp(float v, float limit)
{
    if (limit <= 0.0f)
    {
        return v;
    }
    if (v > limit)
    {
        return limit;
    }
    if (v < -limit)
    {
        return -limit;
    }
    return v;
}

static float motor_link_resolve_kt(void)
{
    if (MOTOR_LINK_KT_NM_PER_A > 0.0f)
    {
        return MOTOR_LINK_KT_NM_PER_A;
    }
    if (MOTOR_LINK_KV_RPM_PER_V > 0.0f)
    {
        return 60.0f / (2.0f * MOTOR_LINK_PI * MOTOR_LINK_KV_RPM_PER_V);
    }
    return 0.0f;
}

bool motor_link_init(void)
{
    memset(&s_left_telem, 0, sizeof(s_left_telem));
    memset(&s_right_telem, 0, sizeof(s_right_telem));
    s_left_cmd = 0.0f;
    s_right_cmd = 0.0f;
    s_control_mode = MOTOR_CONTROL_TORQUE;
    s_enable_applied = 0U;
    s_left_dir = 1.0f;
    s_right_dir = 1.0f;
    s_left_parser_drops = 0U;
    s_right_parser_drops = 0U;

    s_config_in_progress = 1U;
    motor_link_port_init(&s_left_port, &huart1, s_left_rx_buffer,
                         (uint16_t)sizeof(s_left_rx_buffer),
                         s_left_tx_buffer,
                         (uint16_t)sizeof(s_left_tx_buffer), 0U);
    motor_link_port_init(&s_right_port, &huart6, s_right_rx_buffer,
                         (uint16_t)sizeof(s_right_rx_buffer),
                         s_right_tx_buffer,
                         (uint16_t)sizeof(s_right_tx_buffer), 0U);

    (void)motor_link_port_wait_for_traffic(&s_left_port, 2000U);
    (void)motor_link_port_wait_for_traffic(&s_right_port, 2000U);

    (void)motor_link_port_set_telemetry_downsample(&s_left_port,
                                                   MOTOR_LINK_CONFIG_DOWNSAMPLE,
                                                   MOTOR_LINK_ACK_TIMEOUT_MS,
                                                   false);
    (void)motor_link_port_set_telemetry_downsample(&s_right_port,
                                                   MOTOR_LINK_CONFIG_DOWNSAMPLE,
                                                   MOTOR_LINK_ACK_TIMEOUT_MS,
                                                   false);
    HAL_Delay(400);

    const uint8_t telem_regs[] = {MOTOR_LINK_REG_VELOCITY, MOTOR_LINK_REG_STATUS};
        while (!motor_link_port_set_telemetry_registers(&s_left_port, telem_regs,
                                                    sizeof(telem_regs), 0U,
                                                    MOTOR_LINK_ACK_TIMEOUT_MS))
    {
        APP_LOG_ERROR("Motor link: left telemetry setup failed, retrying");
    }

        while (!motor_link_port_set_telemetry_registers(&s_right_port, telem_regs,
                                                    sizeof(telem_regs), 0U,
                                                    MOTOR_LINK_ACK_TIMEOUT_MS))
    {
        APP_LOG_ERROR("Motor link: right telemetry setup failed, retrying");
    }






    for (int attempt = 0; attempt < 10; ++attempt)
    {
        if (motor_link_port_write_reg_u8(&s_left_port,
                                         MOTOR_LINK_REG_CONTROL_MODE,
                                         (uint8_t)MOTOR_CONTROL_TORQUE, true) &&
            motor_link_port_wait_ack(&s_left_port, MOTOR_LINK_REG_CONTROL_MODE,
                                     MOTOR_LINK_ACK_TIMEOUT_MS, NULL))
        {
            break;
        }
        HAL_Delay(10);
    }
    for (int attempt = 0; attempt < 10; ++attempt)
    {
        if (motor_link_port_write_reg_u8(&s_right_port,
                                         MOTOR_LINK_REG_CONTROL_MODE,
                                         (uint8_t)MOTOR_CONTROL_TORQUE, true) &&
            motor_link_port_wait_ack(&s_right_port, MOTOR_LINK_REG_CONTROL_MODE,
                                     MOTOR_LINK_ACK_TIMEOUT_MS, NULL))
        {
            break;
        }
        HAL_Delay(10);
    }

    for (int attempt = 0; attempt < 10; ++attempt)
    {
        if (motor_link_port_write_reg_u8(&s_left_port,
                                         MOTOR_LINK_REG_MODULATION_MODE, 1U,
                                         true) &&
            motor_link_port_wait_ack(&s_left_port,
                                     MOTOR_LINK_REG_MODULATION_MODE,
                                     MOTOR_LINK_ACK_TIMEOUT_MS, NULL))
        {
            break;
        }
        HAL_Delay(10);
    }
    for (int attempt = 0; attempt < 10; ++attempt)
    {
        if (motor_link_port_write_reg_u8(&s_right_port,
                                         MOTOR_LINK_REG_MODULATION_MODE, 1U,
                                         true) &&
            motor_link_port_wait_ack(&s_right_port,
                                     MOTOR_LINK_REG_MODULATION_MODE,
                                     MOTOR_LINK_ACK_TIMEOUT_MS, NULL))
        {
            break;
        }
        HAL_Delay(10);
    }

    for (int attempt = 0; attempt < 10; ++attempt)
    {
        if (motor_link_port_write_reg_u8(&s_left_port, MOTOR_LINK_REG_ENABLE, 0U,
                                         true) &&
            motor_link_port_wait_ack(&s_left_port, MOTOR_LINK_REG_ENABLE,
                                     MOTOR_LINK_ACK_TIMEOUT_MS, NULL))
        {
            break;
        }
        HAL_Delay(10);
    }
    for (int attempt = 0; attempt < 10; ++attempt)
    {
        if (motor_link_port_write_reg_u8(&s_right_port, MOTOR_LINK_REG_ENABLE,
                                         0U, true) &&
            motor_link_port_wait_ack(&s_right_port, MOTOR_LINK_REG_ENABLE,
                                     MOTOR_LINK_ACK_TIMEOUT_MS, NULL))
        {
            break;
        }
        HAL_Delay(10);
    }

    while (!motor_link_port_set_telemetry_rate(&s_left_port, MOTOR_LINK_TELEM_RATE_HZ,
                                               MOTOR_LINK_ACK_TIMEOUT_MS))
    {
        APP_LOG_ERROR("Motor link: left telemetry rate not confirmed, retrying");
    }
    while (!motor_link_port_set_telemetry_rate(&s_right_port, MOTOR_LINK_TELEM_RATE_HZ,
                                               MOTOR_LINK_ACK_TIMEOUT_MS))
    {
        APP_LOG_ERROR("Motor link: right telemetry rate not confirmed, retrying");
    }

    s_initialized = 1U;
    s_config_in_progress = 0U;
    APP_LOG_INFO("Motor link initialized (BinaryIO)");
    return true;
}

void motor_link_poll(void)
{
    if (!s_initialized)
    {
        return;
    }

    motor_link_port_pump_rx(&s_left_port);
    motor_link_port_pump_rx(&s_right_port);
    s_left_parser_drops = s_left_port.parser.drops_total;
    s_right_parser_drops = s_right_port.parser.drops_total;
}

void motor_link_enable(bool on)
{
    if (!s_initialized)
    {
        return;
    }

    s_enable_applied = on ? 1U : 0U;

#if defined(MOTORS_ENABLE_GPIO_Port) && defined(MOTORS_ENABLE_Pin)
    HAL_GPIO_WritePin(MOTORS_ENABLE_GPIO_Port,
                      MOTORS_ENABLE_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif

    (void)motor_link_port_write_reg_u8(&s_left_port, MOTOR_LINK_REG_ENABLE,
                                       on ? 1U : 0U, false);
    (void)motor_link_port_write_reg_u8(&s_right_port, MOTOR_LINK_REG_ENABLE,
                                       on ? 1U : 0U, false);
}

void motor_link_set_control_mode(motor_control_mode_t mode)
{
    if (!s_initialized)
    {
        return;
    }

    s_control_mode = mode;
    s_config_in_progress = 1U;
    for (int attempt = 0; attempt < 5; ++attempt)
    {
        if (motor_link_port_write_reg_u8(&s_left_port,
                                         MOTOR_LINK_REG_CONTROL_MODE,
                                         (uint8_t)mode, true) &&
            motor_link_port_wait_ack(&s_left_port, MOTOR_LINK_REG_CONTROL_MODE,
                                     MOTOR_LINK_ACK_TIMEOUT_MS, NULL))
        {
            break;
        }
        HAL_Delay(10);
    }
    for (int attempt = 0; attempt < 5; ++attempt)
    {
        if (motor_link_port_write_reg_u8(&s_right_port,
                                         MOTOR_LINK_REG_CONTROL_MODE,
                                         (uint8_t)mode, true) &&
            motor_link_port_wait_ack(&s_right_port, MOTOR_LINK_REG_CONTROL_MODE,
                                     MOTOR_LINK_ACK_TIMEOUT_MS, NULL))
        {
            break;
        }
        HAL_Delay(10);
    }
    s_config_in_progress = 0U;
}

motor_control_mode_t motor_link_get_control_mode(void)
{
    return s_control_mode;
}

void motor_link_set_wheel_Iq(float left_A, float right_A, float max_A)
{
    if (!s_initialized || s_control_mode != MOTOR_CONTROL_TORQUE)
    {
        return;
    }

    float left = motor_link_clamp(left_A * s_left_dir, max_A);
    float right = motor_link_clamp(right_A * s_right_dir, max_A);
    if (!s_enable_applied)
    {
        left = 0.0f;
        right = 0.0f;
    }

    s_left_cmd = left;
    s_right_cmd = right;

    if (s_config_in_progress)
    {
        return;
    }
    if (!s_left_port.pending.active)
    {
        (void)motor_link_port_write_reg_f32(&s_left_port, MOTOR_LINK_REG_TARGET,
                                            left, false);
    }
    if (!s_right_port.pending.active)
    {
        (void)motor_link_port_write_reg_f32(&s_right_port, MOTOR_LINK_REG_TARGET,
                                            right, false);
    }
}

void motor_link_set_wheel_torques(float left_Nm, float right_Nm, float max_Nm)
{
    float kt = motor_link_resolve_kt();
    if (kt <= 0.0f)
    {
        return;
    }

    float left_A = left_Nm / kt;
    float right_A = right_Nm / kt;
    float max_A = (max_Nm > 0.0f) ? (max_Nm / kt) : 0.0f;
    motor_link_set_wheel_Iq(left_A, right_A, max_A);
}

bool motor_link_get_wheel_velocities(float *left_rad_s, float *right_rad_s)
{
    if (left_rad_s == NULL || right_rad_s == NULL)
    {
        return false;
    }

    uint32_t now = HAL_GetTick();
    uint32_t age_left = s_left_telem.last_vel_ms ? (now - s_left_telem.last_vel_ms) : UINT32_MAX;
    uint32_t age_right = s_right_telem.last_vel_ms ? (now - s_right_telem.last_vel_ms) : UINT32_MAX;

    bool left_stale = !s_left_telem.vel_valid || age_left > MOTOR_LINK_TELEM_STALE_MS;
    bool right_stale = !s_right_telem.vel_valid || age_right > MOTOR_LINK_TELEM_STALE_MS;
    bool left_late = !left_stale && age_left > MOTOR_LINK_TELEM_WARN_MS;
    bool right_late = !right_stale && age_right > MOTOR_LINK_TELEM_WARN_MS;

    *left_rad_s = left_stale ? 0.0f : s_left_telem.velocity;
    *right_rad_s = right_stale ? 0.0f : s_right_telem.velocity;

    if (left_stale)
    {
        s_left_telem.stale_count++;
    }
    else if (left_late)
    {
        s_left_telem.late_count++;
    }
    if (right_stale)
    {
        s_right_telem.stale_count++;
    }
    else if (right_late)
    {
        s_right_telem.late_count++;
    }

    return !left_stale && !right_stale;
}

uint32_t motor_link_get_left_parser_drops(void)
{
    return s_left_parser_drops;
}

uint32_t motor_link_get_right_parser_drops(void)
{
    return s_right_parser_drops;
}

uint32_t motor_link_get_left_telem_stale(void)
{
    return s_left_telem.stale_count;
}

uint32_t motor_link_get_right_telem_stale(void)
{
    return s_right_telem.stale_count;
}

uint32_t motor_link_get_left_telem_late(void)
{
    return s_left_telem.late_count;
}

uint32_t motor_link_get_right_telem_late(void)
{
    return s_right_telem.late_count;
}
