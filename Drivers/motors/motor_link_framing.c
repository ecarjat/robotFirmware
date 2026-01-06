#include "motor_link_framing.h"
#include <string.h>

/*
 * STM32 Hardware CRC support.
 * The STM32H7 and STM32F1 both have CRC peripherals with polynomial 0x04C11DB7.
 */
#if defined(STM32H7xx) || defined(STM32H723xx) || defined(STM32H743xx) || defined(STM32H750xx)
#include "stm32h7xx_hal.h"
#define USE_HW_CRC 1
#elif defined(STM32F1xx) || defined(STM32F103xB)
#include "stm32f1xx_hal.h"
#define USE_HW_CRC 1
#else
#define USE_HW_CRC 0
#endif

#if USE_HW_CRC
extern CRC_HandleTypeDef hcrc;  /* Must be defined and initialized in main.c */
#pragma message("motor_link_framing: Using HARDWARE CRC")
#else
#pragma message("motor_link_framing: Using SOFTWARE CRC (no hardware acceleration)")
#endif

/*
 * CRC-32 implementation.
 * Uses hardware CRC peripheral if available, otherwise software fallback.
 * Polynomial: 0x04C11DB7 (same as Ethernet CRC, STM32 hardware CRC)
 */
uint32_t frame_crc32(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0U) {
        return 0xFFFFFFFFU;
    }

#if USE_HW_CRC
    /*
     * STM32 hardware CRC operates on 32-bit words.
     * We need to handle byte-level data carefully.
     *
     * The H7 CRC peripheral supports configurable input data size,
     * but for compatibility we'll use word-aligned access.
     */
    __HAL_CRC_DR_RESET(&hcrc);

    /* Process full 32-bit words */
    size_t words = len / 4U;
    const uint32_t *word_ptr = (const uint32_t *)data;

    for (size_t i = 0U; i < words; i++) {
        hcrc.Instance->DR = __REV(word_ptr[i]);  /* Reverse byte order for CRC */
    }

    /* Handle remaining bytes (0-3) */
    size_t remaining = len % 4U;
    if (remaining > 0U) {
        const uint8_t *tail = data + (words * 4U);
#if defined(STM32H7xx) || defined(STM32H723xx) || defined(STM32H743xx) || defined(STM32H750xx)
        /* H7 supports 8-bit input directly; pad with zeros to 32-bit boundary */
        for (size_t i = 0U; i < remaining; i++) {
            *((volatile uint8_t *)&hcrc.Instance->DR) = tail[i];
        }
        for (size_t i = remaining; i < 4U; i++) {
            *((volatile uint8_t *)&hcrc.Instance->DR) = 0U;
        }
#else
        /* F1 only supports 32-bit, pack remaining bytes and pad with zeros */
        uint32_t last_word = 0U;
        for (size_t i = 0U; i < remaining; i++) {
            last_word |= ((uint32_t)tail[i]) << (24U - (i * 8U));
        }
        hcrc.Instance->DR = last_word;
#endif
    }

    return hcrc.Instance->DR;

#else
    /*
     * Software CRC-32 fallback (polynomial 0x04C11DB7).
     * Bit-by-bit implementation - slower but portable.
     */
    uint32_t crc = 0xFFFFFFFFU;
    size_t pad_len = (4U - (len & 3U)) & 3U;
    for (size_t i = 0U; i < (len + pad_len); i++) {
        uint8_t byte = (i < len) ? data[i] : 0U;
        crc ^= ((uint32_t)byte) << 24U;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80000000U) {
                crc = (crc << 1U) ^ 0x04C11DB7U;
            } else {
                crc <<= 1U;
            }
        }
    }
    return crc;
#endif
}

/*
 * Escape a single byte into the output buffer.
 * Returns number of bytes written (1 or 2).
 */
static size_t escape_byte(uint8_t byte, uint8_t *out, size_t out_remaining)
{
    if (byte == FRAME_MARKER) {
        if (out_remaining < 2U) {
            return 0U;
        }
        out[0] = FRAME_ESC;
        out[1] = FRAME_ESC_MARKER;
        return 2U;
    }
    if (byte == FRAME_ESC) {
        if (out_remaining < 2U) {
            return 0U;
        }
        out[0] = FRAME_ESC;
        out[1] = FRAME_ESC_ESC;
        return 2U;
    }
    if (out_remaining < 1U) {
        return 0U;
    }
    out[0] = byte;
    return 1U;
}

size_t frame_encode(uint8_t type,
                    const uint8_t *payload,
                    size_t payload_len,
                    uint8_t *out,
                    size_t out_size)
{
    if (out == NULL || out_size < 8U) {
        return 0U;
    }

    /* Unescaped length: TYPE(1) + PAYLOAD(N) + CRC32(4) */
    size_t unescaped_len = 1U + payload_len + FRAME_CRC_SIZE;
    if (unescaped_len > 255U) {
        return 0U;  /* Length field is 1 byte */
    }
    if ((1U + unescaped_len) > FRAME_MAX_UNESCAPED_SIZE) {
        return 0U;  /* LEN + TYPE + PAYLOAD + CRC must fit parser buffer */
    }

    /* Build unescaped frame for CRC calculation: [LEN][TYPE][PAYLOAD] */
    uint8_t temp[FRAME_MAX_UNESCAPED_SIZE];
    if (1U + 1U + payload_len > sizeof(temp)) {
        return 0U;
    }
    temp[0] = (uint8_t)unescaped_len;
    temp[1] = type;
    if (payload_len > 0U && payload != NULL) {
        memcpy(&temp[2], payload, payload_len);
    }

    /* Calculate CRC-32 over LEN + TYPE + PAYLOAD */
    uint32_t crc = frame_crc32(temp, 1U + 1U + payload_len);

    /* Now encode with escaping: MARKER + escaped(LEN, TYPE, PAYLOAD, CRC32) */
    size_t pos = 0U;
    out[pos++] = FRAME_MARKER;

    /* Escape LEN */
    size_t written = escape_byte(temp[0], &out[pos], out_size - pos);
    if (written == 0U) {
        return 0U;
    }
    pos += written;

    /* Escape TYPE */
    written = escape_byte(type, &out[pos], out_size - pos);
    if (written == 0U) {
        return 0U;
    }
    pos += written;

    /* Escape PAYLOAD */
    for (size_t i = 0U; i < payload_len; i++) {
        written = escape_byte(payload[i], &out[pos], out_size - pos);
        if (written == 0U) {
            return 0U;
        }
        pos += written;
    }

    /* Escape CRC32 (4 bytes, little-endian) */
    for (size_t i = 0U; i < FRAME_CRC_SIZE; i++) {
        uint8_t crc_byte = (uint8_t)(crc >> (i * 8U));
        written = escape_byte(crc_byte, &out[pos], out_size - pos);
        if (written == 0U) {
            return 0U;
        }
        pos += written;
    }

    return pos;
}

void frame_parser_init(frame_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->state = FRAME_STATE_IDLE;
}

static void frame_parser_reset(frame_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }
    parser->state = FRAME_STATE_IDLE;
    parser->pos = 0U;
    parser->expected_len = 0U;
    parser->esc_pending = 0U;
}

/*
 * Process a single unescaped byte.
 */
static void parser_handle_byte(frame_parser_t *parser, uint8_t byte)
{
    switch (parser->state) {
    case FRAME_STATE_IDLE:
        /* Waiting for marker - ignore other bytes */
        if (byte == FRAME_MARKER) {
            parser->state = FRAME_STATE_LEN;
            parser->pos = 0U;
            parser->esc_pending = 0U;
        }
        break;

    case FRAME_STATE_LEN:
        /* First byte after marker is length */
        parser->expected_len = byte;
        /* Minimum length: TYPE(1) + CRC32(4) = 5 */
        if (parser->expected_len < (1U + FRAME_CRC_SIZE) ||
            ((size_t)parser->expected_len + 1U) > sizeof(parser->buf)) {
            /* Invalid length - back to idle */
            parser->sync_losses++;
            frame_parser_reset(parser);
        } else {
            /* Store length in buffer for CRC calculation */
            parser->buf[0] = byte;
            parser->pos = 1U;
            parser->state = FRAME_STATE_DATA;
        }
        break;

    case FRAME_STATE_DATA:
        /* Accumulate TYPE + PAYLOAD + CRC */
        if (parser->pos >= sizeof(parser->buf)) {
            parser->sync_losses++;
            frame_parser_reset(parser);
            break;
        }
        parser->buf[parser->pos++] = byte;
        /* Check if we have all data: LEN(1) + TYPE(1) + PAYLOAD(N) + CRC(1) = expected_len + 1 */
        if (parser->pos >= (size_t)(parser->expected_len + 1U)) {
            parser->state = FRAME_STATE_IDLE;
            /* Frame complete - will be validated in frame_parser_pop */
        }
        break;

    default:
        parser->state = FRAME_STATE_IDLE;
        break;
    }
}

void frame_parser_feed(frame_parser_t *parser, const uint8_t *data, size_t len)
{
    if (parser == NULL || data == NULL || len == 0U) {
        return;
    }

    for (size_t i = 0U; i < len; i++) {
        uint8_t byte = data[i];

        /* Handle marker - always resets state (except during escape) */
        if (byte == FRAME_MARKER && !parser->esc_pending) {
            if (parser->state != FRAME_STATE_IDLE &&
                parser->state != FRAME_STATE_LEN) {
                /* Unexpected marker in middle of frame - sync loss */
                parser->sync_losses++;
            }
            parser->state = FRAME_STATE_LEN;
            parser->pos = 0U;
            parser->esc_pending = 0U;
            continue;
        }

        /* Handle escape sequences */
        if (parser->esc_pending) {
            parser->esc_pending = 0U;
            if (byte == FRAME_ESC_MARKER) {
                parser_handle_byte(parser, FRAME_MARKER);
            } else if (byte == FRAME_ESC_ESC) {
                parser_handle_byte(parser, FRAME_ESC);
            } else {
                /* Invalid escape sequence - sync loss */
                parser->sync_losses++;
                frame_parser_reset(parser);
            }
            continue;
        }

        if (byte == FRAME_ESC) {
            parser->esc_pending = 1U;
            continue;
        }

        /* Regular byte */
        parser_handle_byte(parser, byte);
    }
}

bool frame_parser_pop(frame_parser_t *parser,
                      uint8_t *out_type,
                      const uint8_t **out_payload,
                      uint8_t *out_len)
{
    if (parser == NULL) {
        return false;
    }

    /* Check if we have a complete frame */
    if (parser->state != FRAME_STATE_IDLE) {
        return false;
    }
    if (parser->pos < (1U + 1U + FRAME_CRC_SIZE)) {  /* Minimum: LEN + TYPE + CRC32 */
        return false;
    }

    uint8_t len_field = parser->buf[0];
    if (parser->pos != (size_t)(len_field + 1U)) {
        /* Size mismatch */
        frame_parser_reset(parser);
        return false;
    }

    /* Minimum frame: LEN + TYPE + CRC32 = 1 + 1 + 4 = 6 bytes */
    if (parser->pos < (1U + 1U + FRAME_CRC_SIZE)) {
        frame_parser_reset(parser);
        return false;
    }

    /* Verify CRC32: computed over buf[0..pos-5], stored in buf[pos-4..pos-1] */
    uint32_t computed_crc = frame_crc32(parser->buf, parser->pos - FRAME_CRC_SIZE);
    uint32_t received_crc = 0U;
    for (size_t i = 0U; i < FRAME_CRC_SIZE; i++) {
        received_crc |= ((uint32_t)parser->buf[parser->pos - FRAME_CRC_SIZE + i]) << (i * 8U);
    }

    if (computed_crc != received_crc) {
        parser->crc_errors++;
        frame_parser_reset(parser);
        return false;
    }

    /* Valid frame! Extract fields */
    /* buf[0] = LEN, buf[1] = TYPE, buf[2..pos-5] = PAYLOAD, buf[pos-4..pos-1] = CRC32 */
    if (out_type != NULL) {
        *out_type = parser->buf[1];
    }

    /* Payload length = LEN - TYPE(1) - CRC32(4) */
    size_t payload_len = (len_field > (1U + FRAME_CRC_SIZE)) ? (len_field - 1U - FRAME_CRC_SIZE) : 0U;
    if (out_payload != NULL) {
        *out_payload = (payload_len > 0U) ? &parser->buf[2] : NULL;
    }
    if (out_len != NULL) {
        *out_len = (uint8_t)payload_len;
    }

    /* Clear for next frame */
    frame_parser_reset(parser);
    return true;
}
