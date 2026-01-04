#ifndef MOTOR_LINK_FRAMING_H
#define MOTOR_LINK_FRAMING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Robust binary framing with byte stuffing and CRC-32.
 *
 * Frame format:
 *   [0xA5][LEN][TYPE][PAYLOAD...][CRC32]
 *
 * - 0xA5: Frame marker (never appears in stuffed data)
 * - LEN:  Unescaped length of TYPE + PAYLOAD + CRC32
 * - TYPE: Packet type byte
 * - PAYLOAD: Variable length data
 * - CRC32: CRC-32 (polynomial 0x04C11DB7) over unescaped LEN+TYPE+PAYLOAD
 *         Uses same polynomial as STM32 hardware CRC peripheral
 *
 * Byte stuffing (SLIP-style):
 *   0xA5 -> 0xDB 0xDC
 *   0xDB -> 0xDB 0xDD
 */

#define FRAME_MARKER      0xA5U
#define FRAME_ESC         0xDBU
#define FRAME_ESC_MARKER  0xDCU  /* 0xDB 0xDC = 0xA5 */
#define FRAME_ESC_ESC     0xDDU  /* 0xDB 0xDD = 0xDB */

#define FRAME_MAX_UNESCAPED_SIZE  64U
#define FRAME_MAX_ESCAPED_SIZE    (2U + FRAME_MAX_UNESCAPED_SIZE * 2U)
#define FRAME_CRC_SIZE            4U

/*
 * CRC-32 (polynomial 0x04C11DB7) - compatible with STM32 hardware CRC.
 * On STM32H7, uses hardware CRC peripheral for acceleration.
 */
uint32_t frame_crc32(const uint8_t *data, size_t len);

/*
 * Encode a frame with byte stuffing and CRC.
 *
 * @param type     Packet type byte
 * @param payload  Payload data (can be NULL if payload_len == 0)
 * @param payload_len  Length of payload
 * @param out      Output buffer (must be at least FRAME_MAX_ESCAPED_SIZE)
 * @param out_size Size of output buffer
 * @return         Number of bytes written to out, or 0 on error
 */
size_t frame_encode(uint8_t type,
                    const uint8_t *payload,
                    size_t payload_len,
                    uint8_t *out,
                    size_t out_size);

/*
 * Frame parser state machine.
 */
typedef enum {
    FRAME_STATE_IDLE,       /* Waiting for marker */
    FRAME_STATE_LEN,        /* Reading length byte */
    FRAME_STATE_DATA,       /* Reading type+payload+crc */
    FRAME_STATE_ESC         /* Previous byte was escape */
} frame_parse_state_t;

typedef struct {
    frame_parse_state_t state;
    uint8_t buf[FRAME_MAX_UNESCAPED_SIZE];
    uint8_t expected_len;   /* Unescaped length (type+payload+crc) */
    uint8_t pos;            /* Current position in buf */
    uint8_t esc_pending;    /* In escape sequence */
    uint32_t sync_losses;   /* Counter for debugging */
    uint32_t crc_errors;    /* Counter for debugging */
} frame_parser_t;

/*
 * Initialize frame parser.
 */
void frame_parser_init(frame_parser_t *parser);

/*
 * Feed bytes to the parser.
 *
 * @param parser   Parser state
 * @param data     Input bytes
 * @param len      Number of input bytes
 */
void frame_parser_feed(frame_parser_t *parser, const uint8_t *data, size_t len);

/*
 * Try to extract a complete frame.
 *
 * @param parser   Parser state
 * @param out_type Receives packet type (can be NULL)
 * @param out_payload  Receives pointer to payload in parser buffer (can be NULL)
 * @param out_len  Receives payload length (can be NULL)
 * @return         true if a valid frame was extracted
 */
bool frame_parser_pop(frame_parser_t *parser,
                      uint8_t *out_type,
                      const uint8_t **out_payload,
                      uint8_t *out_len);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_LINK_FRAMING_H */
