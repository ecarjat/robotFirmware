#ifndef APP_FILE_H
#define APP_FILE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize file transfer module
 */
void app_file_init(void);

/**
 * @brief Handle incoming file channel messages
 *
 * @param msg_type Message type (ROBOT_MSG_FILE_*)
 * @param seq Sequence number for response
 * @param payload Message payload
 * @param payload_len Payload length
 * @return true if message was handled
 */
bool app_file_handle_message(uint8_t msg_type,
                              uint16_t seq,
                              const uint8_t *payload,
                              uint16_t payload_len);

#ifdef __cplusplus
}
#endif

#endif /* APP_FILE_H */
