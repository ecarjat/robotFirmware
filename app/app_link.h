#ifndef APP_LINK_H
#define APP_LINK_H

#include "robot_protocol.h"
#include "stm32h7xx_hal.h"

#define APP_LINK_RX_BUFFER_BYTES 512U
#define APP_LINK_TX_BUFFER_BYTES 320U
#ifndef APP_LINK_DEBUG_FRAMES
#define APP_LINK_DEBUG_FRAMES 1U
#endif
#ifndef APP_LINK_DEBUG_MAX_REPORTS
#define APP_LINK_DEBUG_MAX_REPORTS 5U
#endif
#ifndef APP_LINK_DEBUG_MAX_BYTES
#define APP_LINK_DEBUG_MAX_BYTES 48U
#endif
#define APP_LINK_FRAME_BUFFER_BYTES ROBOT_FRAME_MAX_ENCODED
#define APP_LINK_TX_QUEUE_DEPTH 4

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  APP_LINK_SEND_OK = 0,
  APP_LINK_SEND_ERR_UART_NULL,
  APP_LINK_SEND_ERR_ISR,
  APP_LINK_SEND_ERR_FRAME_INIT,
  APP_LINK_SEND_ERR_ENCODE,
  APP_LINK_SEND_ERR_CTS_BLOCKED,
  APP_LINK_SEND_ERR_UART_BUSY,
  APP_LINK_SEND_ERR_UART_TIMEOUT,
  APP_LINK_SEND_ERR_UART_ERROR,
} app_link_send_err_t;

extern uint32_t s_last_cmd_ms;

void app_link_get_last_send_error(app_link_send_err_t *err, uint32_t *status,
                                  uint32_t *hal_state, uint32_t *hal_err);
void app_link_start(void);
void app_link_poll(void);
void app_log_link_errors(void);
void app_link_process_chunk(const uint8_t *data, size_t len);
bool app_link_send(uint8_t type, uint16_t flags, const uint8_t *payload,
                   uint16_t len, uint16_t seq_override);
void app_link_debug_frame(const uint8_t *frame, size_t len);
void app_link_dispatch(const robot_frame_t *frame);
const char *app_link_send_err_str(app_link_send_err_t err);

#ifdef __cplusplus
}
#endif

#endif /* APP_LINK_H */