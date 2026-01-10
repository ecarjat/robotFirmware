#ifndef UTILS_LOG_H
#define UTILS_LOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void app_log_printf(const char *fmt, ...);
void app_log_usb_tx_complete(uint16_t len);
void app_log_flush_blocking(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* UTILS_LOG_H */
