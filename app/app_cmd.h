#ifndef APP_CMD_H
#define APP_CMD_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void app_cmd_handler(uint8_t msg_type, const uint8_t *payload,
                            size_t len, void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* APP_CMD_H */