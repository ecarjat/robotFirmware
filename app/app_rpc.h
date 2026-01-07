#ifndef APP_RPC_H
#define APP_RPC_H

#include "app_main.h"
#include "robot_protocol.h"
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_rpc_handle(const robot_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* APP_RPC_H */