#ifndef STM32_APP_CONFIG_H
#define STM32_APP_CONFIG_H

#include "log.h"
#include "app_log_macros.h"
#include "main.h"
#include "usb_device.h"

#define APP_IDLE_TICK_MS 300U
#define APP_IDLE_BUDGET_US 1216U
#define APP_POWER_SETTLE_DELAY_MS 3000U
#define APP_HEARTBEAT_TIMEOUT_MS 200U
#define APP_LOG_PERIOD_MS 500U
#define APP_LOG_FLUSH_TIMEOUT_MS 2000U
#define APP_TELEM_PERIOD_MS 500U
#define TELEM_FAIL_THRESHOLD 3U
#define APP_MOTOR_MANUAL_PERIOD_MS 20U
#define APP_IMU_CALIB_TIMEOUT_MS 3000U
#define APP_IMU_BOOT_STABILIZE_MS 100U
#define APP_LOG_BUFFER_BYTES 256U
#define APP_LOG_RING_BYTES 2048U
#define APP_LOG_USB_CHUNK_BYTES 256U

#ifndef DEBUG_FAULTS
#define DEBUG_FAULTS 0
#endif

#ifndef MOTOR_LINK_DEBUG
#define MOTOR_LINK_DEBUG 0
#endif
#define ENABLE_MOTORS

#ifndef MOTOR_BACKEND_STEADYWIN_CAN
#define MOTOR_BACKEND_STEADYWIN_CAN 1
#endif

#ifndef MOTOR_LINK_CAN_LEFT_ID
#define MOTOR_LINK_CAN_LEFT_ID 0x01U
#endif

#ifndef MOTOR_LINK_CAN_RIGHT_ID
#define MOTOR_LINK_CAN_RIGHT_ID 0x02U
#endif

#ifndef MOTOR_LINK_CAN_REQ_ID_PREFIX
#define MOTOR_LINK_CAN_REQ_ID_PREFIX 0x100U
#endif

#ifndef MOTOR_LINK_CAN_REQ_ID
#define MOTOR_LINK_CAN_REQ_ID(addr)                                            \
  (MOTOR_LINK_CAN_REQ_ID_PREFIX | ((addr) & 0x7FFU))
#endif

#ifndef APP_ENABLE_PROFILING
#define APP_ENABLE_PROFILING 1
#endif

extern USBD_HandleTypeDef hUsbDeviceHS;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern CRC_HandleTypeDef hcrc;
#define APP_LINK_UART (&huart2)
#define APP_MOTOR_LEFT_UART (&huart6)
#define APP_MOTOR_RIGHT_UART (&huart1)


#endif /* STM32_APP_CONFIG_H */
