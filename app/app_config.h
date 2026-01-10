#ifndef STM32_APP_CONFIG_H
#define STM32_APP_CONFIG_H

#include "log.h"
#include "main.h"
#include "usb_device.h"

#define APP_IDLE_TICK_MS 300U
#define APP_LOG_BUFFER_BYTES 256U
#define APP_LOG_RING_BYTES 2048U
#define APP_LOG_USB_CHUNK_BYTES 256U

#ifndef DEBUG_FAULTS
#define DEBUG_FAULTS 0
#endif

#ifndef MOTOR_LINK_DEBUG
#define MOTOR_LINK_DEBUG 1
#endif
#define ENABLE_MOTORS

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

#define APP_LOG_INFO(fmt, ...)                                                 \
  app_log_printf("[APP][INFO] " fmt "\r\n", ##__VA_ARGS__)
#define APP_LOG_WARN(fmt, ...)                                                 \
  app_log_printf("[APP][WARN] " fmt "\r\n", ##__VA_ARGS__)
#define APP_LOG_ERROR(fmt, ...)                                                \
  app_log_printf("[APP][ERROR] " fmt "\r\n", ##__VA_ARGS__)

#endif /* STM32_APP_CONFIG_H */
