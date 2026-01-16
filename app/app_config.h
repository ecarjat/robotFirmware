#ifndef STM32_APP_CONFIG_H
#define STM32_APP_CONFIG_H

#include "log.h"
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

#define APP_LOG_LEVEL_OFF 0U
#define APP_LOG_LEVEL_ERROR 1U
#define APP_LOG_LEVEL_WARN 2U
#define APP_LOG_LEVEL_INFO 3U
#define APP_LOG_LEVEL_DEBUG 4U


#ifndef APP_LOG_LEVEL
#ifdef NDEBUG
#define APP_LOG_LEVEL APP_LOG_LEVEL_WARN
#else
#define APP_LOG_LEVEL APP_LOG_LEVEL_INFO
#endif
#endif

#ifndef DEBUG_FAULTS
#define DEBUG_FAULTS 0
#endif

#ifndef MOTOR_LINK_DEBUG
#define MOTOR_LINK_DEBUG 0
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


#if APP_LOG_LEVEL >= APP_LOG_LEVEL_DEBUG
#define APP_LOG_DEBUG(fmt, ...)                                                 \
  app_log_printf("[APP][DEBUG] " fmt "\r\n", ##__VA_ARGS__)
#else
#define APP_LOG_DEBUG(fmt, ...)                                                 \
  do                                                                           \
  {                                                                            \
    if (0)                                                                     \
    {                                                                          \
      app_log_printf(fmt, ##__VA_ARGS__);                                      \
    }                                                                          \
  } while (0)
#endif

#if APP_LOG_LEVEL >= APP_LOG_LEVEL_INFO
#define APP_LOG_INFO(fmt, ...)                                                 \
  app_log_printf("[APP][INFO] " fmt "\r\n", ##__VA_ARGS__)
#else
#define APP_LOG_INFO(fmt, ...)                                                 \
  do                                                                           \
  {                                                                            \
    if (0)                                                                     \
    {                                                                          \
      app_log_printf(fmt, ##__VA_ARGS__);                                      \
    }                                                                          \
  } while (0)
#endif

#if APP_LOG_LEVEL >= APP_LOG_LEVEL_WARN
#define APP_LOG_WARN(fmt, ...)                                                 \
  app_log_printf("[APP][WARN] " fmt "\r\n", ##__VA_ARGS__)
#else
#define APP_LOG_WARN(fmt, ...)                                                 \
  do                                                                           \
  {                                                                            \
    if (0)                                                                     \
    {                                                                          \
      app_log_printf(fmt, ##__VA_ARGS__);                                      \
    }                                                                          \
  } while (0)
#endif

#if APP_LOG_LEVEL >= APP_LOG_LEVEL_ERROR
#define APP_LOG_ERROR(fmt, ...)                                                \
  app_log_printf("[APP][ERROR] " fmt "\r\n", ##__VA_ARGS__)
#else
#define APP_LOG_ERROR(fmt, ...)                                                \
  do                                                                           \
  {                                                                            \
    if (0)                                                                     \
    {                                                                          \
      app_log_printf(fmt, ##__VA_ARGS__);                                      \
    }                                                                          \
  } while (0)
#endif

#endif /* STM32_APP_CONFIG_H */
