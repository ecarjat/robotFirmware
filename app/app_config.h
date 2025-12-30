#ifndef STM32_APP_CONFIG_H
#define STM32_APP_CONFIG_H

#include "main.h"
#include "usb_device.h"
#include "log.h"

#define APP_IDLE_TICK_MS   300U
#define APP_LOG_BUFFER_BYTES 256U
#define APP_LOG_RING_BYTES 2048U
#define APP_LOG_USB_CHUNK_BYTES 256U

extern USBD_HandleTypeDef hUsbDeviceHS;
extern UART_HandleTypeDef huart2;
extern CRC_HandleTypeDef hcrc;
#define APP_LINK_UART (&huart2)

#define APP_LOG_INFO(fmt, ...) app_log_printf("[APP][INFO] " fmt "\r\n", ##__VA_ARGS__)
#define APP_LOG_ERROR(fmt, ...) app_log_printf("[APP][ERROR] " fmt "\r\n", ##__VA_ARGS__)

#endif /* STM32_APP_CONFIG_H */
