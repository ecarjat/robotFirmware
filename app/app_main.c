#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "app_main.h"
#include "crc32.h"
#include "framing_cobs.h"
#include "imu_bmi270.h"
#include "imu_bus.h"
#include "imu_icm42688.h"
#include "imu_sched.h"
#include "mux_channels.h"
#include "shared_protocol/robot_protocol.h"
#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"

#define APP_LINK_RX_BUFFER_BYTES 512U
#define APP_LINK_FRAME_BUFFER_BYTES ROBOT_FRAME_MAX_ENCODED
#define APP_LINK_TX_BUFFER_BYTES 320U
#define APP_TELEM_PERIOD_MS 20U
#define APP_HEARTBEAT_TIMEOUT_MS 250U
#define APP_LOG_PERIOD_MS 500U
#ifndef APP_LINK_DEBUG_FRAMES
#define APP_LINK_DEBUG_FRAMES 1U
#endif
#ifndef APP_LINK_DEBUG_MAX_REPORTS
#define APP_LINK_DEBUG_MAX_REPORTS 5U
#endif
#ifndef APP_LINK_DEBUG_MAX_BYTES
#define APP_LINK_DEBUG_MAX_BYTES 48U
#define ROBOT_USE_HW_CRC 1
#endif

static void app_init(void);
static void app_idle_tick(void);
static void app_link_start(void);
static void app_link_process_chunk(const uint8_t *data, size_t len);
static void app_link_flush_frame(void);
static void app_link_handle_encoded_frame(const uint8_t *frame, size_t len);
static void app_link_debug_frame(const uint8_t *frame, size_t len);
static void app_link_log_bytes(const char *label, const uint8_t *data,
                               size_t len);
static void app_link_dispatch(const robot_frame_t *frame);
static void app_cmd_handler(uint8_t msg_type, const uint8_t *payload,
                            size_t len, void *ctx);
static void app_link_restart_rx(void);
static bool app_link_send(uint8_t type, uint16_t flags, const uint8_t *payload,
                          uint16_t len, uint16_t seq_override);
static void app_send_telem(void);
static void app_link_clear_uart_errors(UART_HandleTypeDef *huart);
static void app_link_invalidate_rx_cache(size_t len);
static bool app_link_dma_is_circular(void);

void app_main(void) {
  app_init();

  while (1) {
    app_idle_tick();
  }
}

static robot_mux_t s_mux;
static uint8_t s_uart_rx_buffer[APP_LINK_RX_BUFFER_BYTES]
    __attribute__((section(".dma_buffer"), aligned(32)));
static uint8_t s_cobs_frame_buffer[APP_LINK_FRAME_BUFFER_BYTES];
static size_t s_cobs_frame_len = 0U;
static size_t s_uart_rx_last_pos = 0U;
static uint16_t s_seq_counters[ROBOT_CHANNEL_MAX + 1U] = {0};
static uint32_t s_last_telem_ms = 0U;
static uint32_t s_last_cmd_ms = 0U;
static uint32_t s_last_log_ms = 0U;
static uint32_t s_imu_seq = 0U;
static uint32_t s_bmi_seq = 0U;

static void app_init(void) {
  HAL_Delay(2000);
  robot_mux_init(&s_mux);
  robot_mux_register(&s_mux, ROBOT_CHANNEL_CMD, app_cmd_handler, NULL);

  APP_LOG_INFO("Booting robot firmware (frame v%u)", ROBOT_FRAME_VERSION);
  APP_LOG_INFO("CMD channel id: %u", ROBOT_CHANNEL_CMD);

  app_link_start();
  s_last_cmd_ms = HAL_GetTick();

  imu_sched_init();

  bool bmi_ok = imu_bmi270_init();
  if (!bmi_ok) {
    APP_LOG_ERROR("BMI270 init failed");
  }

  bool icm_ok = imu_icm42688_init();
  if (!icm_ok) {
    APP_LOG_ERROR("ICM42688 init failed");
  }

  if (bmi_ok && icm_ok) {
    imu_bus_set_ready(1U);
    imu_sched_request(IMU_SCHED_SENSOR_BMI270);
    imu_sched_request(IMU_SCHED_SENSOR_ICM42688);
    imu_sched_run();
  } else {
    APP_LOG_ERROR("IMU bus not ready; one or more inits failed");
  }
}

static void app_idle_tick(void) {
  uint32_t now = HAL_GetTick();
  if ((now - s_last_telem_ms) >= APP_TELEM_PERIOD_MS) {
    app_send_telem();
    s_last_telem_ms = now;
  }

  imu_sched_tick();

  // if ((now - s_last_cmd_ms) > APP_HEARTBEAT_TIMEOUT_MS) {
  //   APP_LOG_ERROR("Link heartbeat timeout");
  //   s_last_cmd_ms = now; // rate-limit log spam
  // }
  if ((now - s_last_log_ms) >= APP_LOG_PERIOD_MS) {
    s_last_log_ms = now;
    imu_bmi270_sample_t bmi_sample;
    if (imu_bmi270_try_get_latest(&bmi_sample, &s_bmi_seq)) {
      APP_LOG_INFO("BMI270 accel [mg] = %ld, %ld, %ld gyro [mdps] = %ld,%ld, "
                   "%ld temp=%ld",
                   (long)bmi_sample.accel[0], (long)bmi_sample.accel[1],
                   (long)bmi_sample.accel[2], (long)bmi_sample.gyro[0],
                   (long)bmi_sample.gyro[1], (long)bmi_sample.gyro[2],
                   (long)bmi_sample.temperature);
    }

    imu_icm42688_sample_t imu_sample;
    if (imu_icm42688_try_get_latest(&imu_sample, &s_imu_seq)) {
      APP_LOG_INFO("ICM42688 accel [mg] = %ld, %ld, %ld gyro [mdps] = %ld,%ld, "
                   "%ld temp=%ld",
                   (long)imu_sample.accel[0], (long)imu_sample.accel[1],
                   (long)imu_sample.accel[2], (long)imu_sample.gyro[0],
                   (long)imu_sample.gyro[1], (long)imu_sample.gyro[2],
                   (long)imu_sample.temperature);
    }
  }

  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
  HAL_Delay(APP_IDLE_TICK_MS);
}

static void app_link_start(void) {
  if (APP_LINK_UART == NULL) {
    APP_LOG_ERROR("Link UART not initialized");
    return;
  }

  app_link_clear_uart_errors(APP_LINK_UART);
  if (HAL_UARTEx_ReceiveToIdle_DMA(APP_LINK_UART, s_uart_rx_buffer,
                                   sizeof(s_uart_rx_buffer)) != HAL_OK) {
    APP_LOG_ERROR("UART RX start failed");
    return;
  }
  __HAL_DMA_DISABLE_IT(APP_LINK_UART->hdmarx, DMA_IT_HT);
  s_uart_rx_last_pos = 0U;
}

static void app_link_restart_rx(void) {
  if (app_link_dma_is_circular()) {
    return;
  }
  app_link_clear_uart_errors(APP_LINK_UART);
  if (HAL_UARTEx_ReceiveToIdle_DMA(APP_LINK_UART, s_uart_rx_buffer,
                                   sizeof(s_uart_rx_buffer)) != HAL_OK) {
    APP_LOG_ERROR("UART RX restart failed");
    return;
  }
  __HAL_DMA_DISABLE_IT(APP_LINK_UART->hdmarx, DMA_IT_HT);
  s_uart_rx_last_pos = 0U;
}

static void app_link_process_chunk(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    uint8_t byte = data[i];
    if (byte == 0x00U) {
      app_link_flush_frame();
    } else if (s_cobs_frame_len < sizeof(s_cobs_frame_buffer)) {
      s_cobs_frame_buffer[s_cobs_frame_len++] = byte;
    } else {
      APP_LOG_ERROR("UART frame overflow, dropping data");
      s_cobs_frame_len = 0U;
    }
  }
}

static void app_link_flush_frame(void) {
  if (s_cobs_frame_len == 0U) {
    return;
  }

  app_link_handle_encoded_frame(s_cobs_frame_buffer, s_cobs_frame_len);
  s_cobs_frame_len = 0U;
}

static void app_link_handle_encoded_frame(const uint8_t *frame, size_t len) {
  robot_frame_t decoded;
  robot_frame_t ack;
  uint8_t encoded_ack[ROBOT_FRAME_MAX_ENCODED];
  size_t encoded_ack_len = 0U;

  if (len + 1U > ROBOT_FRAME_MAX_ENCODED) {
    APP_LOG_ERROR("Encoded frame too long (%u)", (unsigned int)len);
    return;
  }

  uint8_t encoded_buf[ROBOT_FRAME_MAX_ENCODED];
  memcpy(encoded_buf, frame, len);
  encoded_buf[len] = 0x00U;

  if (!robot_frame_decode(encoded_buf, len + 1U, &decoded)) {
    APP_LOG_ERROR("Frame decode failed (len=%u)", (unsigned int)len);
    app_link_debug_frame(frame, len);
    return;
  }

  /* Auto-ACK if requested */
  if ((decoded.hdr.flags & ROBOT_FLAG_ACK_REQ) != 0U) {
    if (robot_frame_init(&ack, ROBOT_MSG_ACK, decoded.hdr.seq,
                         ROBOT_FLAG_IS_ACK, NULL, 0U) &&
        robot_frame_encode(&ack, encoded_ack, sizeof(encoded_ack),
                           &encoded_ack_len)) {
      HAL_UART_Transmit(APP_LINK_UART, encoded_ack, (uint16_t)encoded_ack_len,
                        10U);
    }
  }

  app_link_dispatch(&decoded);
}

static void app_link_log_bytes(const char *label, const uint8_t *data,
                               size_t len) {
  if (label == NULL) {
    return;
  }
  if (data == NULL || len == 0U) {
    APP_LOG_ERROR("%s: <empty>", label);
    return;
  }

  size_t max_len = len;
  if (max_len > APP_LINK_DEBUG_MAX_BYTES) {
    max_len = APP_LINK_DEBUG_MAX_BYTES;
  }

  char line[APP_LOG_BUFFER_BYTES];
  int written =
      snprintf(line, sizeof(line), "%s (%u):", label, (unsigned int)len);
  if (written < 0) {
    return;
  }

  for (size_t i = 0U; i < max_len; ++i) {
    if ((size_t)written >= sizeof(line)) {
      break;
    }
    int ret = snprintf(line + written, sizeof(line) - (size_t)written, " %02X",
                       data[i]);
    if (ret < 0) {
      break;
    }
    written += ret;
  }

  if (max_len < len && (size_t)written < sizeof(line)) {
    (void)snprintf(line + written, sizeof(line) - (size_t)written, " ...");
  }

  APP_LOG_ERROR("%s", line);
}

static void app_link_debug_frame(const uint8_t *frame, size_t len) {
#if APP_LINK_DEBUG_FRAMES
  static uint32_t s_debug_reports = 0U;
  if (s_debug_reports >= APP_LINK_DEBUG_MAX_REPORTS) {
    if (s_debug_reports == APP_LINK_DEBUG_MAX_REPORTS) {
      APP_LOG_ERROR("Frame debug suppressed (max reports reached)");
    }
    ++s_debug_reports;
    return;
  }
  ++s_debug_reports;

  APP_LOG_ERROR("Frame debug: encoded_len=%u", (unsigned int)len);
  app_link_log_bytes("Encoded", frame, len);

  if (len == 0U) {
    APP_LOG_ERROR("Frame debug: empty encoded payload");
    return;
  }

  uint8_t decoded[ROBOT_FRAME_MAX_DECODED];
  size_t decoded_len = robot_cobs_decode(frame, len, decoded, sizeof(decoded));
  if (decoded_len == 0U) {
    APP_LOG_ERROR("Frame debug: COBS decode failed");
    return;
  }

  APP_LOG_ERROR("Frame debug: decoded_len=%u", (unsigned int)decoded_len);
  app_link_log_bytes("Decoded", decoded, decoded_len);

  if (decoded_len < sizeof(robot_frame_header_t) + sizeof(uint32_t)) {
    APP_LOG_ERROR("Frame debug: decoded too short for header+CRC");
    return;
  }

  robot_frame_header_t hdr;
  memcpy(&hdr, decoded, sizeof(hdr));
  APP_LOG_ERROR(
      "Frame debug: hdr magic=0x%04x ver=%u type=0x%02x seq=%u len=%u "
      "flags=0x%04x",
      hdr.magic, hdr.version, hdr.type, hdr.seq, hdr.len, hdr.flags);

  if (hdr.magic != ROBOT_FRAME_MAGIC) {
    APP_LOG_ERROR("Frame debug: header magic mismatch");
  }
  if (hdr.version != ROBOT_FRAME_VERSION) {
    APP_LOG_ERROR("Frame debug: header version mismatch");
  }
  if (hdr.len > ROBOT_FRAME_MAX_PAYLOAD) {
    APP_LOG_ERROR("Frame debug: header len too large");
    return;
  }

  size_t crc_offset = sizeof(hdr) + (size_t)hdr.len;
  size_t expected_len = crc_offset + sizeof(uint32_t);
  if (expected_len != decoded_len) {
    APP_LOG_ERROR("Frame debug: length mismatch expected=%u decoded=%u",
                  (unsigned int)expected_len, (unsigned int)decoded_len);
  }
  if (expected_len > decoded_len) {
    return;
  }

  uint32_t crc_rx = 0U;
  memcpy(&crc_rx, decoded + crc_offset, sizeof(crc_rx));
  uint32_t crc_calc = robot_crc32(decoded, crc_offset);
  if (crc_rx != crc_calc) {
    APP_LOG_ERROR("Frame debug: crc rx=0x%08lx calc=0x%08lx",
                  (unsigned long)crc_rx, (unsigned long)crc_calc);
  } else {
    APP_LOG_ERROR("Frame debug: crc ok 0x%08lx", (unsigned long)crc_rx);
  }
#else
  (void)frame;
  (void)len;
#endif
}

static void app_link_dispatch(const robot_frame_t *frame) {
  if (frame == NULL) {
    return;
  }

  /* Track link liveness on CMD heartbeats */
  if (frame->hdr.type == ROBOT_MSG_CMD_HEARTBEAT) {
    s_last_cmd_ms = HAL_GetTick();
  }

  robot_mux_dispatch(&s_mux, frame->hdr.type, frame->payload, frame->hdr.len);
}

static void app_cmd_handler(uint8_t msg_type, const uint8_t *payload,
                            size_t len, void *ctx) {
  (void)ctx;
  if (msg_type == ROBOT_MSG_CMD_TELEOP) {
    if (len < sizeof(robot_cmd_teleop_t)) {
      APP_LOG_ERROR("CMD teleop size mismatch (%u)", (unsigned int)len);
      return;
    }
    const robot_cmd_teleop_t *cmd = (const robot_cmd_teleop_t *)payload;
    APP_LOG_INFO("Teleop fwd=%.2f turn=%.2f flags=0x%04x", (double)cmd->vx_mps,
                 (double)cmd->wz_radps, cmd->flags);
  } else if (msg_type == ROBOT_MSG_CMD_HEARTBEAT) {
    APP_LOG_INFO("Heartbeat received");
  } else {
    APP_LOG_INFO("CMD handler invoked, msg_type=0x%02x", msg_type);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart != APP_LINK_UART) {
    return;
  }
  if (app_link_dma_is_circular()) {
    app_link_invalidate_rx_cache(sizeof(s_uart_rx_buffer));
    HAL_UART_RxEventTypeTypeDef evt = HAL_UARTEx_GetRxEventType(huart);
    size_t buf_size = sizeof(s_uart_rx_buffer);
    uint16_t remaining = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    size_t pos = buf_size - (size_t)remaining;
    if (pos >= buf_size) {
      pos = 0U;
    }

    if (evt == HAL_UART_RXEVENT_TC) {
      if (s_uart_rx_last_pos < buf_size) {
        app_link_process_chunk(&s_uart_rx_buffer[s_uart_rx_last_pos],
                               buf_size - s_uart_rx_last_pos);
      }
      if (s_uart_rx_last_pos > 0U) {
        app_link_process_chunk(&s_uart_rx_buffer[0], s_uart_rx_last_pos);
      }
      s_uart_rx_last_pos = pos;
      return;
    }

    if (pos != s_uart_rx_last_pos) {
      if (pos > s_uart_rx_last_pos) {
        app_link_process_chunk(&s_uart_rx_buffer[s_uart_rx_last_pos],
                               pos - s_uart_rx_last_pos);
      } else {
        app_link_process_chunk(&s_uart_rx_buffer[s_uart_rx_last_pos],
                               buf_size - s_uart_rx_last_pos);
        if (pos > 0U) {
          app_link_process_chunk(&s_uart_rx_buffer[0], pos);
        }
      }
      s_uart_rx_last_pos = pos;
    }
    return;
  }

  app_link_invalidate_rx_cache(Size);
  app_link_process_chunk(s_uart_rx_buffer, Size);
  app_link_restart_rx();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart != APP_LINK_UART) {
    return;
  }
  APP_LOG_ERROR("UART error 0x%lx", (unsigned long)huart->ErrorCode);
  if (app_link_dma_is_circular()) {
    (void)HAL_UART_AbortReceive(huart);
    app_link_clear_uart_errors(huart);
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, s_uart_rx_buffer,
                                     sizeof(s_uart_rx_buffer)) != HAL_OK) {
      APP_LOG_ERROR("UART RX restart failed");
      return;
    }
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    s_uart_rx_last_pos = 0U;
    return;
  }

  app_link_restart_rx();
}

static void app_link_clear_uart_errors(UART_HandleTypeDef *huart) {
  if (huart == NULL) {
    return;
  }

  __HAL_UART_CLEAR_PEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_IDLEFLAG(huart);
  __HAL_UART_FLUSH_DRREGISTER(huart);
}

static void app_link_invalidate_rx_cache(size_t len) {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  if (len == 0U) {
    return;
  }
  uintptr_t start = (uintptr_t)s_uart_rx_buffer;
  uintptr_t end = start + len;
  uintptr_t aligned_start = start & ~(uintptr_t)(32U - 1U);
  uintptr_t aligned_end = (end + (32U - 1U)) & ~(uintptr_t)(32U - 1U);
  SCB_InvalidateDCache_by_Addr((uint32_t *)aligned_start,
                               (int32_t)(aligned_end - aligned_start));
#else
  (void)len;
#endif
}

static bool app_link_dma_is_circular(void) {
  if (APP_LINK_UART == NULL || APP_LINK_UART->hdmarx == NULL) {
    return false;
  }
  return (APP_LINK_UART->hdmarx->Init.Mode == DMA_CIRCULAR);
}

static bool app_link_send(uint8_t type, uint16_t flags, const uint8_t *payload,
                          uint16_t len, uint16_t seq_override) {
  robot_frame_t frame;
  uint8_t encoded[ROBOT_FRAME_MAX_ENCODED];
  size_t encoded_len = 0U;

  uint16_t seq = (seq_override != 0U)
                     ? seq_override
                     : ++s_seq_counters[robot_channel_from_type(type)];
  if (!robot_frame_init(&frame, type, seq, flags, payload, len)) {
    return false;
  }
  if (!robot_frame_encode(&frame, encoded, sizeof(encoded), &encoded_len)) {
    return false;
  }

  return (HAL_UART_Transmit(APP_LINK_UART, encoded, (uint16_t)encoded_len,
                            10U) == HAL_OK);
}

static void app_send_telem(void) {
  robot_telem_v1_t telem;
  memset(&telem, 0, sizeof(telem));
  telem.version = 1U;
  telem.status = 0U;
  telem.timestamp_ms = HAL_GetTick();
  telem.batt_v = 0.0f;
  telem.batt_pct = 0.0f;
  telem.temp_c = 0.0f;

  if (!app_link_send(ROBOT_MSG_TELEM_FRAME, 0U, (const uint8_t *)&telem,
                     sizeof(telem), 0U)) {
    APP_LOG_ERROR("Failed to send telem frame");
  }
}
