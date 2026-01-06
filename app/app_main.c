#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "app_main.h"
#include "crc32.h"
#include "debug_wdog.h"
#include "framing_cobs.h"
#include "imu_bus.h"
#include "imu_sched.h"
#include "motor_link.h"
#include "motion_control.h"
#include "mux_channels.h"
#include "param_storage.h"
#include "sensors.h"
#include "config_control.h"
#include "control_timer.h"
#include "led_status.h"
#if SENSOR_ENABLE_BMI270
#include "imu_bmi270.h"
#endif
#if SENSOR_ENABLE_ICM42688
#include "imu_icm42688.h"
#endif
#if SENSOR_ENABLE_BMM150
#include "imu_bmm150.h"
#endif
#include "shared_protocol/robot_protocol.h"
#include "stm32h7xx_hal.h"
#include "system_reboot.h"
#include "usbd_cdc_if.h"

#define APP_LINK_RX_BUFFER_BYTES 512U
#define APP_LINK_FRAME_BUFFER_BYTES ROBOT_FRAME_MAX_ENCODED
#define APP_LINK_TX_BUFFER_BYTES 320U
#define APP_TELEM_PERIOD_MS 500U
#define APP_HEARTBEAT_TIMEOUT_MS 200U
#define APP_LOG_PERIOD_MS 500U
#define APP_IMU_CALIB_FACE_COUNT 6U
#define APP_IMU_CALIB_IMU_COUNT 2U
#define APP_IMU_CALIB_DEFAULT_SAMPLES 800U
#define APP_IMU_CALIB_MAX_SAMPLES 800U
#define APP_IMU_CALIB_TIMEOUT_MS 3000U
#define APP_IMU_ACCEL_RANGE_G 4.0f
#define APP_IMU_GYRO_RANGE_DPS 500.0f
#ifndef APP_LINK_DEBUG_FRAMES
#define APP_LINK_DEBUG_FRAMES 1U
#endif
#ifndef APP_LINK_DEBUG_MAX_REPORTS
#define APP_LINK_DEBUG_MAX_REPORTS 5U
#endif
#ifndef APP_LINK_DEBUG_MAX_BYTES
#define APP_LINK_DEBUG_MAX_BYTES 48U
#define ROBOT_USE_HW_CRC 1
#define ENABLE_MOTORS
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
static void app_rpc_handle(const robot_frame_t *frame);
static void app_cmd_handler(uint8_t msg_type, const uint8_t *payload,
                            size_t len, void *ctx);
static void app_link_poll(void);
static void app_link_restart_rx(void);
static bool app_link_send(uint8_t type, uint16_t flags, const uint8_t *payload,
                          uint16_t len, uint16_t seq_override);
static void app_send_telem(void);
static void app_link_clear_uart_errors(UART_HandleTypeDef *huart);
static void app_link_invalidate_rx_cache(size_t len);
static bool app_link_dma_is_circular(void);
static void app_log_sensors(void);
static void app_log_link_errors(void);
static void app_motor_manual_apply(void);
static uint8_t app_motor_manual_enable(bool enable);

static bool app_try_arm_balancing(bool prepare_balance);
static uint8_t app_motor_manual_run(uint8_t side, float intensity);
static bool app_rpc_send_param_resp(uint8_t method,
                                    uint8_t status,
                                    uint16_t offset,
                                    uint16_t length,
                                    const uint8_t *data,
                                    uint16_t data_len,
                                    uint16_t seq_override);

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

static const char *app_link_send_err_str(app_link_send_err_t err) {
  switch (err) {
    case APP_LINK_SEND_OK:
      return "ok";
    case APP_LINK_SEND_ERR_UART_NULL:
      return "uart_null";
    case APP_LINK_SEND_ERR_ISR:
      return "in_isr";
    case APP_LINK_SEND_ERR_FRAME_INIT:
      return "frame_init";
    case APP_LINK_SEND_ERR_ENCODE:
      return "encode";
    case APP_LINK_SEND_ERR_CTS_BLOCKED:
      return "cts_blocked";
    case APP_LINK_SEND_ERR_UART_BUSY:
      return "uart_busy";
    case APP_LINK_SEND_ERR_UART_TIMEOUT:
      return "uart_timeout";
    case APP_LINK_SEND_ERR_UART_ERROR:
      return "uart_error";
    default:
      return "unknown";
  }
}

static const char *app_imu_name(uint8_t active) {
  return (active == 1U) ? "ICM42688" : "BMI270";
}

typedef struct {
  uint8_t valid_mask;
  float accel[APP_IMU_CALIB_FACE_COUNT][3];
  float gyro[APP_IMU_CALIB_FACE_COUNT][3];
} app_imu_calib_state_t;

static app_imu_calib_state_t s_imu_calib[APP_IMU_CALIB_IMU_COUNT];

typedef struct {
  uint8_t enabled;
  float left;
  float right;
} app_motor_manual_t;

static app_motor_manual_t s_motor_manual = {0U, 0.0f, 0.0f};

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
static uint32_t s_last_led_ms = 0U;
static uint8_t s_estop_active = 0U;
static uint8_t s_last_teleop_flags = 0U;
static uint8_t s_last_imu_active = 0xFFU;
static uint8_t s_last_imu_active_valid = 0U;
static volatile uint32_t s_link_decode_failures = 0U;
static volatile uint32_t s_link_decode_last_len = 0U;
static volatile uint32_t s_link_overflows = 0U;
static volatile uint32_t s_link_uart_errors = 0U;
static volatile uint32_t s_link_uart_last_err = 0U;
static volatile uint16_t s_uart_rx_pending_size = 0U;
static volatile uint8_t s_uart_rx_event_pending = 0U;
static volatile app_link_send_err_t s_link_send_last_err = APP_LINK_SEND_OK;
static volatile uint32_t s_link_send_last_status = 0U;
static volatile uint32_t s_link_send_last_hal_state = 0U;
static volatile uint32_t s_link_send_last_hal_err = 0U;
static uint32_t s_telem_fail_count = 0U;
#define TELEM_FAIL_THRESHOLD 3U  /* Consecutive failures before LED indication */

/* Global robot parameters (loaded from flash at startup) */
robot_params_t g_robot_params;

static void app_init(void) {
  WDOG_CHECKPOINT(WDOG_CP_APP_INIT_START);
  HAL_Delay(2000);

  /* Load robot parameters from flash (or use defaults) */
  param_storage_init();
  int param_rc = param_storage_load(&g_robot_params);
  if (param_rc == PARAM_ERR_NOT_FOUND) {
    APP_LOG_INFO("No saved params, using defaults");
  } else if (param_rc == PARAM_OK) {
    APP_LOG_INFO("Loaded params from flash");
  } else {
    APP_LOG_ERROR("Param load error: %d", param_rc);
  }

  motion_control_init();
  APP_LOG_INFO("Booting robot firmware (frame v%u)", ROBOT_FRAME_VERSION);
  APP_LOG_INFO("CMD channel id: %u", ROBOT_CHANNEL_CMD);

  s_last_cmd_ms = HAL_GetTick();

  imu_sched_init();

  bool sensors_ok = true;
#if SENSOR_ENABLE_BMI270
  WDOG_CHECKPOINT(WDOG_CP_BMI270_INIT_START);
  bool bmi_ok = imu_bmi270_init();
  WDOG_CHECKPOINT(WDOG_CP_BMI270_INIT_DONE);
  if (!bmi_ok) {
    APP_LOG_ERROR("BMI270 init failed");
  }
  sensors_ok &= bmi_ok;
#endif

#if SENSOR_ENABLE_ICM42688
  WDOG_CHECKPOINT(WDOG_CP_ICM42688_INIT_START);
  bool icm_ok = imu_icm42688_init();
  WDOG_CHECKPOINT(WDOG_CP_ICM42688_INIT_DONE);
  if (!icm_ok) {
    APP_LOG_ERROR("ICM42688 init failed");
  }
  sensors_ok &= icm_ok;
#endif

#if SENSOR_ENABLE_BMM150
  WDOG_CHECKPOINT(WDOG_CP_BMM150_INIT_START);
  bool bmm_ok = imu_bmm150_init();
  WDOG_CHECKPOINT(WDOG_CP_BMM150_INIT_DONE);
  if (!bmm_ok) {
    APP_LOG_ERROR("BMM150 init failed");
  }
  sensors_ok &= bmm_ok;
#endif

  if (SENSOR_ENABLED_COUNT == 0) {
    APP_LOG_ERROR("No IMU sensors enabled");
  } else if (sensors_ok) {
    /*
     * EXTI Re-enable after IMU Init:
     * IMU EXTI interrupts were disabled in main.c to prevent race conditions
     * during cold boot. Now that sensors are initialized and the scheduler
     * is ready, clear any pending interrupts and re-enable EXTI.
     */
#if SENSOR_ENABLE_ICM42688
    __HAL_GPIO_EXTI_CLEAR_IT(ICM42688_INT1_Pin);
#endif
#if SENSOR_ENABLE_BMI270
    __HAL_GPIO_EXTI_CLEAR_IT(BMI270_INT1_Pin);
#endif
#if SENSOR_ENABLE_BMM150
    __HAL_GPIO_EXTI_CLEAR_IT(BMM150_INT1_Pin);
#endif

#if SENSOR_ENABLE_ICM42688
    HAL_NVIC_EnableIRQ(ICM42688_INT1_EXTI_IRQn);
#endif
#if SENSOR_ENABLE_BMI270
    HAL_NVIC_EnableIRQ(BMI270_INT1_EXTI_IRQn);
#endif
#if SENSOR_ENABLE_BMM150
    HAL_NVIC_EnableIRQ(BMM150_INT1_EXTI_IRQn);
#endif
    WDOG_CHECKPOINT(WDOG_CP_EXTI_ENABLE);

    /*
     * Cold Boot Stabilization:
     * After enabling EXTI, give sensors time to generate their first
     * valid data-ready interrupt. On cold boot, sensors may need
     * additional settling time before DMA reads succeed reliably.
     */
    HAL_Delay(100);

    imu_bus_set_ready(1U);
#define SENSOR_REQUEST_ENTRY(name, short_name, prefix) \
    imu_sched_request(IMU_SCHED_SENSOR_##name);
    SENSOR_LIST_ENABLED(SENSOR_REQUEST_ENTRY)
#undef SENSOR_REQUEST_ENTRY
    imu_sched_run();
  } else {
    APP_LOG_ERROR("IMU bus not ready; one or more inits failed");
  }

#ifdef ENABLE_MOTORS
  WDOG_CHECKPOINT(WDOG_CP_MOTOR_INIT);
  if (!motor_link_init()) {
    APP_LOG_ERROR("Motor link init failed");
  }
  motor_link_set_motor_directions(g_robot_params.motor_direction[0],
                                  g_robot_params.motor_direction[1]);
#endif
  robot_mux_init(&s_mux);
  robot_mux_register(&s_mux, ROBOT_CHANNEL_CMD, app_cmd_handler, NULL);
  app_link_start();

  /* Initialize and start control timer based on control_rate_hz */
  control_timer_init();
  control_timer_set_rate_hz(g_robot_params.control_rate_hz);
  control_timer_start();

  /* Initialize LED status state machine */
  led_status_init();
}

static void app_idle_tick(void) {
  /* Refresh watchdog at start of each idle tick */
  debug_wdog_refresh();
  WDOG_CHECKPOINT(WDOG_CP_IDLE_LOOP);
  app_link_poll();

  uint32_t now = HAL_GetTick();
  if ((now - s_last_telem_ms) >= APP_TELEM_PERIOD_MS) {
    app_send_telem();
    s_last_telem_ms = now;
  }
#ifdef ENABLE_MOTORS
  motor_link_poll();
#endif

  /* Heartbeat timeout detection: disarm robot if no commands received */
  if ((now - s_last_cmd_ms) > APP_HEARTBEAT_TIMEOUT_MS) {
    motion_control_set_mode(MOTION_MODE_DISARMED);
#ifdef ENABLE_MOTORS
    motor_link_enable(false);
#endif
    led_status_set_flag(LED_STATUS_TELEM_FAILURE);
    APP_LOG_ERROR("Link timeout: no frames for %lu ms",
                  (unsigned long)(now - s_last_cmd_ms));
  }

  /* Timer-driven control loop: run when TIM2 fires (control_rate_hz) */
  if (control_timer_pending()) {
    control_timer_begin_cycle();
    motion_control_tick(now);
    app_motor_manual_apply();
    control_timer_end_cycle();
  }
  imu_sched_tick();

  if ((now - s_last_log_ms) >= APP_LOG_PERIOD_MS) {
    s_last_log_ms = now;
    app_log_sensors();
    app_log_link_errors();
    motion_control_imu_health_t imu_health;
    if (motion_control_get_imu_health(&imu_health)) {
      APP_LOG_INFO(
          "IMU health active=%u gyro_diff=%.2f dps pitch_diff=%.2f dps "
          "acc_angle=%.2f deg vib=%.3f g gate=%u",
          (unsigned int)imu_health.active_sensor,
          (double)imu_health.gyro_diff_dps,
          (double)imu_health.gyro_pitch_diff_dps,
          (double)imu_health.acc_angle_diff_deg,
          (double)imu_health.vib_rms_g,
          (unsigned int)imu_health.gate_accel);
      if (!s_last_imu_active_valid ||
          imu_health.active_sensor != s_last_imu_active) {
        APP_LOG_INFO("IMU active changed: %s -> %s",
                     s_last_imu_active_valid ? app_imu_name(s_last_imu_active)
                                             : "unknown",
                     app_imu_name(imu_health.active_sensor));
        s_last_imu_active = imu_health.active_sensor;
        s_last_imu_active_valid = 1U;
      }
    }
    motion_control_ekf_log_t ekf_log;
    if (motion_control_get_ekf_log(&ekf_log) && ekf_log.valid) {
      APP_LOG_INFO(
          "EKF log accel=[%.3f %.3f %.3f] gyro=[%.3f %.3f %.3f] norm=%.3f "
          "theta=%.3f theta_acc=%.3f rate=%.3f gate=%u imu_dt=[%lu %lu]ms",
          (double)ekf_log.accel_x, (double)ekf_log.accel_y,
          (double)ekf_log.accel_z, (double)ekf_log.gyro_x,
          (double)ekf_log.gyro_y, (double)ekf_log.gyro_z,
          (double)ekf_log.accel_norm_g, (double)ekf_log.theta,
          (double)ekf_log.theta_acc, (double)ekf_log.gyro_rate,
          (unsigned int)ekf_log.gate,
          (unsigned long)ekf_log.imu_primary_dt_ms,
          (unsigned long)ekf_log.imu_secondary_dt_ms);
    }

#ifdef ENABLE_MOTORS
    float gyro_z = 0.0f;
    float yaw_rate_enc = 0.0f;
    float yaw_rate = 0.0f;
    if (motion_control_get_yaw_debug(&gyro_z, &yaw_rate_enc, &yaw_rate)) {
      APP_LOG_INFO("Yaw debug gyroZ=%.4f enc=%.4f blended=%.4f",
                   (double)gyro_z, (double)yaw_rate_enc, (double)yaw_rate);
    }

    float left_vel = 0.0f;
    float right_vel = 0.0f;
    bool motor_ok = motor_link_get_wheel_velocities(&left_vel, &right_vel);
    APP_LOG_INFO("Motor telemetry vel [rad/s] L=%.3f R=%.3f ok=%u",
                 (double)left_vel, (double)right_vel, motor_ok ? 1U : 0U);
    APP_LOG_INFO(
        "Motor diag drops L=%lu R=%lu stale L=%lu R=%lu late L=%lu R=%lu",
        (unsigned long)motor_link_get_left_parser_drops(),
        (unsigned long)motor_link_get_right_parser_drops(),
        (unsigned long)motor_link_get_left_telem_stale(),
        (unsigned long)motor_link_get_right_telem_stale(),
        (unsigned long)motor_link_get_left_telem_late(),
        (unsigned long)motor_link_get_right_telem_late());
#endif
  }

  if ((now - s_last_led_ms) >= APP_IDLE_TICK_MS) {
    s_last_led_ms = now;

    /* Update LED status flags based on system state */
    motion_mode_t mode = motion_control_get_mode();

    /* Fault/Fallen flags */
    if (mode == MOTION_MODE_FAULT) {
      led_status_set_flag(LED_STATUS_FAULT);
    } else {
      led_status_clear_flag(LED_STATUS_FAULT);
    }

    if (mode == MOTION_MODE_FALLEN) {
      led_status_set_flag(LED_STATUS_FALLEN);
    } else {
      led_status_clear_flag(LED_STATUS_FALLEN);
    }

    /* Motor timeout flag */
    uint32_t last_motor_ok = motion_control_get_last_motor_ok_ms();
    if (last_motor_ok != 0U && (now - last_motor_ok) > MOTOR_LINK_FAULT_FATAL_MS) {
      led_status_set_flag(LED_STATUS_MOTOR_TIMEOUT);
    } else {
      led_status_clear_flag(LED_STATUS_MOTOR_TIMEOUT);
    }

    /* Motor saturation flag (only relevant when balancing) */
    if (mode == MOTION_MODE_BALANCING && motion_control_is_saturated()) {
      led_status_set_flag(LED_STATUS_MOTOR_SATURATED);
    } else {
      led_status_clear_flag(LED_STATUS_MOTOR_SATURATED);
    }

    /* Update LED outputs */
    led_status_update(now);
  }
}

static bool app_in_isr(void) {
  return (__get_IPSR() != 0U);
}

#if SENSOR_ENABLE_BMI270
static void app_log_bmi270(void) {
  /* IMU data now logged via motion_control EKF telemetry */
}
#endif

#if SENSOR_ENABLE_ICM42688
static void app_log_icm42688(void) {
  /* IMU data now logged via motion_control EKF telemetry */
}
#endif

#if SENSOR_ENABLE_BMM150
static void app_log_bmm150(void) {
  /* Magnetometer logging not currently used */
}
#endif

static void app_log_sensors(void) {
#if SENSOR_ENABLED_COUNT > 0
#define SENSOR_LOG_ENTRY(name, short_name, prefix) app_log_##short_name();
  SENSOR_LIST_ENABLED(SENSOR_LOG_ENTRY)
#undef SENSOR_LOG_ENTRY
#endif
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
  s_uart_rx_event_pending = 0U;
  s_uart_rx_pending_size = 0U;
}

static void app_link_poll(void) {
  if (APP_LINK_UART == NULL) {
    return;
  }

  if (app_link_dma_is_circular()) {
    size_t buf_size = sizeof(s_uart_rx_buffer);
    uint16_t remaining = __HAL_DMA_GET_COUNTER(APP_LINK_UART->hdmarx);
    size_t pos = buf_size - (size_t)remaining;
    if (pos >= buf_size) {
      pos = 0U;
    }
    if (pos == s_uart_rx_last_pos) {
      return;
    }

    app_link_invalidate_rx_cache(sizeof(s_uart_rx_buffer));
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
    return;
  }

  if (!s_uart_rx_event_pending) {
    return;
  }

  __disable_irq();
  uint16_t size = s_uart_rx_pending_size;
  s_uart_rx_pending_size = 0U;
  s_uart_rx_event_pending = 0U;
  __enable_irq();

  if (size == 0U) {
    return;
  }

  app_link_invalidate_rx_cache(size);
  app_link_process_chunk(s_uart_rx_buffer, size);
  app_link_restart_rx();
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
      if (app_in_isr()) {
        s_link_overflows++;
      } else {
        APP_LOG_ERROR("UART frame overflow, dropping data");
      }
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
    s_link_decode_failures++;
    s_link_decode_last_len = (uint32_t)len;
    if (!app_in_isr()) {
      APP_LOG_ERROR("Frame decode failed (len=%u)", (unsigned int)len);
      app_link_debug_frame(frame, len);
    }
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

  /* Any valid frame counts as link liveness. */
  s_last_cmd_ms = HAL_GetTick();
  led_status_clear_flag(LED_STATUS_TELEM_FAILURE);

  if (frame->hdr.type == ROBOT_MSG_RPC_REQ) {
    app_rpc_handle(frame);
    return;
  }

  robot_mux_dispatch(&s_mux, frame->hdr.type, frame->payload, frame->hdr.len);
}

static bool app_rpc_send_param_resp(uint8_t method,
                                    uint8_t status,
                                    uint16_t offset,
                                    uint16_t length,
                                    const uint8_t *data,
                                    uint16_t data_len,
                                    uint16_t seq_override) {
  robot_rpc_param_t resp_hdr;
  uint8_t payload[ROBOT_FRAME_MAX_PAYLOAD];
  size_t total_len = sizeof(resp_hdr) + (size_t)data_len;

  if (total_len > sizeof(payload)) {
    return false;
  }

  resp_hdr.method = method;
  resp_hdr.flags = status;
  resp_hdr.offset = offset;
  resp_hdr.length = length;
  memcpy(payload, &resp_hdr, sizeof(resp_hdr));
  if (data_len > 0U && data != NULL) {
    memcpy(payload + sizeof(resp_hdr), data, data_len);
  }

  return app_link_send(ROBOT_MSG_RPC_RESP, ROBOT_FLAG_ACK_REQ, payload,
                       (uint16_t)total_len, seq_override);
}

static float app_vec_norm(const float v[3]) {
  return sqrtf((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]));
}

static float app_vec_dot(const float a[3], const float b[3]) {
  return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
}

static void app_vec_cross(const float a[3], const float b[3], float out[3]) {
  out[0] = (a[1] * b[2]) - (a[2] * b[1]);
  out[1] = (a[2] * b[0]) - (a[0] * b[2]);
  out[2] = (a[0] * b[1]) - (a[1] * b[0]);
}

static bool app_vec_normalize(float v[3]) {
  float norm = app_vec_norm(v);
  if (norm < 1e-6f) {
    return false;
  }
  float inv = 1.0f / norm;
  v[0] *= inv;
  v[1] *= inv;
  v[2] *= inv;
  return true;
}

static int16_t app_clamp_i16(int32_t value) {
  if (value > INT16_MAX) {
    return INT16_MAX;
  }
  if (value < INT16_MIN) {
    return INT16_MIN;
  }
  return (int16_t)value;
}

static void app_motor_manual_apply(void) {
#ifdef ENABLE_MOTORS
  if (!s_motor_manual.enabled) {
    return;
  }
  float max_A = fabsf(g_robot_params.balance.IqMax);
  if (max_A <= 0.0f) {
    return;
  }
  float left = s_motor_manual.left;
  float right = s_motor_manual.right;
  if (left > 1.0f) {
    left = 1.0f;
  } else if (left < -1.0f) {
    left = -1.0f;
  }
  if (right > 1.0f) {
    right = 1.0f;
  } else if (right < -1.0f) {
    right = -1.0f;
  }
  motor_link_set_wheel_Iq(left * max_A, right * max_A, max_A);
#else
  (void)0;
#endif
}

static uint8_t app_motor_manual_enable(bool enable) {
#ifdef ENABLE_MOTORS
  if (enable) {
    s_motor_manual.enabled = 1U;
    s_motor_manual.left = 0.0f;
    s_motor_manual.right = 0.0f;
    motion_control_set_mode(MOTION_MODE_DISARMED);
    motion_control_set_output_enabled(false);
    motor_link_set_control_mode(MOTOR_CONTROL_TORQUE);
    motor_link_enable(true);
  } else {
    s_motor_manual.enabled = 0U;
    s_motor_manual.left = 0.0f;
    s_motor_manual.right = 0.0f;
    motor_link_set_wheel_Iq(0.0f, 0.0f, 0.0f);
    motor_link_enable(false);
    motion_control_set_output_enabled(true);
  }
  return ROBOT_RPC_STATUS_OK;
#else
  (void)enable;
  return ROBOT_RPC_STATUS_NOT_READY;
#endif
}

static bool app_try_arm_balancing(bool prepare_balance) {
  if (!motion_control_can_arm()) {
    return false;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (!motion_control_can_arm()) {
    __set_PRIMASK(primask);
    return false;
  }

  if (prepare_balance) {
    s_motor_manual.enabled = 0U;
    s_motor_manual.left = 0.0f;
    s_motor_manual.right = 0.0f;
    motion_control_set_output_enabled(true);
  }

  motion_control_set_mode(MOTION_MODE_BALANCING);
#ifdef ENABLE_MOTORS
  motor_link_enable(true);
#endif
  __set_PRIMASK(primask);
  return true;
}

static uint8_t app_motor_manual_run(uint8_t side, float intensity) {
#ifdef ENABLE_MOTORS
  if (!s_motor_manual.enabled) {
    return ROBOT_RPC_STATUS_NOT_READY;
  }
  if (side == ROBOT_MOTOR_SIDE_LEFT) {
    s_motor_manual.left = intensity;
  } else if (side == ROBOT_MOTOR_SIDE_RIGHT) {
    s_motor_manual.right = intensity;
  } else {
    return ROBOT_RPC_STATUS_BAD_PARAM;
  }
  app_motor_manual_apply();
  return ROBOT_RPC_STATUS_OK;
#else
  (void)side;
  (void)intensity;
  return ROBOT_RPC_STATUS_NOT_READY;
#endif
}

static bool app_imu_calib_build_rotation(const app_imu_calib_state_t *state,
                                         float rot[9]) {
  if (state == NULL || rot == NULL) {
    return false;
  }
  if (state->valid_mask != ((1U << APP_IMU_CALIB_FACE_COUNT) - 1U)) {
    return false;
  }

  float x_vec[3];
  float y_vec[3];
  float z_vec[3];
  for (size_t i = 0U; i < 3U; ++i) {
    x_vec[i] = state->accel[ROBOT_IMU_FACE_X_POS_UP][i] -
               state->accel[ROBOT_IMU_FACE_X_NEG_UP][i];
    y_vec[i] = state->accel[ROBOT_IMU_FACE_Y_POS_UP][i] -
               state->accel[ROBOT_IMU_FACE_Y_NEG_UP][i];
    z_vec[i] = state->accel[ROBOT_IMU_FACE_Z_POS_UP][i] -
               state->accel[ROBOT_IMU_FACE_Z_NEG_UP][i];
  }

  if (!app_vec_normalize(x_vec) || !app_vec_normalize(y_vec) ||
      !app_vec_normalize(z_vec)) {
    return false;
  }

  float r0[3] = {x_vec[0], x_vec[1], x_vec[2]};
  float r1[3] = {y_vec[0], y_vec[1], y_vec[2]};

  float proj = app_vec_dot(r1, r0);
  for (size_t i = 0U; i < 3U; ++i) {
    r1[i] -= proj * r0[i];
  }
  if (!app_vec_normalize(r1)) {
    return false;
  }

  float r2[3];
  app_vec_cross(r0, r1, r2);
  if (app_vec_dot(r2, z_vec) < 0.0f) {
    for (size_t i = 0U; i < 3U; ++i) {
      r1[i] = -r1[i];
      r2[i] = -r2[i];
    }
  }

  rot[0] = r0[0];
  rot[1] = r0[1];
  rot[2] = r0[2];
  rot[3] = r1[0];
  rot[4] = r1[1];
  rot[5] = r1[2];
  rot[6] = r2[0];
  rot[7] = r2[1];
  rot[8] = r2[2];
  return true;
}

static bool app_imu_calib_capture_bmi270(float accel[3],
                                         float gyro[3],
                                         uint16_t samples) {
#if SENSOR_ENABLE_BMI270
  uint16_t target = samples;
  if (target == 0U) {
    target = APP_IMU_CALIB_DEFAULT_SAMPLES;
  }
  if (target > APP_IMU_CALIB_MAX_SAMPLES) {
    target = APP_IMU_CALIB_MAX_SAMPLES;
  }

  uint32_t seq = 0U;
  uint32_t count = 0U;
  float sum_accel[3] = {0.0f, 0.0f, 0.0f};
  float sum_gyro[3] = {0.0f, 0.0f, 0.0f};
  const uint32_t start = HAL_GetTick();

  while (count < target &&
         (HAL_GetTick() - start) < APP_IMU_CALIB_TIMEOUT_MS) {
    imu_bmi270_sample_t sample;
    if (imu_bmi270_try_get_latest(&sample, &seq)) {
      sum_accel[0] += (float)sample.accel[0];
      sum_accel[1] += (float)sample.accel[1];
      sum_accel[2] += (float)sample.accel[2];
      sum_gyro[0] += (float)sample.gyro[0];
      sum_gyro[1] += (float)sample.gyro[1];
      sum_gyro[2] += (float)sample.gyro[2];
      ++count;
    }
    imu_sched_run();
    HAL_Delay(1U);
  }

  if (count == 0U) {
    return false;
  }

  const float accel_scale =
      (APP_IMU_ACCEL_RANGE_G * 9.80665f) / 32768.0f;
  const float gyro_scale = APP_IMU_GYRO_RANGE_DPS / 32768.0f;
  const float inv = 1.0f / (float)count;
  accel[0] = sum_accel[0] * inv * accel_scale;
  accel[1] = sum_accel[1] * inv * accel_scale;
  accel[2] = sum_accel[2] * inv * accel_scale;
  gyro[0] = sum_gyro[0] * inv * gyro_scale;
  gyro[1] = sum_gyro[1] * inv * gyro_scale;
  gyro[2] = sum_gyro[2] * inv * gyro_scale;
  return true;
#else
  (void)accel;
  (void)gyro;
  (void)samples;
  return false;
#endif
}

static bool app_imu_calib_capture_icm42688(float accel[3],
                                           float gyro[3],
                                           uint16_t samples) {
#if SENSOR_ENABLE_ICM42688
  uint16_t target = samples;
  if (target == 0U) {
    target = APP_IMU_CALIB_DEFAULT_SAMPLES;
  }
  if (target > APP_IMU_CALIB_MAX_SAMPLES) {
    target = APP_IMU_CALIB_MAX_SAMPLES;
  }

  uint32_t seq = 0U;
  uint32_t count = 0U;
  float sum_accel[3] = {0.0f, 0.0f, 0.0f};
  float sum_gyro[3] = {0.0f, 0.0f, 0.0f};
  const uint32_t start = HAL_GetTick();

  while (count < target &&
         (HAL_GetTick() - start) < APP_IMU_CALIB_TIMEOUT_MS) {
    imu_icm42688_sample_t sample;
    if (imu_icm42688_try_get_latest(&sample, &seq)) {
      sum_accel[0] += (float)sample.accel[0];
      sum_accel[1] += (float)sample.accel[1];
      sum_accel[2] += (float)sample.accel[2];
      sum_gyro[0] += (float)sample.gyro[0];
      sum_gyro[1] += (float)sample.gyro[1];
      sum_gyro[2] += (float)sample.gyro[2];
      ++count;
    }
    imu_sched_run();
    HAL_Delay(1U);
  }

  if (count == 0U) {
    return false;
  }

  const float accel_scale =
      (APP_IMU_ACCEL_RANGE_G * 9.80665f) / 32768.0f;
  const float gyro_scale = APP_IMU_GYRO_RANGE_DPS / 32768.0f;
  const float inv = 1.0f / (float)count;
  accel[0] = sum_accel[0] * inv * accel_scale;
  accel[1] = sum_accel[1] * inv * accel_scale;
  accel[2] = sum_accel[2] * inv * accel_scale;
  gyro[0] = sum_gyro[0] * inv * gyro_scale;
  gyro[1] = sum_gyro[1] * inv * gyro_scale;
  gyro[2] = sum_gyro[2] * inv * gyro_scale;
  return true;
#else
  (void)accel;
  (void)gyro;
  (void)samples;
  return false;
#endif
}

static uint8_t app_imu_calib_apply(uint8_t imu_id,
                                   const app_imu_calib_state_t *state) {
  if (state == NULL) {
    return ROBOT_RPC_STATUS_BAD_PARAM;
  }

  float accel_bias[3] = {0.0f, 0.0f, 0.0f};
  float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
  for (size_t face = 0U; face < APP_IMU_CALIB_FACE_COUNT; ++face) {
    accel_bias[0] += state->accel[face][0];
    accel_bias[1] += state->accel[face][1];
    accel_bias[2] += state->accel[face][2];
    gyro_bias[0] += state->gyro[face][0];
    gyro_bias[1] += state->gyro[face][1];
    gyro_bias[2] += state->gyro[face][2];
  }

  const float inv_faces = 1.0f / (float)APP_IMU_CALIB_FACE_COUNT;
  accel_bias[0] *= inv_faces;
  accel_bias[1] *= inv_faces;
  accel_bias[2] *= inv_faces;
  gyro_bias[0] *= inv_faces;
  gyro_bias[1] *= inv_faces;
  gyro_bias[2] *= inv_faces;

  const float inv_g = 1.0f / 9.80665f;
  int32_t accel_bias_mg[3] = {
      (int32_t)lrintf(accel_bias[0] * inv_g * 1000.0f),
      (int32_t)lrintf(accel_bias[1] * inv_g * 1000.0f),
      (int32_t)lrintf(accel_bias[2] * inv_g * 1000.0f),
  };
  int32_t gyro_bias_mdps[3] = {
      (int32_t)lrintf(gyro_bias[0] * 1000.0f),
      (int32_t)lrintf(gyro_bias[1] * 1000.0f),
      (int32_t)lrintf(gyro_bias[2] * 1000.0f),
  };

  float rotation[9];
  if (!app_imu_calib_build_rotation(state, rotation)) {
    return ROBOT_RPC_STATUS_BAD_PARAM;
  }

  imu_calib_t *calib = NULL;
  if (imu_id == 0U) {
    calib = &g_robot_params.imu_bmi270;
  } else if (imu_id == 1U) {
    calib = &g_robot_params.imu_icm42688;
  } else {
    return ROBOT_RPC_STATUS_BAD_PARAM;
  }

  calib->accel_bias[0] = app_clamp_i16(accel_bias_mg[0]);
  calib->accel_bias[1] = app_clamp_i16(accel_bias_mg[1]);
  calib->accel_bias[2] = app_clamp_i16(accel_bias_mg[2]);
  calib->gyro_bias[0] = app_clamp_i16(gyro_bias_mdps[0]);
  calib->gyro_bias[1] = app_clamp_i16(gyro_bias_mdps[1]);
  calib->gyro_bias[2] = app_clamp_i16(gyro_bias_mdps[2]);
  memcpy(calib->rotation, rotation, sizeof(rotation));

  motion_control_apply_params();
  return ROBOT_RPC_STATUS_OK;
}

static uint8_t app_imu_calib_capture_face(uint8_t imu_id,
                                          uint8_t face,
                                          uint16_t samples) {
  if (face >= ROBOT_IMU_FACE_COUNT) {
    return ROBOT_RPC_STATUS_BAD_PARAM;
  }
  if (imu_id >= APP_IMU_CALIB_IMU_COUNT) {
    return ROBOT_RPC_STATUS_BAD_PARAM;
  }

  float accel[3];
  float gyro[3];
  bool ok = false;
  if (imu_id == 0U) {
#if SENSOR_ENABLE_BMI270
    ok = app_imu_calib_capture_bmi270(accel, gyro, samples);
#else
    return ROBOT_RPC_STATUS_NOT_READY;
#endif
  } else {
#if SENSOR_ENABLE_ICM42688
    ok = app_imu_calib_capture_icm42688(accel, gyro, samples);
#else
    return ROBOT_RPC_STATUS_NOT_READY;
#endif
  }

  if (!ok) {
    return ROBOT_RPC_STATUS_TIMEOUT;
  }

  app_imu_calib_state_t *state = &s_imu_calib[imu_id];
  for (size_t i = 0U; i < 3U; ++i) {
    state->accel[face][i] = accel[i];
    state->gyro[face][i] = gyro[i];
  }
  state->valid_mask |= (uint8_t)(1U << face);

  if (state->valid_mask != ((1U << APP_IMU_CALIB_FACE_COUNT) - 1U)) {
    return ROBOT_RPC_STATUS_INCOMPLETE;
  }

  return app_imu_calib_apply(imu_id, state);
}

static void app_rpc_handle(const robot_frame_t *frame) {
  if (frame == NULL) {
    return;
  }
  if (frame->hdr.len < sizeof(robot_rpc_param_t)) {
    APP_LOG_ERROR("RPC payload too short (%u)",
                  (unsigned int)frame->hdr.len);
    return;
  }

  robot_rpc_param_t req;
  memcpy(&req, frame->payload, sizeof(req));

  const size_t params_size = sizeof(robot_params_t);
  const size_t req_data_len = frame->hdr.len - sizeof(req);
  const uint8_t *req_data = frame->payload + sizeof(req);
  const uint16_t offset = req.offset;
  const uint16_t length = req.length;
  const uint16_t resp_seq = (frame->hdr.seq != 0U) ? frame->hdr.seq : 0U;

  if ((size_t)offset > params_size) {
    app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_OFFSET, offset,
                            length, NULL, 0U, resp_seq);
    return;
  }
  if ((size_t)offset + (size_t)length > params_size) {
    app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                            length, NULL, 0U, resp_seq);
    return;
  }

  switch (req.method) {
    case ROBOT_RPC_METHOD_GET_PARAM: {
      if (req_data_len != 0U || length == 0U) {
        app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                                length, NULL, 0U, resp_seq);
        return;
      }
      if ((size_t)length >
          (ROBOT_FRAME_MAX_PAYLOAD - sizeof(robot_rpc_param_t))) {
        app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                                length, NULL, 0U, resp_seq);
        return;
      }
      const uint8_t *src = (const uint8_t *)&g_robot_params + offset;
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_OK, offset, length,
                              src, length, resp_seq);
      return;
    }
    case ROBOT_RPC_METHOD_IMU_CALIB_FACE: {
      if (req_data_len != sizeof(robot_rpc_imu_calib_face_t)) {
        app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                                length, NULL, 0U, resp_seq);
        return;
      }
      robot_rpc_imu_calib_face_t calib_req;
      memcpy(&calib_req, req_data, sizeof(calib_req));

      uint8_t status = app_imu_calib_capture_face(calib_req.imu, calib_req.face,
                                                  calib_req.samples);
      app_rpc_send_param_resp(req.method, status, calib_req.face, 0U, NULL, 0U,
                              resp_seq);
      return;
    }
    case ROBOT_RPC_METHOD_MOTOR_ENABLE: {
      if (req_data_len != 0U) {
        app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                                length, NULL, 0U, resp_seq);
        return;
      }
      uint8_t status = app_motor_manual_enable(true);
      app_rpc_send_param_resp(req.method, status, 0U, 0U, NULL, 0U, resp_seq);
      return;
    }
    case ROBOT_RPC_METHOD_MOTOR_DISABLE: {
      if (req_data_len != 0U) {
        app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                                length, NULL, 0U, resp_seq);
        return;
      }
      uint8_t status = app_motor_manual_enable(false);
      app_rpc_send_param_resp(req.method, status, 0U, 0U, NULL, 0U, resp_seq);
      return;
    }
    case ROBOT_RPC_METHOD_MOTOR_RUN: {
      if (req_data_len != sizeof(robot_rpc_motor_run_t)) {
        app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                                length, NULL, 0U, resp_seq);
        return;
      }
      robot_rpc_motor_run_t run_req;
      memcpy(&run_req, req_data, sizeof(run_req));
      uint8_t status = app_motor_manual_run(run_req.side, run_req.intensity);
      app_rpc_send_param_resp(req.method, status, run_req.side, 0U, NULL, 0U,
                              resp_seq);
      return;
    }
    case ROBOT_RPC_METHOD_BALANCE_ENABLE: {
      if (req_data_len != 0U) {
        app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                                length, NULL, 0U, resp_seq);
        return;
      }
      if (!app_try_arm_balancing(true)) {
        app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_NOT_READY, 0U, 0U,
                                NULL, 0U, resp_seq);
        return;
      }
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_OK, 0U, 0U, NULL, 0U,
                              resp_seq);
      return;
    }
    case ROBOT_RPC_METHOD_BALANCE_DISABLE: {
      if (req_data_len != 0U) {
        app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                                length, NULL, 0U, resp_seq);
        return;
      }
      s_motor_manual.enabled = 0U;
      s_motor_manual.left = 0.0f;
      s_motor_manual.right = 0.0f;
      motion_control_set_output_enabled(true);
      motion_control_set_mode(MOTION_MODE_DISARMED);
#ifdef ENABLE_MOTORS
      motor_link_set_wheel_Iq(0.0f, 0.0f, 0.0f);
      motor_link_enable(false);
#endif
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_OK, 0U, 0U, NULL, 0U,
                              resp_seq);
      return;
    }
    case ROBOT_RPC_METHOD_SET_PARAM: {
      float old_rate_hz = g_robot_params.control_rate_hz;
      if (length == 0U) {
        if (req_data_len != 0U ||
            (req.flags & ROBOT_RPC_FLAG_SAVE) == 0U) {
          app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                                  length, NULL, 0U, resp_seq);
          return;
        }
        uint8_t status = ROBOT_RPC_STATUS_OK;
        int rc = param_storage_save(&g_robot_params);
        if (rc != PARAM_OK) {
          status = ROBOT_RPC_STATUS_STORAGE;
        }
        app_rpc_send_param_resp(req.method, status, offset, length, NULL, 0U,
                                resp_seq);
        return;
      }
      if (req_data_len != length) {
        app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                                length, NULL, 0U, resp_seq);
        return;
      }

      robot_params_t updated;
      memcpy(&updated, &g_robot_params, sizeof(updated));
      memcpy((uint8_t *)&updated + offset, req_data, length);

      uint32_t primask = __get_PRIMASK();
      __disable_irq();
      bool changed = (memcmp(&updated, &g_robot_params, sizeof(updated)) != 0);
      if (changed) {
        memcpy(&g_robot_params, &updated, sizeof(updated));
      }
      __set_PRIMASK(primask);
      if (changed) {
        motion_control_apply_params();
        if (fabsf(g_robot_params.control_rate_hz - old_rate_hz) > 1e-3f) {
          control_timer_set_rate_hz(g_robot_params.control_rate_hz);
        }
      }

      uint8_t status = ROBOT_RPC_STATUS_OK;
      if ((req.flags & ROBOT_RPC_FLAG_SAVE) != 0U && changed) {
        int rc = param_storage_save(&g_robot_params);
        if (rc != PARAM_OK) {
          status = ROBOT_RPC_STATUS_STORAGE;
        }
      }

      app_rpc_send_param_resp(req.method, status, offset, length, NULL, 0U,
                              resp_seq);
      return;
    }
    default:
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_METHOD, offset,
                              length, NULL, 0U, resp_seq);
      return;
  }
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
    uint8_t flags = cmd->flags;
    bool estop = (flags & ROBOT_TELEOP_FLAG_ESTOP) != 0U;
    bool arm = (flags & ROBOT_TELEOP_FLAG_ARM) != 0U;
    uint8_t rising = (uint8_t)(flags & (uint8_t)~s_last_teleop_flags);
    bool estop_rise = (rising & ROBOT_TELEOP_FLAG_ESTOP) != 0U;
    bool arm_rise = (rising & ROBOT_TELEOP_FLAG_ARM) != 0U;
    bool mode_cycle_rise = (rising & ROBOT_TELEOP_FLAG_MODE_CYCLE) != 0U;
    s_last_teleop_flags = flags;

    s_estop_active = estop ? 1U : 0U;
    if (estop) {
      if (estop_rise) {
        APP_LOG_INFO("Teleop estop requested");
      }
      motion_control_set_mode(MOTION_MODE_DISARMED);
#ifdef ENABLE_MOTORS
      motor_link_enable(false);
#endif
      motion_control_set_teleop(0.0f, 0.0f);
    } else {
      if (arm_rise) {
        APP_LOG_INFO("Teleop arm requested");
        if (!app_try_arm_balancing(false)) {
          APP_LOG_ERROR("Arm rejected (not ready)");
        }
      }
      if (mode_cycle_rise) {
        APP_LOG_INFO("Teleop mode cycle requested (not implemented)");
      }
      motion_control_set_teleop(cmd->vx_mps, cmd->wz_radps);
    }
    APP_LOG_INFO("Teleop fwd=%.2f turn=%.2f flags=0x%04x", (double)cmd->vx_mps,
                 (double)cmd->wz_radps, cmd->flags);
  } else if (msg_type == ROBOT_MSG_CMD_HEARTBEAT) {
    /* Heartbeat handled silently - logging would spam */
  } else if (msg_type == ROBOT_MSG_CMD_ARM) {
    APP_LOG_INFO("Arm requested");
    if (!app_try_arm_balancing(false)) {
      APP_LOG_ERROR("Arm rejected (not ready)");
      return;
    }
  } else if (msg_type == ROBOT_MSG_CMD_DISARM) {
    APP_LOG_INFO("Disarm requested");
    motion_control_set_mode(MOTION_MODE_DISARMED);
#ifdef ENABLE_MOTORS
    motor_link_enable(false);
#endif
  } else if (msg_type == ROBOT_MSG_CMD_REBOOT) {
    uint8_t mode = (len > 0U) ? payload[0] : 0U;
    if (mode == 1U) {
      APP_LOG_INFO("Reboot to bootloader requested");
      HAL_Delay(50U); /* Allow log/response to flush */
      system_reboot_to_bootloader();
    } else {
      APP_LOG_INFO("Normal reboot requested");
      HAL_Delay(50U);
      system_reboot();
    }
  } else {
    APP_LOG_INFO("CMD handler invoked, msg_type=0x%02x", msg_type);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart != APP_LINK_UART) {
    return;
  }
  if (app_link_dma_is_circular()) {
    s_uart_rx_event_pending = 1U;
    return;
  }

  s_uart_rx_pending_size = Size;
  s_uart_rx_event_pending = 1U;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart != APP_LINK_UART) {
    return;
  }
  s_link_uart_errors++;
  s_link_uart_last_err = (uint32_t)huart->ErrorCode;
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

static void app_log_link_errors(void) {
  uint32_t decode_fail = 0U;
  uint32_t decode_len = 0U;
  uint32_t overflow = 0U;
  uint32_t uart_errs = 0U;
  uint32_t uart_last = 0U;

  __disable_irq();
  decode_fail = s_link_decode_failures;
  decode_len = s_link_decode_last_len;
  s_link_decode_failures = 0U;
  overflow = s_link_overflows;
  s_link_overflows = 0U;
  uart_errs = s_link_uart_errors;
  uart_last = s_link_uart_last_err;
  s_link_uart_errors = 0U;
  __enable_irq();

  if (decode_fail > 0U) {
    APP_LOG_ERROR("Frame decode failed x%lu (last len=%lu)",
                  (unsigned long)decode_fail, (unsigned long)decode_len);
  }
  if (overflow > 0U) {
    APP_LOG_ERROR("UART frame overflow x%lu", (unsigned long)overflow);
  }
  if (uart_errs > 0U) {
    APP_LOG_ERROR("UART error 0x%lx x%lu", (unsigned long)uart_last,
                  (unsigned long)uart_errs);
  }
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

  s_link_send_last_err = APP_LINK_SEND_OK;
  s_link_send_last_status = 0U;
  s_link_send_last_hal_state = 0U;
  s_link_send_last_hal_err = 0U;

  if (APP_LINK_UART == NULL) {
    s_link_send_last_err = APP_LINK_SEND_ERR_UART_NULL;
    return false;
  }
  if (app_in_isr()) {
    s_link_send_last_err = APP_LINK_SEND_ERR_ISR;
    return false;
  }

  uint16_t seq = (seq_override != 0U)
                     ? seq_override
                     : ++s_seq_counters[robot_channel_from_type(type)];
  if (!robot_frame_init(&frame, type, seq, flags, payload, len)) {
    s_link_send_last_err = APP_LINK_SEND_ERR_FRAME_INIT;
    return false;
  }
  if (!robot_frame_encode(&frame, encoded, sizeof(encoded), &encoded_len)) {
    s_link_send_last_err = APP_LINK_SEND_ERR_ENCODE;
    return false;
  }

  HAL_StatusTypeDef status =
      HAL_UART_Transmit(APP_LINK_UART, encoded, (uint16_t)encoded_len, 10U);
  if (status == HAL_OK) {
    return true;
  }
  s_link_send_last_status = (uint32_t)status;
  s_link_send_last_hal_state = (uint32_t)APP_LINK_UART->gState;
  s_link_send_last_hal_err = (uint32_t)APP_LINK_UART->ErrorCode;
  if (status == HAL_BUSY) {
    s_link_send_last_err = APP_LINK_SEND_ERR_UART_BUSY;
  } else if (status == HAL_TIMEOUT) {
    s_link_send_last_err = APP_LINK_SEND_ERR_UART_TIMEOUT;
  } else {
    s_link_send_last_err = APP_LINK_SEND_ERR_UART_ERROR;
  }
  return false;
}

static void app_send_telem(void) {
  robot_telem_v2_t telem;
  memset(&telem, 0, sizeof(telem));
  telem.version = 2U;
  telem.timestamp_ms = HAL_GetTick();

  /* Motion mode and status */
  motion_mode_t mode = motion_control_get_mode();
  telem.motion_mode = (uint8_t)mode;
  telem.status = 0U;
  telem.faults = 0U;
  if (mode == MOTION_MODE_BALANCING) {
    telem.status |= ROBOT_STATUS_ARMED;
  }
  if (mode == MOTION_MODE_FAULT) {
    telem.status |= ROBOT_STATUS_FAULT;
  }
  if (s_estop_active) {
    telem.status |= ROBOT_STATUS_ESTOP;
  }
  if (motion_control_is_calibrated()) {
    telem.status |= ROBOT_STATUS_IMU_CAL;
  }

  /* EKF state estimate */
  motion_control_estimate_t est;
  if (motion_control_get_estimate(&est)) {
    telem.theta_rad = est.theta_rad;
    telem.theta_dot = est.theta_dot;
    telem.x_m = est.x_m;
    telem.x_dot_mps = est.x_dot_mps;
    telem.gyro_bias = est.gyro_bias;
    telem.estimate_valid = est.valid;
    if (est.valid && g_robot_params.balance.thetaKill > 0.0f &&
        fabsf(est.theta_rad) > g_robot_params.balance.thetaKill) {
      telem.faults |= ROBOT_FAULT_KILL_ANGLE;
    }
  }

  /* IMU health metrics */
  motion_control_imu_health_t imu_health;
  if (motion_control_get_imu_health(&imu_health)) {
    telem.imu_active = imu_health.active_sensor;
    telem.imu_gate_accel = imu_health.gate_accel;
    telem.imu_gyro_diff_dps = imu_health.gyro_diff_dps;
    telem.imu_vib_rms_g = imu_health.vib_rms_g;
  }

  uint32_t now_ms = telem.timestamp_ms;
  uint32_t last_imu_ok_ms = motion_control_get_last_imu_ok_ms();
  if (last_imu_ok_ms != 0U &&
      (now_ms - last_imu_ok_ms) > IMU_FAULT_FATAL_MS) {
    telem.faults |= ROBOT_FAULT_IMU_TIMEOUT;
  }
  uint32_t last_motor_ok_ms = motion_control_get_last_motor_ok_ms();
  if (last_motor_ok_ms != 0U &&
      (now_ms - last_motor_ok_ms) > MOTOR_LINK_FAULT_FATAL_MS) {
    telem.faults |= ROBOT_FAULT_MOTOR_TIMEOUT;
  }

  /* Wheel velocities */
#ifdef ENABLE_MOTORS
  float left_w = 0.0f;
  float right_w = 0.0f;
  if (motor_link_get_wheel_velocities(&left_w, &right_w)) {
    telem.wheel_left_rps = left_w;
    telem.wheel_right_rps = right_w;
    telem.status |= ROBOT_STATUS_LINK_OK;
  }
  telem.motor_left_ack_timeouts = motor_link_get_left_ack_timeouts();
  telem.motor_right_ack_timeouts = motor_link_get_right_ack_timeouts();
#endif

  /* Control outputs */
  motion_control_output_t ctrl_out;
  if (motion_control_get_control_output(&ctrl_out)) {
    telem.iq_left = ctrl_out.iq_left;
    telem.iq_right = ctrl_out.iq_right;
    telem.pitch_target_rad = ctrl_out.pitch_target_rad;
  }

  if (!app_link_send(ROBOT_MSG_TELEM_FRAME_V2, 0U, (const uint8_t *)&telem,
                     sizeof(telem), 0U)) {
    s_telem_fail_count++;
    if (s_telem_fail_count >= TELEM_FAIL_THRESHOLD) {
      led_status_set_flag(LED_STATUS_TELEM_FAILURE);
    }
    unsigned long cts = (unsigned long)(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET);
    unsigned long rts = (unsigned long)(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET);
    APP_LOG_ERROR("Failed to send telem frame reason=%s cts=%lu rts=%lu status=0x%lx state=0x%lx err=0x%lx",
                  app_link_send_err_str(s_link_send_last_err),
                  cts,
                  rts,
                  (unsigned long)s_link_send_last_status,
                  (unsigned long)s_link_send_last_hal_state,
                  (unsigned long)s_link_send_last_hal_err);
  } else {
    /* Success: clear failure counter and LED flag */
    if (s_telem_fail_count > 0U) {
      s_telem_fail_count = 0U;
      led_status_clear_flag(LED_STATUS_TELEM_FAILURE);
      APP_LOG_INFO("Telem send recovered; clearing telem failure flag");
    }
  }
}
