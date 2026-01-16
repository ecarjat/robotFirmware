#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_adc.h"
#include "app_arm.h"
#include "app_config.h"
#include "app_file.h"
#include "app_imu.h"
#include "app_link.h"
#include "app_main.h"
#include "app_motor.h"
#include "app_telem.h"
#include "app_profiling.h"

#include "debug_wdog.h"

#include "imu_sched.h"
#include "motion_control.h"
#include "motor_link.h"

#include "config_control.h"
#include "control_timer.h"
#include "led_status.h"
#include "sensors.h"

#include "system_reboot.h"
#include "usbd_cdc_if.h"

#include "logging/blackbox.h"
#include "logging/blackbox_dump.h"
#include "qspi_w25q64.h"

#define ROBOT_USE_HW_CRC 1

static void app_init(void);
static void app_idle_tick(void);
void app_cdc_handle_frame(const uint8_t *data, uint32_t len);

void app_main(void) {
  app_init();

  /* Main Super-Loop */
  while (1) {
    /* 1. High Priority: Control Loop
     * Run this immediately when the timer flag is set to minimize jitter.
     */
    if (control_timer_pending()) {
      control_timer_begin_cycle();
      motion_control_tick(HAL_GetTick());
      control_timer_end_cycle();
    }

    /* 2. High Priority: Link Polling
     * Keep communication buffers drained.
     */
    uint32_t start = APP_PROFILE_GET_TIME();
    app_link_poll();
    app_profiling_record(APP_PROF_LINK, start);

#ifdef ENABLE_MOTORS
    start = APP_PROFILE_GET_TIME();
    motor_link_poll();
    app_profiling_record(APP_PROF_MOTOR, start);
#endif
    start = APP_PROFILE_GET_TIME();
    imu_sched_tick();
    app_profiling_record(APP_PROF_IMU, start);

    /* 3. Background Tasks (Telemetry, Logging, LED, etc.) */
    uint32_t idle_budget_us = control_timer_time_to_deadline_us();
    if (!control_timer_pending() && idle_budget_us >= APP_IDLE_BUDGET_US) {
      start = APP_PROFILE_GET_TIME();
      app_idle_tick();
      app_profiling_record(APP_PROF_IDLE, start);
    }
  }
}

static uint32_t s_last_log_ms = 0U;
static uint32_t s_last_led_ms = 0U;
static uint8_t s_last_imu_active = 0xFFU;
static uint8_t s_last_imu_active_valid = 0U;

static const char *app_imu_name(uint8_t active) {
  return (active == 1U) ? "ICM42688" : "BMI270";
}


/* Global robot parameters (loaded from flash at startup) */
robot_params_t g_robot_params;
/* global reboot */
volatile uint8_t g_reboot_request = 0U; /* 0=none, 1=bootloader, x =normal*/
volatile uint8_t g_estop_active = 0U;

static void app_init(void) {
  WDOG_CHECKPOINT(WDOG_CP_APP_INIT_START);
  /* Short delay for sensor power stabilization */
  HAL_Delay(APP_POWER_SETTLE_DELAY_MS);
  APP_LOG_INFO("Booting robot firmware (frame v%u)", ROBOT_FRAME_VERSION);
  APP_LOG_INFO("CMD channel id: %u", ROBOT_CHANNEL_CMD);

  /* Load robot parameters from flash (or use defaults) */
  param_storage_init();
  int param_rc = param_storage_load(&g_robot_params);
  if (param_rc != PARAM_OK && param_rc != PARAM_ERR_NOT_FOUND) {
    APP_LOG_ERROR("Param load error: %d", param_rc);
  }

  /* Initialize blackbox logging */
  qspi_w25q64_init();
  uint8_t manufacturer;
  uint16_t device;
  qspi_w25q64_read_id(&manufacturer, &device);
  APP_LOG_INFO("QSPI W25Q64 flash manufacturer=0x%02x device=0x%04x",
               manufacturer, device);


  if (qspi_w25q64_is_ready()) {
    APP_LOG_INFO("QSPI W25Q64 flash ready");
  } else {
    APP_LOG_ERROR("QSPI W25Q64 flash NOT ready - blackbox disabled");
  }
  log_init(&g_robot_params);
  log_dump_init();
  if (log_is_initialized()) {
    APP_LOG_INFO("Blackbox logging initialized (fields=0x%08lx)",
                 (unsigned long)g_robot_params.log_fields_mask);
  } else {
    APP_LOG_ERROR("Blackbox logging failed to initialize");
  }

  /* Initialize file transfer */
  app_file_init();

  motion_control_init();

  s_last_cmd_ms = HAL_GetTick();
  app_imu_init();
  app_adc_init();

#ifdef ENABLE_MOTORS
  WDOG_CHECKPOINT(WDOG_CP_MOTOR_INIT);
  if (!motor_link_init()) {
    APP_LOG_ERROR("Motor link init failed");
  }
  motor_link_set_motor_directions(g_robot_params.motor_direction[0],
                                  g_robot_params.motor_direction[1]);
#endif

  app_profiling_init();
  app_telem_init();
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
  uint32_t now = HAL_GetTick();
  app_telem_tick(now);
  app_motor_manual_apply();

  if (g_reboot_request != 0U) {
    uint8_t mode = g_reboot_request;
    g_reboot_request = 0U;
    APP_LOG_INFO("Reboot requested (mode=%u)", (unsigned int)mode);
    app_log_flush_blocking(APP_LOG_FLUSH_TIMEOUT_MS);
    if (mode == 1U) {
      system_reboot_to_bootloader();
    } else {
      system_reboot();
    }
  }

  /* Heartbeat timeout detection: disarm robot if no commands received */
  if ((now - s_last_cmd_ms) > APP_HEARTBEAT_TIMEOUT_MS) {
    app_disarm_robot();
    led_status_set_flag(LED_STATUS_TELEM_FAILURE);
    // APP_LOG_ERROR("Link timeout: no frames for %lu ms",
    //               (unsigned long)(now - s_last_cmd_ms));
  }
  
  if ((now - s_last_log_ms) >= APP_LOG_PERIOD_MS) {
    s_last_log_ms = now;
    app_log_link_errors();
    motion_control_imu_health_t imu_health;
    if (motion_control_get_imu_health(&imu_health)) {
      APP_LOG_INFO(
          "IMU health active=%u gyro_diff=%.2f dps pitch_diff=%.2f dps "
          "acc_angle=%.2f deg vib=%.3f g gate=%u",
          (unsigned int)imu_health.active_sensor,
          (double)imu_health.gyro_diff_dps,
          (double)imu_health.gyro_pitch_diff_dps,
          (double)imu_health.acc_angle_diff_deg, (double)imu_health.vib_rms_g,
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
          (unsigned int)ekf_log.gate, (unsigned long)ekf_log.imu_primary_dt_ms,
          (unsigned long)ekf_log.imu_secondary_dt_ms);
    }

#ifdef ENABLE_MOTORS
    float gyro_z = 0.0f;
    float yaw_rate_enc = 0.0f;
    float yaw_rate = 0.0f;
    if (motion_control_get_yaw_debug(&gyro_z, &yaw_rate_enc, &yaw_rate)) {
      APP_LOG_INFO("Yaw debug gyroZ=%.4f enc=%.4f blended=%.4f", (double)gyro_z,
                   (double)yaw_rate_enc, (double)yaw_rate);
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
    app_profiling_log();
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
    if (last_motor_ok != 0U &&
        (now - last_motor_ok) > MOTOR_LINK_FAULT_FATAL_MS) {
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

  /* Blackbox logging background tasks */
  log_writer_tick();
  log_erase_tick();
  log_dump_tick();
}

void app_cdc_handle_frame(const uint8_t *data, uint32_t len) {
  if (data == NULL || len == 0U) {
    return;
  }
  /* Push to ring buffer for processing in main loop to avoid blocking ISR */
  app_link_feed_cdc(data, len);
}
