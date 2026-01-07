#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_arm.h"
#include "app_config.h"
#include "app_imu.h"
#include "app_link.h"
#include "app_main.h"
#include "app_motor.h"
#include "app_telem.h"

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

#define APP_HEARTBEAT_TIMEOUT_MS 200U
#define APP_LOG_PERIOD_MS 500U

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
    app_link_poll();
#ifdef ENABLE_MOTORS
    motor_link_poll();
#endif
    imu_sched_tick();

    /* 3. Background Tasks (Telemetry, Logging, LED, etc.) */
    app_idle_tick();
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
  HAL_Delay(2000);
  /* TODO: Verify if 2000ms delay is strictly necessary for hardware settling */
  HAL_Delay(500); 
  APP_LOG_INFO("Booting robot firmware (frame v%u)", ROBOT_FRAME_VERSION);
  APP_LOG_INFO("CMD channel id: %u", ROBOT_CHANNEL_CMD);

  /* Load robot parameters from flash (or use defaults) */
  param_storage_init();
  int param_rc = param_storage_load(&g_robot_params);
  if (param_rc != PARAM_OK && param_rc != PARAM_ERR_NOT_FOUND) {
    APP_LOG_ERROR("Param load error: %d", param_rc);
  }

  motion_control_init();

  s_last_cmd_ms = HAL_GetTick();
  app_imu_init();

#ifdef ENABLE_MOTORS
  WDOG_CHECKPOINT(WDOG_CP_MOTOR_INIT);
  if (!motor_link_init()) {
    APP_LOG_ERROR("Motor link init failed");
  }
  motor_link_set_motor_directions(g_robot_params.motor_direction[0],
                                  g_robot_params.motor_direction[1]);
#endif

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
    app_log_flush_blocking(2000U);
    if (mode == 1U) {
      system_reboot_to_bootloader();
    } else {
      system_reboot();
    }
  }

  /* Heartbeat timeout detection: disarm robot if no commands received */
  if ((now - s_last_cmd_ms) > APP_HEARTBEAT_TIMEOUT_MS) {
    motion_control_set_mode(MOTION_MODE_DISARMED);
#ifdef ENABLE_MOTORS
    motor_link_enable(false);
#endif
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
}

void app_cdc_handle_frame(const uint8_t *data, uint32_t len) {
  if (data == NULL || len == 0U) {
    return;
  }
  /* CDC frames share the link decoder with UART frames. */
  /* Reuse the UART COBS accumulator so partial CDC packets are handled. */
  app_link_process_chunk(data, len);
}
