#include "app_telem.h"

#include <math.h>
#include <string.h>

#include "app_adc.h"
#include "app_config.h"
#include "app_link.h"
#include "app_main.h"
#include "led_status.h"
#include "motion_control.h"
#include "motor_link.h"
#include "robot_protocol.h"
#include "stm32h7xx_hal.h"
#include "config_control.h"



static uint32_t s_last_telem_ms = 0U;
static uint32_t s_telem_fail_count = 0U;

void app_telem_init(void) {
  s_last_telem_ms = HAL_GetTick();
  s_telem_fail_count = 0U;
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
  if (g_estop_active) {
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
  if (last_imu_ok_ms != 0U && (now_ms - last_imu_ok_ms) > IMU_FAULT_FATAL_MS) {
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

  /* ADC voltage reading */
  float adc_voltage = 0.0f;
  if (app_adc_read_voltage(&adc_voltage)) {
    telem.adc_voltage = adc_voltage;
  }

  if (!app_link_send(ROBOT_MSG_TELEM_FRAME_V2, 0U, (const uint8_t *)&telem,
                     sizeof(telem), 0U)) {
    app_link_send_err_t last_err;
    uint32_t last_status, last_hal_state, last_hal_err;
    app_link_get_last_send_error(&last_err, &last_status, &last_hal_state,
                                 &last_hal_err);

    if (last_err != APP_LINK_SEND_ERR_UART_BUSY) {
      s_telem_fail_count++;
      if (s_telem_fail_count >= TELEM_FAIL_THRESHOLD) {
        led_status_set_flag(LED_STATUS_TELEM_FAILURE);
      }
      unsigned long cts =
          (unsigned long)(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET);
      unsigned long rts =
          (unsigned long)(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET);
      APP_LOG_ERROR(
          "Failed to send telem frame reason=%s cts=%lu rts=%lu "
          "status=0x%lx state=0x%lx err=0x%lx",
          app_link_send_err_str(last_err), cts, rts,
          (unsigned long)last_status, (unsigned long)last_hal_state,
          (unsigned long)last_hal_err);
    }
  } else {
    /* Success: clear failure counter and LED flag */
    if (s_telem_fail_count > 0U) {
      s_telem_fail_count = 0U;
      led_status_clear_flag(LED_STATUS_TELEM_FAILURE);
      APP_LOG_INFO("Telem send recovered; clearing telem failure flag");
    }
  }
}

void app_telem_tick(uint32_t now_ms) {
  if ((now_ms - s_last_telem_ms) >= APP_TELEM_PERIOD_MS) {
    app_send_telem();
    s_last_telem_ms = now_ms;
  }
}
