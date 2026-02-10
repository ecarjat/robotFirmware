#include "app_telem.h"

#include <math.h>
#include <string.h>

#include "app_adc.h"
#include "app_config.h"
#include "app_link.h"
#include "app_main.h"
#include "led_status.h"
#include "logging/blackbox_dump.h"
#include "motion_control.h"
#include "hip_control.h"
#include "motor_link.h"
#include "robot_protocol.h"
#include "stm32h7xx_hal.h"
#include "config_control.h"



static uint32_t s_last_telem_ms = 0U;
static uint32_t s_telem_fail_count = 0U;
static uint32_t s_telem_next_send_ms = 0U;

#define TELEM_BACKOFF_BASE_MS 200U
#define TELEM_BACKOFF_MAX_MS 2000U
#define TELEM_RESTART_THRESHOLD 5U

void app_telem_init(void) {
  s_last_telem_ms = HAL_GetTick();
  s_telem_fail_count = 0U;
  s_telem_next_send_ms = 0U;
}

static uint32_t telem_next_backoff(uint32_t fail_count) {
  uint32_t backoff = TELEM_BACKOFF_BASE_MS;
  uint32_t steps = (fail_count > 0U) ? (fail_count - 1U) : 0U;
  while (steps > 0U && backoff < TELEM_BACKOFF_MAX_MS) {
    if (backoff > (TELEM_BACKOFF_MAX_MS / 2U)) {
      backoff = TELEM_BACKOFF_MAX_MS;
      break;
    }
    backoff *= 2U;
    steps--;
  }
  if (backoff > TELEM_BACKOFF_MAX_MS) {
    backoff = TELEM_BACKOFF_MAX_MS;
  }
  return backoff;
}

static void app_send_telem(void) {
  robot_telem_v3_t telem;
  memset(&telem, 0, sizeof(telem));
  telem.v2.version = 3U;
  telem.v2.timestamp_ms = HAL_GetTick();

  /* Motion mode and status */
  motion_mode_t mode = motion_control_get_mode();
  telem.v2.motion_mode = (uint8_t)mode;
  telem.v2.status = 0U;
  telem.v2.faults = 0U;
  if (mode == MOTION_MODE_BALANCING) {
    telem.v2.status |= ROBOT_STATUS_ARMED;
  }
  if (mode == MOTION_MODE_FAULT) {
    telem.v2.status |= ROBOT_STATUS_FAULT;
  }
  if (g_estop_active) {
    telem.v2.status |= ROBOT_STATUS_ESTOP;
  }
  if (motion_control_is_calibrated()) {
    telem.v2.status |= ROBOT_STATUS_IMU_CAL;
  }
  if (log_is_dumping()) {
    telem.v2.status |= ROBOT_STATUS_DUMPING;
  }

  /* EKF state estimate */
  motion_control_estimate_t est;
  if (motion_control_get_estimate(&est)) {
    telem.v2.theta_rad = est.theta_rad;
    telem.v2.theta_dot = est.theta_dot;
    telem.v2.x_m = est.x_m;
    telem.v2.x_dot_mps = est.x_dot_mps;
    telem.v2.gyro_bias = est.gyro_bias;
    telem.v2.estimate_valid = est.valid;
    if (est.valid && g_robot_params.balance.thetaKill > 0.0f &&
        fabsf(est.theta_rad) > g_robot_params.balance.thetaKill) {
      telem.v2.faults |= ROBOT_FAULT_KILL_ANGLE;
    }
  }

  /* IMU health metrics */
  motion_control_imu_health_t imu_health;
  if (motion_control_get_imu_health(&imu_health)) {
    telem.v2.imu_active = imu_health.active_sensor;
    telem.v2.imu_gate_accel = imu_health.gate_accel;
    telem.v2.imu_gyro_diff_dps = imu_health.gyro_diff_dps;
    telem.v2.imu_vib_rms_g = imu_health.vib_rms_g;
  }

  uint32_t now_ms = telem.v2.timestamp_ms;
  uint32_t last_imu_ok_ms = motion_control_get_last_imu_ok_ms();
  if (last_imu_ok_ms != 0U && (now_ms - last_imu_ok_ms) > IMU_FAULT_FATAL_MS) {
    telem.v2.faults |= ROBOT_FAULT_IMU_TIMEOUT;
  }
  uint32_t last_motor_ok_ms = motion_control_get_last_motor_ok_ms();
  if (last_motor_ok_ms != 0U &&
      (now_ms - last_motor_ok_ms) > MOTOR_LINK_FAULT_FATAL_MS) {
    telem.v2.faults |= ROBOT_FAULT_MOTOR_TIMEOUT;
  }

  /* Wheel velocities */
#ifdef ENABLE_MOTORS
  float left_w = 0.0f;
  float right_w = 0.0f;
  if (motor_link_get_wheel_velocities(&left_w, &right_w)) {
    telem.v2.wheel_left_rps = left_w;
    telem.v2.wheel_right_rps = right_w;
    telem.v2.status |= ROBOT_STATUS_LINK_OK;
  }
  telem.v2.motor_left_ack_timeouts = motor_link_get_left_ack_timeouts();
  telem.v2.motor_right_ack_timeouts = motor_link_get_right_ack_timeouts();
#endif

  /* Control outputs */
  motion_control_output_t ctrl_out;
  if (motion_control_get_control_output(&ctrl_out)) {
    telem.v2.torque_left_nm = ctrl_out.torque_left_nm;
    telem.v2.torque_right_nm = ctrl_out.torque_right_nm;
    telem.v2.pitch_target_rad = ctrl_out.pitch_target_rad;
  }

  /* ADC voltage reading */
  float adc_voltage = 0.0f;
  if (app_adc_read_voltage(&adc_voltage)) {
    telem.v2.adc_voltage = adc_voltage;
  }

  hip_state_t hip_left = {0};
  hip_state_t hip_right = {0};
  hip_command_t hip_cmd_left = {0};
  hip_command_t hip_cmd_right = {0};
  hip_control_get_state(&hip_left, &hip_right);
  hip_control_get_command(&hip_cmd_left, &hip_cmd_right);
  telem.hip_left_cmd_pos_rev = hip_cmd_left.pos_cmd_rev;
  telem.hip_left_cmd_vel_rev_s = hip_cmd_left.vel_ff_rev_s;
  telem.hip_left_cmd_torque_nm = hip_cmd_left.torque_ff_nm;
  telem.hip_right_cmd_pos_rev = hip_cmd_right.pos_cmd_rev;
  telem.hip_right_cmd_vel_rev_s = hip_cmd_right.vel_ff_rev_s;
  telem.hip_right_cmd_torque_nm = hip_cmd_right.torque_ff_nm;
  telem.hip_left_pos_rev = hip_left.pos_rev;
  telem.hip_left_vel_rev_s = hip_left.vel_rev_s;
  telem.hip_left_torque_nm = hip_left.torque_nm;
  telem.hip_right_pos_rev = hip_right.pos_rev;
  telem.hip_right_vel_rev_s = hip_right.vel_rev_s;
  telem.hip_right_torque_nm = hip_right.torque_nm;
  if (hip_left.valid && hip_right.valid) {
    telem.hip_height_m = 0.5f * (hip_left.height_m + hip_right.height_m);
    telem.hip_height_dot_m_s = 0.5f * (hip_left.height_dot_m_s + hip_right.height_dot_m_s);
  } else if (hip_left.valid) {
    telem.hip_height_m = hip_left.height_m;
    telem.hip_height_dot_m_s = hip_left.height_dot_m_s;
  } else if (hip_right.valid) {
    telem.hip_height_m = hip_right.height_m;
    telem.hip_height_dot_m_s = hip_right.height_dot_m_s;
  }
  telem.hip_mode = motion_control_get_hip_behavior_mode();
  telem.hip_phase_progress_pct = motion_control_get_hip_phase_progress();
  telem.hip_faults = (uint16_t)hip_control_get_faults();
  telem.hip_left_rx_age_ms = (hip_left.last_rx_ms != 0U) ?
                             (now_ms - hip_left.last_rx_ms) : 0U;
  telem.hip_right_rx_age_ms = (hip_right.last_rx_ms != 0U) ?
                              (now_ms - hip_right.last_rx_ms) : 0U;

  if (telem.hip_faults & (HIP_FAULT_HEARTBEAT_TIMEOUT | HIP_FAULT_ENCODER_TIMEOUT | HIP_FAULT_TORQUE_TIMEOUT)) {
    telem.v2.faults |= ROBOT_FAULT_HIP_TIMEOUT;
  }
  if (telem.hip_faults & HIP_FAULT_BUS_OFF) {
    telem.v2.faults |= ROBOT_FAULT_HIP_BUS_OFF;
  }
  if (telem.hip_faults & HIP_FAULT_KINEMATICS) {
    telem.v2.faults |= ROBOT_FAULT_HIP_KINEMATICS;
  }
  if (telem.hip_faults & HIP_FAULT_STALL) {
    telem.v2.faults |= ROBOT_FAULT_HIP_STALL;
  }

  if (!app_link_send(ROBOT_MSG_TELEM_FRAME_V3, 0U, (const uint8_t *)&telem,
                     sizeof(telem), 0U)) {
    app_link_send_err_t last_err;
    uint32_t last_status, last_hal_state, last_hal_err;
    app_link_get_last_send_error(&last_err, &last_status, &last_hal_state,
                                 &last_hal_err);

    if (last_err != APP_LINK_SEND_ERR_UART_BUSY) {
      s_telem_fail_count++;
      uint32_t backoff_ms = telem_next_backoff(s_telem_fail_count);
      s_telem_next_send_ms = telem.v2.timestamp_ms + backoff_ms;
      if (s_telem_fail_count >= TELEM_FAIL_THRESHOLD) {
        led_status_set_flag(LED_STATUS_TELEM_FAILURE);
      }
      if (s_telem_fail_count == TELEM_RESTART_THRESHOLD) {
        APP_LOG_WARN("Telem failures reached %lu; restarting link",
                     (unsigned long)s_telem_fail_count);
        app_link_start();
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
      s_telem_next_send_ms = 0U;
      led_status_clear_flag(LED_STATUS_TELEM_FAILURE);
      APP_LOG_INFO("Telem send recovered; clearing telem failure flag");
    }
  }
}

void app_telem_tick(uint32_t now_ms) {
  if ((now_ms - s_last_telem_ms) >= APP_TELEM_PERIOD_MS) {
    s_last_telem_ms = now_ms;
    if (s_telem_next_send_ms != 0U &&
        (int32_t)(now_ms - s_telem_next_send_ms) < 0) {
      return;
    }
    app_send_telem();
  }
}
