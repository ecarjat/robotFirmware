#include "app_motor.h"
#include "app_config.h"
#include "app_main.h"
#include "motion_control.h"
#include "motor_link.h"
#include "robot_protocol.h"


app_motor_manual_t s_motor_manual = {0U, 0.0f, 0.0f};

void app_motor_manual_apply(void) {
#ifdef ENABLE_MOTORS
  if (!s_motor_manual.enabled) {
    return;
  }

  static uint32_t s_last_manual_ms = 0U;
  uint32_t now = HAL_GetTick();
  if ((now - s_last_manual_ms) < APP_MOTOR_MANUAL_PERIOD_MS) {
    return;
  }
  s_last_manual_ms = now;

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

uint8_t app_motor_manual_run(uint8_t side, float intensity) {
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

uint8_t app_motor_manual_enable(bool enable) {
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
