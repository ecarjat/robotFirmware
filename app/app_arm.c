#include "app_config.h"
#include "app_motor.h"
#include "motion_control.h"
#include "motor_link.h"
#include "stm32h7xx_hal.h"

bool app_try_arm_balancing(bool prepare_balance) {
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
  if (!motor_link_enable(true)) {
    APP_LOG_ERROR("Motor link enable failed");
    motion_control_set_mode(MOTION_MODE_DISARMED);
    motion_control_set_output_enabled(true);
    s_motor_manual.enabled = 0U;
    s_motor_manual.left = 0.0f;
    s_motor_manual.right = 0.0f;
    __set_PRIMASK(primask);
    return false;
  }
#endif
  __set_PRIMASK(primask);
  return true;
}

void app_disarm_robot(void) {
  /* Check if already disarmed to avoid spamming motor link */
  if (motion_control_get_mode() == MOTION_MODE_DISARMED &&
      s_motor_manual.enabled == 0U) {
    return;
  }

  s_motor_manual.enabled = 0U;
  s_motor_manual.left = 0.0f;
  s_motor_manual.right = 0.0f;
  /* Re-enable motion controller output processing (it will be zero in
   * disarmed state) */
  motion_control_set_output_enabled(true);
  motion_control_set_mode(MOTION_MODE_DISARMED);
#ifdef ENABLE_MOTORS
  /* Explicitly command zero torque before disabling motor drivers */
  motor_link_set_wheel_Iq(0.0f, 0.0f, 0.0f);
  if (!motor_link_enable(false)) {
    APP_LOG_ERROR("Motor link disable failed");
  }
#endif
}
