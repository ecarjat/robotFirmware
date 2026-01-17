#include "app_arm.h"
#include "app_config.h"
#include "motion_control.h"
#include "motor_link.h"
#include "robot_protocol.h"
#include "logging/blackbox_dump.h"
#include "param_storage.h"

extern uint8_t g_reboot_request;
extern robot_params_t g_robot_params;
static uint8_t s_last_teleop_flags = 0U;
extern uint8_t g_estop_active;

void app_cmd_handler(uint8_t msg_type, const uint8_t *payload, size_t len,
                     void *ctx) {
  (void)ctx;
  if (msg_type == ROBOT_MSG_CMD_TELEOP) {
    if (len < sizeof(robot_cmd_teleop_t)) {
      APP_LOG_ERROR("CMD teleop size mismatch (%u)", (unsigned int)len);
      return;
    }
    const robot_cmd_teleop_t *cmd = (const robot_cmd_teleop_t *)payload;
    uint8_t flags = cmd->flags;
    bool estop = (flags & ROBOT_TELEOP_FLAG_ESTOP) != 0U;
    uint8_t rising = (uint8_t)(flags & (uint8_t)~s_last_teleop_flags);
    bool estop_rise = (rising & ROBOT_TELEOP_FLAG_ESTOP) != 0U;
    bool arm_rise = (rising & ROBOT_TELEOP_FLAG_ARM) != 0U;
    bool mode_cycle_rise = (rising & ROBOT_TELEOP_FLAG_MODE_CYCLE) != 0U;
    bool dump_rise = (rising & ROBOT_TELEOP_FLAG_DUMP) != 0U;
    bool lqr_mode = (flags & ROBOT_TELEOP_FLAG_LQR_MODE) != 0U;

    s_last_teleop_flags = flags;

    /* Set inner loop mode directly from flag (0=PID, 1=LQR) */
    motion_control_set_inner_mode(lqr_mode ? 1U : 0U);

    if (dump_rise) {
      uint32_t dump_seconds = g_robot_params.dump_seconds_default;
      if (dump_seconds == 0U) {
        dump_seconds = 30U;  /* Fallback default */
      }
      APP_LOG_INFO("Teleop dump requested (%u seconds)", (unsigned int)dump_seconds);
      if (!log_dump_last_seconds(dump_seconds)) {
        APP_LOG_ERROR("Dump rejected (already in progress or SD not ready)");
      }
    }

    g_estop_active = estop ? 1U : 0U;
    if (estop) {
      if (estop_rise) {
        APP_LOG_INFO("Teleop estop requested");
      }
      app_disarm_robot();
      motion_control_set_teleop(0.0f, 0.0f);
    } else {
      if (arm_rise) {
        APP_LOG_INFO("Teleop arm requested");
        if (!app_try_arm_balancing(true)) {
          APP_LOG_ERROR("Arm rejected (not ready)");
        }
      }
      if (mode_cycle_rise) {
        APP_LOG_INFO("Teleop mode cycle requested (not implemented)");
      }
      motion_control_set_teleop(cmd->vx_mps, cmd->wz_radps);
    }
    // APP_LOG_INFO("Teleop fwd=%.2f turn=%.2f flags=0x%04x",
    // (double)cmd->vx_mps,
    //              (double)cmd->wz_radps, cmd->flags);
  } else if (msg_type == ROBOT_MSG_CMD_HEARTBEAT) {
    /* Heartbeat handled silently - logging would spam */
  } else if (msg_type == ROBOT_MSG_CMD_ARM) {
    APP_LOG_INFO("Arm requested");
    if (!app_try_arm_balancing(true)) {
      APP_LOG_ERROR("Arm rejected (not ready)");
      return;
    }
  } else if (msg_type == ROBOT_MSG_CMD_DISARM) {
    APP_LOG_INFO("Disarm requested");
    app_disarm_robot();
  } else if (msg_type == ROBOT_MSG_CMD_REBOOT) {
    uint8_t mode = (len > 0U) ? payload[0] : 0U;
    /* mode: 0=normal, 1=bootloader */
    g_reboot_request = (mode == 1U) ? 1U : 0U; /* 1 = bootloader, 0 = normal */
  } else {
    APP_LOG_INFO("CMD handler invoked, msg_type=0x%02x", msg_type);
  }
}
