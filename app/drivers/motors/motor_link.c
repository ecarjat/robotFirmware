#include "motor_backend.h"

#include <stddef.h>

static const motor_backend_ops_t *s_ops = NULL;

static const motor_backend_ops_t *motor_link_ops(void) {
  if (s_ops == NULL) {
    s_ops = motor_backend_get();
  }
  return s_ops;
}

bool motor_link_init(void) {
  s_ops = motor_backend_get();
  if (s_ops == NULL || s_ops->init == NULL) {
    return false;
  }
  return s_ops->init();
}

void motor_link_poll(void) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->poll != NULL) {
    ops->poll();
  }
}

bool motor_link_enable(bool on) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops == NULL || ops->enable == NULL) {
    return false;
  }
  return ops->enable(on);
}

void motor_link_set_control_mode(motor_control_mode_t mode) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->set_control_mode != NULL) {
    ops->set_control_mode(mode);
  }
}

motor_control_mode_t motor_link_get_control_mode(void) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->get_control_mode != NULL) {
    return ops->get_control_mode();
  }
  return MOTOR_CONTROL_TORQUE;
}

void motor_link_set_motor_directions(int8_t left_dir, int8_t right_dir) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->set_motor_directions != NULL) {
    ops->set_motor_directions(left_dir, right_dir);
  }
}

void motor_link_set_wheel_Iq(float left_A, float right_A, float max_A) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->set_wheel_Iq != NULL) {
    ops->set_wheel_Iq(left_A, right_A, max_A);
  }
}

void motor_link_set_wheel_torques(float left_Nm, float right_Nm, float max_Nm) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->set_wheel_torques != NULL) {
    ops->set_wheel_torques(left_Nm, right_Nm, max_Nm);
  }
}

bool motor_link_torque_to_iq(float torque_nm, float *iq_out) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->torque_to_iq != NULL) {
    return ops->torque_to_iq(torque_nm, iq_out);
  }
  return false;
}

bool motor_link_get_wheel_velocities(float *left_rad_s, float *right_rad_s) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->get_wheel_velocities != NULL) {
    return ops->get_wheel_velocities(left_rad_s, right_rad_s);
  }
  return false;
}

bool motor_link_send_command(motor_side_t side, uint8_t cmd_id) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->send_command != NULL) {
    return ops->send_command(side, cmd_id);
  }
  return false;
}

bool motor_link_send_write(motor_side_t side) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->send_write != NULL) {
    return ops->send_write(side);
  }
  return false;
}

bool motor_link_send_calibrate(motor_side_t side) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->send_calibrate != NULL) {
    return ops->send_calibrate(side);
  }
  return false;
}

bool motor_link_send_bootloader(motor_side_t side) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->send_bootloader != NULL) {
    return ops->send_bootloader(side);
  }
  return false;
}

motor_cmd_result_t motor_link_wait_command(motor_side_t side, uint8_t cmd_id,
                                           uint32_t timeout_ms) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->wait_command != NULL) {
    return ops->wait_command(side, cmd_id, timeout_ms);
  }
  return MOTOR_CMD_ERROR;
}

uint32_t motor_link_get_left_parser_drops(void) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->get_left_parser_drops != NULL) {
    return ops->get_left_parser_drops();
  }
  return 0U;
}

uint32_t motor_link_get_right_parser_drops(void) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->get_right_parser_drops != NULL) {
    return ops->get_right_parser_drops();
  }
  return 0U;
}

uint32_t motor_link_get_left_sync_losses(void) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->get_left_sync_losses != NULL) {
    return ops->get_left_sync_losses();
  }
  return 0U;
}

uint32_t motor_link_get_right_sync_losses(void) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->get_right_sync_losses != NULL) {
    return ops->get_right_sync_losses();
  }
  return 0U;
}

uint32_t motor_link_get_left_crc_errors(void) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->get_left_crc_errors != NULL) {
    return ops->get_left_crc_errors();
  }
  return 0U;
}

uint32_t motor_link_get_right_crc_errors(void) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->get_right_crc_errors != NULL) {
    return ops->get_right_crc_errors();
  }
  return 0U;
}

uint32_t motor_link_get_left_telem_stale(void) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->get_left_telem_stale != NULL) {
    return ops->get_left_telem_stale();
  }
  return 0U;
}

uint32_t motor_link_get_right_telem_stale(void) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->get_right_telem_stale != NULL) {
    return ops->get_right_telem_stale();
  }
  return 0U;
}

uint32_t motor_link_get_left_telem_late(void) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->get_left_telem_late != NULL) {
    return ops->get_left_telem_late();
  }
  return 0U;
}

uint32_t motor_link_get_right_telem_late(void) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->get_right_telem_late != NULL) {
    return ops->get_right_telem_late();
  }
  return 0U;
}

uint32_t motor_link_get_left_ack_timeouts(void) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->get_left_ack_timeouts != NULL) {
    return ops->get_left_ack_timeouts();
  }
  return 0U;
}

uint32_t motor_link_get_right_ack_timeouts(void) {
  const motor_backend_ops_t *ops = motor_link_ops();
  if (ops != NULL && ops->get_right_ack_timeouts != NULL) {
    return ops->get_right_ack_timeouts();
  }
  return 0U;
}
