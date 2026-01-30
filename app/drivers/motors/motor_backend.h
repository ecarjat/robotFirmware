#ifndef MOTOR_BACKEND_H
#define MOTOR_BACKEND_H

#include "motor_link.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool (*init)(void);
  void (*poll)(void);
  bool (*enable)(bool on);
  void (*set_control_mode)(motor_control_mode_t mode);
  motor_control_mode_t (*get_control_mode)(void);
  void (*set_motor_directions)(int8_t left_dir, int8_t right_dir);
  void (*set_wheel_Iq)(float left_A, float right_A, float max_A);
  void (*set_wheel_torques)(float left_Nm, float right_Nm, float max_Nm);
  bool (*get_wheel_velocities)(float *left_rad_s, float *right_rad_s);
  bool (*send_command)(motor_side_t side, uint8_t cmd_id);
  bool (*send_write)(motor_side_t side);
  bool (*send_calibrate)(motor_side_t side);
  bool (*send_bootloader)(motor_side_t side);
  motor_cmd_result_t (*wait_command)(motor_side_t side, uint8_t cmd_id,
                                     uint32_t timeout_ms);
  uint32_t (*get_left_parser_drops)(void);
  uint32_t (*get_right_parser_drops)(void);
  uint32_t (*get_left_sync_losses)(void);
  uint32_t (*get_right_sync_losses)(void);
  uint32_t (*get_left_crc_errors)(void);
  uint32_t (*get_right_crc_errors)(void);
  uint32_t (*get_left_telem_stale)(void);
  uint32_t (*get_right_telem_stale)(void);
  uint32_t (*get_left_telem_late)(void);
  uint32_t (*get_right_telem_late)(void);
  uint32_t (*get_left_ack_timeouts)(void);
  uint32_t (*get_right_ack_timeouts)(void);
} motor_backend_ops_t;

const motor_backend_ops_t *motor_backend_get(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_BACKEND_H */
