#include "motor_backend.h"

#define motor_link_init motor_backend_robust_uart_init
#define motor_link_poll motor_backend_robust_uart_poll
#define motor_link_enable motor_backend_robust_uart_enable
#define motor_link_set_control_mode motor_backend_robust_uart_set_control_mode
#define motor_link_get_control_mode motor_backend_robust_uart_get_control_mode
#define motor_link_set_motor_directions                                         \
  motor_backend_robust_uart_set_motor_directions
#define motor_link_set_wheel_Iq motor_backend_robust_uart_set_wheel_Iq
#define motor_link_set_wheel_torques motor_backend_robust_uart_set_wheel_torques
#define motor_link_get_wheel_velocities                                         \
  motor_backend_robust_uart_get_wheel_velocities
#define motor_link_send_command motor_backend_robust_uart_send_command
#define motor_link_send_write motor_backend_robust_uart_send_write
#define motor_link_send_calibrate motor_backend_robust_uart_send_calibrate
#define motor_link_send_bootloader motor_backend_robust_uart_send_bootloader
#define motor_link_wait_command motor_backend_robust_uart_wait_command
#define motor_link_get_left_parser_drops                                        \
  motor_backend_robust_uart_get_left_parser_drops
#define motor_link_get_right_parser_drops                                       \
  motor_backend_robust_uart_get_right_parser_drops
#define motor_link_get_left_sync_losses                                         \
  motor_backend_robust_uart_get_left_sync_losses
#define motor_link_get_right_sync_losses                                        \
  motor_backend_robust_uart_get_right_sync_losses
#define motor_link_get_left_crc_errors                                          \
  motor_backend_robust_uart_get_left_crc_errors
#define motor_link_get_right_crc_errors                                         \
  motor_backend_robust_uart_get_right_crc_errors
#define motor_link_get_left_telem_stale                                         \
  motor_backend_robust_uart_get_left_telem_stale
#define motor_link_get_right_telem_stale                                        \
  motor_backend_robust_uart_get_right_telem_stale
#define motor_link_get_left_telem_late                                          \
  motor_backend_robust_uart_get_left_telem_late
#define motor_link_get_right_telem_late                                         \
  motor_backend_robust_uart_get_right_telem_late
#define motor_link_get_left_ack_timeouts                                       \
  motor_backend_robust_uart_get_left_ack_timeouts
#define motor_link_get_right_ack_timeouts                                      \
  motor_backend_robust_uart_get_right_ack_timeouts

#include "motor_backend_robust_uart_impl.inc"

const motor_backend_ops_t motor_backend_robust_uart_ops = {
    .init = motor_backend_robust_uart_init,
    .poll = motor_backend_robust_uart_poll,
    .enable = motor_backend_robust_uart_enable,
    .set_control_mode = motor_backend_robust_uart_set_control_mode,
    .get_control_mode = motor_backend_robust_uart_get_control_mode,
    .set_motor_directions = motor_backend_robust_uart_set_motor_directions,
    .set_wheel_Iq = motor_backend_robust_uart_set_wheel_Iq,
    .set_wheel_torques = motor_backend_robust_uart_set_wheel_torques,
    .get_wheel_velocities = motor_backend_robust_uart_get_wheel_velocities,
    .send_command = motor_backend_robust_uart_send_command,
    .send_write = motor_backend_robust_uart_send_write,
    .send_calibrate = motor_backend_robust_uart_send_calibrate,
    .send_bootloader = motor_backend_robust_uart_send_bootloader,
    .wait_command = motor_backend_robust_uart_wait_command,
    .get_left_parser_drops = motor_backend_robust_uart_get_left_parser_drops,
    .get_right_parser_drops = motor_backend_robust_uart_get_right_parser_drops,
    .get_left_sync_losses = motor_backend_robust_uart_get_left_sync_losses,
    .get_right_sync_losses = motor_backend_robust_uart_get_right_sync_losses,
    .get_left_crc_errors = motor_backend_robust_uart_get_left_crc_errors,
    .get_right_crc_errors = motor_backend_robust_uart_get_right_crc_errors,
    .get_left_telem_stale = motor_backend_robust_uart_get_left_telem_stale,
    .get_right_telem_stale = motor_backend_robust_uart_get_right_telem_stale,
    .get_left_telem_late = motor_backend_robust_uart_get_left_telem_late,
    .get_right_telem_late = motor_backend_robust_uart_get_right_telem_late,
    .get_left_ack_timeouts = motor_backend_robust_uart_get_left_ack_timeouts,
    .get_right_ack_timeouts = motor_backend_robust_uart_get_right_ack_timeouts,
};
