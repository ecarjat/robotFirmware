#ifndef MOTOR_LINK_H
#define MOTOR_LINK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MOTOR_SIDE_LEFT = 0,
    MOTOR_SIDE_RIGHT = 1
} motor_side_t;

typedef enum
{
    MOTOR_CONTROL_TORQUE = 0x00,
    MOTOR_CONTROL_VELOCITY = 0x01,
    MOTOR_CONTROL_ANGLE = 0x02,
    MOTOR_CONTROL_VELOCITY_OPENLOOP = 0x03,
    MOTOR_CONTROL_ANGLE_OPENLOOP = 0x04
} motor_control_mode_t;

/* Command result for async command operations */
typedef enum
{
    MOTOR_CMD_PENDING = 0,
    MOTOR_CMD_OK,
    MOTOR_CMD_ERROR,
    MOTOR_CMD_BUSY,
    MOTOR_CMD_TIMEOUT,
    MOTOR_CMD_UNKNOWN
} motor_cmd_result_t;

bool motor_link_init(void);
void motor_link_poll(void);
bool motor_link_enable(bool on);
void motor_link_set_control_mode(motor_control_mode_t mode);
motor_control_mode_t motor_link_get_control_mode(void);
void motor_link_set_motor_directions(int8_t left_dir, int8_t right_dir);

void motor_link_set_wheel_Iq(float left_A, float right_A, float max_A);
void motor_link_set_wheel_torques(float left_Nm, float right_Nm, float max_Nm);
bool motor_link_torque_to_iq(float torque_nm, float *iq_out);

bool motor_link_get_wheel_velocities(float *left_rad_s, float *right_rad_s);

/* Command API (v2 protocol) */
bool motor_link_send_command(motor_side_t side, uint8_t cmd_id);
bool motor_link_send_write(motor_side_t side);
bool motor_link_send_calibrate(motor_side_t side);
bool motor_link_send_bootloader(motor_side_t side);
motor_cmd_result_t motor_link_wait_command(motor_side_t side, uint8_t cmd_id,
                                            uint32_t timeout_ms);

/* Statistics */
uint32_t motor_link_get_left_parser_drops(void);
uint32_t motor_link_get_right_parser_drops(void);
uint32_t motor_link_get_left_sync_losses(void);
uint32_t motor_link_get_right_sync_losses(void);
uint32_t motor_link_get_left_crc_errors(void);
uint32_t motor_link_get_right_crc_errors(void);
uint32_t motor_link_get_left_telem_stale(void);
uint32_t motor_link_get_right_telem_stale(void);
uint32_t motor_link_get_left_telem_late(void);
uint32_t motor_link_get_right_telem_late(void);
uint32_t motor_link_get_left_ack_timeouts(void);
uint32_t motor_link_get_right_ack_timeouts(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_LINK_H */
