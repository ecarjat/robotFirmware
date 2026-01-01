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

bool motor_link_init(void);
void motor_link_poll(void);
void motor_link_enable(bool on);
void motor_link_set_control_mode(motor_control_mode_t mode);
motor_control_mode_t motor_link_get_control_mode(void);

void motor_link_set_wheel_Iq(float left_A, float right_A, float max_A);
void motor_link_set_wheel_torques(float left_Nm, float right_Nm, float max_Nm);

bool motor_link_get_wheel_velocities(float *left_rad_s, float *right_rad_s);

uint32_t motor_link_get_left_parser_drops(void);
uint32_t motor_link_get_right_parser_drops(void);
uint32_t motor_link_get_left_telem_stale(void);
uint32_t motor_link_get_right_telem_stale(void);
uint32_t motor_link_get_left_telem_late(void);
uint32_t motor_link_get_right_telem_late(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_LINK_H */
