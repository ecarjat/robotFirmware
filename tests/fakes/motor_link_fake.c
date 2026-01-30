#include "motor_link.h"

#include <stddef.h>

static bool s_motor_link_ok = true;
static float s_left_vel = 0.0f;
static float s_right_vel = 0.0f;
static bool s_motor_enabled = true;

bool motor_link_init(void) { return true; }
void motor_link_poll(void) {}
bool motor_link_enable(bool on)
{
    s_motor_enabled = on;
    return true;
}
void motor_link_set_control_mode(motor_control_mode_t mode) { (void)mode; }
motor_control_mode_t motor_link_get_control_mode(void) { return MOTOR_CONTROL_TORQUE; }
void motor_link_set_motor_directions(int8_t left_dir, int8_t right_dir)
{
    (void)left_dir;
    (void)right_dir;
}

void motor_link_set_wheel_Iq(float left_A, float right_A, float max_A)
{
    (void)left_A;
    (void)right_A;
    (void)max_A;
}

void motor_link_set_wheel_torques(float left_Nm, float right_Nm, float max_Nm)
{
    (void)left_Nm;
    (void)right_Nm;
    (void)max_Nm;
}

bool motor_link_get_wheel_velocities(float *left_rad_s, float *right_rad_s)
{
    if (!s_motor_link_ok || left_rad_s == NULL || right_rad_s == NULL) {
        return false;
    }
    *left_rad_s = s_left_vel;
    *right_rad_s = s_right_vel;
    return true;
}

/* Test helpers */
void motor_link_fake_set_ok(bool ok) { s_motor_link_ok = ok; }
void motor_link_fake_set_velocities(float left_rad_s, float right_rad_s)
{
    s_left_vel = left_rad_s;
    s_right_vel = right_rad_s;
}
bool motor_link_fake_get_enabled(void) { return s_motor_enabled; }
