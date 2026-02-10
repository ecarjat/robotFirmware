#include "motors/motor_link.h"

#include <stddef.h>

static bool s_motor_link_ok = true;
static float s_left_vel = 0.0f;
static float s_right_vel = 0.0f;
static bool s_motor_enabled = true;
static float s_last_iq_left = 0.0f;
static float s_last_iq_right = 0.0f;
static float s_last_iq_max = 0.0f;
static float s_last_torque_left = 0.0f;
static float s_last_torque_right = 0.0f;
static float s_last_torque_max = 0.0f;
static float s_motor_kt = 1.0f;

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
    s_last_iq_left = left_A;
    s_last_iq_right = right_A;
    s_last_iq_max = max_A;
}

void motor_link_set_wheel_torques(float left_Nm, float right_Nm, float max_Nm)
{
    s_last_torque_left = left_Nm;
    s_last_torque_right = right_Nm;
    s_last_torque_max = max_Nm;
}

bool motor_link_torque_to_iq(float torque_nm, float *iq_out)
{
    if (iq_out == NULL || s_motor_kt <= 0.0f) {
        return false;
    }
    *iq_out = torque_nm / s_motor_kt;
    return true;
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

uint32_t motor_link_get_left_ack_timeouts(void)
{
    return 0U;
}

uint32_t motor_link_get_right_ack_timeouts(void)
{
    return 0U;
}

/* Test helpers */
void motor_link_fake_set_ok(bool ok) { s_motor_link_ok = ok; }
void motor_link_fake_set_velocities(float left_rad_s, float right_rad_s)
{
    s_left_vel = left_rad_s;
    s_right_vel = right_rad_s;
}
bool motor_link_fake_get_enabled(void) { return s_motor_enabled; }
void motor_link_fake_get_last_iq(float *left_A, float *right_A, float *max_A)
{
    if (left_A) {
        *left_A = s_last_iq_left;
    }
    if (right_A) {
        *right_A = s_last_iq_right;
    }
    if (max_A) {
        *max_A = s_last_iq_max;
    }
}

void motor_link_fake_get_last_torque(float *left_nm, float *right_nm, float *max_nm)
{
    if (left_nm) {
        *left_nm = s_last_torque_left;
    }
    if (right_nm) {
        *right_nm = s_last_torque_right;
    }
    if (max_nm) {
        *max_nm = s_last_torque_max;
    }
}

void motor_link_fake_set_kt(float kt)
{
    s_motor_kt = kt;
}
