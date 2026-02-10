#include "motion_control.h"

static motion_control_estimate_t s_estimate = {0};
static motion_control_imu_health_t s_imu_health = {0};
static motion_control_output_t s_output = {0};
static motion_mode_t s_mode = MOTION_MODE_DISARMED;
static uint32_t s_last_imu_ok = 0U;
static uint32_t s_last_motor_ok = 0U;
static uint8_t s_hip_behavior_mode = 0U;
static uint8_t s_hip_phase_progress = 0U;

void motion_control_init(void) {}
void motion_control_apply_params(void) {}
void motion_control_tick(uint32_t now_ms) { (void)now_ms; }
void motion_control_set_mode(motion_mode_t mode) { s_mode = mode; }
motion_mode_t motion_control_get_mode(void) { return s_mode; }
void motion_control_set_output_enabled(bool enabled) { (void)enabled; }
void motion_control_set_teleop(float forward_cmd, float turn_cmd)
{
    (void)forward_cmd;
    (void)turn_cmd;
}
bool motion_control_can_arm(void) { return true; }
bool motion_control_is_calibrated(void) { return true; }
uint32_t motion_control_get_last_imu_ok_ms(void) { return s_last_imu_ok; }
uint32_t motion_control_get_last_motor_ok_ms(void) { return s_last_motor_ok; }
bool motion_control_get_yaw_debug(float *gyro_z, float *yaw_rate_enc, float *yaw_rate)
{
    if (gyro_z) *gyro_z = 0.0f;
    if (yaw_rate_enc) *yaw_rate_enc = 0.0f;
    if (yaw_rate) *yaw_rate = 0.0f;
    return true;
}
bool motion_control_get_imu_health(motion_control_imu_health_t *out)
{
    if (!out) return false;
    *out = s_imu_health;
    out->valid = 1U;
    return true;
}
bool motion_control_get_ekf_log(motion_control_ekf_log_t *out)
{
    (void)out;
    return false;
}
bool motion_control_get_estimate(motion_control_estimate_t *out)
{
    if (!out) return false;
    *out = s_estimate;
    out->valid = 1U;
    return true;
}
bool motion_control_get_control_output(motion_control_output_t *out)
{
    if (!out) return false;
    *out = s_output;
    return true;
}
bool motion_control_is_saturated(void) { return false; }
void motion_control_set_inner_mode(uint8_t mode) { (void)mode; }
uint8_t motion_control_get_inner_mode(void) { return 0U; }
uint8_t motion_control_get_hip_behavior_mode(void) { return s_hip_behavior_mode; }
uint8_t motion_control_get_hip_phase_progress(void) { return s_hip_phase_progress; }

void motion_control_fake_set_estimate(const motion_control_estimate_t *est)
{
    if (est) s_estimate = *est;
}
void motion_control_fake_set_output(const motion_control_output_t *out)
{
    if (out) s_output = *out;
}
void motion_control_fake_set_imu_health(const motion_control_imu_health_t *health)
{
    if (health) s_imu_health = *health;
}
void motion_control_fake_set_last_ok(uint32_t imu_ms, uint32_t motor_ms)
{
    s_last_imu_ok = imu_ms;
    s_last_motor_ok = motor_ms;
}
void motion_control_fake_set_hip_behavior_mode(uint8_t mode)
{
    s_hip_behavior_mode = mode;
}
void motion_control_fake_set_hip_phase_progress(uint8_t pct)
{
    s_hip_phase_progress = pct;
}
