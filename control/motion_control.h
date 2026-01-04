#ifndef CONTROL_MOTION_CONTROL_H
#define CONTROL_MOTION_CONTROL_H

#include <stdint.h>

#include "motion_modes.h"

#ifdef __cplusplus
extern "C" {
#endif

void motion_control_init(void);
void motion_control_apply_params(void);
void motion_control_tick(uint32_t now_ms);
void motion_control_set_mode(motion_mode_t mode);
motion_mode_t motion_control_get_mode(void);
void motion_control_set_output_enabled(bool enabled);
void motion_control_set_teleop(float forward_cmd, float turn_cmd);
bool motion_control_can_arm(void);
bool motion_control_get_yaw_debug(float *gyro_z, float *yaw_rate_enc, float *yaw_rate);

typedef struct
{
    uint8_t valid;
    uint8_t active_sensor;
    float gyro_diff_dps;
    float gyro_pitch_diff_dps;
    float acc_angle_diff_deg;
    float vib_rms_g;
    uint8_t gate_accel;
} motion_control_imu_health_t;

bool motion_control_get_imu_health(motion_control_imu_health_t *out);

typedef struct
{
    uint8_t valid;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_norm_g;
    float theta;
    float theta_acc;
    float gyro_rate;
    uint32_t imu_primary_dt_ms;
    uint32_t imu_secondary_dt_ms;
    uint8_t gate;
} motion_control_ekf_log_t;

bool motion_control_get_ekf_log(motion_control_ekf_log_t *out);

/**
 * @brief State estimate from EKF (for telemetry)
 */
typedef struct
{
    float theta_rad;
    float theta_dot;
    float x_m;
    float x_dot_mps;
    float gyro_bias;
    uint8_t valid;
} motion_control_estimate_t;

bool motion_control_get_estimate(motion_control_estimate_t *out);

/**
 * @brief Control output (for telemetry)
 */
typedef struct
{
    float iq_left;
    float iq_right;
    float pitch_target_rad;
} motion_control_output_t;

bool motion_control_get_control_output(motion_control_output_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_MOTION_CONTROL_H */
