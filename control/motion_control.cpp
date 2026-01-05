#include "motion_control.h"

#include "MotionController.h"
#include "StateEstimator.h"
#include "config_control.h"
#include "types.h"

#include <math.h>
#include <string.h>

#include "app_main.h"
#include "motor_link.h"
#include "sensors.h"
#include "stm32h7xx_hal.h"
#if SENSOR_ENABLE_BMI270
#include "imu_bmi270.h"
#endif
#if SENSOR_ENABLE_ICM42688
#include "imu_icm42688.h"
#endif

static RobotParams s_params;
static MotionController s_controller(s_params);
static StateEstimator s_estimator;

namespace {
constexpr float kGravity = 9.80665f;
constexpr float kDegToRad = 0.01745329252f;
constexpr float kBmiAccelRangeG = 4.0f;
constexpr float kBmiGyroRangeDps = 500.0f;
constexpr float kIcmAccelRangeG = 4.0f;
constexpr float kIcmGyroRangeDps = 500.0f;
constexpr float kBmiAccelScale = (kBmiAccelRangeG * kGravity) / 32768.0f;
constexpr float kBmiGyroScale = (kBmiGyroRangeDps * kDegToRad) / 32768.0f;
constexpr float kIcmAccelScale = (kIcmAccelRangeG * kGravity) / 32768.0f;
constexpr float kIcmGyroScale = (kIcmGyroRangeDps * kDegToRad) / 32768.0f;
}

static uint32_t s_last_tick_ms = 0U;
static float s_max_turn_rate = PARAM_MAX_TURN_RATE;
static uint32_t s_last_imu_ok_ms = 0U;
static uint32_t s_last_motor_ok_ms = 0U;

/* Last control output for telemetry */
static float s_last_iq_left = 0.0f;
static float s_last_iq_right = 0.0f;
static bool s_output_enabled = true;

#if SENSOR_ENABLE_BMI270
static imu_bmi270_sample_t s_bmi_latest;
static uint32_t s_bmi_seq = 0U;
static bool s_bmi_have = false;
#endif

#if SENSOR_ENABLE_ICM42688
static imu_icm42688_sample_t s_icm_latest;
static uint32_t s_icm_seq = 0U;
static bool s_icm_have = false;
#endif

static void motion_control_update_params(void)
{
    s_params = RobotParams();
    s_params.wheelRadius = g_robot_params.wheel_radius_m;
    s_params.wheelBase = g_robot_params.wheel_base_m;
    s_params.maxForwardVelocity = g_robot_params.max_linear_vel_mps;
    s_controller.setRobotParams(s_params);
    s_controller.setBalanceGains(g_robot_params.balance);
    s_max_turn_rate = g_robot_params.max_angular_vel_rps;
}

void motion_control_apply_params(void)
{
    motion_control_update_params();
    s_estimator.setImuRotations(g_robot_params.imu_bmi270.rotation,
                                g_robot_params.imu_icm42688.rotation);
}

static bool motion_control_build_primary(ImuReading &out, uint32_t now_ms)
{
    (void)now_ms;
#if SENSOR_ENABLE_BMI270
    imu_bmi270_sample_t sample;
    if (imu_bmi270_try_get_latest(&sample, &s_bmi_seq))
    {
        s_bmi_latest = sample;
        s_bmi_have = true;
    }
    if (!s_bmi_have)
    {
        out.valid = false;
        return false;
    }

    sample = s_bmi_latest;
    out.timestamp_ms = sample.timestamp_ms;
    out.accel_x = (float)sample.accel[0] * kBmiAccelScale -
                  (float)g_robot_params.imu_bmi270.accel_bias[0] * (kGravity / 1000.0f);
    out.accel_y = (float)sample.accel[1] * kBmiAccelScale -
                  (float)g_robot_params.imu_bmi270.accel_bias[1] * (kGravity / 1000.0f);
    out.accel_z = (float)sample.accel[2] * kBmiAccelScale -
                  (float)g_robot_params.imu_bmi270.accel_bias[2] * (kGravity / 1000.0f);

    out.gyro_x = (float)sample.gyro[0] * kBmiGyroScale -
                 (float)g_robot_params.imu_bmi270.gyro_bias[0] * (kDegToRad / 1000.0f);
    out.gyro_y = (float)sample.gyro[1] * kBmiGyroScale -
                 (float)g_robot_params.imu_bmi270.gyro_bias[1] * (kDegToRad / 1000.0f);
    out.gyro_z = (float)sample.gyro[2] * kBmiGyroScale -
                 (float)g_robot_params.imu_bmi270.gyro_bias[2] * (kDegToRad / 1000.0f);

    out.pitch_rad = 0.0f;
    out.roll_rad = 0.0f;
    out.yaw_rad = 0.0f;

    uint32_t age_ms = now_ms - sample.timestamp_ms;
    out.valid = (age_ms <= IMU_SAMPLE_MAX_AGE_MS);
    return out.valid;
#else
    (void)out;
    return false;
#endif
}

static bool motion_control_build_secondary(ImuReading &out, uint32_t now_ms)
{
    (void)now_ms;
#if SENSOR_ENABLE_ICM42688
    imu_icm42688_sample_t sample;
    if (imu_icm42688_try_get_latest(&sample, &s_icm_seq))
    {
        s_icm_latest = sample;
        s_icm_have = true;
    }
    if (!s_icm_have)
    {
        out.valid = false;
        return false;
    }

    sample = s_icm_latest;
    out.timestamp_ms = sample.timestamp_ms;
    out.accel_x = (float)sample.accel[0] * kIcmAccelScale -
                  (float)g_robot_params.imu_icm42688.accel_bias[0] * (kGravity / 1000.0f);
    out.accel_y = (float)sample.accel[1] * kIcmAccelScale -
                  (float)g_robot_params.imu_icm42688.accel_bias[1] * (kGravity / 1000.0f);
    out.accel_z = (float)sample.accel[2] * kIcmAccelScale -
                  (float)g_robot_params.imu_icm42688.accel_bias[2] * (kGravity / 1000.0f);

    out.gyro_x = (float)sample.gyro[0] * kIcmGyroScale -
                 (float)g_robot_params.imu_icm42688.gyro_bias[0] * (kDegToRad / 1000.0f);
    out.gyro_y = (float)sample.gyro[1] * kIcmGyroScale -
                 (float)g_robot_params.imu_icm42688.gyro_bias[1] * (kDegToRad / 1000.0f);
    out.gyro_z = (float)sample.gyro[2] * kIcmGyroScale -
                 (float)g_robot_params.imu_icm42688.gyro_bias[2] * (kDegToRad / 1000.0f);

    out.pitch_rad = 0.0f;
    out.roll_rad = 0.0f;
    out.yaw_rad = 0.0f;

    uint32_t age_ms = now_ms - sample.timestamp_ms;
    out.valid = (age_ms <= IMU_SAMPLE_MAX_AGE_MS);
    return out.valid;
#else
    (void)out;
    return false;
#endif
}

void motion_control_init(void)
{
    motion_control_update_params();
    s_controller.resetPidState();
    s_estimator.begin(s_params);
    s_estimator.setImuRotations(g_robot_params.imu_bmi270.rotation,
                                g_robot_params.imu_icm42688.rotation);
    motion_modes_init();
    s_last_tick_ms = 0U;
    s_last_imu_ok_ms = 0U;
    s_last_motor_ok_ms = 0U;
#if SENSOR_ENABLE_BMI270
    s_bmi_have = false;
    s_bmi_seq = 0U;
#endif
#if SENSOR_ENABLE_ICM42688
    s_icm_have = false;
    s_icm_seq = 0U;
#endif
}

void motion_control_set_mode(motion_mode_t mode)
{
    motion_modes_set(mode);
    // Reset PID state on any mode that shouldn't carry over integral terms.
    // This includes entering BALANCING to prevent windup from previous sessions.
    if (mode == MOTION_MODE_DISARMED || mode == MOTION_MODE_FAULT ||
        mode == MOTION_MODE_FALLEN || mode == MOTION_MODE_BALANCING)
    {
        s_controller.resetPidState();
    }
}

motion_mode_t motion_control_get_mode(void)
{
    return motion_modes_get();
}

void motion_control_set_output_enabled(bool enabled)
{
    s_output_enabled = enabled;
}

uint32_t motion_control_get_last_imu_ok_ms(void)
{
    return s_last_imu_ok_ms;
}

uint32_t motion_control_get_last_motor_ok_ms(void)
{
    return s_last_motor_ok_ms;
}

/**
 * @brief Check if a single IMU calibration has been performed
 *
 * Calibration is considered valid if at least one gyro or accel bias value
 * is non-zero. A fresh/uncalibrated sensor has all biases = 0.
 */
static bool imu_calib_is_valid(const imu_calib_t *calib)
{
    /* Check if any gyro bias is non-zero (indicates calibration was done) */
    if (calib->gyro_bias[0] != 0 ||
        calib->gyro_bias[1] != 0 ||
        calib->gyro_bias[2] != 0)
    {
        return true;
    }

    /* Also accept if accel bias is set (partial calibration) */
    if (calib->accel_bias[0] != 0 ||
        calib->accel_bias[1] != 0 ||
        calib->accel_bias[2] != 0)
    {
        return true;
    }

    return false;
}

/**
 * @brief Check if all enabled IMUs have been calibrated
 *
 * Per MainControl.md Section 8.4:
 * "Robot cannot enter BALANCING mode until valid calibration exists in params."
 *
 * Both BMI270 and ICM42688 (if enabled) must have valid calibration.
 */
static bool motion_control_is_calibrated(void)
{
    /* BMI270 is always required */
    if (!imu_calib_is_valid(&g_robot_params.imu_bmi270))
    {
        return false;
    }

#if SENSOR_ENABLE_ICM42688
    /* ICM42688 calibration also required when enabled */
    if (!imu_calib_is_valid(&g_robot_params.imu_icm42688))
    {
        return false;
    }
#endif

    return true;
}

bool motion_control_can_arm(void)
{
    motion_mode_t mode = motion_modes_get();
    if (mode != MOTION_MODE_DISARMED && mode != MOTION_MODE_FALLEN)
    {
        return false;
    }

    /* Require valid calibration before allowing balance */
    if (!motion_control_is_calibrated())
    {
        return false;
    }

    StateEstimate estimate = s_estimator.getEstimate();
    if (!estimate.valid)
    {
        return false;
    }
    if (fabsf(estimate.theta) > MOTION_ARM_UPRIGHT_RAD)
    {
        return false;
    }
    motion_control_imu_health_t health;
    if (!motion_control_get_imu_health(&health) || !health.valid)
    {
        return false;
    }
    float left_w = 0.0f;
    float right_w = 0.0f;
    if (!motor_link_get_wheel_velocities(&left_w, &right_w))
    {
        return false;
    }
    return true;
}

void motion_control_set_teleop(float forward_cmd, float turn_cmd)
{
    float forward_norm = 0.0f;
    if (fabsf(s_params.maxForwardVelocity) > 1e-6f)
    {
        forward_norm = forward_cmd / s_params.maxForwardVelocity;
    }
    float turn_norm = 0.0f;
    if (fabsf(s_max_turn_rate) > 1e-6f)
    {
        turn_norm = turn_cmd / s_max_turn_rate;
    }
    s_controller.setTeleopCommands(forward_norm, turn_norm);
}

void motion_control_tick(uint32_t now_ms)
{
    float dt_s = CONTROL_DT;
    if (s_last_tick_ms != 0U)
    {
        uint32_t delta_ms = now_ms - s_last_tick_ms;
        if (delta_ms > 0U)
        {
            dt_s = 0.001f * (float)delta_ms;
        }
    }
    s_last_tick_ms = now_ms;

    ImuReading primary{};
    ImuReading secondary{};
    primary.valid = false;
    secondary.valid = false;
    (void)motion_control_build_primary(primary, now_ms);
    (void)motion_control_build_secondary(secondary, now_ms);

    float left_w = 0.0f;
    float right_w = 0.0f;
    bool wheel_ok = motor_link_get_wheel_velocities(&left_w, &right_w);
    if (wheel_ok)
    {
        s_last_motor_ok_ms = now_ms;
    }

    float v_enc = 0.0f;
    float yaw_rate_enc = 0.0f;
    if (wheel_ok)
    {
        v_enc = 0.5f * (left_w + right_w) * s_params.wheelRadius;
        if (fabsf(s_params.wheelBase) > 1e-6f)
        {
            yaw_rate_enc = (right_w - left_w) * s_params.wheelRadius / s_params.wheelBase;
        }
    }

    bool imu_ok = primary.valid || secondary.valid;
    if (imu_ok)
    {
        s_last_imu_ok_ms = now_ms;
    }

    s_estimator.update(primary, secondary, now_ms, v_enc);

    float gyro_z = s_estimator.getLastGyroZ();
    s_controller.setYawRates(gyro_z, yaw_rate_enc);

    StateEstimate estimate = s_estimator.getEstimate();

    /* Populate state machine input and evaluate transitions */
    motion_modes_input_t modes_input = {
        .now_ms = now_ms,
        .estimate_valid = estimate.valid,
        .theta_rad = estimate.theta,
        .theta_kill_rad = g_robot_params.balance.thetaKill,
        .imu_ok = imu_ok,
        .last_imu_ok_ms = s_last_imu_ok_ms,
        .motor_ok = wheel_ok,
        .last_motor_ok_ms = s_last_motor_ok_ms,
    };

    motion_modes_output_t modes_output;
    motion_modes_step(&modes_input, &modes_output);

    /* Handle state machine outputs */
    if (modes_output.disable_motors)
    {
        motor_link_enable(false);
    }
    if (modes_output.reset_pid)
    {
        s_controller.resetPidState();
    }

    if (motion_modes_allows_output())
    {
        estimate.xDot = v_enc;
        if (estimate.valid)
        {
            MotionController::Command cmd = s_controller.computeControl(estimate, dt_s);
            s_last_iq_left = cmd.iq.iqLeft;
            s_last_iq_right = cmd.iq.iqRight;
            if (s_output_enabled)
            {
                motor_link_set_wheel_Iq(cmd.iq.iqLeft, cmd.iq.iqRight,
                                        g_robot_params.balance.IqMax);
            }
        }
        else
        {
            s_last_iq_left = 0.0f;
            s_last_iq_right = 0.0f;
            if (s_output_enabled)
            {
                motor_link_set_wheel_Iq(0.0f, 0.0f, 0.0f);
            }
        }
    }
    else
    {
        s_last_iq_left = 0.0f;
        s_last_iq_right = 0.0f;
        if (s_output_enabled)
        {
            motor_link_set_wheel_Iq(0.0f, 0.0f, 0.0f);
        }
    }
}

bool motion_control_get_yaw_debug(float *gyro_z, float *yaw_rate_enc, float *yaw_rate)
{
    if (gyro_z == NULL || yaw_rate_enc == NULL || yaw_rate == NULL)
    {
        return false;
    }
    *gyro_z = s_controller.getLastGyroZ();
    *yaw_rate_enc = s_controller.getLastYawRateEnc();
    *yaw_rate = s_controller.getLastYawRate();
    return true;
}

bool motion_control_get_imu_health(motion_control_imu_health_t *out)
{
    if (out == NULL)
    {
        return false;
    }
    ImuHealthMetrics metrics{};
    if (!s_estimator.getImuHealthMetrics(metrics))
    {
        return false;
    }
    out->valid = metrics.valid ? 1U : 0U;
    out->active_sensor = metrics.active_sensor;
    out->gyro_diff_dps = metrics.gyro_diff_dps;
    out->gyro_pitch_diff_dps = metrics.gyro_pitch_diff_dps;
    out->acc_angle_diff_deg = metrics.acc_angle_diff_deg;
    out->vib_rms_g = metrics.vib_rms_g;
    out->gate_accel = metrics.gate_accel;
    return true;
}

bool motion_control_get_ekf_log(motion_control_ekf_log_t *out)
{
    if (out == NULL)
    {
        return false;
    }
    EkfLogData log{};
    if (!s_estimator.getEkfLogData(log))
    {
        return false;
    }
    out->valid = log.valid ? 1U : 0U;
    out->accel_x = log.accel_x;
    out->accel_y = log.accel_y;
    out->accel_z = log.accel_z;
    out->gyro_x = log.gyro_x;
    out->gyro_y = log.gyro_y;
    out->gyro_z = log.gyro_z;
    out->accel_norm_g = log.accel_norm_g;
    out->theta = log.theta;
    out->theta_acc = log.theta_acc;
    out->gyro_rate = log.gyro_rate;
    out->imu_primary_dt_ms = log.imu_primary_dt_ms;
    out->imu_secondary_dt_ms = log.imu_secondary_dt_ms;
    out->gate = log.gate;
    return true;
}

bool motion_control_get_estimate(motion_control_estimate_t *out)
{
    if (out == NULL)
    {
        return false;
    }
    StateEstimate est = s_estimator.getEstimate();
    out->theta_rad = est.theta;
    out->theta_dot = est.thetaDot;
    out->x_m = est.x;
    out->x_dot_mps = est.xDot;
    out->gyro_bias = est.gyroBias;
    out->valid = est.valid ? 1U : 0U;
    return true;
}

bool motion_control_get_control_output(motion_control_output_t *out)
{
    if (out == NULL)
    {
        return false;
    }
    out->iq_left = s_last_iq_left;
    out->iq_right = s_last_iq_right;
    out->pitch_target_rad = s_controller.getLastPitchTarget();
    return true;
}

bool motion_control_is_saturated(void)
{
    return s_controller.isOutputSaturated();
}
