#include "motion_control.h"

#include "MotionController.h"
#include "StateEstimator.h"
#include "config_control.h"
#include "types.h"
#include "app_log_macros.h"
#include "hip_control.h"
#include "hip_behavior.h"
#include "hip_kinematics.h"
#include "lqr_lut.h"

#include <math.h>
#include <string.h>

#include "app_main.h"
#include "motor_link.h"
#include "sensors.h"
/* stm32h7xx_hal.h not required for host tests */
#if SENSOR_ENABLE_BMI270
#include "imu_bmi270.h"
#endif
#if SENSOR_ENABLE_ICM42688
#include "imu_icm42688.h"
#endif

extern "C" {
#include "../app/logging/blackbox.h"
#include "../app/logging/blackbox_format.h"
#include "crc32.h"
}

static RobotParams s_params;
static MotionController s_controller(s_params);
static StateEstimator s_estimator;

namespace {
constexpr float kGravity = 9.80665f;
constexpr float kDegToRad = 0.01745329252f;

/**
 * @brief Check if a control output value is safe to send to motors
 * @return true if value is finite (not NaN, not inf), false otherwise
 */
inline bool is_control_value_safe(float value)
{
    return isfinite(value);
}
constexpr float kBmiAccelRangeG = 4.0f;
constexpr float kBmiGyroRangeDps = 500.0f;
constexpr float kIcmAccelRangeG = 4.0f;
constexpr float kIcmGyroRangeDps = 500.0f;
constexpr float kBmiAccelScale = (kBmiAccelRangeG * kGravity) / 32768.0f;
constexpr float kBmiGyroScale = (kBmiGyroRangeDps * kDegToRad) / 32768.0f;
constexpr float kIcmAccelScale = (kIcmAccelRangeG * kGravity) / 32768.0f;
constexpr float kIcmGyroScale = (kIcmGyroRangeDps * kDegToRad) / 32768.0f;
}

#ifdef UNIT_TEST
static bool s_test_estimate_override = false;
static motion_control_estimate_t s_test_estimate = {};
static bool s_test_imu_health_override = false;
static motion_control_imu_health_t s_test_imu_health = {};
#endif

static uint32_t s_last_tick_ms = 0U;
static float s_max_turn_rate = PARAM_MAX_TURN_RATE;
static uint32_t s_last_imu_ok_ms = 0U;
static uint32_t s_last_imu_irq_ms = 0U;
static uint32_t s_last_motor_ok_ms = 0U;
static float s_control_dt = CONTROL_DT;
static uint32_t s_last_hip_coord_ms = 0U;
static uint32_t s_last_lqr_lut_ms = 0U;
static hip_behavior_mode_t s_hip_behavior_mode = HIP_BEHAVIOR_NORMAL;

/* Last control output for telemetry */
static float s_last_torque_left = 0.0f;
static float s_last_torque_right = 0.0f;
static bool s_output_enabled = true;
static bool s_motor_disable_failed = false;

/* Blackbox logging */
static uint32_t s_log_seq = 0U;

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
    s_controller.setLqrParams(g_robot_params.lqr);
    s_max_turn_rate = g_robot_params.max_angular_vel_rps;
    if (g_robot_params.control_rate_hz > 1e-3f) {
        s_control_dt = 1.0f / g_robot_params.control_rate_hz;
    } else {
        s_control_dt = CONTROL_DT;
    }
    s_controller.setControlDt(s_control_dt);
    s_estimator.setControlDt(s_control_dt);
}

void motion_control_apply_params(void)
{
    motion_control_update_params();
    s_estimator.setImuRotations(g_robot_params.imu_bmi270.rotation,
                                g_robot_params.imu_icm42688.rotation);
#if HIP_CONTROL_ENABLE
    hip_control_apply_params(&g_robot_params);
#endif
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
#if HIP_CONTROL_ENABLE
    hip_control_init();
    hip_control_apply_params(&g_robot_params);
    float min_h = 0.0f;
    float max_h = 0.0f;
    float nominal_h = HIP_HEIGHT_DEFAULT_M;
    if (hip_kinematics_get_height_range(&min_h, &max_h)) {
        nominal_h = 0.5f * (min_h + max_h);
    }
    hip_behavior_init(nominal_h);
#endif

    /* Initialize LQR params and set default mode */
    s_controller.setLqrParams(g_robot_params.lqr);
    if (g_robot_params.lqr.default_mode != 0) {
        s_controller.setRequestedMode(InnerLongMode::LQR);
    }

    s_last_tick_ms = 0U;
    s_last_imu_ok_ms = 0U;
    s_last_imu_irq_ms = 0U;
    s_last_motor_ok_ms = 0U;
    s_last_hip_coord_ms = 0U;
    s_hip_behavior_mode = HIP_BEHAVIOR_NORMAL;
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
    // Reset PID/LQR state on any mode that shouldn't carry over integral terms.
    // This includes entering BALANCING to prevent windup from previous sessions.
    if (mode == MOTION_MODE_DISARMED || mode == MOTION_MODE_FAULT ||
        mode == MOTION_MODE_FALLEN || mode == MOTION_MODE_BALANCING)
    {
        s_controller.resetPidState();  /* Also resets LQR state */
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
bool motion_control_is_calibrated(void)
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
        APP_LOG_ERROR("Arm rejected: mode=%u", (unsigned int)mode);
        return false;
    }

    /* Require valid calibration before allowing balance */
    if (!motion_control_is_calibrated())
    {
        APP_LOG_ERROR("Arm rejected: not calibrated");
        return false;
    }

    StateEstimate estimate = s_estimator.getEstimate();
#ifdef UNIT_TEST
    if (s_test_estimate_override)
    {
        estimate.theta = s_test_estimate.theta_rad;
        estimate.thetaDot = s_test_estimate.theta_dot;
        estimate.x = s_test_estimate.x_m;
        estimate.xDot = s_test_estimate.x_dot_mps;
        estimate.gyroBias = s_test_estimate.gyro_bias;
        estimate.valid = s_test_estimate.valid != 0U;
    }
#endif
    if (!estimate.valid)
    {
        APP_LOG_ERROR("Arm rejected: estimate invalid");
        return false;
    }
    if (fabsf(estimate.theta) > MOTION_ARM_UPRIGHT_RAD)
    {
        APP_LOG_ERROR("Arm rejected: not upright (%.2f rad)", (double)estimate.theta);
        return false;
    }
    motion_control_imu_health_t health;
    if (!motion_control_get_imu_health(&health) || !health.valid)
    {
        APP_LOG_ERROR("Arm rejected: IMU unhealthy");
        return false;
    }
    float left_w = 0.0f;
    float right_w = 0.0f;
    if (!motor_link_get_wheel_velocities(&left_w, &right_w))
    {
        APP_LOG_ERROR("Arm rejected: motor link down");
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
    float dt_s = s_control_dt;
    if (s_last_tick_ms != 0U)
    {
        uint32_t delta_ms = now_ms - s_last_tick_ms;
        if (delta_ms > 0U)
        {
            dt_s = 0.001f * (float)delta_ms;
        }
    }
    if (dt_s > EKF_MAX_DT) {
        dt_s = EKF_MAX_DT;
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
    float accel_z_g = 0.0f;
    bool accel_valid = false;
    if (primary.valid) {
        accel_z_g = primary.accel_z / kGravity;
        accel_valid = true;
    } else if (secondary.valid) {
        accel_z_g = secondary.accel_z / kGravity;
        accel_valid = true;
    }
    uint32_t last_irq_ms = 0U;
#if SENSOR_ENABLE_BMI270
    uint32_t bmi_irq_ms = imu_bmi270_get_last_irq_ms();
    if (bmi_irq_ms > last_irq_ms)
    {
        last_irq_ms = bmi_irq_ms;
    }
#endif
#if SENSOR_ENABLE_ICM42688
    uint32_t icm_irq_ms = imu_icm42688_get_last_irq_ms();
    if (icm_irq_ms > last_irq_ms)
    {
        last_irq_ms = icm_irq_ms;
    }
#endif
    if (last_irq_ms != 0U)
    {
        s_last_imu_irq_ms = last_irq_ms;
    }
    if (imu_ok)
    {
        s_last_imu_ok_ms = now_ms;
    }

    s_estimator.update(primary, secondary, now_ms, v_enc, yaw_rate_enc);

    // Use EKF-filtered yaw rate (gyroZ - yawBias) for control
    float gyro_z_filtered = s_estimator.getEstimatedYawRate();
    s_controller.setYawRates(gyro_z_filtered, yaw_rate_enc);

    StateEstimate estimate = s_estimator.getEstimate();
#ifdef UNIT_TEST
    if (s_test_estimate_override)
    {
        estimate.theta = s_test_estimate.theta_rad;
        estimate.thetaDot = s_test_estimate.theta_dot;
        estimate.x = s_test_estimate.x_m;
        estimate.xDot = s_test_estimate.x_dot_mps;
        estimate.gyroBias = s_test_estimate.gyro_bias;
        estimate.valid = s_test_estimate.valid != 0U;
    }
#endif
    float theta_acc = s_estimator.getLastThetaAcc();
    bool theta_acc_valid = imu_ok && isfinite(theta_acc);

    /* Populate state machine input and evaluate transitions */
    motion_modes_input_t modes_input = {
        .now_ms = now_ms,
        .estimate_valid = estimate.valid,
        .theta_rad = estimate.theta,
        .theta_kill_rad = g_robot_params.balance.thetaKill,
        .theta_acc_valid = theta_acc_valid,
        .theta_acc_rad = theta_acc,
        .imu_ok = imu_ok,
        .last_imu_ok_ms = s_last_imu_ok_ms,
        .last_imu_irq_ms = s_last_imu_irq_ms,
        .motor_ok = wheel_ok,
        .last_motor_ok_ms = s_last_motor_ok_ms,
    };

    motion_modes_output_t modes_output;
    motion_modes_step(&modes_input, &modes_output);

    /* Handle state machine outputs */
    if (modes_output.disable_motors)
    {
        if (!motor_link_enable(false))
        {
            if (!s_motor_disable_failed)
            {
                APP_LOG_ERROR("Motor link disable failed");
                s_motor_disable_failed = true;
            }
        }
        else if (s_motor_disable_failed)
        {
            APP_LOG_INFO("Motor link disable recovered");
            s_motor_disable_failed = false;
        }
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
            hip_state_t hip_left{};
            hip_state_t hip_right{};
            hip_control_get_state(&hip_left, &hip_right);
            float hip_height_dot = 0.0f;
            int hip_count = 0;
            if (hip_left.valid) {
                hip_height_dot += hip_left.height_dot_m_s;
                hip_count++;
            }
            if (hip_right.valid) {
                hip_height_dot += hip_right.height_dot_m_s;
                hip_count++;
            }
            if (hip_count > 0) {
                hip_height_dot /= (float)hip_count;
            }
            if (fabsf(HIP_WHEEL_FF_PITCH_GAIN) > 1e-6f) {
                estimate.theta += HIP_WHEEL_FF_PITCH_GAIN * hip_height_dot;
            }

            MotionController::Command cmd = s_controller.computeControl(estimate, dt_s);
            float wheel_scale = 1.0f;
            if (s_hip_behavior_mode == HIP_BEHAVIOR_IMPULSE) {
                wheel_scale = HIP_WHEEL_SCALE_IMPULSE;
            } else if (s_hip_behavior_mode == HIP_BEHAVIOR_FLIGHT) {
                wheel_scale = HIP_WHEEL_SCALE_FLIGHT;
            } else if (s_hip_behavior_mode == HIP_BEHAVIOR_LANDING) {
                wheel_scale = HIP_WHEEL_SCALE_LANDING;
            }
            cmd.torque.torqueLeftNm *= wheel_scale;
            cmd.torque.torqueRightNm *= wheel_scale;

            /* Safety check: validate control outputs before sending to motors */
            bool left_safe = is_control_value_safe(cmd.torque.torqueLeftNm);
            bool right_safe = is_control_value_safe(cmd.torque.torqueRightNm);

            if (left_safe && right_safe)
            {
                s_last_torque_left = cmd.torque.torqueLeftNm;
                s_last_torque_right = cmd.torque.torqueRightNm;
                if (s_output_enabled)
                {
                    float max_torque = 0.0f;
                    if (g_robot_params.balance.IqMax > 0.0f &&
                        PARAM_MOTOR_KT > 0.0f) {
                        max_torque = g_robot_params.balance.IqMax * PARAM_MOTOR_KT;
                    }
                    motor_link_set_wheel_torques(cmd.torque.torqueLeftNm,
                                                 cmd.torque.torqueRightNm,
                                                 max_torque);
                }
            }
            else
            {
                /* NaN/Inf detected in control output - emergency stop */
                APP_LOG_ERROR("SAFETY: Invalid control output detected "
                              "(L=%d R=%d) - emergency zero",
                              left_safe ? 1 : 0, right_safe ? 1 : 0);
                s_last_torque_left = 0.0f;
                s_last_torque_right = 0.0f;
                if (s_output_enabled)
                {
                    motor_link_set_wheel_torques(0.0f, 0.0f, 0.0f);
                }
                /* Reset controller state to clear any corrupted integrators */
                s_controller.resetPidState();
            }
        }
        else
        {
            s_last_torque_left = 0.0f;
            s_last_torque_right = 0.0f;
            if (s_output_enabled)
            {
                motor_link_set_wheel_torques(0.0f, 0.0f, 0.0f);
            }
        }
    }
    else
    {
        s_last_torque_left = 0.0f;
        s_last_torque_right = 0.0f;
        if (s_output_enabled)
        {
            motor_link_set_wheel_torques(0.0f, 0.0f, 0.0f);
        }
    }

#if HIP_CONTROL_ENABLE
    hip_behavior_set_imu_accel(accel_z_g, accel_valid);
    if (s_last_hip_coord_ms == 0U || (now_ms - s_last_hip_coord_ms) >= 20U) {
        s_last_hip_coord_ms = now_ms;
        hip_state_t hip_left{};
        hip_state_t hip_right{};
        hip_control_get_state(&hip_left, &hip_right);
        if (s_last_lqr_lut_ms == 0U || (now_ms - s_last_lqr_lut_ms) >= 20U) {
            s_last_lqr_lut_ms = now_ms;
            float hip_theta = 0.0f;
            int hip_count = 0;
            if (hip_left.valid) {
                hip_theta += hip_left.theta_rad;
                hip_count++;
            }
            if (hip_right.valid) {
                hip_theta += hip_right.theta_rad;
                hip_count++;
            }
            if (hip_count > 0) {
                hip_theta /= (float)hip_count;
                float K_lut[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                float theta_eq = 0.0f;
                float u_eq = 0.0f;
                if (lqr_lut_eval_full(hip_theta, K_lut, &theta_eq, &u_eq)) {
                    lqr_params_t lqr = g_robot_params.lqr;
                    /* Cascaded architecture: K[0] is position-to-theta gain (Kx).
                     * Keep K[0] from config (not LUT) for now — LUT was tuned
                     * for direct 4-state where K[0] was position-to-torque.
                     * Inner loop gains K[1,2,3] can still be gain-scheduled. */
                    /* lqr.K[0] = K_lut[0]; -- skip: keep config value */
                    for (int i = 1; i < 4; ++i) {
                        lqr.K[i] = K_lut[i];
                    }
                    s_controller.setLqrParams(lqr);
                    /* Apply equilibrium pitch and feedforward torque from LUT */
                    s_controller.setLqrEquilibrium(theta_eq, u_eq);
                }
            }
        }
        hip_target_t target{};
        hip_behavior_tick(now_ms,
                          motion_modes_get(),
                          &hip_left,
                          &hip_right,
                          &target,
                          &s_hip_behavior_mode);
        hip_control_set_target(&target);
    }
    hip_control_tick(now_ms);
#endif

    /* Blackbox logging */
    LogRecord rec;
    memset(&rec, 0, sizeof(rec));

    /* Header */
    rec.magic = LOG_RECORD_MAGIC;
    rec.version = LOG_RECORD_VERSION;
    rec.seq = s_log_seq++;
    rec.t_us = now_ms * 1000U;

    /* Get IMU health metrics for active sensor */
    ImuHealthMetrics health_metrics;
    bool health_valid = s_estimator.getImuHealthMetrics(health_metrics);
    rec.active_imu = health_valid ? health_metrics.active_sensor : 0;

    /* Flags */
    rec.flags = 0;
    if (health_valid && health_metrics.gate_accel) {
        rec.flags |= LOGF_REC_ACCEL_GATED;
    }
    if (health_valid && health_metrics.active_sensor == 1) {
        rec.flags |= LOGF_REC_IMU_FALLBACK;
    }
    if (s_controller.isOutputSaturated()) {
        rec.flags |= LOGF_REC_UL_SAT | LOGF_REC_UR_SAT;
    }
    motion_mode_t mode = motion_modes_get();
    if (mode == MOTION_MODE_FALLEN) {
        rec.flags |= LOGF_REC_FALLEN;
    }
    if (mode == MOTION_MODE_BALANCING) {
        rec.flags |= LOGF_REC_ARMED;
    }

    uint32_t mask = g_robot_params.log_fields_mask;

    /* IMU raw data (LOGF_IMU_RAW) */
    if (mask & LOGF_IMU_RAW) {
#if SENSOR_ENABLE_BMI270
        if (s_bmi_have && health_valid && health_metrics.active_sensor == 0) {
            rec.acc_raw[0] = s_bmi_latest.accel[0];
            rec.acc_raw[1] = s_bmi_latest.accel[1];
            rec.acc_raw[2] = s_bmi_latest.accel[2];
            rec.gyro_raw[0] = s_bmi_latest.gyro[0];
            rec.gyro_raw[1] = s_bmi_latest.gyro[1];
            rec.gyro_raw[2] = s_bmi_latest.gyro[2];
        }
#endif
#if SENSOR_ENABLE_ICM42688
        if (s_icm_have && health_valid && health_metrics.active_sensor == 1) {
            rec.acc_raw[0] = s_icm_latest.accel[0];
            rec.acc_raw[1] = s_icm_latest.accel[1];
            rec.acc_raw[2] = s_icm_latest.accel[2];
            rec.gyro_raw[0] = s_icm_latest.gyro[0];
            rec.gyro_raw[1] = s_icm_latest.gyro[1];
            rec.gyro_raw[2] = s_icm_latest.gyro[2];
        }
#endif
    }

    /* Dual-IMU health metrics (LOGF_IMU2_HEALTH) */
    if ((mask & LOGF_IMU2_HEALTH) && health_valid) {
        rec.gyro_diff_dps = health_metrics.gyro_pitch_diff_dps;
        rec.acc_angle_deg = health_metrics.acc_angle_diff_deg;
        rec.vib_grms = health_metrics.vib_rms_g;
    }

    /* EKF state (LOGF_EKF) */
    if (mask & LOGF_EKF) {
        rec.theta_rad = estimate.theta;
        rec.thetaDot_rads = estimate.thetaDot;
        rec.gyro_bias_rads = estimate.gyroBias;
        rec.x_m = estimate.x;
        rec.x_dot_mps = estimate.xDot;
    }

    /* Wheel velocities (LOGF_WHEELS) */
    if (mask & LOGF_WHEELS) {
        rec.wL_rads = left_w;
        rec.wR_rads = right_w;
        rec.v_mps = v_enc;
    }

    /* PID controller internals (LOGF_PID) */
    if (mask & LOGF_PID) {
        rec.theta_ref_rad = s_controller.getLastPitchTarget();
        rec.e_theta_rad = s_controller.getLastPitchError();
        rec.P = s_controller.getLastPitchP();
        rec.I = s_controller.getLastPitchI();
        rec.D = s_controller.getLastPitchD();
        rec.uL_cmd = s_last_torque_left;
        rec.uR_cmd = s_last_torque_right;

        /* LQR diagnostics */
        InnerCtrlDiag lqr_diag{};
        if (s_controller.getInnerCtrlDiag(lqr_diag)) {
            rec.u_common = lqr_diag.u_sum_cmd;
            rec.u_turn = lqr_diag.u_diff_cmd;
            rec.u_sum_lqr = lqr_diag.u_sum_lqr;
            rec.lqr_alpha = static_cast<uint8_t>(lqr_diag.alpha * 255.0f);
            if (lqr_diag.active_mode == InnerLongMode::LQR) {
                rec.flags |= LOGF_REC_LQR_ACTIVE;
            }
        } else {
            rec.u_common = 0.0f;
            rec.u_turn = 0.0f;
            rec.u_sum_lqr = 0.0f;
            rec.lqr_alpha = 0;
        }
    }

    /* Compute CRC */
    rec.crc32 = robot_crc32((const uint8_t *)&rec, sizeof(LogRecord) - sizeof(uint32_t));

    /* Push to blackbox */
    log_push_record(&rec);
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
#ifdef UNIT_TEST
    if (s_test_imu_health_override)
    {
        *out = s_test_imu_health;
        return true;
    }
#endif
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

#ifdef UNIT_TEST
void motion_control_test_set_estimate(const motion_control_estimate_t *est)
{
    if (est == NULL)
    {
        s_test_estimate_override = false;
        return;
    }
    s_test_estimate = *est;
    s_test_estimate_override = true;
}

void motion_control_test_set_imu_health(const motion_control_imu_health_t *health)
{
    if (health == NULL)
    {
        s_test_imu_health_override = false;
        return;
    }
    s_test_imu_health = *health;
    s_test_imu_health_override = true;
}

void motion_control_test_clear_overrides(void)
{
    s_test_estimate_override = false;
    s_test_imu_health_override = false;
}
#endif

bool motion_control_get_control_output(motion_control_output_t *out)
{
    if (out == NULL)
    {
        return false;
    }
    out->torque_left_nm = s_last_torque_left;
    out->torque_right_nm = s_last_torque_right;
    out->pitch_target_rad = s_controller.getLastPitchTarget();
    return true;
}

bool motion_control_is_saturated(void)
{
    return s_controller.isOutputSaturated();
}

void motion_control_set_inner_mode(uint8_t mode)
{
    InnerLongMode m = (mode != 0U) ? InnerLongMode::LQR : InnerLongMode::PID;
    s_controller.setRequestedMode(m);
}

uint8_t motion_control_get_inner_mode(void)
{
    return (s_controller.getActiveMode() == InnerLongMode::LQR) ? 1U : 0U;
}

uint8_t motion_control_get_hip_behavior_mode(void)
{
    return (uint8_t)s_hip_behavior_mode;
}

uint8_t motion_control_get_hip_phase_progress(void)
{
    return hip_behavior_get_phase_progress();
}
