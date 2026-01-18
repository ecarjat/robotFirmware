#include "StateEstimator.h"

#include <math.h>
#include <string.h>

namespace {
constexpr float kRadToDeg = 57.2957795f;
constexpr float kGravity = 9.80665f;

void set_identity(float rot[9]) {
    memset(rot, 0, sizeof(float) * 9U);
    rot[0] = 1.0f;
    rot[4] = 1.0f;
    rot[8] = 1.0f;
}

void apply_rotation(const float rot[9], const float in[3], float out[3]) {
    out[0] = rot[0] * in[0] + rot[1] * in[1] + rot[2] * in[2];
    out[1] = rot[3] * in[0] + rot[4] * in[1] + rot[5] * in[2];
    out[2] = rot[6] * in[0] + rot[7] * in[1] + rot[8] * in[2];
}

float vec_norm(const float v[3]) {
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

float vec_dot(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
}  // namespace

StateEstimator::StateEstimator()
    : ekf_(),
      estimate_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false},
      lastUpdateMs_(0U),
      bootInitDone_(false),
      lastThetaAcc_(0.0f),
      lastVEnc_(0.0f),
      lastGyroPitch_(0.0f),
      lastWheelAngleL_(0.0f),
      lastWheelAngleR_(0.0f),
      lastWheelMechL_(0.0f),
      lastWheelMechR_(0.0f),
      haveWheelAngles_(false),
      odomX_(0.0f),
      last_primary_ts_(0U),
      last_secondary_ts_(0U),
      initSampleCount_(0),
      initPitchSum_(0.0f),
      wheelRadius_(PARAM_WHEEL_RADIUS),
      control_dt_(CONTROL_DT),
      initialized_(false),
      ekf_log_data_{},
      ekf_log_valid_(false),
      imu_health_{},
      imu_health_valid_(false),
      imu_primary_rot_{},
      imu_secondary_rot_{},
      vib_primary_{},
      vib_secondary_{},
      use_secondary_(false),
      last_switch_ms_(0U),
      primary_unhealthy_since_ms_(0U),
      primary_healthy_since_ms_(0U)
{
    set_identity(imu_primary_rot_);
    set_identity(imu_secondary_rot_);
    memset(&vib_primary_, 0, sizeof(vib_primary_));
    memset(&vib_secondary_, 0, sizeof(vib_secondary_));
}

bool StateEstimator::begin(const RobotParams &params)
{
    wheelRadius_ = params.wheelRadius;
    ekf_.begin();
    initialized_ = true;
    bootInitDone_ = false;
    lastUpdateMs_ = 0U;
    use_secondary_ = false;
    last_switch_ms_ = 0U;
    primary_unhealthy_since_ms_ = 0U;
    primary_healthy_since_ms_ = 0U;
    imu_health_valid_ = false;
    return true;
}

StateEstimate StateEstimator::getEstimate() const
{
    return estimate_;
}

bool StateEstimator::getEkfLogData(EkfLogData& out) const
{
    if (!ekf_log_valid_) {
        return false;
    }
    out = ekf_log_data_;
    return true;
}

bool StateEstimator::getImuHealthMetrics(ImuHealthMetrics& out) const
{
    if (!imu_health_valid_) {
        return false;
    }
    out = imu_health_;
    return true;
}

void StateEstimator::resetTiming()
{
    lastUpdateMs_ = 0U;
}

void StateEstimator::setControlDt(float dtSeconds)
{
    if (dtSeconds > 0.0f) {
        control_dt_ = dtSeconds;
        ekf_.setFallbackDt(dtSeconds);
    }
}

void StateEstimator::setImuRotations(const float *primary, const float *secondary)
{
    if (primary != NULL) {
        memcpy(imu_primary_rot_, primary, sizeof(imu_primary_rot_));
    }
    if (secondary != NULL) {
        memcpy(imu_secondary_rot_, secondary, sizeof(imu_secondary_rot_));
    }
}

void StateEstimator::update(const ImuReading &primary, const ImuReading &secondary,
                            uint32_t now_ms, float v_enc, float yaw_rate_enc)
{
    if (!initialized_)
    {
        return;
    }

    uint32_t primary_dt_ms = 0U;
    uint32_t secondary_dt_ms = 0U;
    if (primary.timestamp_ms != 0U)
    {
        if (last_primary_ts_ != 0U && primary.timestamp_ms >= last_primary_ts_)
        {
            primary_dt_ms = primary.timestamp_ms - last_primary_ts_;
        }
        last_primary_ts_ = primary.timestamp_ms;
    }
    if (secondary.timestamp_ms != 0U)
    {
        if (last_secondary_ts_ != 0U && secondary.timestamp_ms >= last_secondary_ts_)
        {
            secondary_dt_ms = secondary.timestamp_ms - last_secondary_ts_;
        }
        last_secondary_ts_ = secondary.timestamp_ms;
    }

    float dt_s = 0.0f;
    if (lastUpdateMs_ != 0U)
    {
        uint32_t delta_ms = now_ms - lastUpdateMs_;
        dt_s = (delta_ms > 0U) ? (0.001f * (float)delta_ms) : control_dt_;
    }
    else
    {
        dt_s = control_dt_;
    }
    lastUpdateMs_ = now_ms;

    float accel_primary[3] = {primary.accel_x, primary.accel_y, primary.accel_z};
    float gyro_primary[3] = {primary.gyro_x, primary.gyro_y, primary.gyro_z};
    float accel_secondary[3] = {secondary.accel_x, secondary.accel_y, secondary.accel_z};
    float gyro_secondary[3] = {secondary.gyro_x, secondary.gyro_y, secondary.gyro_z};

    float accel_primary_body[3];
    float gyro_primary_body[3];
    float accel_secondary_body[3];
    float gyro_secondary_body[3];
    apply_rotation(imu_primary_rot_, accel_primary, accel_primary_body);
    apply_rotation(imu_primary_rot_, gyro_primary, gyro_primary_body);
    apply_rotation(imu_secondary_rot_, accel_secondary, accel_secondary_body);
    apply_rotation(imu_secondary_rot_, gyro_secondary, gyro_secondary_body);

    if (primary.valid) {
        float norm_g = vec_norm(accel_primary_body) / kGravity;
        float delta = norm_g - 1.0f;
        VibWindow *w = &vib_primary_;
        float old = (w->count < IMU_VIB_WINDOW_SAMPLES) ? 0.0f : w->samples[w->index];
        if (w->count < IMU_VIB_WINDOW_SAMPLES) {
            w->count++;
        }
        w->samples[w->index] = delta;
        w->index = (w->index + 1U) % IMU_VIB_WINDOW_SAMPLES;
        w->sum_sq += (delta * delta) - (old * old);
        if (w->sum_sq < 0.0f) w->sum_sq = 0.0f;  /* Guard against FP rounding errors */
        w->rms = (w->count > 0U) ? sqrtf(w->sum_sq / (float)w->count) : 0.0f;
    }

    if (secondary.valid) {
        float norm_g = vec_norm(accel_secondary_body) / kGravity;
        float delta = norm_g - 1.0f;
        VibWindow *w = &vib_secondary_;
        float old = (w->count < IMU_VIB_WINDOW_SAMPLES) ? 0.0f : w->samples[w->index];
        if (w->count < IMU_VIB_WINDOW_SAMPLES) {
            w->count++;
        }
        w->samples[w->index] = delta;
        w->index = (w->index + 1U) % IMU_VIB_WINDOW_SAMPLES;
        w->sum_sq += (delta * delta) - (old * old);
        if (w->sum_sq < 0.0f) w->sum_sq = 0.0f;  /* Guard against FP rounding errors */
        w->rms = (w->count > 0U) ? sqrtf(w->sum_sq / (float)w->count) : 0.0f;
    }

    bool primary_healthy = primary.valid;
    bool secondary_healthy = secondary.valid;
    float gyro_diff_dps = NAN;
    float gyro_pitch_diff_dps = NAN;
    float acc_angle_diff_deg = NAN;

    if (primary.valid && secondary.valid)
    {
        float gyro_diff_vec[3] = {
            gyro_primary_body[0] - gyro_secondary_body[0],
            gyro_primary_body[1] - gyro_secondary_body[1],
            gyro_primary_body[2] - gyro_secondary_body[2]};
        float gyro_diff = vec_norm(gyro_diff_vec);
        gyro_diff_dps = gyro_diff * kRadToDeg;
        gyro_pitch_diff_dps = fabsf(gyro_primary_body[1] - gyro_secondary_body[1]) * kRadToDeg;

        float acc_norm_p = vec_norm(accel_primary_body);
        float acc_norm_s = vec_norm(accel_secondary_body);
        float acc_norm_p_g = acc_norm_p / kGravity;
        float acc_norm_s_g = acc_norm_s / kGravity;
        if (acc_norm_p_g > 0.8f && acc_norm_p_g < 1.2f &&
            acc_norm_s_g > 0.8f && acc_norm_s_g < 1.2f)
        {
            float denom = acc_norm_p * acc_norm_s;
            if (denom > 1e-6f)
            {
                float cosang = vec_dot(accel_primary_body, accel_secondary_body) / denom;
                if (cosang > 1.0f) cosang = 1.0f;
                if (cosang < -1.0f) cosang = -1.0f;
                acc_angle_diff_deg = acosf(cosang) * kRadToDeg;
            }
        }

        if (!isnan(gyro_diff_dps) &&
            (gyro_diff_dps > IMU_GYRO_DISAGREE_FAULT_DPS ||
             gyro_pitch_diff_dps > IMU_GYRO_DISAGREE_FAULT_DPS))
        {
            primary_healthy = false;
        }

        if (!isnan(acc_angle_diff_deg) &&
            acc_angle_diff_deg > IMU_ACC_ANGLE_FAULT_DEG)
        {
            primary_healthy = false;
        }
    }

    bool switched = false;
    bool prev_use_secondary = use_secondary_;

    if (!use_secondary_)
    {
        if (!primary_healthy)
        {
            if (primary_unhealthy_since_ms_ == 0U)
            {
                primary_unhealthy_since_ms_ = now_ms;
            }
            if (secondary_healthy &&
                (now_ms - primary_unhealthy_since_ms_) >= IMU_SWITCH_TO_SECONDARY_MS &&
                (now_ms - last_switch_ms_) >= IMU_SWITCH_DWELL_MS)
            {
                use_secondary_ = true;
                last_switch_ms_ = now_ms;
                primary_unhealthy_since_ms_ = 0U;
                primary_healthy_since_ms_ = 0U;
            }
        }
        else
        {
            primary_unhealthy_since_ms_ = 0U;
        }
    }
    else
    {
        if (!secondary.valid && primary.valid)
        {
            use_secondary_ = false;
            last_switch_ms_ = now_ms;
            primary_unhealthy_since_ms_ = 0U;
            primary_healthy_since_ms_ = 0U;
        }
        else if (primary_healthy)
        {
            if (primary_healthy_since_ms_ == 0U)
            {
                primary_healthy_since_ms_ = now_ms;
            }
            if ((now_ms - primary_healthy_since_ms_) >= IMU_SWITCH_BACK_MS &&
                (now_ms - last_switch_ms_) >= IMU_SWITCH_DWELL_MS)
            {
                use_secondary_ = false;
                last_switch_ms_ = now_ms;
                primary_unhealthy_since_ms_ = 0U;
                primary_healthy_since_ms_ = 0U;
            }
        }
        else
        {
            primary_healthy_since_ms_ = 0U;
        }
    }

    if (use_secondary_ != prev_use_secondary)
    {
        switched = true;
    }

    const bool active_valid = use_secondary_ ? secondary.valid : primary.valid;
    const float *accel_body = use_secondary_ ? accel_secondary_body : accel_primary_body;
    const float *gyro_body = use_secondary_ ? gyro_secondary_body : gyro_primary_body;

    float accel_norm_g = vec_norm(accel_body) / kGravity;
    float vib_rms = use_secondary_ ? vib_secondary_.rms : vib_primary_.rms;

    bool gate_accel = imu_health_.gate_accel != 0U;
    if (!gate_accel && vib_rms > IMU_VIB_ON_G)
    {
        gate_accel = true;
    }
    else if (gate_accel && vib_rms < IMU_VIB_OFF_G)
    {
        gate_accel = false;
    }

    imu_health_.valid = active_valid;
    imu_health_.active_sensor = use_secondary_ ? 1U : 0U;
    imu_health_.gyro_diff_dps = gyro_diff_dps;
    imu_health_.gyro_pitch_diff_dps = gyro_pitch_diff_dps;
    imu_health_.acc_angle_diff_deg = acc_angle_diff_deg;
    imu_health_.vib_rms_g = vib_rms;
    imu_health_.gate_accel = gate_accel ? 1U : 0U;
    imu_health_valid_ = true;

    if (!active_valid)
    {
        estimate_.valid = false;
        return;
    }

    float theta_acc = NAN;
    float accel_yz = sqrtf(accel_body[1] * accel_body[1] + accel_body[2] * accel_body[2]);
    if (accel_yz > 1e-6f && isfinite(accel_norm_g)) {
        theta_acc = atan2f(-accel_body[0], accel_yz);
    }
    float gyro_pitch = gyro_body[1];
    float gyro_yaw = gyro_body[2];

    if (switched)
    {
        float theta_init = estimate_.valid ? estimate_.theta : theta_acc;
        if (!isfinite(theta_init))
        {
            theta_init = 0.0f;
        }
        float pos_init = estimate_.valid ? estimate_.x : 0.0f;
        ekf_.reset(theta_init, pos_init);
    }

    float theta_var = ekf_.getThetaMeasurementVarianceBase();
    if (gate_accel)
    {
        theta_var *= EKF_TUNE_R_MULT;
    }

    // Enable encoder velocity and yaw rate measurements
    float v_enc_ekf = v_enc;
    float pos_enc = NAN;  // Position measurement still disabled
    bool ekf_ok = ekf_.step(theta_acc, v_enc_ekf, pos_enc,
                            gyro_pitch, gyro_yaw, yaw_rate_enc,
                            dt_s, theta_var);
    BalancerState st = ekf_.getState();

    estimate_.theta = st.theta;
    estimate_.thetaDot = st.thetaDot;
    estimate_.x = st.x;
    estimate_.xDot = st.xDot;
    estimate_.gyroBias = st.gyroBias;
    estimate_.yaw = st.yaw;
    estimate_.yawBias = st.yawBias;
    estimate_.valid = ekf_ok;

    lastThetaAcc_ = theta_acc;
    lastVEnc_ = v_enc;
    lastGyroPitch_ = gyro_pitch;

    ekf_log_data_.valid = ekf_ok;
    ekf_log_data_.dt_s = dt_s;
    ekf_log_data_.accel_x = accel_body[0];
    ekf_log_data_.accel_y = accel_body[1];
    ekf_log_data_.accel_z = accel_body[2];
    ekf_log_data_.gyro_x = gyro_body[0];
    ekf_log_data_.gyro_y = gyro_body[1];
    ekf_log_data_.gyro_z = gyro_body[2];
    ekf_log_data_.accel_norm_g = accel_norm_g;
    ekf_log_data_.theta = st.theta;
    ekf_log_data_.theta_acc = theta_acc;
    ekf_log_data_.gyro_rate = gyro_pitch;
    ekf_log_data_.imu_primary_dt_ms = primary_dt_ms;
    ekf_log_data_.imu_secondary_dt_ms = secondary_dt_ms;
    ekf_log_data_.gate = gate_accel ? 1U : 0U;
    ekf_log_data_.still = 0U;

    BalancerEkfDiag diag;
    if (ekf_.getDiag(diag)) {
        ekf_log_data_.innov = diag.innov_theta;
        ekf_log_data_.s = diag.s_theta;
        ekf_log_data_.k0 = diag.k_theta;
        ekf_log_data_.k1 = diag.k_bias;
        ekf_log_data_.p00 = diag.p00;
        ekf_log_data_.p01 = diag.p01;
        ekf_log_data_.p11 = diag.p11;
        ekf_log_data_.r_used = theta_var;
    }

    ekf_log_valid_ = true;
}

bool StateEstimator::getLastWheelMechanicalAngles(float &angle_left, float &angle_right) const
{
    (void)angle_left;
    (void)angle_right;
    return false;
}

float StateEstimator::getEstimatedYawRate() const
{
    // Return gyro yaw rate minus estimated bias
    return ekf_log_data_.gyro_z - estimate_.yawBias;
}
