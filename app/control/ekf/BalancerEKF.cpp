#include "ekf/BalancerEKF.h"
#include "config_control.h"
#include <math.h>
#include <cstring>

namespace {
constexpr float MAX_DT = EKF_MAX_DT;  // guard against large time steps
constexpr int POST_RESET_DAMPING_STEPS = 10;  // Steps to apply extra damping after reset
constexpr float DAMPING_R_MULTIPLIER = 5.0f;  // Inflate measurement variance during damping
constexpr float INNOV_GATE_R_MULTIPLIER = 100.0f;  // Inflate R when gating instead of resetting
constexpr float MAX_R_INFLATION = 100.0f;  // Cap combined inflation to preserve observability
}  // namespace

BalancerEKF::BalancerEKF()
    : ekf_{},
      Q_{},
      R_{},
      lastGyroPitch_(0.0f),
      lastDt_(0.0f),
      fallback_dt_(CONTROL_DT),
      diag_{},
      diag_valid_(false),
      theta_r_base_(0.0f),
      damping_steps_remaining_(0),
      nan_reset_count_(0)
{
}

void BalancerEKF::begin()
{
    initState();
    initNoiseCovariances();
}

void BalancerEKF::reset(float theta_init, float pos_init)
{
    initState();
    setInitialState(theta_init, pos_init);
    damping_steps_remaining_ = POST_RESET_DAMPING_STEPS;
}

void BalancerEKF::partialReset(float theta_init, float pos_init)
{
    // Preserve velocity (x[3]) and gyro bias (x[4]) estimates
    float saved_vel = ekf_.x[3];
    float saved_bias = ekf_.x[4];

    // Reset theta and position only
    ekf_.x[0] = theta_init;
    ekf_.x[1] = 0.0f;  // Reset theta_dot to zero
    ekf_.x[2] = pos_init;
    ekf_.x[3] = saved_vel;
    ekf_.x[4] = saved_bias;

    // Increase covariance for reset states only
    ekf_.P[0 * EKF_N + 0] = EKF_P0_THETA;
    ekf_.P[1 * EKF_N + 1] = EKF_P0_THETA_DOT;
    ekf_.P[2 * EKF_N + 2] = EKF_P0_X;
    ekf_.P[4 * EKF_N + 4] = EKF_P0_BIAS;  // Inflate bias covariance after reset
    // Keep P[3][3] unchanged (velocity covariance)

    // Clear cross-covariances involving reset states to avoid inconsistency.
    const int reset_states[] = {0, 1, 2};
    for (int idx = 0; idx < (int)(sizeof(reset_states) / sizeof(reset_states[0])); ++idx) {
        int i = reset_states[idx];
        for (int j = 0; j < EKF_N; ++j) {
            if (j == i) {
                continue;
            }
            ekf_.P[i * EKF_N + j] = 0.0f;
            ekf_.P[j * EKF_N + i] = 0.0f;
        }
    }

    // Start post-reset damping period
    damping_steps_remaining_ = POST_RESET_DAMPING_STEPS;
}

void BalancerEKF::initState()
{
    // Upright, stationary, zero bias with diagonal covariance.
    const float Pdiag[EKF_N] = {
        EKF_P0_THETA,
        EKF_P0_THETA_DOT,
        EKF_P0_X,
        EKF_P0_X_DOT,
        EKF_P0_BIAS,
        EKF_P0_YAW,
        EKF_P0_YAW_BIAS,
    };
    ekf_initialize(&ekf_, Pdiag);
}

void BalancerEKF::initNoiseCovariances()
{
    // Zero all then set diagonal noise terms.
    for (int i = 0; i < EKF_N * EKF_N; ++i) {
        Q_[i] = 0.0f;
    }
    Q_[0 * EKF_N + 0] = EKF_Q_THETA;
    Q_[1 * EKF_N + 1] = EKF_Q_THETA_DOT;
    Q_[2 * EKF_N + 2] = EKF_Q_X;
    Q_[3 * EKF_N + 3] = EKF_Q_X_DOT;
    Q_[4 * EKF_N + 4] = EKF_Q_BIAS;
    Q_[5 * EKF_N + 5] = EKF_Q_YAW;
    Q_[6 * EKF_N + 6] = EKF_Q_YAW_BIAS;

    for (int i = 0; i < EKF_M * EKF_M; ++i) {
        R_[i] = 0.0f;
    }
    R_[0 * EKF_M + 0] = EKF_R_THETA_ACC;
    R_[1 * EKF_M + 1] = EKF_R_V_ENC;
    R_[2 * EKF_M + 2] = EKF_R_X_POS;
    R_[3 * EKF_M + 3] = EKF_R_YAW_RATE_ENC;
    theta_r_base_ = R_[0 * EKF_M + 0];
}

void BalancerEKF::setInitialTheta(float theta_rad)
{
    ekf_.x[0] = theta_rad;
}

void BalancerEKF::setInitialState(float theta_rad, float pos_m)
{
    ekf_.x[0] = theta_rad;
    ekf_.x[1] = 0.0f;
    ekf_.x[2] = pos_m;
    ekf_.x[3] = 0.0f;
    ekf_.x[4] = 0.0f;
    ekf_.x[5] = 0.0f;  // yaw
    ekf_.x[6] = 0.0f;  // yawBias
}

void BalancerEKF::setFallbackDt(float dtSeconds)
{
    if (dtSeconds > 0.0f) {
        fallback_dt_ = dtSeconds;
    }
}

bool BalancerEKF::step(float thetaAcc, float vEnc, float posEnc,
                       float gyroPitch, float gyroYaw, float yawRateEnc,
                       float dt, float thetaMeasVar)
{
    if (dt <= 0.0f) {
        dt = fallback_dt_;
    }
    if (dt > MAX_DT) {
        dt = MAX_DT;
    }

    lastGyroPitch_ = gyroPitch;
    lastDt_ = dt;

    // Current state
    float theta = ekf_.x[0];
    float thetaDot = ekf_.x[1];
    float pos = ekf_.x[2];
    float vel = ekf_.x[3];
    float bias = ekf_.x[4];
    float yaw = ekf_.x[5];
    float yawBias = ekf_.x[6];

    // Process model fx
    float fx[EKF_N] = {};
    fx[0] = theta + thetaDot * dt;
    fx[1] = gyroPitch - bias;
    fx[2] = pos + vel * dt;
    fx[3] = vel;
    fx[4] = bias;
    fx[5] = yaw + (gyroYaw - yawBias) * dt;  // Integrate yaw using gyro
    fx[6] = yawBias;                          // Bias random walk

    // Jacobian F (flattened, row-major, 7x7)
    float F[EKF_N * EKF_N] = {};
    F[0 * EKF_N + 0] = 1.0f;   // d(theta)/d(theta)
    F[0 * EKF_N + 1] = dt;     // d(theta)/d(thetaDot)
    F[1 * EKF_N + 4] = -1.0f;  // d(thetaDot)/d(bias)
    F[2 * EKF_N + 2] = 1.0f;   // d(x)/d(x)
    F[2 * EKF_N + 3] = dt;     // d(x)/d(xDot)
    F[3 * EKF_N + 3] = 1.0f;   // d(xDot)/d(xDot)
    F[4 * EKF_N + 4] = 1.0f;   // d(bias)/d(bias)
    F[5 * EKF_N + 5] = 1.0f;   // d(yaw)/d(yaw)
    F[5 * EKF_N + 6] = -dt;    // d(yaw)/d(yawBias)
    F[6 * EKF_N + 6] = 1.0f;   // d(yawBias)/d(yawBias)

    ekf_predict(&ekf_, fx, F, Q_);

    // Validate state after predict - NaN can propagate from bad inputs or numerical issues
    if (!isStateValid()) {
        nan_reset_count_++;
        reset(0.0f, 0.0f);
        diag_valid_ = false;
        return false;
    }

    // Measurement model h(x) = [theta, xDot, x, yawRate]
    // yawRate predicted = gyroYaw - yawBias
    float predictedYawRate = gyroYaw - ekf_.x[6];

    float hx[EKF_M] = {};
    hx[0] = fx[0];              // theta
    hx[1] = fx[3];              // xDot
    hx[2] = fx[2];              // x
    hx[3] = predictedYawRate;   // yaw rate from gyro minus bias

    // Jacobian H (4x7)
    float H[EKF_M * EKF_N] = {};
    H[0 * EKF_N + 0] = 1.0f;    // d(theta_meas)/d(theta)
    H[1 * EKF_N + 3] = 1.0f;    // d(v_meas)/d(xDot)
    H[2 * EKF_N + 2] = 1.0f;    // d(x_meas)/d(x)
    H[3 * EKF_N + 6] = -1.0f;   // d(yawRate_meas)/d(yawBias)

    bool theta_valid = isfinite(thetaAcc);
    bool vel_valid = isfinite(vEnc);
    bool pos_valid = isfinite(posEnc);
    bool yawRate_valid = isfinite(yawRateEnc);

    float z[EKF_M];
    z[0] = thetaAcc;
    z[1] = vEnc;
    z[2] = posEnc;
    z[3] = yawRateEnc;

    // If a measurement is invalid (NaN), replace it with the predicted value and
    // zero the corresponding H row so the update truly skips that channel.
    if (!theta_valid) {
        z[0] = hx[0];
        for (int i = 0; i < EKF_N; ++i) {
            H[0 * EKF_N + i] = 0.0f;
        }
    }
    if (!vel_valid) {
        z[1] = hx[1];
        for (int i = 0; i < EKF_N; ++i) {
            H[1 * EKF_N + i] = 0.0f;
        }
    }
    if (!pos_valid) {
        z[2] = hx[2];
        for (int i = 0; i < EKF_N; ++i) {
            H[2 * EKF_N + i] = 0.0f;
        }
    }
    if (!yawRate_valid) {
        z[3] = hx[3];
        for (int i = 0; i < EKF_N; ++i) {
            H[3 * EKF_N + i] = 0.0f;
        }
    }

    // Innovation gating - check each measurement channel
    float innov_theta = z[0] - hx[0];
    float innov_vel = z[1] - hx[1];
    float innov_pos = z[2] - hx[2];
    float innov_yawRate = z[3] - hx[3];

    bool bad_theta = fabsf(innov_theta) > EKF_INNOV_THETA_MAX_RAD;
    bool bad_vel = fabsf(innov_vel) > EKF_INNOV_VEL_MAX_MPS;
    bool bad_pos = fabsf(innov_pos) > EKF_INNOV_POS_MAX_M;
    bool bad_yawRate = fabsf(innov_yawRate) > EKF_INNOV_YAW_RATE_MAX_RPS;

    // For severe position/velocity errors, do a partial reset (preserves bias/velocity estimates)
    // For theta errors, use gating (inflate R) instead of hard reset
    if (bad_pos || bad_vel) {
        float pos_reset = pos_valid ? posEnc : pos;
        float theta_reset = theta_valid ? thetaAcc : theta;
        partialReset(theta_reset, pos_reset);
        diag_valid_ = false;
        return false;
    }

    float base_measurement_var = isnan(thetaMeasVar) ? R_[0] : thetaMeasVar;
    float r_inflation = 1.0f;

    // Apply innovation gating for theta: inflate R instead of resetting
    // This allows smoother recovery from transient disturbances
    if (bad_theta) {
        r_inflation *= INNOV_GATE_R_MULTIPLIER;
    }

    // Apply post-reset damping: inflate measurement variance to reduce correction magnitude
    if (damping_steps_remaining_ > 0) {
        r_inflation *= DAMPING_R_MULTIPLIER;
        damping_steps_remaining_--;
    }

    if (r_inflation > MAX_R_INFLATION) {
        r_inflation = MAX_R_INFLATION;
    }

    float measurement_var = base_measurement_var * r_inflation;

    float R_step[EKF_M * EKF_M];
    memcpy(R_step, R_, sizeof(R_step));
    R_step[0] = theta_valid ? measurement_var : 1e6f;
    if (!vel_valid) {
        R_step[1 * EKF_M + 1] = 1e6f;
    }
    if (!pos_valid) {
        R_step[2 * EKF_M + 2] = 1e6f;
    }
    if (!yawRate_valid || bad_yawRate) {
        R_step[3 * EKF_M + 3] = 1e6f;
    }

    float p00 = ekf_.P[0 * EKF_N + 0];
    float p40 = ekf_.P[4 * EKF_N + 0];
    float s_theta = p00 + measurement_var;
    float inv_s = (s_theta > 1e-9f) ? (1.0f / s_theta) : 0.0f;
    float k_theta = p00 * inv_s;
    float k_bias = p40 * inv_s;
    diag_.innov_theta = innov_theta;
    diag_.s_theta = s_theta;
    diag_.k_theta = k_theta;
    diag_.k_bias = k_bias;
    diag_.p00 = 0.0f;
    diag_.p01 = 0.0f;
    diag_.p11 = 0.0f;
    diag_valid_ = false;

    bool ok = ekf_update(&ekf_, z, hx, H, R_step);

    // Validate state after update - catch NaN propagation from measurement fusion
    if (!isStateValid()) {
        nan_reset_count_++;
        reset(0.0f, 0.0f);
        diag_valid_ = false;
        return false;
    }

    if (ok) {
        diag_.p00 = ekf_.P[0 * EKF_N + 0];
        diag_.p01 = ekf_.P[0 * EKF_N + 4];
        diag_.p11 = ekf_.P[4 * EKF_N + 4];
        diag_valid_ = true;
    }
    return ok;
}

BalancerState BalancerEKF::getState() const
{
    BalancerState s{};
    s.theta = ekf_.x[0];
    s.thetaDot = ekf_.x[1];
    s.x = ekf_.x[2];
    s.xDot = ekf_.x[3];
    s.gyroBias = ekf_.x[4];
    s.yaw = ekf_.x[5];
    s.yawBias = ekf_.x[6];
    return s;
}

bool BalancerEKF::getDiag(BalancerEkfDiag& out) const
{
    if (!diag_valid_) {
        return false;
    }
    out = diag_;
    return true;
}

float BalancerEKF::getThetaMeasurementVarianceBase() const
{
    return theta_r_base_;
}

bool BalancerEKF::isStateValid() const
{
    for (int i = 0; i < EKF_N; ++i) {
        if (!isfinite(ekf_.x[i])) {
            return false;
        }
    }
    return true;
}
