#include "ekf/BalancerEKF.h"
#include "config_control.h"
#include <math.h>
#include <cstring>

namespace {
constexpr float MAX_DT = EKF_MAX_DT;  // guard against large time steps
}  // namespace

BalancerEKF::BalancerEKF()
    : ekf_{},
      Q_{},
      R_{},
      lastGyroPitch_(0.0f),
      lastDt_(0.0f),
      diag_{},
      diag_valid_(false),
      theta_r_base_(0.0f)
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

    for (int i = 0; i < EKF_M * EKF_M; ++i) {
        R_[i] = 0.0f;
    }
    R_[0 * EKF_M + 0] = EKF_R_THETA_ACC;
    R_[1 * EKF_M + 1] = EKF_R_V_ENC;
    R_[2 * EKF_M + 2] = EKF_R_X_POS;
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
}

bool BalancerEKF::step(float thetaAcc, float vEnc, float posEnc, float gyroPitch, float dt,
                       float thetaMeasVar)
{
    if (dt <= 0.0f) {
        dt = 0.001f;
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

    // Process model fx
    float fx[EKF_N] = {};
    fx[0] = theta + thetaDot * dt;
    fx[1] = gyroPitch - bias;
    fx[2] = pos + vel * dt;
    fx[3] = vel;
    fx[4] = bias;

    // Jacobian F (flattened, row-major)
    float F[EKF_N * EKF_N] = {};
    F[0 * EKF_N + 0] = 1.0f;
    F[0 * EKF_N + 1] = dt;
    F[1 * EKF_N + 4] = -1.0f;
    F[2 * EKF_N + 2] = 1.0f;
    F[2 * EKF_N + 3] = dt;
    F[3 * EKF_N + 3] = 1.0f;
    F[4 * EKF_N + 4] = 1.0f;

    ekf_predict(&ekf_, fx, F, Q_);

    // Measurement model h(x) = [theta, xDot, x]
    float hx[EKF_M] = {};
    hx[0] = fx[0];
    hx[1] = fx[3];
    hx[2] = fx[2];

    float H[EKF_M * EKF_N] = {};
    H[0 * EKF_N + 0] = 1.0f;
    H[1 * EKF_N + 3] = 1.0f;
    H[2 * EKF_N + 2] = 1.0f;

    float z[EKF_M];
    z[0] = thetaAcc;
    z[1] = vEnc;
    z[2] = posEnc;

    // If any measurement is invalid (NaN), replace it with the predicted value so the innovation is zero,
    // effectively skipping that measurement update while still advancing the process model.
    if (!isfinite(z[1])) {
        z[1] = hx[1];
    }
    if (!isfinite(z[2])) {
        z[2] = hx[2];
    }

    // Innovation gating
    float innov_theta = z[0] - hx[0];
    float innov_vel = z[1] - hx[1];
    float innov_pos = z[2] - hx[2];
    bool bad_innov =
        fabsf(innov_theta) > EKF_INNOV_THETA_MAX_RAD ||
        fabsf(innov_vel) > EKF_INNOV_VEL_MAX_MPS ||
        fabsf(innov_pos) > EKF_INNOV_POS_MAX_M;
    if (bad_innov) {
        reset(thetaAcc, posEnc);
        diag_valid_ = false;
        return false;
    }

    float measurement_var = isnan(thetaMeasVar) ? R_[0] : thetaMeasVar;
    float R_step[EKF_M * EKF_M];
    memcpy(R_step, R_, sizeof(R_step));
    R_step[0] = measurement_var;

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
