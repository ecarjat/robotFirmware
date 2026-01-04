#ifndef CONTROL_BALANCER_EKF_H
#define CONTROL_BALANCER_EKF_H

// TinyEKF uses compile-time dimensions via macros.
#define EKF_N 5  // [theta, theta_dot, x, x_dot, b_g]
#define EKF_M 3  // [theta_acc, v_enc, x_pos]

#include <stdint.h>
#include <math.h>
#include "tinyekf.h"

struct BalancerState {
    float theta;      // rad
    float thetaDot;   // rad/s
    float x;          // m
    float xDot;       // m/s
    float gyroBias;   // rad/s
};

struct BalancerEkfDiag {
    float innov_theta;
    float s_theta;
    float k_theta;
    float k_bias;
    float p00;
    float p01;
    float p11;
};

class BalancerEKF {
public:
    BalancerEKF();

    void begin();
    void reset(float theta_init, float pos_init);

    // Partial reset: only resets theta and position, preserves velocity and bias.
    // Use this instead of full reset when innovation gating triggers.
    void partialReset(float theta_init, float pos_init);

    // Run one EKF step using latest measurements; returns true on update success.
    // thetaMeasVar: optional override for theta measurement variance (use NAN for default)
    bool step(float thetaAcc, float vEnc, float posEnc, float gyroPitch, float dt,
              float thetaMeasVar = NAN);

    BalancerState getState() const;
    bool getDiag(BalancerEkfDiag& out) const;
    float getThetaMeasurementVarianceBase() const;
    void setInitialTheta(float theta_rad);
    void setInitialState(float theta_rad, float pos_m);

    // Check if we're in a post-reset damping period
    bool isInDampingPeriod() const { return damping_steps_remaining_ > 0; }

private:
    void initState();
    void initNoiseCovariances();

    ekf_t ekf_;
    float Q_[EKF_N * EKF_N];
    float R_[EKF_M * EKF_M];
    float lastGyroPitch_;
    float lastDt_;
    BalancerEkfDiag diag_;
    bool diag_valid_;
    float theta_r_base_;
    int damping_steps_remaining_;  // Post-reset damping countdown
};

#endif /* CONTROL_BALANCER_EKF_H */
