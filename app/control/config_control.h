#ifndef CONTROL_CONFIG_CONTROL_H
#define CONTROL_CONFIG_CONTROL_H

/* Control loop timing (legacy defaults; adjust during integration). */
#define CONTROL_DT 0.005f
#define IMU_DT     0.005f
#define CONTROL_DEFAULT_HZ 400.0f

/* EKF / estimator timing guard */
#define EKF_MAX_DT 0.05f

/* Limits / safety thresholds */
#define MAX_TILT_ANGLE_RAD 1.5f
#define MIN_FRONT_DISTANCE 0.25f
#define MIN_REAR_DISTANCE  0.25f

#ifndef EKF_TUNING_LOG
#define EKF_TUNING_LOG 0
#endif
#ifndef EKF_TUNE_LOG_DECIM
#define EKF_TUNE_LOG_DECIM 1
#endif
#ifndef EKF_ADAPTIVE_R
#define EKF_ADAPTIVE_R 1
#endif

/* EKF innovation gating thresholds */
#define EKF_INNOV_THETA_MAX_RAD    0.5f
#define EKF_INNOV_VEL_MAX_MPS      1.5f
#define EKF_INNOV_POS_MAX_M        0.5f
#define EKF_INNOV_YAW_RATE_MAX_RPS 1.0f  /* Yaw rate innovation gate (rad/s) */

/* EKF init parameters */
#define EKF_INIT_ACCEL_SAMPLES   50
#define EKF_INIT_ACCEL_DELAY_MS  2.0f
#define EKF_GRACE_MS             50U

/* EKF noise tuning */
#define EKF_Q_THETA      1e-4f
#define EKF_Q_THETA_DOT  5e-3f
#define EKF_Q_X          1e-2f
#define EKF_Q_X_DOT      5e-2f
#define EKF_Q_BIAS       1e-5f
#define EKF_Q_YAW        1e-3f   /* Yaw angle process noise */
#define EKF_Q_YAW_BIAS   1e-6f   /* Yaw gyro bias random walk */

#define EKF_R_THETA_ACC     4e-3f
#define EKF_R_V_ENC         5e-2f
#define EKF_R_X_POS         1e-3f
#define EKF_R_YAW_RATE_ENC  1e-2f  /* Encoder differential yaw rate noise */

#define EKF_P0_THETA     0.05f
#define EKF_P0_THETA_DOT 0.05f
#define EKF_P0_X         0.2f
#define EKF_P0_X_DOT     0.2f
#define EKF_P0_BIAS      0.01f
#define EKF_P0_YAW       0.1f    /* Initial yaw uncertainty (rad^2) */
#define EKF_P0_YAW_BIAS  0.01f   /* Initial yaw bias uncertainty */

#define EKF_TUNE_GYRO_STILL_THRESH_RAD_S 0.0174533f
#define EKF_TUNE_ANORM_STILL_THRESH_G    0.12f
#define EKF_TUNE_ANORM_GATE_THRESH_G     0.18f
#define EKF_TUNE_R_MULT                  20.0f

/* IMU configuration (design defaults). */
#define IMU_GYRO_ODR_HZ 800.0f
#define IMU_ACCEL_ODR_HZ 400.0f
#define IMU_SAMPLE_MAX_AGE_MS 5U

/* Dual-IMU disagreement thresholds (stage-1 defaults). */
#define IMU_GYRO_DISAGREE_WARN_DPS 30.0f
#define IMU_GYRO_DISAGREE_FAULT_DPS 60.0f
#define IMU_ACC_ANGLE_WARN_DEG 7.0f
#define IMU_ACC_ANGLE_FAULT_DEG 12.0f

/* Vibration gating (stage-1 defaults).
 * VIB_WINDOW_MS = 100 ms, ACCEL_ODR = 400 Hz → 40 samples */
#define IMU_VIB_WINDOW_SAMPLES 40U
#define IMU_VIB_ON_G 0.06f
#define IMU_VIB_OFF_G 0.04f

/* IMU switching timing (stage-1 defaults). */
#define IMU_SWITCH_TO_SECONDARY_MS 100U
#define IMU_SWITCH_BACK_MS 500U
#define IMU_SWITCH_DWELL_MS 500U

/* Mode / safety thresholds (stage-1 defaults). */
#define MOTION_ARM_UPRIGHT_RAD 0.174533f
#define IMU_FAULT_FALLEN_MS 200U
#define IMU_FAULT_FATAL_MS 500U
#define MOTOR_LINK_FAULT_FALLEN_MS 100U
#define MOTOR_LINK_FAULT_FATAL_MS 500U

/* Yaw damping blend. */
#define YAW_BLEND_ALPHA 0.8f
#define VEL_DAMP_GAIN 0.0f
#define TURN_TORQUE_GAIN PARAM_MAX_TURN_TORQUE
#define YAW_DAMP_GAIN 0.0f

/* Balance defaults (stage-1). */
#define BALANCE_DEFAULT_KP_THETA 2.0f
#define BALANCE_DEFAULT_KD_THETA 1.0f
#define BALANCE_DEFAULT_KP_V_TO_THETA 0.5f
#define BALANCE_DEFAULT_KI_V_TO_THETA 0.1f
#define BALANCE_DEFAULT_MAX_TILT_REF 0.15f
#define BALANCE_DEFAULT_KV_DAMP 0.1f
#define BALANCE_DEFAULT_K_TURN 0.5f
#define BALANCE_DEFAULT_K_YAW_DAMP 0.2f
#define BALANCE_DEFAULT_ALPHA_YAW 0.8f
#define BALANCE_DEFAULT_IQ_MAX 3.0f
#define BALANCE_DEFAULT_THETA_KILL 0.785f
#define BALANCE_DEFAULT_IV_MAX 0.5f

/* Cascaded PID gains */
#define VEL_PID_KP 0.0f
#define VEL_PID_KI 0.0f
#define VEL_PID_KD 0.0f

#define PITCH_PID_KP 2.0f
#define PITCH_PID_KI 0.0f
#define PITCH_PID_KD 1.0f

#define MAX_PITCH_TARGET_RAD 0.15f

/* Physical parameters (defaults from legacy). */
#define PARAM_WHEEL_RADIUS 0.083f
#define PARAM_WHEEL_BASE   0.1204f
#define PARAM_BODY_MASS    0.43f
#define PARAM_WHEEL_MASS   0.111f
#define PARAM_COM_HEIGHT   0.036f
#define PARAM_BODY_INERTIA (PARAM_BODY_MASS * PARAM_COM_HEIGHT * PARAM_COM_HEIGHT)
#define PARAM_MOTOR_KT         0.0735f
#define PARAM_MOTOR_RESISTANCE 8.0f
#define PARAM_GEAR_RATIO       1.0f
#define PARAM_MAX_VOLTAGE      5.0f
#define PARAM_GRAVITY          9.81f
#define PARAM_MAX_WHEEL_TORQUE 6.0f
#define PARAM_MAX_FORWARD_VEL  0.6f
#define PARAM_MAX_TURN_TORQUE  0.6f

#define PARAM_MAX_WHEEL_VELOCITY (PARAM_MAX_FORWARD_VEL / PARAM_WHEEL_RADIUS)
#define VELOCITY_SLEW_RATE_RAD_PER_S2 20.0f
#define PARAM_MAX_TURN_RATE 2.0f

/* ======== LQR Direct Full-State Controller ======== */

/* Direct full-state feedback:
 *   u = -(K[0]*x_err + K[1]*v_err + K[2]*theta_err + K[3]*thetaDot)
 *
 * K[2]/K[3] deliberately reduced ~10x from original LQR (which had
 * K[2]=-4187.5, K[3]=-165.1) to avoid bang-bang saturation at ±5 Nm.
 * This leaves torque headroom for velocity feedback (K[1]).
 *
 * At K[2]=-400: pitch of 0.01 rad uses 4 Nm, leaving 1 Nm for velocity.
 * At K[1]=2: velocity of 0.5 m/s uses 1 Nm of braking torque.
 */
#define LQR_K0_X         (-0.2f)    /* Position gain — NEGATIVE: lean back when drifting forward */
#define LQR_K1_V         (-1.5f)    /* Velocity gain — NEGATIVE: forward torque when moving forward → lean back → decelerate */
#define LQR_K2_THETA     (-400.0f)  /* Pitch angle gain (Nm/rad) — ~10x smaller to avoid saturation */
#define LQR_K3_THETADOT  (-20.0f)   /* Pitch rate gain (Nm/(rad/s)) — ~10x smaller */

/* LQR limits */
#define LQR_U_LIMIT            5.0f  /* Max |u_sum| (Nm) */
#define LQR_DU_LIMIT           10.0f   /* Max |du_sum| per second (Nm/s) */
#define LQR_THETA_REF_LIMIT    0.01f    /* Max |theta_ref| from vel loop (rad, ~0.57°) */
#define LQR_V_REF_LIMIT        2.0f     /* Max |v_ref| from position loop (m/s) */

/* Mode switching ramp times */
#define LQR_ENGAGE_RAMP_MS     200U     /* PID→LQR blend time (ms) */
#define LQR_DISENGAGE_RAMP_MS  100U     /* LQR→PID blend time (ms) */

/* Estimator staleness threshold for LQR fallback */
#define EST_MAX_AGE_MS         10U      /* Max estimator age before fallback (ms) */

/* IMU staleness tolerances (consecutive stale samples allowed). */
#define IMU_STALE_ALLOWED_NORMAL        50
#define IMU_STALE_ALLOWED_WHILE_ENABLING 1000

#endif /* CONTROL_CONFIG_CONTROL_H */
