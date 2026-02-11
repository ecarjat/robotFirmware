#ifndef CONTROL_MOTION_CONTROLLER_H
#define CONTROL_MOTION_CONTROLLER_H

#include "RobotParams.h"
#include "StateEstimate.h"
#include "config_control.h"
#include "param_storage.h"

/**
 * @brief Inner longitudinal loop mode selector
 */
enum class InnerLongMode {
    PID = 0,  /**< Cascaded PID inner loop (baseline) */
    LQR = 1,  /**< LQR inner loop */
};

/**
 * @brief Inner controller diagnostics for LQR mode
 */
struct InnerCtrlDiag {
    float u_sum_cmd;      /**< Final blended u_sum output (Nm) */
    float u_sum_pid;      /**< PID-computed u_sum before blend (Nm) */
    float u_sum_lqr;      /**< LQR-computed u_sum before blend (Nm) */
    float u_diff_cmd;     /**< Yaw loop differential output (Nm) */
    float alpha;          /**< Current blend factor (0=PID, 1=LQR) */
    float theta_ref_pos;  /**< Position-induced theta reference (rad) */
    float x_err;          /**< Position error (m) */
    bool sat_left;        /**< Left wheel saturated flag */
    bool sat_right;       /**< Right wheel saturated flag */
    bool fallback_to_pid; /**< Forced fallback to PID due to safety */
    bool cross_antiwindup_active; /**< Cross-controller anti-windup engaged */
    InnerLongMode requested_mode;
    InnerLongMode active_mode;
};

class MotionController {
public:
    struct ControlOutput {
        float torqueLeftNm;
        float torqueRightNm;
    };

    struct Command {
        ControlOutput torque{0.0f, 0.0f};
    };

    explicit MotionController(const RobotParams& robotParams);

    void setRobotParams(const RobotParams& params);
    void setBalanceGains(const balance_gains_t& gains);
    void setLqrParams(const lqr_params_t& lqr);
    void setLqrEquilibrium(float thetaRef, float uEq);
    void setControlDt(float dtSeconds);

    void setTeleopCommands(float forwardCmd, float turnCmd);
    void setTargetVelocity(float linearVelocityMps);
    void setYawRates(float gyroZ, float yawRateEnc);

    /* LQR mode control */
    void setRequestedMode(InnerLongMode mode);
    InnerLongMode getRequestedMode() const { return _requestedMode; }
    InnerLongMode getActiveMode() const { return _activeMode; }

    float getTeleopForward() const { return _teleopForward; }
    float getTeleopTurn() const { return _teleopTurn; }
    float getLastPitchTarget() const { return _lastPitchTarget; }
    float getLastYawRate() const { return _lastYawRate; }
    float getLastYawRateEnc() const { return _lastYawRateEnc; }
    float getLastGyroZ() const { return _lastGyroZ; }

    float getLastPitchError() const { return _lastPitchError; }
    float getLastPitchP() const { return _lastPitchP; }
    float getLastPitchI() const { return _lastPitchI; }
    float getLastPitchD() const { return _lastPitchD; }

    bool isOutputSaturated() const { return _lastSaturated; }
    bool getInnerCtrlDiag(InnerCtrlDiag& diag) const;

    Command computeControl(const StateEstimate& state, float dtSeconds);

    void resetPidState();

#ifdef UNIT_TEST
    float test_computePid(float setpoint,
                          float measurement,
                          float dt,
                          float Kp, float Ki, float Kd,
                          float outputLimit,
                          float integralLimit)
    {
        return computePid(_velocityPid, setpoint, measurement, dt,
                          Kp, Ki, Kd, outputLimit, integralLimit);
    }

    ControlOutput test_applyVelocitySlew(ControlOutput desired, float dt)
    {
        return applyVelocitySlew(desired, dt);
    }

    float test_computeLqrUSumNm(const StateEstimate& state, float vRef, float thetaRef)
    {
        return computeLqrUSumNm(state, vRef, thetaRef);
    }
#endif

private:
    struct PidState {
        float integral = 0.0f;
        float prevMeasurement = 0.0f;
        float derivativeFiltered = 0.0f;
        bool initialized = false;
    };

    struct PitchPidState {
        float integral = 0.0f;
    };

    float computePid(PidState& state,
                     float setpoint,
                     float measurement,
                     float dt,
                     float Kp, float Ki, float Kd,
                     float outputLimit,
                     float integralLimit);

    ControlOutput applyVelocitySlew(ControlOutput desired, float dt);

    RobotParams _robot;

    float _teleopForward;
    float _teleopTurn;
    float _targetVel;

    float _maxForwardVelocity;
    float _maxWheelTorque;
    float _motorKt;
    float _iqMax;
    float _velPid_iMax;

    PidState _velocityPid;
    PitchPidState _pitchPid;

    float _velPid_Kp;
    float _velPid_Ki;
    float _velPid_Kd;

    float _pitchPid_Kp;
    float _pitchPid_Ki;
    float _pitchPid_Kd;

    float _maxPitchTarget;
    float _maxWheelVelocity;
    float _maxTurnRate;
    float _velDampGain;
    float _turnGain;
    float _yawDampGain;
    float _yawBlendAlpha;
    float _controlDt;

    float _prevVelocityLeft = 0.0f;
    float _prevVelocityRight = 0.0f;
    float _velocitySlewRate;

    float _lastGyroZ = 0.0f;
    float _lastYawRateEnc = 0.0f;
    float _lastYawRate = 0.0f;

    float _lastPitchTarget = 0.0f;
    float _lastPitchError = 0.0f;
    float _lastPitchP = 0.0f;
    float _lastPitchI = 0.0f;
    float _lastPitchD = 0.0f;
    bool _lastSaturated = false;

    /* LQR inner loop state */
    lqr_params_t _lqr;
    InnerLongMode _requestedMode = InnerLongMode::PID;
    InnerLongMode _activeMode = InnerLongMode::PID;
    float _alpha = 0.0f;  /* Blend factor: 0=PID, 1=LQR */
    float _prevUSum = 0.0f;
    bool _prevUSumValid = false;
    float _xRef = 0.0f;
    bool _xRefValid = false;
    float _thetaRef = 0.0f;
    float _uEq = 0.0f;
    float _thetaRefFromPos = 0.0f;  /* Position-induced theta ref (cascaded) */
    float _lastXErr = 0.0f;         /* Last position error for diagnostics */
    InnerCtrlDiag _diag = {};
    bool _diagValid = false;

    float computeLqrUSumNm(const StateEstimate& state, float vRef, float thetaRef);
    void updateBlendAlpha(float dt);

    static constexpr float DERIVATIVE_FILTER_ALPHA = 0.1f;
};

#endif /* CONTROL_MOTION_CONTROLLER_H */
