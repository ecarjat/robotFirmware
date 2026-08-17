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
    float cruise_alpha;   /**< Cruise scheduler blend (0=rest, 1=cruise) */
    float k0_eff;         /**< Effective K0 after cruise scheduling */
    float v_ref_limit_eff;    /**< Effective v_ref limit after scheduling (m/s) */
    float theta_ref_limit_eff; /**< Effective theta_ref limit after scheduling (rad) */
    float yaw_damp_eff;       /**< Effective yaw damping gain */
    bool sat_left;        /**< Left wheel saturated flag */
    bool sat_right;       /**< Right wheel saturated flag */
    bool cruise_mode;     /**< Cruise scheduler target mode */
    bool stop_mode_active; /**< Deceleration-aware stop mode active */
    bool fallback_to_pid; /**< Forced fallback to PID due to safety */
    bool cross_antiwindup_active; /**< Cross-controller anti-windup engaged */
    InnerLongMode requested_mode;
    InnerLongMode active_mode;
};

struct LqrSpeedSchedule {
    bool enabled = (LQR_SPEED_SCHED_ENABLE != 0);
    float cruise_enter_cmd_mps = LQR_CRUISE_ENTER_CMD_MPS;
    float cruise_exit_cmd_mps = LQR_CRUISE_EXIT_CMD_MPS;
    float cruise_enter_meas_mps = LQR_CRUISE_ENTER_MEAS_MPS;
    float cruise_exit_meas_mps = LQR_CRUISE_EXIT_MEAS_MPS;
    float cruise_blend_tau_s = LQR_CRUISE_BLEND_TAU_S;
    float v_ref_margin_mps = LQR_VREF_MARGIN_MPS;
    float theta_ref_limit_rest_cap_rad = LQR_THETA_LIMIT_REST_CAP_RAD;
    float theta_ref_limit_cruise_cap_rad = LQR_THETA_LIMIT_CRUISE_CAP_RAD;
    float yaw_damp_cruise_mult = LQR_YAW_DAMP_CRUISE_MULT;
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
    void setLqrSpeedSchedule(const LqrSpeedSchedule& schedule);
    LqrSpeedSchedule getLqrSpeedSchedule() const { return _speedSchedule; }
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
    LqrSpeedSchedule _speedSchedule{};
    InnerLongMode _requestedMode = InnerLongMode::PID;
    InnerLongMode _activeMode = InnerLongMode::PID;
    float _alpha = 0.0f;  /* Blend factor: 0=PID, 1=LQR */
    bool _cruiseMode = false;
    float _cruiseAlpha = 0.0f;
    float _prevUSum = 0.0f;
    bool _prevUSumValid = false;
    float _xRef = 0.0f;
    bool _xRefValid = false;
    float _thetaRef = 0.0f;
    float _uEq = 0.0f;
    float _thetaRefFromPos = 0.0f;  /* Position-induced theta ref (cascaded) */
    float _lastXErr = 0.0f;         /* Last position error for diagnostics */
    float _velIntegral = 0.0f;      /* Velocity error integral for LQI (unused) */
    
    /* Slow trim loop: integrates velocity error into pitch reference */
    float _thetaTrim = 0.0f;        /* Pitch trim from velocity integrator (rad) */
    float _k0Eff = 0.0f;            /* Effective K0 after scheduling */
    float _vRefLimitEff = 0.0f;     /* Effective v_ref limit after scheduling */
    float _thetaRefLimitEff = 0.0f; /* Effective theta_ref limit after scheduling */
    float _yawDampEff = 0.0f;       /* Effective yaw damping gain after scheduling */
    bool _turnSuppressActive = false; /* Hysteresis latch for turn-aware K0 suppression */
    bool _turnInPlaceActive = false;  /* In-place turn mode (freeze xRef/thetaTrim integration) */
    float _turnK0Scale = 1.0f;        /* Additional K0 scale during turning */
    bool _stopModeActive = false;   /* Deceleration-aware stop mode after release */
    bool _stopModeArmed = false;    /* True after nontrivial cmd, allows ramped-release entry */
    float _stopModeVRef = 0.0f;     /* Stop-mode velocity reference ramping to zero */
    float _prevTargetVelCmd = 0.0f; /* Previous user-commanded target velocity */
    
    InnerCtrlDiag _diag = {};
    bool _diagValid = false;

    float computeLqrUSumNm(const StateEstimate& state, float vRef, float thetaRef);
    bool updateStopMode(float xDot, float dt);
    void updateCruiseSchedule(float cmdVel, float xDot, float dt);
    void updateBlendAlpha(float dt);

    static constexpr float DERIVATIVE_FILTER_ALPHA = 0.1f;
};

#endif /* CONTROL_MOTION_CONTROLLER_H */
