#ifndef CONTROL_MOTION_CONTROLLER_H
#define CONTROL_MOTION_CONTROLLER_H

#include "RobotParams.h"
#include "StateEstimate.h"
#include "config_control.h"
#include "param_storage.h"

class MotionController {
public:
    struct ControlOutput {
        float iqLeft;
        float iqRight;
    };

    struct Command {
        ControlOutput iq{0.0f, 0.0f};
    };

    explicit MotionController(const RobotParams& robotParams);

    void setRobotParams(const RobotParams& params);
    void setBalanceGains(const balance_gains_t& gains);

    void setTeleopCommands(float forwardCmd, float turnCmd);
    void setTargetVelocity(float linearVelocityMps);
    void setYawRates(float gyroZ, float yawRateEnc);

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

    Command computeControl(const StateEstimate& state, float dtSeconds);

    void resetPidState();

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

    static constexpr float DERIVATIVE_FILTER_ALPHA = 0.1f;
};

#endif /* CONTROL_MOTION_CONTROLLER_H */
