#include "MotionController.h"

#include <math.h>

MotionController::MotionController(const RobotParams& robotParams)
    : _robot(robotParams),
      _teleopForward(0.0f),
      _teleopTurn(0.0f),
      _targetVel(0.0f),
      _maxForwardVelocity(robotParams.maxForwardVelocity),
      _maxWheelTorque(robotParams.maxWheelTorque),
      _motorKt(robotParams.motorKt),
      _iqMax(0.0f),
      _velPid_iMax(0.0f),
      _velocityPid{},
      _pitchPid{},
      _velPid_Kp(VEL_PID_KP),
      _velPid_Ki(VEL_PID_KI),
      _velPid_Kd(VEL_PID_KD),
      _pitchPid_Kp(PITCH_PID_KP),
      _pitchPid_Ki(PITCH_PID_KI),
      _pitchPid_Kd(PITCH_PID_KD),
      _maxPitchTarget(MAX_PITCH_TARGET_RAD),
      _maxWheelVelocity(PARAM_MAX_WHEEL_VELOCITY),
      _maxTurnRate(PARAM_MAX_TURN_RATE),
      _velDampGain(VEL_DAMP_GAIN),
      _turnGain(TURN_TORQUE_GAIN),
      _yawDampGain(YAW_DAMP_GAIN),
      _yawBlendAlpha(YAW_BLEND_ALPHA),
      _velocitySlewRate(VELOCITY_SLEW_RATE_RAD_PER_S2)
{
}

void MotionController::setRobotParams(const RobotParams& params)
{
    _robot = params;
    _maxForwardVelocity = params.maxForwardVelocity;
    _maxWheelTorque = params.maxWheelTorque;
    _motorKt = params.motorKt;
}

void MotionController::setBalanceGains(const balance_gains_t& gains)
{
    _pitchPid_Kp = gains.Kp_theta;
    _pitchPid_Ki = 0.0f;
    _pitchPid_Kd = gains.Kd_theta;

    _velPid_Kp = gains.Kp_v_to_theta;
    _velPid_Ki = gains.Ki_v_to_theta;
    _velPid_Kd = 0.0f;

    _maxPitchTarget = gains.max_tilt_ref;
    _velDampGain = gains.Kv_damp;
    _turnGain = gains.K_turn;
    _yawDampGain = gains.K_yawDamp;
    _yawBlendAlpha = gains.alpha_yaw;
    if (_yawBlendAlpha < 0.0f) _yawBlendAlpha = 0.0f;
    if (_yawBlendAlpha > 1.0f) _yawBlendAlpha = 1.0f;

    _iqMax = gains.IqMax;
    _velPid_iMax = gains.iV_max;
}

void MotionController::setTeleopCommands(float forwardCmd, float turnCmd)
{
    if (forwardCmd > 1.0f) forwardCmd = 1.0f;
    if (forwardCmd < -1.0f) forwardCmd = -1.0f;
    if (turnCmd > 1.0f) turnCmd = 1.0f;
    if (turnCmd < -1.0f) turnCmd = -1.0f;

    _teleopForward = forwardCmd;
    _teleopTurn    = turnCmd;
    _targetVel = _maxForwardVelocity * _teleopForward;
    if (_targetVel > _maxForwardVelocity)  _targetVel = _maxForwardVelocity;
    if (_targetVel < -_maxForwardVelocity) _targetVel = -_maxForwardVelocity;
}

void MotionController::setTargetVelocity(float linearVelocityMps)
{
    _targetVel = linearVelocityMps;
    if (_targetVel > _maxForwardVelocity)  _targetVel = _maxForwardVelocity;
    if (_targetVel < -_maxForwardVelocity) _targetVel = -_maxForwardVelocity;
}

void MotionController::resetPidState()
{
    _velocityPid = PidState{};
    _pitchPid = PitchPidState{};
    _prevVelocityLeft = 0.0f;
    _prevVelocityRight = 0.0f;
    _lastPitchTarget = 0.0f;
    _lastPitchError = 0.0f;
    _lastPitchP = 0.0f;
    _lastPitchI = 0.0f;
    _lastPitchD = 0.0f;
    _lastGyroZ = 0.0f;
    _lastYawRateEnc = 0.0f;
    _lastYawRate = 0.0f;
}

void MotionController::setYawRates(float gyroZ, float yawRateEnc)
{
    _lastGyroZ = gyroZ;
    _lastYawRateEnc = yawRateEnc;
    _lastYawRate = _yawBlendAlpha * gyroZ + (1.0f - _yawBlendAlpha) * yawRateEnc;
}

float MotionController::computePid(PidState& state,
                                   float setpoint,
                                   float measurement,
                                   float dt,
                                   float Kp, float Ki, float Kd,
                                   float outputLimit,
                                   float integralLimit)
{
    if (dt <= 0.0f) dt = 1e-4f;

    float error = setpoint - measurement;

    float p = Kp * error;

    float d = 0.0f;
    if (state.initialized)
    {
        float derivativeRaw = -(measurement - state.prevMeasurement) / dt;
        state.derivativeFiltered = DERIVATIVE_FILTER_ALPHA * derivativeRaw
                                 + (1.0f - DERIVATIVE_FILTER_ALPHA) * state.derivativeFiltered;
        d = Kd * state.derivativeFiltered;
    }
    else
    {
        state.initialized = true;
        state.derivativeFiltered = 0.0f;
    }
    state.prevMeasurement = measurement;

    float outputPreSat = p + Ki * state.integral + d;

    bool saturatedHigh = outputPreSat >= outputLimit;
    bool saturatedLow = outputPreSat <= -outputLimit;
    bool integrationWouldHelp = (saturatedHigh && error < 0.0f) || (saturatedLow && error > 0.0f);

    if (!saturatedHigh && !saturatedLow)
    {
        state.integral += error * dt;
    }
    else if (integrationWouldHelp)
    {
        state.integral += error * dt;
    }

    if (state.integral > integralLimit) state.integral = integralLimit;
    if (state.integral < -integralLimit) state.integral = -integralLimit;

    float i = Ki * state.integral;

    float output = p + i + d;

    if (output > outputLimit) output = outputLimit;
    if (output < -outputLimit) output = -outputLimit;

    return output;
}

MotionController::Command
MotionController::computeControl(const StateEstimate& state, float dtSeconds)
{
    if (dtSeconds <= 0.0f)
    {
        dtSeconds = 1e-4f;
    }

    Command cmd{};
    cmd.iq = {0.0f, 0.0f};

    float pitchTarget = computePid(
        _velocityPid,
        _targetVel,
        state.xDot,
        dtSeconds,
        _velPid_Kp, _velPid_Ki, _velPid_Kd,
        _maxPitchTarget,
        (_velPid_Ki > 1e-6f && _velPid_iMax > 0.0f) ? (_velPid_iMax / _velPid_Ki)
                                                    : _maxPitchTarget
    );

    float pitchError = state.theta - pitchTarget;

    float p = _pitchPid_Kp * pitchError;
    float d = _pitchPid_Kd * state.thetaDot;

    float maxIq = 0.0f;
    float maxIqPhysical = 0.0f;
    if (_motorKt > 1e-6f) {
        maxIqPhysical = _maxWheelTorque / _motorKt;
    }
    if (_iqMax > 0.0f && maxIqPhysical > 0.0f) {
        maxIq = (maxIqPhysical < _iqMax) ? maxIqPhysical : _iqMax;
    } else if (_iqMax > 0.0f) {
        maxIq = _iqMax;
    } else if (maxIqPhysical > 0.0f) {
        maxIq = maxIqPhysical;
    }

    float outputLimit = (maxIq > 0.0f) ? maxIq : _maxWheelVelocity;

    float outputPreSat = p + _pitchPid_Ki * _pitchPid.integral + d;
    bool saturated = fabsf(outputPreSat) >= outputLimit;
    bool integrationWouldHelp = (outputPreSat > outputLimit && pitchError < 0.0f) ||
                                (outputPreSat < -outputLimit && pitchError > 0.0f);

    if (!saturated || integrationWouldHelp)
    {
        _pitchPid.integral += pitchError * dtSeconds;
    }
    float integralLimit = (_pitchPid_Ki > 1e-6f)
        ? outputLimit / _pitchPid_Ki
        : outputLimit;
    if (_pitchPid.integral > integralLimit) _pitchPid.integral = integralLimit;
    if (_pitchPid.integral < -integralLimit) _pitchPid.integral = -integralLimit;

    float i = _pitchPid_Ki * _pitchPid.integral;

    float uBal = p + i + d;

    _lastPitchError = pitchError;
    _lastPitchP = p;
    _lastPitchI = i;
    _lastPitchD = d;

    float uCommon = uBal - _velDampGain * state.xDot;
    float uTurn = _turnGain * _teleopTurn - _yawDampGain * _lastYawRate;

    ControlOutput out;
    out.iqLeft = uCommon - uTurn;
    out.iqRight = uCommon + uTurn;

    if (maxIq > 0.0f) {
        if (out.iqLeft > maxIq) out.iqLeft = maxIq;
        if (out.iqLeft < -maxIq) out.iqLeft = -maxIq;
        if (out.iqRight > maxIq) out.iqRight = maxIq;
        if (out.iqRight < -maxIq) out.iqRight = -maxIq;
    }

    _lastPitchTarget = pitchTarget;

    cmd.iq = out;
    return cmd;
}

MotionController::ControlOutput
MotionController::applyVelocitySlew(ControlOutput desired, float dt)
{
    if (dt <= 0.0f) dt = 1e-3f;
    float maxStep = _velocitySlewRate * dt;

    ControlOutput out;
    float deltaLeft = desired.iqLeft - _prevVelocityLeft;
    if (deltaLeft > maxStep) deltaLeft = maxStep;
    if (deltaLeft < -maxStep) deltaLeft = -maxStep;
    out.iqLeft = _prevVelocityLeft + deltaLeft;

    float deltaRight = desired.iqRight - _prevVelocityRight;
    if (deltaRight > maxStep) deltaRight = maxStep;
    if (deltaRight < -maxStep) deltaRight = -maxStep;
    out.iqRight = _prevVelocityRight + deltaRight;

    _prevVelocityLeft = out.iqLeft;
    _prevVelocityRight = out.iqRight;
    return out;
}
