#include "MotionController.h"
#include "app_log_macros.h"
#include "config_control.h"

#include <math.h>
#include <string.h>

/* Threshold for detecting near-zero gain values to avoid division by zero.
 * Gains smaller than this are treated as disabled (effectively zero). */
static constexpr float kGainEpsilon = 1e-6f;

/* Default integral limit when Ki is disabled or iMax not configured.
 * Uses a large but finite value to prevent unbounded windup. */
static constexpr float kDefaultIntegralLimit = 1000.0f;

namespace {

inline float clampf(float val, float limit)
{
    if (val > limit) return limit;
    if (val < -limit) return -limit;
    return val;
}

}  /* anonymous namespace */

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
      _controlDt(CONTROL_DT),
      _velocitySlewRate(VELOCITY_SLEW_RATE_RAD_PER_S2),
      _lqr{},
      _requestedMode(InnerLongMode::PID),
      _activeMode(InnerLongMode::PID),
      _alpha(0.0f),
      _prevUSum(0.0f),
      _prevUSumValid(false),
      _diag{},
      _diagValid(false)
{
    /* Initialize LQR with defaults */
    _lqr.K[0] = LQR_K0_X;
    _lqr.K[1] = LQR_K1_V;
    _lqr.K[2] = LQR_K2_THETA;
    _lqr.K[3] = LQR_K3_THETADOT;
    _lqr.u_limit = LQR_U_LIMIT;
    _lqr.du_limit = LQR_DU_LIMIT;
    _lqr.theta_ref_limit = LQR_THETA_REF_LIMIT;
    _lqr.v_ref_limit = LQR_V_REF_LIMIT;
    _lqr.engage_ramp_ms = LQR_ENGAGE_RAMP_MS;
    _lqr.disengage_ramp_ms = LQR_DISENGAGE_RAMP_MS;
    _lqr.default_mode = 0;
}

void MotionController::setRobotParams(const RobotParams& params)
{
    _robot = params;
    _maxForwardVelocity = params.maxForwardVelocity;
    _maxWheelTorque = params.maxWheelTorque;

    /* Validate motorKt to prevent division by zero and ensure valid torque calculations.
     * A zero or negative motorKt would cause maxIqPhysical calculation to fail. */
    if (params.motorKt > kGainEpsilon) {
        _motorKt = params.motorKt;
    } else {
        APP_LOG_WARN("Invalid motorKt=%.4f, using default %.4f",
                     (double)params.motorKt, (double)PARAM_MOTOR_KT);
        _motorKt = PARAM_MOTOR_KT;
    }
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

void MotionController::setLqrParams(const lqr_params_t& lqr)
{
    /* Only log when gains actually change (avoids spamming every LUT update) */
    bool changed = false;
    for (int i = 0; i < 4; ++i) {
        if (_lqr.K[i] != lqr.K[i]) { changed = true; break; }
    }
    if (_lqr.u_limit != lqr.u_limit) changed = true;

    _lqr = lqr;

    if (changed) {
        APP_LOG_INFO("LQR params: K=[%.2f,%.2f,%.2f,%.2f] u_lim=%.1f",
                     (double)lqr.K[0], (double)lqr.K[1],
                     (double)lqr.K[2], (double)lqr.K[3],
                     (double)lqr.u_limit);
    }
}

void MotionController::setLqrEquilibrium(float thetaRef, float uEq)
{
    _thetaRef = thetaRef;
    _uEq = uEq;
}

void MotionController::setRequestedMode(InnerLongMode mode)
{
    if (mode != _requestedMode) {
        InnerLongMode old_mode = _requestedMode;
        _requestedMode = mode;
        APP_LOG_INFO("Inner mode: %s -> %s (ramp=%u ms)",
                     old_mode == InnerLongMode::LQR ? "LQR" : "PID",
                     mode == InnerLongMode::LQR ? "LQR" : "PID",
                     mode == InnerLongMode::LQR ? _lqr.engage_ramp_ms
                                                : _lqr.disengage_ramp_ms);
    }
}

void MotionController::updateBlendAlpha(float dt)
{
    float target_alpha = (_requestedMode == InnerLongMode::LQR) ? 1.0f : 0.0f;

    if (_alpha < target_alpha) {
        /* Ramping toward LQR */
        float ramp_time = _lqr.engage_ramp_ms * 0.001f;
        if (ramp_time > 0.0f) {
            float step = dt / ramp_time;
            _alpha += step;
            if (_alpha > target_alpha) _alpha = target_alpha;
        } else {
            _alpha = target_alpha;
        }
    } else if (_alpha > target_alpha) {
        /* Ramping toward PID */
        float ramp_time = _lqr.disengage_ramp_ms * 0.001f;
        if (ramp_time > 0.0f) {
            float step = dt / ramp_time;
            _alpha -= step;
            if (_alpha < target_alpha) _alpha = target_alpha;
        } else {
            _alpha = target_alpha;
        }
    }

    /* Update active mode based on blend position */
    if (_alpha >= 1.0f) {
        _activeMode = InnerLongMode::LQR;
    } else if (_alpha <= 0.0f) {
        _activeMode = InnerLongMode::PID;
    }
}

float MotionController::computeLqrUSumNm(const StateEstimate& state, float vRef, float thetaRef)
{
    /* Direct full-state feedback:
     *
     *   u = -(K[0]*x_err + K[1]*v_err + K[2]*theta_err + K[3]*thetaDot)
     *
     * K[2]/K[3] are reduced ~10x from original LQR so the controller doesn't
     * bang-bang on pitch alone, leaving torque budget for velocity feedback.
     *
     * K[0]: position (disabled when 0)
     * K[1]: velocity feedback, direct to torque
     * K[2]: pitch angle (negative — forward lean → positive torque)
     * K[3]: pitch rate (negative — damping)
     */
    float x_err = state.x - _xRef;
    float v_err = state.xDot - vRef;
    float theta_err = state.theta - thetaRef;
    float thetaDot = state.thetaDot;

    /* Store for diagnostics */
    _lastXErr = x_err;
    _thetaRefFromPos = 0.0f;

    float u_sum = 0.0f;

    /* Position term (optional, clamped to avoid dominating torque budget) */
    if (fabsf(_lqr.K[0]) > kGainEpsilon) {
        float u_pos = -_lqr.K[0] * x_err;
        /* Clamp position contribution to leave budget for pitch and velocity */
        float pos_limit = _lqr.u_limit * 0.4f;  /* max 40% of torque budget */
        u_sum += clampf(u_pos, pos_limit);
    }

    /* Velocity term (unclamped — needs full authority to lean for braking) */
    if (fabsf(_lqr.K[1]) > kGainEpsilon) {
        u_sum -= _lqr.K[1] * v_err;
    }

    /* Pitch terms (unclamped — balance is priority) */
    u_sum -= _lqr.K[2] * theta_err + _lqr.K[3] * thetaDot;

    return u_sum;
}

bool MotionController::getInnerCtrlDiag(InnerCtrlDiag& diag) const
{
    if (!_diagValid) {
        return false;
    }
    diag = _diag;
    return true;
}

void MotionController::setControlDt(float dtSeconds)
{
    if (dtSeconds > 0.0f) {
        _controlDt = dtSeconds;
    }
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
    _xRef = 0.0f;
    _xRefValid = false;
    _thetaRef = 0.0f;
    _uEq = 0.0f;

    /* Reset LQR state */
    _alpha = (_requestedMode == InnerLongMode::LQR) ? 1.0f : 0.0f;
    _activeMode = _requestedMode;
    _prevUSum = 0.0f;
    _prevUSumValid = false;
    _velIntegral = 0.0f;
    _diag = InnerCtrlDiag{};
    _diagValid = false;
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
    if (dt <= 0.0f) dt = _controlDt;

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
        dtSeconds = _controlDt;
    }

    Command cmd{};
    cmd.torque = {0.0f, 0.0f};

    /* ========== PID u_sum computation (always computed for blending) ========== */

    /* Compute velocity PID integral limit:
     * - If Ki and iMax are configured, limit = iMax/Ki (in velocity units)
     * - Cap at kDefaultIntegralLimit to prevent huge limits with tiny Ki
     * - Otherwise use a large default to prevent unbounded windup */
    float velIntegralLimit = kDefaultIntegralLimit;
    if ((_velPid_Ki >= kGainEpsilon) && (_velPid_iMax > 0.0f)) {
        float computed = _velPid_iMax / _velPid_Ki;
        velIntegralLimit = (computed < kDefaultIntegralLimit) ? computed : kDefaultIntegralLimit;
    }

    float pitchTarget = computePid(
        _velocityPid,
        _targetVel,
        state.xDot,
        dtSeconds,
        _velPid_Kp, _velPid_Ki, _velPid_Kd,
        _maxPitchTarget,
        velIntegralLimit
    );

    float pitchError = state.theta - pitchTarget;

    float p = _pitchPid_Kp * pitchError;
    float d = _pitchPid_Kd * state.thetaDot;

    float maxIq = 0.0f;
    float maxIqPhysical = 0.0f;
    if (_motorKt > kGainEpsilon) {
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
    /* Pitch PID integral limit: if Ki is configured, limit in pitch error units;
     * Cap at kDefaultIntegralLimit to prevent huge limits with tiny Ki;
     * otherwise use outputLimit directly (effectively disabling anti-windup) */
    float integralLimit = kDefaultIntegralLimit;
    if (_pitchPid_Ki >= kGainEpsilon) {
        float computed = outputLimit / _pitchPid_Ki;
        integralLimit = (computed < kDefaultIntegralLimit) ? computed : kDefaultIntegralLimit;
    }
    if (_pitchPid.integral > integralLimit) _pitchPid.integral = integralLimit;
    if (_pitchPid.integral < -integralLimit) _pitchPid.integral = -integralLimit;

    float i = _pitchPid_Ki * _pitchPid.integral;

    float uBal = p + i + d;

    _lastPitchError = pitchError;
    _lastPitchP = p;
    _lastPitchI = i;
    _lastPitchD = d;

    /* ========== Cross-controller anti-windup ========== */
    /*
     * When the inner (pitch) loop saturates, the outer (velocity) loop's
     * integral can wind up because it doesn't know the system can't achieve
     * the requested pitch. We detect this condition and reduce the velocity
     * integral to prevent excessive windup.
     *
     * Condition: pitch output saturated AND velocity error would push
     * pitchTarget further in the saturating direction.
     *
     * - If uBal is saturated positive (robot falling forward, needs more torque)
     *   and velocity error is negative (going slower than target, pitchTarget
     *   would increase to speed up), then velocity integral is making things worse.
     * - If uBal is saturated negative and velocity error is positive, same issue.
     */
    bool crossAntiWindupActive = false;
    if (saturated)
    {
        float velocityError = _targetVel - state.xDot;
        bool velIntegralMakesWorse = false;

        if (outputPreSat > outputLimit && velocityError < 0.0f)
        {
            /* Saturated positive, velocity error wants more pitch (to accelerate) */
            velIntegralMakesWorse = true;
        }
        else if (outputPreSat < -outputLimit && velocityError > 0.0f)
        {
            /* Saturated negative, velocity error wants more pitch (to decelerate) */
            velIntegralMakesWorse = true;
        }

        if (velIntegralMakesWorse)
        {
            /* Apply decay to velocity integral to help recovery.
             * Time constant ~0.5s means integral decays to 37% in 0.5s.
             * This is aggressive enough to prevent sustained windup while
             * being smooth enough to avoid control discontinuities. */
            constexpr float kCrossAntiWindupTau = 0.5f;  /* seconds */
            float decayFactor = expf(-dtSeconds / kCrossAntiWindupTau);
            _velocityPid.integral *= decayFactor;
            crossAntiWindupActive = true;
        }
    }

    float uSumPid = uBal - _velDampGain * state.xDot;

    /* ========== LQR u_sum computation ========== */

    if (!_xRefValid) {
        _xRef = state.x;
        _xRefValid = true;
    } else {
        // Track position reference when a non-zero velocity command is present.
        if (fabsf(_targetVel) > 1e-4f) {
            _xRef += _targetVel * dtSeconds;
        }
    }

    float thetaRef = _thetaRef;
    if (_lqr.theta_ref_limit > 0.0f) {
        thetaRef = clampf(thetaRef, _lqr.theta_ref_limit);
    }
    /* LQR uses theta_ref from equilibrium LUT and v_ref=_targetVel */
    float uSumLqrNm = computeLqrUSumNm(state, _targetVel, thetaRef) + _uEq;
    float uSumPidNm = 0.0f;
    if (_motorKt > kGainEpsilon) {
        uSumPidNm = uSumPid * _motorKt;
    }

    /* ========== Blend PID and LQR based on alpha ========== */

    updateBlendAlpha(dtSeconds);

    float uSum = (1.0f - _alpha) * uSumPidNm + _alpha * uSumLqrNm;

    /* Rate limiting on u_sum */
    if (_prevUSumValid && _lqr.du_limit > 0.0f) {
        float maxDelta = _lqr.du_limit * dtSeconds;
        float delta = uSum - _prevUSum;
        delta = clampf(delta, maxDelta);
        uSum = _prevUSum + delta;
    }
    _prevUSum = uSum;
    _prevUSumValid = true;

    /* ========== Yaw differential (unchanged) ========== */

    float uTurnIq = _turnGain * _teleopTurn - _yawDampGain * _lastYawRate;
    float uTurn = (_motorKt > kGainEpsilon) ? (uTurnIq * _motorKt) : 0.0f;

    /* ========== Mix u_sum and u_diff into wheel commands ========== */

    ControlOutput out;
    out.torqueLeftNm = uSum - uTurn;
    out.torqueRightNm = uSum + uTurn;

    float maxTorque = 0.0f;
    float maxTorqueIq = 0.0f;
    if (_motorKt > kGainEpsilon && _iqMax > 0.0f) {
        maxTorqueIq = _iqMax * _motorKt;
    }
    if (_maxWheelTorque > 0.0f && maxTorqueIq > 0.0f) {
        maxTorque = (maxTorqueIq < _maxWheelTorque) ? maxTorqueIq : _maxWheelTorque;
    } else if (_maxWheelTorque > 0.0f) {
        maxTorque = _maxWheelTorque;
    } else if (maxTorqueIq > 0.0f) {
        maxTorque = maxTorqueIq;
    }

    float lqrTorqueLimit = (_lqr.u_limit > 0.0f) ? _lqr.u_limit : 0.0f;

    float effectiveLimit = (_alpha > 0.5f) ? lqrTorqueLimit : maxTorque;

    _lastSaturated = false;
    _diag.sat_left = false;
    _diag.sat_right = false;
    if (effectiveLimit > 0.0f) {
        if (out.torqueLeftNm > effectiveLimit) { out.torqueLeftNm = effectiveLimit; _lastSaturated = true; _diag.sat_left = true; }
        if (out.torqueLeftNm < -effectiveLimit) { out.torqueLeftNm = -effectiveLimit; _lastSaturated = true; _diag.sat_left = true; }
        if (out.torqueRightNm > effectiveLimit) { out.torqueRightNm = effectiveLimit; _lastSaturated = true; _diag.sat_right = true; }
        if (out.torqueRightNm < -effectiveLimit) { out.torqueRightNm = -effectiveLimit; _lastSaturated = true; _diag.sat_right = true; }
    }

    _lastPitchTarget = pitchTarget;

    /* ========== Record diagnostics ========== */

    _diag.u_sum_pid = uSumPidNm;
    _diag.u_sum_lqr = uSumLqrNm;
    _diag.u_sum_cmd = uSum;
    _diag.u_diff_cmd = uTurn;
    _diag.alpha = _alpha;
    _diag.theta_ref_pos = _thetaRefFromPos;
    _diag.x_err = _lastXErr;
    _diag.fallback_to_pid = false;
    _diag.cross_antiwindup_active = crossAntiWindupActive;
    _diag.requested_mode = _requestedMode;
    _diag.active_mode = _activeMode;
    _diagValid = true;

    cmd.torque = out;
    return cmd;
}

MotionController::ControlOutput
MotionController::applyVelocitySlew(ControlOutput desired, float dt)
{
    if (dt <= 0.0f) dt = _controlDt;
    float maxStep = _velocitySlewRate * dt;

    ControlOutput out;
    float deltaLeft = desired.torqueLeftNm - _prevVelocityLeft;
    if (deltaLeft > maxStep) deltaLeft = maxStep;
    if (deltaLeft < -maxStep) deltaLeft = -maxStep;
    out.torqueLeftNm = _prevVelocityLeft + deltaLeft;

    float deltaRight = desired.torqueRightNm - _prevVelocityRight;
    if (deltaRight > maxStep) deltaRight = maxStep;
    if (deltaRight < -maxStep) deltaRight = -maxStep;
    out.torqueRightNm = _prevVelocityRight + deltaRight;

    _prevVelocityLeft = out.torqueLeftNm;
    _prevVelocityRight = out.torqueRightNm;
    return out;
}
