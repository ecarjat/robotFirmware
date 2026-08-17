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

/* Deceleration-aware stop mode:
 * when user command is released at non-trivial speed, ramp v_ref down instead
 * of snapping to zero to avoid a large transient position pull-back. */
static constexpr float kStopEnterCmdMps = 0.05f;
static constexpr float kStopReengageCmdMps = 0.12f;
static constexpr float kStopEnterMeasMps = 0.35f;
static constexpr float kStopExitMeasMps = 0.12f;
static constexpr float kStopDecelMps2 = 0.8f;

/* Turn-aware position-loop suppression:
 * reduce/disable K0 during strong yaw maneuvers to avoid position loop
 * fighting in-place turns or spending balance authority on world-frame drift. */
static constexpr float kTurnSuppressEnterCmd = 0.35f;     /* teleop turn command (0..1) */
static constexpr float kTurnSuppressExitCmd = 0.20f;
static constexpr float kTurnSuppressFullCmd = 0.85f;
static constexpr float kTurnSuppressEnterYawRadS = 1.0f;  /* blended yaw-rate estimate */
static constexpr float kTurnSuppressExitYawRadS = 0.6f;
static constexpr float kTurnSuppressFullYawRadS = 2.5f;
static constexpr float kInPlaceTurnVRefMaxMps = 0.20f;

/* Yaw governor:
 * reserve part of wheel torque headroom for balance/longitudinal loop. */
static constexpr float kYawHeadroomReserveNm = 2.0f;

namespace {

inline float clampf(float val, float limit)
{
    if (val > limit) return limit;
    if (val < -limit) return -limit;
    return val;
}

inline float clamp01(float val)
{
    if (val < 0.0f) return 0.0f;
    if (val > 1.0f) return 1.0f;
    return val;
}

inline float lerpf(float a, float b, float t)
{
    return a + (b - a) * t;
}

LqrSpeedSchedule sanitizeSpeedSchedule(LqrSpeedSchedule s)
{
    if (s.cruise_enter_cmd_mps < 0.0f) s.cruise_enter_cmd_mps = 0.0f;
    if (s.cruise_exit_cmd_mps < 0.0f) s.cruise_exit_cmd_mps = 0.0f;
    if (s.cruise_enter_meas_mps < 0.0f) s.cruise_enter_meas_mps = 0.0f;
    if (s.cruise_exit_meas_mps < 0.0f) s.cruise_exit_meas_mps = 0.0f;
    if (s.v_ref_margin_mps < 0.0f) s.v_ref_margin_mps = 0.0f;
    if (s.theta_ref_limit_rest_cap_rad < 0.0f) s.theta_ref_limit_rest_cap_rad = 0.0f;
    if (s.theta_ref_limit_cruise_cap_rad < 0.0f) s.theta_ref_limit_cruise_cap_rad = 0.0f;
    if (s.yaw_damp_cruise_mult < 0.0f) s.yaw_damp_cruise_mult = 0.0f;
    if (s.cruise_blend_tau_s < 0.0f) s.cruise_blend_tau_s = 0.0f;

    if (s.cruise_exit_cmd_mps > s.cruise_enter_cmd_mps) {
        s.cruise_exit_cmd_mps = s.cruise_enter_cmd_mps;
    }
    if (s.cruise_exit_meas_mps > s.cruise_enter_meas_mps) {
        s.cruise_exit_meas_mps = s.cruise_enter_meas_mps;
    }
    return s;
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
    _speedSchedule = sanitizeSpeedSchedule(_speedSchedule);
    _k0Eff = _lqr.K[0];
    _vRefLimitEff = (_lqr.v_ref_limit > 0.0f) ? _lqr.v_ref_limit : 100.0f;
    _thetaRefLimitEff = _lqr.theta_ref_limit;
    _yawDampEff = _yawDampGain;
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
    _yawDampEff = _yawDampGain;

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

    // Removed verbose LQR params logging
    // if (changed) {
    //     APP_LOG_INFO("LQR params: K=[%.2f,%.2f,%.2f,%.2f] u_lim=%.1f",
    //                  (double)lqr.K[0], (double)lqr.K[1],
    //                  (double)lqr.K[2], (double)lqr.K[3],
    //                  (double)lqr.u_limit);
    // }
}

void MotionController::setLqrEquilibrium(float thetaRef, float uEq)
{
    _thetaRef = thetaRef;
    _uEq = uEq;
}

void MotionController::setLqrSpeedSchedule(const LqrSpeedSchedule& schedule)
{
    _speedSchedule = sanitizeSpeedSchedule(schedule);
    if (!_speedSchedule.enabled) {
        _cruiseMode = false;
        _cruiseAlpha = 0.0f;
    }
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

bool MotionController::updateStopMode(float xDot, float dt)
{
    bool just_exited = false;
    const float abs_cmd = fabsf(_targetVel);
    const float abs_meas = fabsf(xDot);

    // Arm stop-mode once command has been non-trivial. This allows smooth
    // teleop ramp-down releases (not only single-step command drops).
    if (abs_cmd >= kStopReengageCmdMps) {
        _stopModeArmed = true;
    }

    if (!_stopModeActive) {
        if (_stopModeArmed &&
            abs_cmd <= kStopEnterCmdMps &&
            abs_meas >= kStopEnterMeasMps) {
            _stopModeActive = true;
            _stopModeArmed = false;
            _stopModeVRef = xDot;
        }
    } else {
        if (abs_cmd >= kStopReengageCmdMps) {
            _stopModeActive = false;
            _stopModeArmed = true;
            _stopModeVRef = 0.0f;
            just_exited = true;
        } else {
            float step = kStopDecelMps2 * dt;
            if (step < 0.0f) step = 0.0f;
            if (_stopModeVRef > step) {
                _stopModeVRef -= step;
            } else if (_stopModeVRef < -step) {
                _stopModeVRef += step;
            } else {
                _stopModeVRef = 0.0f;
            }

            if (fabsf(_stopModeVRef) <= kStopEnterCmdMps &&
                abs_meas <= kStopExitMeasMps) {
                _stopModeActive = false;
                _stopModeVRef = 0.0f;
                just_exited = true;
            }
        }
    }

    _prevTargetVelCmd = _targetVel;
    return just_exited;
}

void MotionController::updateCruiseSchedule(float cmdVel, float xDot, float dt)
{
    if (!_speedSchedule.enabled) {
        _cruiseMode = false;
        _cruiseAlpha = 0.0f;
        return;
    }

    const float abs_cmd = fabsf(cmdVel);
    const float abs_meas = fabsf(xDot);

    if (_cruiseMode) {
        if (abs_cmd <= _speedSchedule.cruise_exit_cmd_mps &&
            abs_meas <= _speedSchedule.cruise_exit_meas_mps) {
            _cruiseMode = false;
        }
    } else {
        if (abs_cmd >= _speedSchedule.cruise_enter_cmd_mps ||
            abs_meas >= _speedSchedule.cruise_enter_meas_mps) {
            _cruiseMode = true;
        }
    }

    const float target_alpha = _cruiseMode ? 1.0f : 0.0f;
    if (_speedSchedule.cruise_blend_tau_s <= kGainEpsilon) {
        _cruiseAlpha = target_alpha;
        return;
    }
    float step = dt / _speedSchedule.cruise_blend_tau_s;
    step = clamp01(step);
    _cruiseAlpha += (target_alpha - _cruiseAlpha) * step;
    _cruiseAlpha = clamp01(_cruiseAlpha);
}

float MotionController::computeLqrUSumNm(const StateEstimate& state, float vRef, float thetaRef)
{
    /*
     * Three-layer architecture (per second opinion feedback):
     *
     * 1. OUTER POSITION LOOP (optional, slow):
     *    v_ref = -k_x * x_err  (saturated)
     *
     * 2. SLOW TRIM LOOP:
     *    θ_trim integrates (v_ref - v) to eliminate velocity drift
     *    This works because changing velocity requires changing lean (non-minimum phase)
     *
     * 3. FAST INNER LQR:
     *    u = -K[1]*(v - v_ref) - K[2]*(θ - θ_ref) - K[3]*θ̇
     *    where θ_ref = θ_eq + θ_trim
     */
    
    const float dt = _controlDt;
    
    /* === 1. OUTER POSITION LOOP === */
    /* Generate velocity reference from position error */
    float x_err = state.x - _xRef;
    _lastXErr = x_err;
    
    const float cruise_alpha = _speedSchedule.enabled ? _cruiseAlpha : 0.0f;

    /* K[0] now acts as position→velocity gain (not position→torque).
     * Suppress K0 in cruise mode to avoid late "catch-up" position action. */
    const float stop_scale = _stopModeActive ? 0.0f : 1.0f;
    const float k0_eff = _lqr.K[0] * (1.0f - cruise_alpha) * stop_scale * _turnK0Scale;
    _k0Eff = k0_eff;

    /* Use configured v_ref_limit; 0 disables clamping.
     * In cruise mode, allow v_ref to expand toward |v_cmd| + margin. */
    const float v_ref_max_base = (_lqr.v_ref_limit > 0.0f) ? _lqr.v_ref_limit : 100.0f;
    const float v_ref_target = fabsf(vRef) + _speedSchedule.v_ref_margin_mps;
    const float v_ref_max_sched = lerpf(v_ref_max_base, v_ref_target, cruise_alpha);
    const float v_ref_max = (_speedSchedule.enabled)
                                ? fmaxf(v_ref_max_base, v_ref_max_sched)
                                : v_ref_max_base;
    _vRefLimitEff = v_ref_max;

    /* Schedule theta reference cap from rest->cruise, bounded by base lqr.theta_ref_limit. */
    float theta_ref_limit_eff = _lqr.theta_ref_limit;
    if (_lqr.theta_ref_limit > 0.0f && _speedSchedule.enabled) {
        const float cap_sched = lerpf(_speedSchedule.theta_ref_limit_rest_cap_rad,
                                      _speedSchedule.theta_ref_limit_cruise_cap_rad,
                                      cruise_alpha);
        theta_ref_limit_eff = fminf(_lqr.theta_ref_limit, cap_sched);
        if (theta_ref_limit_eff < 0.0f) {
            theta_ref_limit_eff = 0.0f;
        }
    }
    _thetaRefLimitEff = theta_ref_limit_eff;

    float v_ref_from_pos = 0.0f;
    if (fabsf(k0_eff) > kGainEpsilon) {
        /* K[0] is negative, x_err positive when ahead of target → v_ref negative (go back) */
        v_ref_from_pos = k0_eff * x_err;  /* Already negative gain */
        v_ref_from_pos = clampf(v_ref_from_pos, v_ref_max);
    }
    
    /* Combine with teleop velocity reference */
    float v_ref_total = vRef + v_ref_from_pos;
    v_ref_total = clampf(v_ref_total, v_ref_max);
    
    /* === 2. SLOW TRIM LOOP === */
    /* Integrate velocity error into pitch trim (leaky integrator with anti-windup) */
    constexpr float k_trim = 0.03f;      /* Trim integrator gain (rad per (m/s)*s) — slow! */
    constexpr float k_leak = 0.005f;     /* Leak rate (1/s) — prevents memory forever */
    constexpr float theta_trim_max = 0.08f;  /* Max trim ~4.6° */
    
    float v_err = v_ref_total - state.xDot;  /* (v_ref - v): positive when going too slow */
    
    /* Leaky integration: trim += ki * v_err * dt - leak * trim * dt
     * Freeze trim integration in in-place turn mode to avoid injecting
     * forward/backward bias while yaw dynamics dominate. */
    if (!_turnInPlaceActive) {
        _thetaTrim += k_trim * v_err * dt;
        _thetaTrim -= k_leak * _thetaTrim * dt;
    }
    _thetaTrim = clampf(_thetaTrim, theta_trim_max);
    
    /* Final theta reference = equilibrium + trim */
    float theta_ref_final = thetaRef + _thetaTrim;
    if (theta_ref_limit_eff > 0.0f) {
        const float lo = thetaRef - theta_ref_limit_eff;
        const float hi = thetaRef + theta_ref_limit_eff;
        if (theta_ref_final < lo) theta_ref_final = lo;
        if (theta_ref_final > hi) theta_ref_final = hi;
    }
    _thetaRefFromPos = _thetaTrim;  /* For diagnostics */
    
    /* === 3. FAST INNER LQR === */
    /* u = -K[1]*(v - v_ref) - K[2]*(θ - θ_ref) - K[3]*θ̇ */
    float theta_err = state.theta - theta_ref_final;
    float thetaDot = state.thetaDot;
    
    float u_sum = 0.0f;
    
    /* Velocity term */
    if (fabsf(_lqr.K[1]) > kGainEpsilon) {
        u_sum -= _lqr.K[1] * (state.xDot - v_ref_total);
    }
    
    /* Pitch terms (balance is priority) */
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
    _thetaTrim = 0.0f;
    _cruiseMode = false;
    _cruiseAlpha = 0.0f;
    _turnSuppressActive = false;
    _turnInPlaceActive = false;
    _turnK0Scale = 1.0f;
    _stopModeActive = false;
    _stopModeArmed = (fabsf(_targetVel) >= kStopReengageCmdMps);
    _stopModeVRef = 0.0f;
    _prevTargetVelCmd = _targetVel;
    _k0Eff = _lqr.K[0];
    _vRefLimitEff = (_lqr.v_ref_limit > 0.0f) ? _lqr.v_ref_limit : 100.0f;
    _thetaRefLimitEff = _lqr.theta_ref_limit;
    _yawDampEff = _yawDampGain;
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

    const bool stop_just_exited = updateStopMode(state.xDot, dtSeconds);
    const float targetVelCtrl = _stopModeActive ? _stopModeVRef : _targetVel;
    updateCruiseSchedule(targetVelCtrl, state.xDot, dtSeconds);

    const bool was_turn_in_place = _turnInPlaceActive;

    /* Turn-aware K0 suppression with hysteresis. */
    const float abs_turn_cmd = fabsf(_teleopTurn);
    const float abs_yaw_rate = fabsf(_lastYawRate);
    if (_turnSuppressActive) {
        if (abs_turn_cmd <= kTurnSuppressExitCmd &&
            abs_yaw_rate <= kTurnSuppressExitYawRadS) {
            _turnSuppressActive = false;
        }
    } else if (abs_turn_cmd >= kTurnSuppressEnterCmd ||
               abs_yaw_rate >= kTurnSuppressEnterYawRadS) {
        _turnSuppressActive = true;
    }

    float turn_suppress_alpha = 0.0f;
    if (_turnSuppressActive) {
        const float denom_cmd = fmaxf(kTurnSuppressFullCmd - kTurnSuppressExitCmd, kGainEpsilon);
        const float denom_yaw =
            fmaxf(kTurnSuppressFullYawRadS - kTurnSuppressExitYawRadS, kGainEpsilon);
        const float cmd_alpha = clamp01((abs_turn_cmd - kTurnSuppressExitCmd) / denom_cmd);
        const float yaw_alpha =
            clamp01((abs_yaw_rate - kTurnSuppressExitYawRadS) / denom_yaw);
        turn_suppress_alpha = fmaxf(cmd_alpha, yaw_alpha);
    }

    _turnInPlaceActive = _turnSuppressActive &&
                         fabsf(targetVelCtrl) <= kInPlaceTurnVRefMaxMps;
    if (_turnInPlaceActive) {
        turn_suppress_alpha = 1.0f;
    }
    _turnK0Scale = 1.0f - clamp01(turn_suppress_alpha);
    const bool turn_in_place_just_exited = was_turn_in_place && !_turnInPlaceActive;

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
        targetVelCtrl,
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
        float velocityError = targetVelCtrl - state.xDot;
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
        if (stop_just_exited || turn_in_place_just_exited) {
            // After stop-mode or turn-mode exit, anchor at current pose to
            // prevent a delayed K0 pull-back transient.
            _xRef = state.x;
            _thetaTrim = 0.0f;
        } else if (_turnInPlaceActive) {
            // Freeze reference integration during in-place turns.
        } else if (fabsf(targetVelCtrl) > 1e-4f) {
            // Track position reference while moving, including stop-mode deceleration.
            _xRef += targetVelCtrl * dtSeconds;
        }
    }

    float thetaRef = _thetaRef;
    /* LQR uses theta_ref from equilibrium LUT and effective velocity reference. */
    float uSumLqrNm = computeLqrUSumNm(state, targetVelCtrl, thetaRef) + _uEq;
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

    /* ========== Torque limits ========== */

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

    /* ========== Yaw differential with headroom governor ========== */

    // Yaw split is in torque units (Nm), consistent with uSum.
    const float yaw_damp_eff = _yawDampGain *
        (_speedSchedule.enabled
             ? lerpf(1.0f, _speedSchedule.yaw_damp_cruise_mult, _cruiseAlpha)
             : 1.0f);
    _yawDampEff = yaw_damp_eff;

    float uTurnCmd = _turnGain * _teleopTurn;
    const float uTurnDamp = -yaw_damp_eff * _lastYawRate;
    if (effectiveLimit > 0.0f) {
        float uTurnLimit = effectiveLimit - fabsf(uSum);
        if (uTurnLimit < 0.0f) uTurnLimit = 0.0f;
        float cmdBudget = uTurnLimit - fabsf(uTurnDamp) - kYawHeadroomReserveNm;
        if (cmdBudget < 0.0f) cmdBudget = 0.0f;
        uTurnCmd = clampf(uTurnCmd, cmdBudget);
    }
    float uTurn = uTurnCmd + uTurnDamp;

    /* ========== Mix u_sum and u_diff into wheel commands ========== */

    ControlOutput out;
    out.torqueLeftNm = uSum - uTurn;
    out.torqueRightNm = uSum + uTurn;

    // Final hard bound for safety.
    if (effectiveLimit > 0.0f) {
        float uTurnLimit = effectiveLimit - fabsf(uSum);
        if (uTurnLimit < 0.0f) uTurnLimit = 0.0f;
        if (uTurn > uTurnLimit) uTurn = uTurnLimit;
        if (uTurn < -uTurnLimit) uTurn = -uTurnLimit;
        out.torqueLeftNm = uSum - uTurn;
        out.torqueRightNm = uSum + uTurn;
    }

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
    _diag.cruise_alpha = _cruiseAlpha;
    _diag.k0_eff = _k0Eff;
    _diag.v_ref_limit_eff = _vRefLimitEff;
    _diag.theta_ref_limit_eff = _thetaRefLimitEff;
    _diag.yaw_damp_eff = _yawDampEff;
    _diag.cruise_mode = _cruiseMode;
    _diag.stop_mode_active = _stopModeActive;
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
