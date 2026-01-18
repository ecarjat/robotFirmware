# Motion Control v2 - Review Notes and Suggestions

This document captures improvement ideas after the Phase 1 motion control code
reached a functional baseline. The focus is on reliability, safety, and clearer
operational behavior before deeper tuning work.

## Quick strengths observed in Phase 1
- Clear separation between mode logic (`motion_modes.*`) and control loop
  orchestration (`motion_control.*`).
- Dual-IMU plumbing with health metrics and blackbox logging is in place.
- Control loop is bounded and non-blocking, with output gating by mode.

## Suggested improvements (priority order)

### 1) Harden fault detection when no data has ever arrived
Right now, the fault timers only trigger if `last_imu_ok_ms` or
`last_motor_ok_ms` is non-zero. If the system enters BALANCING with no valid
sensor or motor feedback yet (or those timestamps never get set), the fault
logic never trips. Consider:
- On entry to BALANCING, seed `last_imu_ok_ms` and `last_motor_ok_ms` with the
  current time, then treat missing data as a fault after the configured timeout.
- Alternatively, add an explicit “no data yet” grace timer that starts on
  BALANCING entry and flips to FALLEN/FAULT if no valid samples arrive in time.

### 2) Use or remove the unused `imu_ok` input
`motion_modes_input_t` includes `imu_ok`, but it is not referenced in
`motion_modes_step()`. Either:
- Use it to guard transitions or to provide richer reason codes, or
- Remove it from the input struct to avoid confusion and keep the interface
  minimal.

### 3) Make arming and disarming fully atomic in one place
Arming spans `motion_control_can_arm()`, `motion_control_set_mode()`, and
`motor_link_enable()` in `app_arm.c`. This works but spreads the safety
contract across multiple files. Consider:
- Create a single `motion_control_arm()` entry point that performs all checks
  and performs the mode transition plus motor enable under a critical section.
- For disarm, centralize the "set zero torque then disable" sequence to ensure
  a single source of truth.

### 4) Add a latched fault reason and surface it in telemetry
The log prints the reason when BALANCING exits, but the reason is not retained
for later inspection. Consider:
- Latching the last fault reason (kill, IMU timeout, motor timeout, etc.)
  in `motion_modes` and exposing it via a small getter for telemetry.
- Use this in UI/telemetry to distinguish operator errors from hardware faults.

### 5) Gate log/blackbox work more aggressively under load
`motion_control_tick()` always builds and pushes a log record each tick. At
1 kHz this can become a significant CPU and SD load. Options:
- Make the log write rate configurable (e.g., 200 Hz) while still capturing
  high-rate internal signals in RAM for a short window.
- Skip log record assembly entirely when `log_fields_mask == 0`.

### 6) Revisit calibration validity criteria
Calibration validity currently checks whether any bias entry is non-zero. This
is pragmatic, but it can produce false negatives when a valid bias happens to
be zero (rare, but possible). Consider:
- Adding an explicit `calibrated` flag in the stored params, set by the
  calibration procedure.
- Recording a calibration timestamp or version as part of the params to track
  staleness across sensor swaps.

### 7) Make mode transitions explicit (include CALIBRATION in state machine)
CALIBRATION exists in the enum but is not part of transition logic in
`motion_modes_step()`. Consider:
- Defining explicit entry/exit behavior for CALIBRATION, including output
  gating and logging, even if only used during factory/test flows.

### 8) Add explicit "output disable" semantics
`motion_control_set_output_enabled(false)` stops sending wheel commands but does
not ensure that the motor drivers stop acting on the last command. Consider:
- When output is disabled, send a single zero-Iq command and/or force
  `motor_link_enable(false)`.
- Track the last enable state so transitions can perform one-time safety
  actions without spamming the link.

### 9) Improve timebase resolution for control/estimator
`motion_control_tick()` uses millisecond resolution from `now_ms`. At 1 kHz,
`dt` is often inferred from the configured rate rather than the actual tick
spacing. Consider:
- Using a microsecond timer for `dt` (while keeping `now_ms` for timeouts).
- Alternatively, compute `dt` from the timer ISR period and avoid mixing
  tick-time and wall-time within the estimator.

## EKF review and proposed improvements
The EKF is solidly structured, but a few gaps limit accuracy and fault
handling. Suggested changes below are based on `app/control/StateEstimator.cpp`
and `app/control/ekf/BalancerEKF.cpp`.

### 1) Actually fuse wheel velocity (or remove it cleanly)
Right now the EKF measurement vector is `[theta_acc, v_enc, x]`, but
`StateEstimator::update()` passes `v_enc_ekf = NAN` and `pos_enc = NAN`, so the
velocity and position channels are always skipped. As a result:
- `bad_vel` / `bad_pos` innovation logic in `BalancerEKF::step()` never runs.
- The `x` / `xDot` states are effectively dead unless driven by process noise.

If wheel velocity is desired (recommended), pass `v_enc` into the EKF and tune
`EKF_R_V_ENC`. If not, consider removing the velocity/position measurements
from the model to simplify behavior and avoid misleading logic.

### 2) Use a statistically grounded innovation gate
Innovation gating is based on fixed absolute thresholds. Consider switching to
a normalized (Mahalanobis) gate using `S` so thresholds scale with uncertainty.
This is more robust during startup, after resets, and when `R` is inflated.

### 3) Improve accelerometer gating criteria
Accel gating only uses vibration RMS (`IMU_VIB_*`). Add a simple norm gate
based on `|norm_g - 1|` to reject dynamic acceleration that is not vibration
(e.g., fast forward motion), or inflate `R` based on that deviation.

### 4) Tighten timebase handling
The EKF uses `dt` derived from `HAL_GetTick()` (ms). At high rates this is
coarse. Use IMU timestamps or a microsecond timer for `dt`, and only fall back
to `control_dt_` when timestamps are missing.

### 5) Clarify partial reset behavior
`partialReset()` is a good idea, but its triggers are tied to position/velocity
innovations that are never active. If you keep the partial reset mechanism,
tie it to a valid measurement channel (e.g., theta) or enable wheel velocity
to make it meaningful.

### 6) Consider bias observability during accel gating
When accel is gated for long periods, gyro bias observability weakens. Options:
- Increase `EKF_Q_BIAS` temporarily during gate periods.
- Add a slow bias leak toward zero when accel is gated for too long.

## Nice-to-have enhancements (lower priority)
- Add hysteresis for `theta_kill_rad` to prevent rapid oscillation around the
  threshold when the robot is near the fall boundary.
- Export a short "control status" struct (mode, output enabled, saturated,
  last fault reason) for UI/telemetry.
- Cache `motion_modes_get()` in `motion_control_tick()` to avoid multiple reads
  and ensure consistency within one tick.

## Open questions to resolve before tuning
- Should BALANCING permit recovery from FALLEN automatically after a quiet
  window, or must it always require manual re-arm?
- What is the intended behavior when only one IMU is healthy? (Current
  estimator supports it, but mode logic treats any missing IMU as a fault.)
- Is the long-term plan to keep the loop at 1 kHz, or to split estimator and
  controller rates (e.g., 400 Hz EKF with 1 kHz control)?
