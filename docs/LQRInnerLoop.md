# LQR Inner Loop With Runtime PID↔LQR Switching

## Goal
Implement a **new inner-loop LQR stabilizer** for the two-wheeled robot that can be switched at runtime between:
- the existing **cascaded PID** controller (baseline)
- the new **LQR inner loop** (for longitudinal + pitch stabilization)

Switching must be triggered by a **teleop command** and must be **safe** (bumpless transfer / blending, watchdog, clean fallback).

This document specifies what a coding agent must implement.

---

## Scope and Control Architecture

### Current baseline
- Outer loop: PID (e.g., position/velocity → pitch/lean reference)
- Inner loop: PID (pitch/velocity stabilization)
- Yaw: handled via differential wheel command (existing)

### Target (v1)
- **Outer loop:** unchanged initially (still PID). Produces longitudinal references (e.g., `v_ref`, optionally `x_ref`, and/or `theta_ref`).
- **Inner longitudinal loop:** selectable at runtime:
  - `INNER_LONG_PID` (existing)
  - `INNER_LONG_LQR` (new)
- **Yaw loop:** unchanged (continues to produce `u_diff`).

The inner controller outputs are split into:
- `u_sum` (symmetric command for both wheels) → balance + longitudinal control
- `u_diff` (differential command) → yaw control (existing)

Final wheel commands:
- `u_left  = saturate(u_sum - u_diff)`
- `u_right = saturate(u_sum + u_diff)`

> v1 requirement: only `u_sum` is switched between PID and LQR. Yaw loop remains unchanged.

### Rates
- Control tick: currently **400 Hz**.
- Sensors / EKF may run at the same rate or higher (e.g., 800 Hz). LQR uses the latest estimator state at the control tick.

---

## State Estimator Interface (Amended)
Your EKF/state estimator now fuses **yaw** and **wheel velocities**. The controller shall consume the fused estimates.

### Required estimator outputs
The estimator must provide (directly or via an accessor struct), with consistent units:

- Pitch:
  - `theta` (pitch angle, **rad**)
  - `thetaDot` (pitch rate, **rad/s**, bias-corrected)

- Yaw:
  - `yaw` (yaw angle, **rad**)
  - `yawDot` (yaw rate, **rad/s**, fused)

- Longitudinal translation:
  - `x` (forward position, **m**) — optional but supported
  - `v` / `xDot` (forward velocity, **m/s**) — fused estimate

- Wheel velocities (fused or raw + flags):
  - `v_left` (wheel velocity; document whether **rad/s** or **m/s**)
  - `v_right` (wheel velocity; document whether **rad/s** or **m/s**)

- Health/time:
  - `valid` (bool)
  - `timestamp_ms` (or equivalent)

### State source selection rules
- Prefer **fused `v`** for balance/longitudinal control.
- Prefer fused `yawDot` for yaw loop.
- Wheel velocities are used for:
  - yaw mixing / differential control
  - slip diagnostics (optional)
  - sanity checks

### Staleness
Controller must treat estimator data as stale if:
- `!valid`, OR
- `now_ms - timestamp_ms > EST_MAX_AGE_MS` (new parameter)

On stale/invalid state, controller must immediately fall back to `INNER_LONG_PID`.

---

## Controller Modes and Teleop Commands

### Modes
Add a mode selector for the **longitudinal inner loop** only:

```c
typedef enum {
  INNER_LONG_PID = 0,
  INNER_LONG_LQR = 1,
} inner_long_mode_t;
```

- Requested mode: set via teleop.
- Active mode: may differ if safety fallback forces PID.

### Teleop commands
Add a teleop command to select inner longitudinal mode:
- `SET_LONG_INNER_MODE {PID|LQR}`

Optional (nice-to-have, not required for v1):
- `SET_LQR_K` to update LQR gains at runtime

### Safe switching (bumpless transfer)
Switching between PID and LQR must not introduce a step change in wheel commands.

v1 requirement: implement **blended output**:

- Compute both commands each tick:
  - `u_sum_pid`
  - `u_sum_lqr`
- Blend with ramp:
  - `u_sum = (1 - alpha) * u_sum_pid + alpha * u_sum_lqr`
- `alpha` ramps:
  - PID→LQR: `alpha: 0 → 1` over `lqr.engage_ramp_ms`
  - LQR→PID: `alpha: 1 → 0` over `lqr.disengage_ramp_ms`

The blend happens **before** final saturation/rate limiting.

---

## LQR Design

### LQR role (v1)
LQR replaces only the **inner longitudinal stabilizer** (balance + forward velocity tracking) producing `u_sum`.

Yaw remains controlled by the existing yaw loop producing `u_diff`.

### State vector
Select one state vector for v1 and implement it explicitly.

**Recommended v1 (4-state):**
\[
\tilde{x} = \begin{bmatrix}
(x - x_{ref})\\
(v - v_{ref})\\
(\theta - \theta_{ref})\\
\dot\theta
\end{bmatrix}
\]

- `x_ref` optional: if outer loop does not provide position references, set `x_ref = x` (so the first term is 0) or use the 3-state variant.
- `v_ref` from outer loop.
- `theta_ref` from outer loop (or derived from `v_ref` by existing mapping).

**Alternative v1 (3-state):**
\[
\tilde{x} = \begin{bmatrix}
(v - v_{ref})\\
(\theta - \theta_{ref})\\
\dot\theta
\end{bmatrix}
\]

> v1 requirement: yaw/yawDot are NOT part of the LQR state vector.

### Control law
The inner-loop LQR computes:
\[
 u_{sum,lqr} = -K\,\tilde{x}
\]

- `K` is provided (computed offline) and stored in parameters.
- No online Riccati solve on STM32 in v1.

### Gain provisioning
Support at least:
1) Compile-time fixed `K` (initial shipping mode)
2) Runtime-configurable `K` (nice-to-have)

### Limits
LQR must respect actuator constraints via:
- `u_sum` clamp: `|u_sum| ≤ u_limit`
- optional rate limit: `|Δu_sum| ≤ du_limit` per control tick

All limits apply after blending and before mixing to left/right.

---

## Outer Loop Interface
Define an outer reference struct that can carry both longitudinal and yaw references without forcing the LQR to use yaw:

```c
typedef struct {
  float x_ref;         // m (optional)
  float v_ref;         // m/s
  float theta_ref;     // rad
  float yaw_ref;       // rad (optional)
  float yaw_rate_ref;  // rad/s (optional)
} outer_ref_t;
```

The yaw loop continues to compute `u_diff` from `yaw/yawDot` and the yaw references.

---

## Implementation Plan

### Files / modules
Add:
- `control_inner_lqr.h/.c` (or `.cpp` if the project is C++)
- optional `lqr_params.h` for config structs

### Public API

```c
typedef struct {
  float u_sum_cmd;
  float u_sum_pid;
  float u_sum_lqr;
  float u_diff_cmd;
  float u_left_cmd;
  float u_right_cmd;
  float alpha;
  uint8_t sat_left;
  uint8_t sat_right;
  uint8_t sat_any;
  uint8_t fallback_to_pid;
} inner_ctrl_diag_t;

void inner_lqr_init(const robot_params_t* params);
void inner_lqr_set_requested_mode(inner_long_mode_t mode);
inner_long_mode_t inner_lqr_get_active_mode(void);

// Called each control tick
void inner_lqr_tick(float dt_s,
                    const state_estimate_t* est,
                    const outer_ref_t* ref,
                    float u_diff_from_yaw,
                    inner_ctrl_diag_t* diag,
                    float* out_u_left,
                    float* out_u_right);
```

### Plug-in point
In `motion_control_tick()`:
1) compute outer loop references (`ref`)
2) compute yaw loop output (`u_diff`)
3) compute inner longitudinal outputs:
   - existing `u_sum_pid`
   - new `u_sum_lqr`
4) blend to `u_sum`
5) mix `u_sum` and `u_diff` into wheel commands
6) apply saturation and (optional) rate limiting
7) send to motor command path

> Requirement: keep the saturation/rate limiting in **one place** after blending and mixing to avoid double-limiting.

---

## Safety, Watchdogs, and Fallback

### Immediate fallback to PID
Force active mode to `INNER_LONG_PID` (same tick) if any of:
- `est->valid == false`
- estimator stale: `(now_ms - est->timestamp_ms) > EST_MAX_AGE_MS`
- NaN/INF in any state input, `K`, or computed command
- teleop link indicates failsafe mode (if applicable)

When fallback triggers:
- set `diag->fallback_to_pid = 1`
- reset blend request to PID (alpha ramps toward 0)

### Saturation behavior
- Saturation must be applied on `u_left` and `u_right` (after sum/diff mix).
- Export `sat_left`, `sat_right`, `sat_any`.

Optional (nice-to-have): revert to PID if persistent saturation + poor tracking persists for configurable time.

### Slip / mismatch diagnostics (optional)
Using wheel velocities, compute:
- `v_fwd_wheels = (v_left + v_right) / 2`
- `yaw_rate_wheels = (v_right - v_left) / TRACK_WIDTH` (if wheel velocities are linear m/s; if rad/s, include wheel radius)

Log mismatch metrics (v1: logging only):
- `|v_fwd_wheels - v_est|`

---

## Parameters to Add
Add (or confirm existing) params:

- `control.inner_long_default_mode` (`PID` or `LQR`)
- `control.est_max_age_ms`

LQR config:
- `lqr.K` (1×NX)
- `lqr.u_limit`
- `lqr.du_limit` (optional)
- `lqr.engage_ramp_ms`
- `lqr.disengage_ramp_ms`

Reference limits (optional but recommended):
- `lqr.theta_ref_limit_rad`
- `lqr.v_ref_limit` (m/s)

---

## Diagnostics and Logging
Expose via telemetry/logging:
- requested vs active longitudinal inner mode
- `alpha`
- `u_sum_pid`, `u_sum_lqr`, `u_sum_cmd`
- `u_diff_cmd`
- `u_left_cmd`, `u_right_cmd`
- saturation flags
- estimator validity and age
- `theta`, `thetaDot`, `v`, `yaw`, `yawDot`
- `v_left`, `v_right`

Add one info log on each mode change:
- `Inner longitudinal mode: PID→LQR (ramp=...)` and reverse

---

## Test Plan

### Unit-ish tests (host or embedded)
- Blend ramp reaches target value in configured time.
- Clamp and rate limiting behave deterministically.
- NaN/INF injection triggers immediate fallback.

### On-robot staged tests
1) Wheels off ground: switch PID↔LQR; verify no command spikes.
2) Low-gain LQR: verify stable balancing.
3) Increase gains gradually; validate saturation behavior.
4) Force estimator invalid/stale; verify fallback to PID within 1 tick.
5) Verify yaw control unchanged while switching longitudinal mode.

---

## Acceptance Criteria
- Teleop command switches longitudinal inner loop mode reliably.
- Switching is bumpless: no visible step in `u_left/u_right` beyond rate limit.
- LQR mode balances at least as well as PID at steady state.
- If estimator becomes invalid/stale, system returns to PID within 1 control tick.
- Yaw behavior does not regress when longitudinal LQR is enabled.
- Telemetry/logs show mode, alpha ramp, command components, and safety fallbacks.

---

## Non-goals (v1)
- No outer MPC in this change.
- No online Riccati solve on STM32.
- No redesign of yaw controller.
- No change to motor driver inner current loop.
