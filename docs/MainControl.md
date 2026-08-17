# Main Control Sketch (GDS68 CAN Simple + Dual-IMU EKF Pattern A)
Target: self-balancing robot where the **STM32H723** runs the state estimator
and balance/speed/steering controller. The two wheel actuators are
GIM8108-8/GDS68 drives on the shared CAN Simple bus; the STM32 sends torque
commands directly in Nm. Two IMUs are used: **IMU1 (BMI270)** as primary and
**IMU2 (ICM-42688)** for validation and fallback.

Wheel encoder telemetry: 100 Hz minimum (10 ms configured on each drive)
Main control: rate set in robot_params  
IMU sampling: DRDY -> DMA (gyro ODR 800 Hz, accel ODR 400 Hz)

---

## 1) Control architecture (cascade with dual-IMU fusion)
- **Wheel drives (GIM8108-8/GDS68)**: run their internal FOC loop and accept
  CAN Simple `Set_Input_Torque` commands in Nm.
- **STM32H723**:
  - Read IMUs using SPI + DMA, DRDY flags
  - EKF estimates tilt, tilt rate, gyro bias
  - Health checks using IMU2 for validation
  - Main PID controller produces torque commands
  - Outputs wheel torque commands (Nm) to the GDS68 drives over FDCAN1.

---

## 2) Signals and conventions
**Primary IMU (IMU1):** BMI270  
**Secondary IMU (IMU2):** ICM-42688

State/measurements:
- `theta, thetaDot` (body pitch & rate)
- `wL, wR` (wheel angular velocities)
- `v` (forward linear velocity)
- `acc1, gyro1`: primary IMU accel/gyro vectors
- `acc2, gyro2`: secondary IMU accel/gyro vectors

Velocity handling:
- EKF is tilt/bias only (no velocity state); acceleration is handled inside the EKF update.
- The controller uses wheel velocity directly (`wL`, `wR`) for `v`; wheel velocity is not fused into the EKF.

### 2.2 IMU frame alignment
IMU1 and IMU2 can be mounted with different axes. The per-IMU rotation matrix is stored in `imu_calib_t` within `robot_params_t` and is applied in the estimator.

**Body frame definition:**
- **+X**: Forward (direction of travel when moving forward)
- **+Y**: Left (perpendicular to forward, in the horizontal plane)
- **+Z**: Up (opposite to gravity when robot is upright)
- **Pitch axis**: Y-axis (positive pitch = nose up)
- **Roll axis**: X-axis (positive roll = right side down)
- **Yaw axis**: Z-axis (positive yaw = counter-clockwise when viewed from above)

**Rotation matrix format:**
The 3×3 rotation matrix `R` transforms sensor-frame vectors into body-frame vectors:
```
v_body = R * v_sensor
```

**Example mounting orientations:**

*IMU mounted flat, chip top facing up, connector toward front:*
```
R = [1  0  0]    // sensor X → body X
    [0  1  0]    // sensor Y → body Y
    [0  0  1]    // sensor Z → body Z
```

*IMU rotated 90° about Z (connector toward left):*
```
R = [0 -1  0]    // sensor Y → body -X
    [1  0  0]    // sensor X → body Y
    [0  0  1]    // sensor Z → body Z
```

*IMU mounted upside-down (chip facing down):*
```
R = [1  0  0]    // sensor X → body X
    [0 -1  0]    // sensor Y → body -Y
    [0  0 -1]    // sensor Z → body -Z
```

---

## 2.1 Motor command model (GDS68 CAN Simple torque control)
The outer controller commands wheel torque in Nm. `motor_link_set_wheel_torques`
clamps each requested torque, applies the configured wheel direction, and sends
CAN Simple `Set_Input_Torque` (`0x00E`) with a float32 Nm payload.

The wheel drives use node IDs `1` and `2`; their arbitration IDs are formed as
`(node_id << 5) | command_id`. Commands are enabled only after the backend has
cleared errors, selected torque/direct mode, sent zero torque, and requested
closed-loop state. Disabling sends zero torque then requests Idle.

Wheel velocity is received from periodic `Get_Encoder_Estimates` (`0x009`)
messages and converted from rev/s to rad/s. Configure
`odrv0.axis0.config.can.encoder_rate_ms = 10` and save it on each wheel drive.

`motor_link_set_wheel_Iq()` remains only for callers that explicitly work in
current. It requires a calibrated motor torque constant to convert amperes to
the GDS68 torque unit. The normal balance path uses Nm directly.

### 2.3 Motor link API (implemented)
Motor command transmission and wheel velocity feedback are implemented in `Drivers/motors/motor_link.h`:

**Sending motor commands:**
```c
// Set Iq targets directly (Amps); requires a calibrated motor Kt.
void motor_link_set_wheel_Iq(float left_A, float right_A, float max_A);

// Set direct GDS68 torque targets (Nm) - preferred for balance control.
void motor_link_set_wheel_torques(float left_Nm, float right_Nm, float max_Nm);
```

**Reading wheel velocities:**
```c
// Returns wheel angular velocities from motor driver telemetry
bool motor_link_get_wheel_velocities(float *left_rad_s, float *right_rad_s);
```

**Implementation notes:**
- Commands are sent through FDCAN1 at 500 kbit/s to nodes `1` and `2`.
- Wheel velocities arrive through periodic GDS68 encoder-estimate telemetry.
- **No latency compensation** is applied to wheel velocity readings; the control loop uses the most recent available value.
- For forward velocity: `v = (wL + wR) / 2 * wheel_radius`
- For yaw rate from encoders: `yawRate_enc = (wR - wL) / wheelbase`

## 3) IMU fusion (Pattern A: best ROI)
The fusion logic augments your EKF with:
- **health checks**
- **outlier rejection**
- **fallback to IMU2**

### 3.0 Implementation details (firmware integration)

**IMU data acquisition:**
IMU samples are read via DRDY-triggered DMA and accessed using lock-free getters in `Drivers/imu/`:
```c
// Get latest BMI270 sample (returns false if no new sample since last call)
bool imu_bmi270_try_get_latest(imu_bmi270_sample_t *out, uint32_t *seq);

// Get latest ICM42688 sample (returns false if no new sample since last call)
bool imu_icm42688_try_get_latest(imu_icm42688_sample_t *out, uint32_t *seq);
```

**When to compute health metrics:**
- Health metrics (`gyro_diff`, `acc_angle_diff`, `vib`) are computed **once per control tick** (not per IMU sample).
- At each control tick, call `imu_bmi270_try_get_latest()` and `imu_icm42688_try_get_latest()` to retrieve the most recent samples from each sensor.
- If either returns `false`, use the previous sample and increment a stale-sample counter for health tracking.

**Variance window (circular buffer):**
For vibration detection, maintain a circular buffer of recent accel magnitude samples:
```
VIB_WINDOW_MS = 100 ms
ACCEL_ODR = 400 Hz
VIB_BUFFER_SIZE = 40 samples  // 100ms × 400Hz
```
- Store `|a|/g` for each accel sample in a circular buffer.
- Compute `vib = RMS(buffer[i] - 1.0)` over the full buffer.
- Update the buffer at accel ODR (400 Hz), but the vibration metric can be queried at control tick rate.

### 3.1 Dual-IMU health metrics
Compute:
```
gyro_diff = norm(gyro1 - gyro2)
acc_angle_diff = angle_between(acc1, acc2)
accel_vib_index = variance_of_recent(acc1_samples)  // vibration indicator
```

### 3.2 Thresholds (tune experimentally)
```
GYRO_DISAGREE_THRESH ≈ 20–50 deg/s
ACC_ANGLE_DISAGREE_THRESH ≈ 5–10 deg
VIBRATION_HIGH_THRESH ≈ empirically determined
```

### 3.3 EKF update gating logic
At each IMU read:
```
valid1 = true
if gyro_diff > GYRO_DISAGREE_THRESH:
    valid1 = false

if acc_angle_diff > ACC_ANGLE_DISAGREE_THRESH:
    valid1 = false

if accel_vib_index > VIBRATION_HIGH_THRESH:
    // high vibration: rely more on gyro
    gate_accel_update = true
else:
    gate_accel_update = false
```

### 3.4 EKF measurement updates
```
if valid1:
    EKF.predict_and_update(gyro1, accel1 if !gate_accel_update)
else:
    // IMU1 is unreliable
    if secondary healthy (gyro2/acc2 OK):
        EKF.predict_and_update(gyro2, accel2 if !gate_accel_update)
    else:
        // Poor data from both IMUs
        EKF.predict_only(gyro1 or gyro2 fallback)
```

### 3.5 Health / fallback rules
```
if IMU1 DRDY stops or FIFO overflows:
    use IMU2 as primary until IMU1 recovers

if IMU1 reports saturated gyro/accel often:
    temp switch to IMU2
```

### 3.6 Online noise estimation
Maintain sliding windows for:
```
acc1_var = variance(acc1 over last N samples)
acc2_var = variance(acc2 over last N samples)
```
Use them to adjust:
- `gate_accel_update`
- EKF accel measurement noise `R` dynamically

### 3.7 Dual-IMU thresholds, vibration gating, and recovery (frozen stage-1 defaults)

We will **freeze stage-1 defaults** (tune later from logs). Use a short *fault* debounce to avoid false switches, and a longer *recovery* debounce to avoid flapping.

**Disagreement metrics (computed in body frame):**
- `gyro_diff = ||gyro1 - gyro2||` (vector norm, deg/s)
- `gyro_pitch_diff = abs(gyro1_pitch - gyro2_pitch)` (deg/s) (optional but recommended)
- `acc_angle_diff = angle(acc1, acc2)` (deg)

**Default thresholds (stage-1):**
- `GYRO_DISAGREE_WARN = 30 deg/s`
- `GYRO_DISAGREE_FAULT = 60 deg/s`
  - Trigger *fault* if `gyro_diff > GYRO_DISAGREE_FAULT` OR `gyro_pitch_diff > GYRO_DISAGREE_FAULT` for `SWITCH_TO_IMU2_AFTER`.
- `ACC_ANGLE_DISAGREE_WARN = 7 deg`
- `ACC_ANGLE_DISAGREE_FAULT = 12 deg`
  - Only evaluate `acc_angle_diff` if both accel norms are near gravity: `0.8g < |a1| < 1.2g` AND `0.8g < |a2| < 1.2g`.

**Vibration gating metric (stage-1):**
- Define `a_norm = |a|/g`.
- Define `vib = RMS(a_norm - 1.0)` over a sliding window of `VIB_WINDOW_MS = 100 ms`.
- Use hysteresis:
  - `VIB_ON = 0.06 g_rms` → gate accel updates ON
  - `VIB_OFF = 0.04 g_rms` → gate accel updates OFF

**How gating is applied:**
- If `vib` is high: prefer **gyro-only** (skip accel measurement update) *or* increase accel measurement noise `R_accel` by a factor (start with `×10`).
- Gate based on the **active IMU**:
  - when EKF is using IMU1, compute `vib` from IMU1 accel
  - when EKF is using IMU2 (fallback), compute `vib` from IMU2 accel

**Switching / recovery timing (stage-1):**
- `SWITCH_TO_IMU2_AFTER = 100 ms` (IMU1 must be continuously unhealthy for this long)
- `SWITCH_BACK_TO_IMU1_AFTER = 500 ms` (IMU1 must be continuously healthy for this long)
- `MIN_DWELL_AFTER_SWITCH = 500 ms` (minimum time to stay on the newly selected IMU to prevent chatter)

**Health definition (stage-1):**
- IMU is *unhealthy* if any of:
  - missing DRDY / no new sample for `> 2 × expected_sample_period`
  - FIFO overflow flag
  - sustained disagreement fault (above)
- IMU is *healthy* if:
  - samples arrive on time
  - no FIFO overflow
  - disagreement is below WARN thresholds for the full recovery window

**Logging required for tuning:**
- Log `gyro_diff`, `gyro_pitch_diff`, `acc_angle_diff`, `vib`, active IMU selection, and whether accel update is gated.

---

## 4) Outer loop control rates
- **Outer control loop (balance + speed + steering):** rate defined in `robot_params`
- **EKF update:** driven by IMU DRDY (gyro 800 Hz, accel 400 Hz) and consumes the latest sample per control tick
- **Telemetry:** 500 Hz

---

## 5) Balance + speed + steering controller
(Combine as before; this section is unchanged except using EKF outputs.)

```
eTheta = thetaRef - theta
eThetaDot = 0 - thetaDot
uBal = Kp_theta*eTheta + Kd_theta*eThetaDot

eV = vRef - v
iV += clamp(eV)*dt
thetaRef = clamp(Kp_v_to_theta*eV + Ki_v_to_theta*iV)

uCommon = uBal - Kv_damp * v
// yawRate is blended from gyro Z and wheel-encoder yaw rate
// yawRate = alpha*gyroZ + (1-alpha)*yawRate_enc, alpha = 0.8 (stage-1 default)
uTurn = K_turn*turnRef - K_yawDamp*yawRate

tauL_ref = clamp(uCommon - uTurn, -tauMax, +tauMax)
tauR_ref = clamp(uCommon + uTurn, -tauMax, +tauMax)
```

Send `{tauL_ref, tauR_ref}` through `motor_link_set_wheel_torques()` to the
GDS68 wheel drives.

### 5.1 Control gains (stored in robot_params)

All control tuning parameters are stored in `robot_params_t` (see `Drivers/param_storage.h`) and persisted to flash via `param_storage_save()`. This allows tuning via RPC without recompiling.

**Required additions to robot_params_t:**

```c
/* Balance controller gains */
typedef struct {
    /* Pitch stabilization PID */
    float Kp_theta;         /* Pitch proportional gain */
    float Kd_theta;         /* Pitch derivative gain (uses gyro, not numerical diff) */

    /* Velocity-to-tilt outer loop */
    float Kp_v_to_theta;    /* Velocity error → tilt reference P gain */
    float Ki_v_to_theta;    /* Velocity error → tilt reference I gain */
    float max_tilt_ref;     /* Max tilt reference from velocity loop (rad) */

    /* Damping */
    float Kv_damp;          /* Velocity damping gain */

    /* Steering */
    float K_turn;           /* Turn command gain */
    float K_yawDamp;        /* Yaw rate damping gain */
    float alpha_yaw;        /* Yaw rate blend: alpha*gyroZ + (1-alpha)*enc (default 0.8) */

    /* Limits */
    float IqMax;            /* Max motor current (A) */
    float thetaKill;        /* Kill-switch angle (rad, default ~0.785 = 45°) */

    /* Velocity integrator anti-windup */
    float iV_max;           /* Max integrated velocity error */
} balance_gains_t;
```

**Stage-1 defaults (starting point for tuning):**

| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| `Kp_theta` | 2.0 | A/rad | Pitch P gain |
| `Kd_theta` | 1.0 | A/(rad/s) | Pitch D gain |
| `Kp_v_to_theta` | 0.5 | rad/(m/s) | Velocity→tilt P |
| `Ki_v_to_theta` | 0.1 | rad/(m·s) | Velocity→tilt I |
| `max_tilt_ref` | 0.15 | rad | Max tilt from velocity loop (~8.5°) |
| `Kv_damp` | 0.1 | A/(m/s) | Velocity damping |
| `K_turn` | 0.5 | A | Turn command gain |
| `K_yawDamp` | 0.2 | A/(rad/s) | Yaw rate damping |
| `alpha_yaw` | 0.8 | - | Gyro/encoder blend factor |
| `IqMax` | 3.0 | A | Motor current limit |
| `thetaKill` | 0.785 | rad | Kill-switch at 45° |
| `iV_max` | 0.5 | rad | Velocity integrator limit |

**Usage:**
```c
// In motion_control.cpp
float Kp_theta = g_robot_params.balance.Kp_theta;
float Kd_theta = g_robot_params.balance.Kd_theta;
// ... etc

// Tuning via RPC:
// 1. SET_PARAM with offset to balance.Kp_theta, new value
// 2. SET_PARAM with SAVE flag to persist to flash
```

---

## 6) IMU scheduler & DMA
**Goal:** read both IMUs without bus conflicts, low latency.

IMU ODRs are fixed at **gyro 800 Hz** and **accel 400 Hz**. Use DRDY to fetch
the latest sample only (no FIFO accumulation), matching the current firmware
behavior.

```
on IMU1 DRDY interrupt:
    mark imu1_ready, store timestamp (latest sample only)

on IMU2 DRDY interrupt:
    mark imu2_ready, store timestamp (latest sample only)
```

Scheduler main loop (high priority):
```
if imu1_ready:
    start SPI6 DMA to read BMI270
if imu2_ready:
    start SPI6 DMA to read ICM42688 (with bus lock)
```

DMA complete callbacks:
```
parse IMU data into acc1/gyro1 or acc2/gyro2
feed data into ring buffer with timestamp
clear ready flag
```

EKF task:
```
if new IMU1 or IMU2 sample available:
    execute fusion logic (section 3)
```

---

## 7) Timestamp alignment
- Timestamp **at DRDY edge** (in ISR) into the sample struct
- EKF uses timestamps to compute `dt` accurately

---

## 8) Safety + operational modes

### 8.1 Motion mode state machine

The robot has two distinct operational flows: **Calibration** (one-time setup) and **Balancing** (normal operation).

```
═══════════════════════════════════════════════════════════════════════════════
                           CALIBRATION FLOW (one-time)
═══════════════════════════════════════════════════════════════════════════════

    ┌─────────────────┐     User initiates      ┌──────────────────┐
    │   DISARMED      │ ───────────────────────►│   CALIBRATION    │
    │                 │     calibration cmd     │  (6-face bias    │
    │                 │                         │   procedure)     │
    └─────────────────┘                         └────────┬─────────┘
                                                         │
                                                         │ All 6 faces captured
                                                         │ Bias computed & stored
                                                         ▼
                                                ┌──────────────────┐
                                                │  Save to params  │
                                                │  + REBOOT        │
                                                └──────────────────┘


═══════════════════════════════════════════════════════════════════════════════
                          BALANCING FLOW (normal operation)
═══════════════════════════════════════════════════════════════════════════════

                    ┌─────────────┐
        Power-on    │  DISARMED   │◄──────────────────────────┐
        ──────────►│  (motors    │                           │
                    │   off)      │                           │
                    └──────┬──────┘                           │
                           │                                  │
                           │ ARM command                      │
                           │ (requires valid calibration      │
                           │  stored in params)               │
                           ▼                                  │
                    ┌─────────────┐                           │
         ┌─────────►│  BALANCING  │────────────┐              │
         │          │  (active    │            │              │
         │          │   control)  │            │              │
         │          └──────┬──────┘            │              │
         │                 │                   │              │
         │                 │ |theta| > thetaKill              │
         │                 │ OR motor timeout                 │
         │                 │ OR IMU fault                     │
         │                 ▼                   │              │
         │          ┌─────────────┐            │              │
         │          │   FALLEN    │            │              │
         │          │  (motors    │            │              │
         │          │   disabled) │            │              │
         │          └──────┬──────┘            │              │
         │                 │                   │              │
         │                 │ User lifts robot  │              │
         │                 │ upright + ARM cmd │              │
         └─────────────────┘                   │              │
                                               │              │
                                               │ DISARM cmd   │
                                               │ OR fatal     │
                                               │ error        │
                                               └──────────────┘

                    ┌─────────────┐
                    │    FAULT    │◄─── Unrecoverable error
                    │  (requires  │     (e.g., both IMUs bad,
                    │   reboot)   │      motor link lost)
                    └─────────────┘
```

### 8.2 State definitions

| State | Motors | EKF | Outputs | Entry condition |
|-------|--------|-----|---------|-----------------|
| **DISARMED** | Off | Running (passive) | Iq = 0 | Power-on, DISARM cmd |
| **CALIBRATION** | Off | Collecting bias | Iq = 0 | Calibration cmd |
| **BALANCING** | Active | Running | Iq per PID | ARM cmd (with valid calib) |
| **FALLEN** | Off | Running | Iq = 0 | Tilt exceeded |
| **FAULT** | Off | Stopped | Iq = 0 | Unrecoverable error |

### 8.3 Calibration procedure (6-face bias calibration)

Calibration is a **separate, one-time procedure** that must be completed before the robot can balance.

**Procedure:**
1. User initiates calibration via command
2. Robot prompts user to place it on each of 6 faces (orientations):
   - +X up, -X up, +Y up, -Y up, +Z up, -Z up
3. At each orientation, accelerometer and gyro samples are collected for 1-2 seconds
4. Gyro bias and accelerometer scale/offset are computed from the 6-face data
5. Calibration data is **stored in `robot_params`** (persistent flash storage)
6. **Robot reboots** to apply new calibration

**Important constraints:**
- **Robot cannot enter BALANCING mode until valid calibration exists in params.**
- If calibration data is missing or invalid, ARM command is rejected.
- Calibration need only be performed once (or when sensors are replaced/remounted).

### 8.4 Balancing mode transitions

**DISARMED → BALANCING:**
- ARM command received via robot protocol
- Valid calibration data exists in params
- Robot pitch is within ±10° of upright
- Both IMUs healthy, motor link established

**BALANCING → FALLEN:**
- `|theta| > thetaKill` (default: 45°)
- OR motor link timeout (no response for > 100ms)
- OR sustained IMU fault (both IMUs unhealthy for > 200ms)

**FALLEN → BALANCING:**
- User issues ARM command
- Robot is detected upright (`|theta| < 10°`)
- PID integrators are reset

**Any → DISARMED:**
- DISARM command received
- Immediate transition, motors disabled

**Any → FAULT:**
- Unrecoverable error detected:
  - Both IMUs dead (no DRDY for > 500ms)
  - Motor link completely lost
- **Requires reboot to exit FAULT state**

### 8.5 Kill-switch behavior
```
if abs(theta) > thetaKill:
    motor_link_set_wheel_Iq(0, 0, 0)  // immediate motor disable
    state = FALLEN
    clear PID integrators
```

**Default threshold:** `thetaKill = 45°` (configurable in robot_params)

---

## 9) Dynamic tuning aids
- Log `gyro_diff`, `acc_angle_diff`, `accel_vib_index` over time
- Use them offline to set:
    - GYRO_DISAGREE_THRESH
    - ACC_ANGLE_DISAGREE_THRESH
    - VIBRATION_HIGH_THRESH
- Log Iq targets, estimated Uq (if reported by motor drivers), and battery voltage to validate the current→voltage conversion and saturation behavior.

---

## 10) Debug / fallback states
- **IMU1 BAD**: a counter of bad samples → switch to IMU2
- **Both IMUs BAD**: freeze EKF rotation state, rely on gyro only predictions
- **Vibration gate active**: suppress accel measurement updates

---

## 11) Summary of additions to original sketch
- Dual IMU inputs → EKF with Pattern A fusion
- Health checks (disagreement thresholds)
- Vibration index gating
- Temporary IMU switch/fallback
- Dynamic accel gating for EKF

---

## 12) Already in place (current algorithm snapshot)
This section summarizes what the **current MotionControl implementation** already does today (baseline), so we can clearly see what changes are required.

- **Porting approach**: Estimator + PID controller math is already ported from legacy C++ modules with minimal math changes; hardware I/O is adapted at the boundaries (sensor adapters + motor link).
- **Scheduling model**: A hardware timer drives a deterministic control tick (currently targeted at **1 kHz**), and the control tick is designed to be bounded and non-blocking.
- **Motion modes**: A mode state machine exists (or is planned) with at least DISARMED / CALIBRATION / BALANCING / FAULT, gating outputs to ensure safety.
- **Sensor adapter boundary**: IMU readings are acquired and converted into the estimator’s expected units at a single adapter boundary; missing samples are handled via valid flags / last-known sample strategy.
- **Motor link abstraction**: Motor commands are sent via a dedicated CAN
  Simple backend so that the main control loop never blocks on motor I/O.
- **Telemetry plumbing**: A low-rate telemetry path exists (or is planned) to publish estimator/controller outputs without stalling the control loop.

## 13) TODO to reach the desired final state (this document)
This section lists what must be amended so the implementation matches the **MainControl** target design (torque via estimated current, dual-IMU Pattern A, and PID-based balance + speed + steering).

### Control-rate alignment
- Ensure telemetry at **500 Hz** does not block control (use queue/ring-buffer + separate task).

### Motor command semantics (torque targets)
- Use `{tauL_ref, tauR_ref}` in Nm as the primary wheel command.
- Ensure wheel node IDs are `1` and `2`, FDCAN1 runs at 500 kbit/s, and
  periodic encoder estimates are configured at 10 ms or faster.
- Characterize and configure the mechanical torque limits before raising
  `tauMax`; the GDS68 command is a direct torque request, not a voltage target.
- Log wheel torque, encoder velocity, bus voltage, and drive fault state during
  tuning.

### Dual-IMU integration (Pattern A)
- Implement the Pattern A health metrics and gating in code:
  - `gyro_diff = ||gyro1 - gyro2||`
  - `acc_angle_diff = angle(acc1, acc2)`
  - `accel_vib_index = variance(acc1 window)` (and optionally acc2)
- Implement robust **fault detection** and **fallback switching**:
  - detect IMU1 stalls (missing DRDY) and FIFO overflows
  - switch EKF input to IMU2 when IMU1 is unhealthy
  - revert to IMU1 only after a clean recovery window
- Implement accel gating and/or dynamic measurement noise `R` adjustment based on vibration index.

### Time alignment and dataflow
- Ensure both IMU samples are timestamped at DRDY edge and stored in a ring buffer with timestamps.
- Define a deterministic rule for which sample pair a given EKF/control tick consumes (e.g., latest IMU1 sample, plus nearest-in-time IMU2 sample for validation).

### Control law implementation details
- Implement the speed→tilt reference path (`vRef → thetaRef`) with anti-windup.
- Confirm derivative term uses `thetaDot` from gyro/EKF (not numerical diff).
- Implement steering split (`uTurn`) with optional yaw damping.

### Safety + modes
- Integrate the tilt kill-switch (`thetaKill`) into the motion mode state machine.
- Define behavior for: FALLEN recovery, IMU fault, motor link timeout, and saturation.

## 14) Outstanding questions / design decisions
These must be resolved (or explicitly frozen as defaults) before final tuning.

### Steering model
- Steering uses torque differential only.
- Yaw damping uses IMU gyro Z as primary, blended with encoder-derived yaw rate:
  - `yawRate = alpha*gyroZ + (1 - alpha)*yawRate_enc`
  - `alpha = 0.8` (stage-1 default)
  - applied in `uTurn = K_turn*turnRef - K_yawDamp*yawRate`
  - log `gyroZ`, `yawRate_enc`, and `yawRate` for tuning

### Torque limits and brownout handling
- Determine `tauMax` from the motor, gearbox, and driver thermal limits.
- Consider battery-voltage-aware torque clamps and regeneration handling to
  prevent undervoltage or overvoltage faults during aggressive correction.

---

End of dual‑IMU fusion integration spec.
