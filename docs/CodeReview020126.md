# Code Review: MainControl Spec Implementation (2026-01-02)

This review covers the implementation of the MainControl.md specification for the self-balancing robot firmware.

---

## 1. Summary of Changes Reviewed

| File | Changes |
|------|---------|
| [param_storage.h](../Drivers/param_storage.h) | Added `balance_gains_t` struct, bumped `PARAM_VERSION` to 3 |
| [param_storage.c](../Drivers/param_storage.c) | Added `param_init_balance_gains()` with defaults from config_control.h |
| [MotionController.cpp](../control/MotionController.cpp) | Wired balance gains, yaw blend, damping, Iq clamping |
| [motion_control.cpp](../control/motion_control.cpp) | Added FALLEN mode, kill-switch, arm gating, fault timers |
| [motion_modes.cpp](../control/motion_modes.cpp) | State machine enum with FALLEN mode |
| [StateEstimator.cpp](../control/StateEstimator.cpp) | Dual-IMU fusion with health metrics, vibration window |
| [imu_bmi270.c](../Drivers/imu/imu_bmi270.c) | DRDY-edge timestamp via `s_dma_timestamp_ms` |
| [config_control.h](../control/config_control.h) | Balance defaults, IMU thresholds, fault timers |
| [app_main.c](../app/app_main.c) | `motion_control_apply_params()` on SET_PARAM, 500 Hz telemetry |

---

## 2. Issues and Concerns

### 2.1 HIGH: Encoder velocity not used by EKF

**Location:** [StateEstimator.cpp:326-328](../control/StateEstimator.cpp#L326-L328)

```cpp
float v_enc_ekf = NAN;
float pos_enc = NAN;
bool ekf_ok = ekf_.step(theta_acc, v_enc_ekf, pos_enc, gyro_pitch, dt_s, theta_var);
```

The EKF is passed `NAN` for both encoder velocity and position, even though `v_enc` is passed to `update()`. The spec states:

> *"The controller uses wheel velocity directly (`wL`, `wR`) for `v`; wheel velocity is not fused into the EKF."*

This is technically correct per spec, but the `v_enc` parameter to `update()` is currently unused except for logging (`lastVEnc_`). Either:
1. Remove the parameter if it's intentionally unused, or
2. Document clearly why it's passed but not used

**Recommendation:** Add comment clarifying design intent.

---

### 2.2 ~~HIGH: `motion_modes_step()` is a placeholder~~ FIXED

**Status:** Resolved on 2026-01-02

State transition logic has been moved from `motion_control_tick()` into `motion_modes_step()`.

**Changes made:**
- Added `motion_modes_input_t` struct with all data needed for transitions
- Added `motion_modes_output_t` struct with action flags (disable_motors, reset_pid)
- `motion_modes_step()` now encapsulates all transition logic:
  - BALANCING → FALLEN: kill-switch, IMU fault, motor fault
  - Any → FAULT: fatal IMU or motor link loss
- `motion_control_tick()` populates input struct and handles output actions

---

### 2.3 MEDIUM: Kill-switch check after EKF update

**Location:** [motion_control.cpp:300-316](../control/motion_control.cpp#L300-L316)

The kill-switch check happens after `s_estimator.update()` is called. If the EKF produces an invalid estimate, the kill-switch condition may not trigger correctly on the first iteration.

```cpp
if (mode == MOTION_MODE_BALANCING)
{
    if (estimate.valid && g_robot_params.balance.thetaKill > 0.0f &&
        fabsf(estimate.theta) > g_robot_params.balance.thetaKill)
```

The `estimate.valid` check is good, but the spec says:
> *"immediate motor disable"*

**Recommendation:** Consider adding a redundant safety check using raw IMU data when EKF is invalid.

---

### 2.4 ~~MEDIUM: Missing calibration validation for ARM~~ FIXED

**Status:** Resolved on 2026-01-02

**Spec requirement (Section 8.4):**
> *"Valid calibration data exists in params"*
> *"Robot cannot enter BALANCING mode until valid calibration exists in params."*

**Changes made:**
- Added `motion_control_is_calibrated()` helper function
- Checks if any gyro or accel bias is non-zero for primary IMU (BMI270)
- Fresh/uncalibrated robots have all biases = 0, so ARM is rejected
- Called early in `motion_control_can_arm()` before other checks

---

### 2.5 ~~MEDIUM: Vibration window sample count mismatch~~ FIXED

**Status:** Resolved on 2026-01-02

**Spec:**
> *"VIB_BUFFER_SIZE = 40 samples // 100ms × 400Hz"*

**Changes made:**
- Updated `IMU_VIB_WINDOW_SAMPLES` from 50U to 40U in config_control.h
- Added comment documenting the derivation: "VIB_WINDOW_MS = 100 ms, ACCEL_ODR = 400 Hz → 40 samples"
- Reduced RAM usage by 80 bytes (10 floats × 2 windows)

---

### 2.6 LOW: Telemetry period mismatch

**Spec:**
> *"Telemetry: 500 Hz"*

**app_main.c:**
```cpp
#define APP_TELEM_PERIOD_MS 2U
```

At 2ms period, telemetry runs at 500 Hz. However, `app_idle_tick()` runs in the main loop without a guaranteed period, so actual telemetry rate depends on loop timing.

**Recommendation:** Document that 500 Hz is best-effort, or use timer interrupt for guaranteed rate.

---

### 2.7 ~~LOW: Control rate fallback magic number~~ FIXED

**Status:** Resolved on 2026-01-02

**Location:** [app_main.c:234-237](../app/app_main.c#L234-L237)

The fallback value now references `CONTROL_DEFAULT_HZ` from config_control.h instead of a magic number.

---

### 2.8 LOW: PID integral limit calculation

**Location:** [MotionController.cpp:183-184](../control/MotionController.cpp#L183-L184)

```cpp
(_velPid_Ki > 1e-6f && _velPid_iMax > 0.0f) ? (_velPid_iMax / _velPid_Ki)
                                            : _maxPitchTarget
```

The integral limit for the velocity PID uses `_velPid_iMax / _velPid_Ki`, which can produce unexpected values if gains are misconfigured. Consider adding bounds checking.

---

## 3. Missing Relative to MainControl.md Spec

### 3.1 CALIBRATION mode not implemented

**Spec Section 8.3:**
> *"6-face bias calibration procedure"*

The `MOTION_MODE_CALIBRATION` enum value exists but the actual calibration procedure is not implemented:
- No calibration state machine
- No 6-face data collection
- No bias computation
- No reboot trigger after calibration

**Status:** Documented as future work (reasonable for stage-1).

---

### 3.2 FAULT state transition not fully wired

**Spec Section 8.4:**
> *"Both IMUs dead (no DRDY for > 500ms) → FAULT"*

The fault detection exists in motion_control.cpp:321-328 with `IMU_FAULT_FATAL_MS`, but:
- The threshold is only checked when IMU samples become stale
- There's no explicit "no DRDY" counter at the driver level

**Status:** Partially implemented; relies on sample staleness rather than DRDY timeout.

---

### 3.3 Motor driver telemetry for validation

**Spec Section 13 (TODO):**
> *"Add (optional but recommended) telemetry from motor drivers for: estimated_Uq, bus_voltage, shaft_velocity, and saturation flags"*

Motor link provides wheel velocities but not:
- Estimated Uq
- Bus voltage
- Saturation flags

**Status:** Not implemented (acceptable for stage-1).

---

### 3.4 Dynamic R adjustment not fully per-spec

**Spec Section 3.6:**
> *"Use them to adjust... EKF accel measurement noise R dynamically"*

The implementation multiplies `R` by `EKF_TUNE_R_MULT` (20×) when vibration is high:

```cpp
if (gate_accel)
{
    theta_var *= EKF_TUNE_R_MULT;
}
```

This is binary (high/low) rather than continuous adjustment based on vibration magnitude.

**Status:** Acceptable simplification for stage-1; may want continuous scaling later.

---

### 3.5 Yaw damping blend logging

**Spec Section 14:**
> *"log gyroZ, yawRate_enc, and yawRate for tuning"*

The debug logging exists in app_main.c via `motion_control_get_yaw_debug()`, but it's only logged at `APP_LOG_PERIOD_MS` (500ms) intervals, not in telemetry.

**Status:** Logging exists but not at full rate. Add to telemetry struct if needed.

---

## 4. Improvement Suggestions

### 4.1 Extract state transition logic

Move the transition logic from `motion_control_tick()` into `motion_modes_step()` for better separation of concerns:

```cpp
void motion_modes_step(const motion_modes_input_t *input)
{
    // Centralize all state transitions here
}
```

---

### 4.2 Add calibration validity flag

Add to `robot_params_t`:
```c
uint8_t calibration_valid;  /* 0 = uncalibrated, 1 = valid */
```

Set during 6-face calibration procedure, check in `motion_control_can_arm()`.

---

### 4.3 Consider Joseph form for EKF P update

The current TinyEKF uses the standard `P = (I - KH) * P` form which can lose positive-definiteness due to numerical issues. The Joseph form is more robust:

```cpp
// Joseph form: P = (I - KH) * P * (I - KH)' + K * R * K'
```

This may help if filter divergence is observed during aggressive maneuvers.

---

### 4.4 Add fault reason enum

When transitioning to FAULT state, store the reason for debugging:

```cpp
typedef enum {
    FAULT_NONE = 0,
    FAULT_BOTH_IMU_DEAD,
    FAULT_MOTOR_LINK_LOST,
    FAULT_WATCHDOG,
} fault_reason_t;
```

---

### 4.5 ~~Telemetry struct for EKF/health data~~ IMPLEMENTED

**Status:** Implemented on 2026-01-02

**Changes made:**
- Added `robot_telem_v2_t` struct to robot_protocol.h with EKF state and IMU health fields
- Added `ROBOT_MSG_TELEM_FRAME_V2` message type (0x02)
- Added `motion_control_get_estimate()` and `motion_control_get_control_output()` accessor functions
- Updated `app_send_telem()` to populate all fields at 500 Hz:
  - EKF state: theta, thetaDot, x, xDot, gyroBias, estimate_valid
  - IMU health: active sensor, gate_accel, gyro_diff_dps, vib_rms_g
  - Wheel velocities: left/right rad/s
  - Control outputs: iq_left, iq_right, pitch_target_rad
  - Motion mode and status bits

---

## 5. Test Coverage Recommendations

| Area | Suggested Tests |
|------|-----------------|
| Kill-switch | Verify motors disable when `theta > thetaKill` |
| IMU failover | Remove BMI270 power, verify switch to ICM42688 |
| Arm rejection | Verify ARM rejected when not upright |
| Fault timeout | Verify FAULT state after 500ms IMU dropout |
| Parameter RPC | Verify live gain update via SET_PARAM |

---

## 6. Conclusion

The implementation is well-aligned with the MainControl.md specification for stage-1 functionality. The core control loop, balance gains, yaw blending, and safety transitions are correctly wired.

**Key strengths:**
- Clean separation between controller (MotionController), estimator (StateEstimator), and modes
- Proper DRDY-edge timestamping for IMU samples
- Dual-IMU health metrics with switchover logic
- Parameter persistence with wear-leveling

**Priority fixes:**
1. ~~Add calibration validity check to `motion_control_can_arm()`~~ DONE
2. ~~Align `IMU_VIB_WINDOW_SAMPLES` to spec (40 vs 50)~~ DONE
3. ~~Consolidate state transition logic in `motion_modes_step()`~~ DONE

**Deferred (acceptable for stage-1):**
- 6-face calibration procedure
- Motor driver telemetry (Uq, bus voltage)
- Continuous R scaling based on vibration magnitude
