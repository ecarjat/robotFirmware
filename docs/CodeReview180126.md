# Firmware Code Review - January 18, 2026

## Executive Summary

This is a well-structured self-balancing robot firmware with a 7-state EKF, dual-controller (PID/LQR) architecture, dual-IMU redundancy, and comprehensive blackbox logging. The code demonstrates solid embedded systems practices but has areas requiring attention for robustness and scalability to more complex motions.

---

## 1. Architecture Overview

### Strengths
- **Clean separation of concerns**: Control logic (`motion_control.cpp`), state estimation (`StateEstimator.cpp`), and hardware abstraction are well-separated
- **Super-loop with priority scheduling**: The main loop in `app_main.c:43-81` correctly prioritizes control loop execution over background tasks
- **Lock-free SPSC queue** for blackbox logging avoids ISR/main contention
- **Dual-IMU redundancy** with automatic failover in StateEstimator

### Concerns
- **Mixed C/C++ codebase** creates friction - control system is C++, drivers are C
- **Global state scattered** across multiple translation units (`g_robot_params`, `s_controller`, etc.)
- **No formal state machine** for overall system modes (only motion_modes handles control state)

---

## 2. Control System Review

### MotionController (`app/control/MotionController.cpp`)

**Strengths:**
- PID with anti-windup (lines 280-294): Conditional integration prevents wind-up during saturation
- LQR/PID blending with smooth ramp transitions (lines 128-160)
- Derivative filtering (DERIVATIVE_FILTER_ALPHA) to reduce noise

**Issues:**

1. ~~**Cascaded controller coupling** (lines 329-388): The velocity-to-pitch cascade PID followed by pitch PID creates complex dynamics. The inner pitch loop uses thetaDot from the EKF, but when saturated, the outer loop continues integrating~~
   **✅ FIXED**: Cross-controller anti-windup implemented in `MotionController.cpp:388-431`. When pitch loop saturates and velocity error would worsen windup, applies exponential decay (τ=0.5s) to velocity integral. Diagnostic field `cross_antiwindup_active` added to track engagement.

2. **LQR gains hardcoded in constructor** (lines 63-73): No runtime validation of gain magnitudes or stability
   **Recommendation**: Add sanity checks for LQR gains (e.g., K values within expected ranges)

3. **Rate limiting on u_sum only** (lines 401-409): Rate limiting is applied to combined output but not to differential turn command
   **Recommendation**: Add rate limiting to yaw control for smoother turning

### StateEstimator (`app/control/StateEstimator.cpp`)

**Strengths:**
- IMU health monitoring with hysteresis-based switching (lines 263-314)
- Vibration-based accelerometer gating (lines 323-331)
- Cross-sensor validation (gyro diff, accel angle diff)

**Issues:**

1. **IMU switching during motion** (lines 274-276): Switching IMUs mid-balance could introduce transients
   ```cpp
   if (secondary_healthy && (now_ms - primary_unhealthy_since_ms_) >= IMU_SWITCH_TO_SECONDARY_MS...)
       use_secondary_ = true;  // Abrupt switch
   ```
   **Recommendation**: Add EKF reset/damping on IMU switch, or blend sensors during transition

2. **Theta from accelerometer** (lines 348-352): Uses `atan2(-accel_x, accel_yz)` which assumes robot frame alignment. No validation that `accel_yz` is reasonable:
   ```cpp
   float accel_yz = sqrtf(accel_body[1] * accel_body[1] + accel_body[2] * accel_body[2]);
   if (accel_yz > 1e-6f && isfinite(accel_norm_g)) {
       theta_acc = atan2f(-accel_body[0], accel_yz);
   }
   ```
   **Recommendation**: Gate theta_acc when `accel_yz` is very small (near gimbal lock)

### EKF (`app/control/ekf/BalancerEKF.cpp`)

**Strengths:**
- 7-state filter with yaw bias estimation
- Innovation gating with graceful degradation (inflate R instead of hard reset)
- Post-reset damping period (lines 258-261)

**Issues:**

1. **Partial reset preserves potentially corrupt bias** (lines 43-51):
   ```cpp
   float saved_bias = ekf_.x[4];  // Could be bad if sensors were faulty
   ```
   **Recommendation**: Consider resetting bias to 0 or reducing its covariance during partial reset

2. **Process noise tuning** (lines 84-90): Fixed Q matrix - no adaptation for different operating conditions (e.g., stationary vs. moving)
   **Recommendation**: Consider adaptive Q based on velocity magnitude

3. **No observability check**: The EKF assumes all states are observable, but position (x) is only weakly observable through velocity integration
   **Recommendation**: For complex motions, consider removing position state or using external position sensing

---

## 3. Blackbox Logging Review (`app/logging/blackbox.c`)

**Strengths:**
- Lock-free SPSC queue with proper memory barriers (lines 26-27)
- Async QSPI writes with state machine (lines 200-332)
- Pre-erase ahead of write pointer (lines 334-369)
- Dual metadata slots for wear leveling

**Issues:**

1. **Queue overflow silently drops records** (lines 173-176):
   ```cpp
   if (next_head == tail) {
       s_dropped_records++;
       return;  // Silent drop
   }
   ```
   **Recommendation**: Consider a flag or telemetry to indicate logging backpressure

2. **Meta save can block write path** (lines 241-243):
   ```cpp
   if (s_meta_op_inflight != LOG_META_OP_NONE) {
       return;  // No logging while meta is being saved
   }
   ```
   **Recommendation**: Meta saves are relatively rare; this is acceptable but should be documented

3. **No ring buffer overflow detection** in `log_validate_ring_tail()`: If the ring wraps during a long dump, old data could be overwritten
   **Recommendation**: Pause logging during dump or detect wrap-around

---

## 4. Driver Layer Review

### QSPI Flash (`app/drivers/qspi_w25q64.c`)

- Async write/erase state machine is well-implemented
- HAL callback integration for DMA completion

**Issue**: Error reporting is limited to a single state variable. Consider per-operation error tracking.

### Parameter Storage (`app/drivers/param_storage.c`)

**Strengths:**
- Wear-leveling with append-only writes
- CRC validation with sequence numbers
- Safety check preventing saves during balancing (line 385)

**Issues:**

1. ~~**Flash erase blocks for 1-2 seconds** (lines 117-148): Interrupts disabled during erase~~
   **✅ FIXED**: Comprehensive documentation added to `param_storage.h:41-79` covering:
   - IWDG configuration (4s timeout: prescaler=256, reload=500, LSI=32kHz)
   - Flash timing analysis (1-2s typical, 4s max vs 4s watchdog)
   - Mitigations (wdog refresh before disable_irq, balancing mode check)
   - Risk assessment and recommendations

2. **No migration path** for parameter version changes (lines 248-252)
   **Recommendation**: Add version migration handlers before reaching production

### Motor Link (`app/drivers/motors/motor_link.c`)

- Comprehensive telemetry handling with stale/late detection
- Direction correction applied per-motor
- Command timeout tracking

---

## 5. Robustness Assessment

### Error Handling Gaps

| Component | Issue | Severity | Status |
|-----------|-------|----------|--------|
| EKF | No NaN propagation check on state vector | High | **✅ FIXED** |
| MotionController | NaN/inf in control output sent to motors | High | **✅ FIXED** |
| MotionController | Division by zero possible if motorKt=0 (line 346) | Medium | **✅ FIXED** |
| StateEstimator | isnan checks but no isinf checks | Medium | Open |
| blackbox | QSPI errors increment counter but don't trigger fallback | Low | Open |

**NaN/inf fix details**: Added `is_control_value_safe()` validation in `motion_control.cpp` before motor commands. Invalid values trigger emergency zero output, error log, and controller reset. Defense-in-depth checks also added in `motor_link.c` at driver level.

**EKF NaN fix details**: Added `isStateValid()` method in `BalancerEKF.cpp` that checks all 7 state variables for finite values. Validation runs after both `ekf_predict()` and `ekf_update()`. On NaN detection, EKF resets to safe state and increments `nan_reset_count_` for diagnostics.

**motorKt fix details**: Added validation in `MotionController::setRobotParams()` that checks motorKt > kGainEpsilon. Invalid values (zero or negative) fall back to PARAM_MOTOR_KT default with warning log. The division at line 346 was already guarded, but this adds early validation at parameter load time.

### Timing/Concurrency

| Issue | Location | Risk |
|-------|----------|------|
| IMU timestamp rollover not handled | StateEstimator:145 | Low (requires 49 days uptime) |
| s_next_seq non-atomic increment | blackbox.c:191 | Low (SPSC safe) |
| Flash operations block control loop if called at wrong time | param_storage.c | Mitigated by mode check |

### Recommended Additions

1. **State sanity checks** in control loop:
   ```cpp
   if (!isfinite(state.theta) || !isfinite(state.thetaDot)) {
       // Emergency stop
   }
   ```

2. **Watchdog checkpoints** at more granular levels (currently only in idle tick)

3. **Error counters exposed via telemetry** for all subsystems

---

## 6. Restructuring Recommendations for Complex Motions

### Current Limitations

The current architecture is optimized for **static balance and simple forward/turn**. For more complex motions (trajectory following, obstacle avoidance, dynamic maneuvers), the following areas need attention:

### 6.1 Motion Planning Layer (New)

**Current**: Teleop commands go directly to velocity setpoint
```
Teleop -> MotionController -> Motor Commands
```

**Recommended**: Add trajectory planning layer
```
High-level Goal -> Motion Planner -> Trajectory -> MotionController -> Motors
                                         ^
                                 Constraint Checker
```

**Implementation**:
- Create `trajectory_planner.h/cpp` with waypoint interpolation
- Add velocity/acceleration profiling
- Integrate with control loop at lower frequency (10-50Hz)

### 6.2 State Machine Refactoring

**Current** (`motion_modes.cpp`): Single state machine handles arm/disarm/fault
```cpp
enum motion_mode_t { DISARMED, BALANCING, FALLEN, FAULT };
```

**Recommended**: Hierarchical state machine
```
SystemState
+-- BOOT
+-- CALIBRATION
+-- READY
|   +-- IDLE
|   +-- BALANCING
|   |   +-- STATIONARY
|   |   +-- MOVING
|   |   +-- MANEUVERING
|   +-- RECOVERY (attempting to stand up)
+-- FAULT
+-- SHUTDOWN
```

### 6.3 Control Architecture Changes

**Current**: Fixed cascade (velocity PID -> pitch PID) + LQR blend

**Recommended** for complex motions:

1. **Model Predictive Control (MPC)** for trajectory tracking:
   - Better constraint handling
   - Predictive obstacle avoidance
   - Coordinated motion planning

2. **Feedforward path**:
   ```cpp
   u = u_feedback + u_feedforward
   // u_feedforward from dynamics model for known trajectory
   ```

3. **Separate yaw controller**:
   - Current: Yaw is a differential addon
   - Recommended: Independent yaw tracking with its own state estimator

### 6.4 State Estimator Enhancements

For complex motions, consider:

1. **Add wheel odometry** to position estimate (partially done, but not fully integrated)
2. **Slip detection** using IMU vs. wheel disagreement
3. **External localization fusion** (if adding camera/lidar)

### 6.5 Suggested File Structure Refactoring

```
app/
+-- control/
|   +-- core/
|   |   +-- BalanceController.cpp    # Inner balance loop
|   |   +-- VelocityController.cpp   # Velocity tracking
|   |   +-- YawController.cpp        # Independent yaw control
|   +-- planning/
|   |   +-- TrajectoryPlanner.cpp    # NEW
|   |   +-- PathFollower.cpp         # NEW
|   |   +-- MotionPrimitives.cpp     # NEW
|   +-- estimation/
|   |   +-- StateEstimator.cpp       # (existing)
|   |   +-- OdometryEstimator.cpp    # NEW
|   |   +-- SensorFusion.cpp         # Consolidate fusion logic
|   +-- safety/
|       +-- SafetyMonitor.cpp        # NEW - consolidate fault detection
|       +-- LimitEnforcer.cpp        # NEW - constraint enforcement
+-- drivers/
    +-- (unchanged)
```

### 6.6 Parameter System Enhancement

For complex motions, parameters need:
1. **Runtime-modifiable gains** without flash writes
2. **Parameter groups** (balance, trajectory, limits)
3. **Profile switching** (aggressive vs. conservative modes)

---

## 7. Priority Actions

### High Priority (Safety)
1. ~~Add NaN/inf checks to control output before motor commands~~ **✅ DONE** - `motion_control.cpp`, `motor_link.c`
2. ~~Add cross-controller anti-windup~~ **✅ DONE** - `MotionController.cpp:388-431`
3. ~~Document flash erase timing vs. watchdog requirements~~ **✅ DONE** - `param_storage.h:41-79`

### Medium Priority (Reliability)
1. ~~Add EKF state validation on each step~~ **✅ DONE** - `BalancerEKF.cpp:174-180, 302-308`
2. Implement telemetry for error counters
3. Add parameter version migration

### For Complex Motions
1. Abstract control loop to support different controllers
2. Add trajectory planning interface
3. Refactor state machine to hierarchical design
4. Separate yaw control into independent subsystem

---

## 8. Conclusion

The firmware is well-engineered for its current purpose of self-balancing with teleop control. The dual-IMU redundancy, EKF-based state estimation, and comprehensive logging provide a solid foundation.

**Update (Jan 18, 2026)**: All high-priority safety items have been addressed:
- NaN/inf protection added at control and driver layers
- Cross-controller anti-windup prevents velocity integral windup during pitch saturation
- Flash timing vs. watchdog constraints fully documented

For evolution toward more complex motions, the primary investment should be in:
1. A trajectory planning layer
2. A hierarchical state machine
3. Controller abstraction to support MPC or other advanced control strategies

The remaining medium-priority items (EKF state validation, error telemetry, parameter migration) should be addressed before adding complexity to ensure a stable foundation.
