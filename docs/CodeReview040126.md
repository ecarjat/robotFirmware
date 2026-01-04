# Code Review: Self-Balancing Robot Firmware (2026-01-04)

This review covers the STM32H723 firmware for the two-wheeled self-balancing robot, building upon the previous review from 2026-01-02.

**Update (2026-01-04):** Several critical and high-priority issues have been fixed during this review session. See individual sections marked with **[FIXED]** for details.

---

## 1. Executive Summary

The firmware has matured significantly with a well-structured architecture. Key strengths include robust communication protocols (RobustBinaryIO v2), comprehensive dual-IMU fusion with health monitoring, and a clean state machine for motion mode management.

**Issue Status:**
| Severity | Original | Fixed | Remaining |
|----------|----------|-------|-----------|
| Critical | 2 | 2 | 0 |
| High Priority | 4 | 3 | 1 |
| Medium Priority | 7 | 1 | 6 |
| Low Priority | 5 | 0 | 5 |

---

## 2. Critical Issues

### 2.1 ~~CRITICAL: Interrupt Latency During Flash Operations~~ [FIXED]

**Location:** [param_storage.c:346-365](../Drivers/param_storage.c#L346-L365)

**Original Issue:** Flash erase operations disable all interrupts (`__disable_irq()`) for potentially hundreds of milliseconds, which could cause the robot to fall if balancing during parameter save.

**Fix Applied:**
- Added `param_storage_can_save()` function that checks if robot is in BALANCING mode
- Added `PARAM_ERR_BUSY` error code
- `param_storage_save()` now returns `PARAM_ERR_BUSY` if called while balancing

```c
bool param_storage_can_save(void)
{
    motion_mode_t mode = motion_modes_get();
    return (mode != MOTION_MODE_BALANCING);
}

int param_storage_save(const robot_params_t *params)
{
    // ...
    if (!param_storage_can_save())
    {
        return PARAM_ERR_BUSY;
    }
    // ...
}
```

---

### 2.2 ~~CRITICAL: Potential Race Condition in IMU Scheduler~~ [FIXED]

**Location:** [imu_sched.c:232-236](../drivers/imu/imu_sched.c#L232-L236)

**Original Issue:** `s_pending_mask` is modified in `imu_sched_set_pending()` which is called from EXTI ISRs. Reading without protection creates a TOCTOU race.

**Fix Applied:** Added interrupt disable/enable around the pending mask read:

```c
void imu_sched_run(void)
{
    // ... atomic flag check ...

    /* Read pending mask atomically to avoid race with EXTI ISRs
     * that call imu_sched_set_pending(). */
    __disable_irq();
    uint32_t pending = s_pending_mask & s_enabled_mask;
    __enable_irq();

    // ...
}
```

---

## 3. High Priority Issues

### 3.1 HIGH: Control Loop Timing Not Guaranteed

**Location:** [app_main.c:310-336](../app/app_main.c#L310-L336)

The control loop runs in `app_idle_tick()` which is a best-effort polling loop, not a timer interrupt:

```c
if ((now - s_last_control_ms) >= control_period_ms) {
    s_last_control_ms = now;
    motion_control_tick(now);
}
```

**Issues:**
1. Jitter from UART/telemetry processing
2. Missed deadlines accumulate (no "catch-up" logic)
3. Worst-case latency is unbounded

**Recommendation:**
- Use a hardware timer (TIM2/TIM3) to trigger control loop via interrupt
- Implement control in a high-priority ISR or flag-based approach
- At minimum, add jitter monitoring to telemetry

---

### 3.2 ~~HIGH: Motor Link Infinite Retry Loops~~ [FIXED]

**Location:** [motor_link.c:1014-1040](../drivers/motors/motor_link.c#L1014-L1040)

**Original Issue:** Telemetry register setup had unbounded `while` loops that could hang forever if motor driver was unresponsive.

**Fix Applied:** Replaced all 4 infinite `while` loops with 5-retry limits:

```c
int telem_retries = 5;
while (!motor_link_port_set_telemetry_registers(&s_left_port, telem_regs,
                                                sizeof(telem_regs), 0U,
                                                MOTOR_LINK_ACK_TIMEOUT_MS) &&
       telem_retries-- > 0)
{
    APP_LOG_ERROR("Motor link: left telemetry setup failed, retrying (%d left)", telem_retries);
    HAL_Delay(100);
}
if (telem_retries < 0)
{
    APP_LOG_ERROR("Motor link: left telemetry setup failed after retries");
}
```

Same pattern applied to right motor and telemetry rate configuration.

---

### 3.3 ~~HIGH: EKF Reset on Innovation Gate Triggers State Discontinuity~~ [FIXED]

**Location:** [BalancerEKF.cpp:39-60, 156-186](../control/ekf/BalancerEKF.cpp#L39-L60)

**Original Issue:** When innovation gating triggered, the EKF was completely reset, losing velocity and bias estimates and causing large transients.

**Fix Applied:** Implemented all three recommendations:

1. **Partial reset** - Added `partialReset()` method that preserves velocity and gyro bias:
```cpp
void BalancerEKF::partialReset(float theta_init, float pos_init)
{
    float saved_vel = ekf_.x[3];
    float saved_bias = ekf_.x[4];
    // Reset theta and position only, restore vel/bias
    ekf_.x[3] = saved_vel;
    ekf_.x[4] = saved_bias;
    damping_steps_remaining_ = POST_RESET_DAMPING_STEPS;
}
```

2. **Post-reset damping** - Added 10-step damping period with 5x measurement variance inflation

3. **Innovation gating instead of hard reset** - For theta innovations, inflate R by 100x instead of resetting:
```cpp
if (bad_theta) {
    measurement_var *= INNOV_GATE_R_MULTIPLIER;  // 100x
}
```

Position/velocity errors still trigger partial reset; theta errors use soft gating.

---

### 3.4 ~~HIGH: Velocity PID Integral Not Reset on Mode Transitions~~ [FIXED]

**Location:** [motion_control.cpp:190-200](../control/motion_control.cpp#L190-L200)

**Original Issue:** When transitioning from DISARMED to BALANCING, the PID state was NOT reset. If integral windup occurred in a previous balance session, it persisted.

**Fix Applied:** Added `MOTION_MODE_BALANCING` to the list of modes that trigger PID reset:

```cpp
void motion_control_set_mode(motion_mode_t mode)
{
    motion_modes_set(mode);
    // Reset PID state on any mode that shouldn't carry over integral terms.
    // This includes entering BALANCING to prevent windup from previous sessions.
    if (mode == MOTION_MODE_DISARMED || mode == MOTION_MODE_FAULT ||
        mode == MOTION_MODE_FALLEN || mode == MOTION_MODE_BALANCING)
    {
        s_controller.resetPidState();
    }
}
```

---

## 4. Medium Priority Issues

### 4.1 MEDIUM: Telemetry Send Failure Silent After First Log

**Location:** [app_main.c:1609-1620](../app/app_main.c#L1609-L1620)

Failed telemetry sends are logged but no corrective action is taken:

```c
if (!app_link_send(ROBOT_MSG_TELEM_FRAME_V2, 0U, ...)) {
    APP_LOG_ERROR("Failed to send telem frame reason=%s ...");
}
```

**Recommendation:**
- Add telemetry failure counter to status byte
- Implement exponential backoff on repeated failures
- Consider link restart after N consecutive failures

---

### 4.2 ~~MEDIUM: Hardcoded GPIO Pin for CTS Check~~ [FIXED]

**Location:** [app_main.c:1523-1527](../app/app_main.c#L1523-L1527)

**Original Issue:** CTS check used hardcoded GPIO that was no longer needed.

**Fix Applied:** Removed the entire CTS check block as it was obsolete:

```c
// REMOVED:
// if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
//     s_link_send_last_err = APP_LINK_SEND_ERR_CTS_BLOCKED;
//     return false;
// }
```

---

### 4.3 MEDIUM: Vibration Window RMS Calculation Accumulates Error

**Location:** [StateEstimator.cpp:155-167](../control/StateEstimator.cpp#L155-L167)

The sliding window RMS uses incremental sum-of-squares:

```cpp
w->sum_sq += (delta * delta) - (old * old);
w->rms = sqrtf(w->sum_sq / (float)w->count);
```

**Issue:** Floating-point subtraction accumulates rounding errors over time. After millions of samples, `sum_sq` could become negative (NaN RMS).

**Recommendation:**
- Periodically recompute sum_sq from scratch (every N samples)
- Or add bounds check: `if (w->sum_sq < 0.0f) w->sum_sq = 0.0f;`

---

### 4.4 MEDIUM: Motor Direction Configuration Not Validated

**Location:** [motor_link.c:1224-1231](../drivers/motors/motor_link.c#L1224-L1231)

Motor direction is stored as float (-1.0 or 1.0) derived from int8_t:

```c
void motor_link_set_motor_directions(int8_t left_dir, int8_t right_dir)
{
    float left = (left_dir < 0) ? -1.0f : 1.0f;
    float right = (right_dir < 0) ? -1.0f : 1.0f;
```

**Issue:** If `left_dir` or `right_dir` is 0 (unconfigured default), motors run in positive direction. This could be unexpected.

**Recommendation:** Add validation and/or explicit zero handling.

---

### 4.5 MEDIUM: Calibration Check Only Looks at BMI270

**Location:** [motion_control.cpp:219-240](../control/motion_control.cpp#L219-L240)

Calibration validity only checks the primary IMU (BMI270):

```cpp
static bool motion_control_is_calibrated(void)
{
    const imu_calib_t *calib = &g_robot_params.imu_bmi270;
    // ... only checks BMI270 ...
}
```

**Issue:** If the system switches to ICM42688 due to BMI270 failure, the secondary IMU calibration is not verified.

**Recommendation:** Check calibration of both IMUs, or at least the active IMU.

---

### 4.6 MEDIUM: Magic Numbers in PID Integral Limit Calculation

**Location:** [MotionController.cpp:183-184](../control/MotionController.cpp#L183-L184)

The integral limit calculation has implicit assumptions:

```cpp
(_velPid_Ki > 1e-6f && _velPid_iMax > 0.0f) ? (_velPid_iMax / _velPid_Ki)
                                            : _maxPitchTarget
```

**Issues:**
- The fallback to `_maxPitchTarget` as integral limit is unintuitive
- `1e-6f` threshold is arbitrary and undocumented

**Recommendation:** Add comments explaining the logic, or use explicit default values.

---

### 4.7 MEDIUM: No Watchdog in Control Path

**Location:** [app_main.c:311-312](../app/app_main.c#L311-L312)

Watchdog is refreshed at start of idle tick:

```c
debug_wdog_refresh();
WDOG_CHECKPOINT(WDOG_CP_IDLE_LOOP);
```

**Issue:** If the control loop takes too long or hangs, the watchdog is still refreshed because it's at the loop start. A control loop hang won't trigger reset.

**Recommendation:** Move watchdog refresh to end of control tick, or add a deadline check.

---

## 5. Low Priority Issues

### 5.1 LOW: Commented-Out Debug Code Left in Source

**Location:** Multiple files (e.g., [app_main.c:193-196](../app/app_main.c#L193-L196))

```c
// for(int i=0; i <10 ; i++){
HAL_Delay(2000);
// WDOG_CHECKPOINT(WDOG_CP_APP_INIT_START);
// }
```

**Recommendation:** Remove or use proper `#if DEBUG` guards.

---

### 5.2 LOW: APP_TELEM_PERIOD_MS Mismatch with Spec

**Location:** [app_main.c:38](../app/app_main.c#L38)

```c
#define APP_TELEM_PERIOD_MS 500U
```

Previous review mentioned "500 Hz telemetry" but actual period is 500ms (2 Hz). This appears intentional now but should be documented.

---

### 5.3 LOW: Unused Function `applyVelocitySlew`

**Location:** [MotionController.cpp:251-271](../control/MotionController.cpp#L251-L271)

`applyVelocitySlew()` is implemented but never called.

**Recommendation:** Either use it in the control path or remove to reduce code size.

---

### 5.4 LOW: Motor Link Uses Blocking HAL_Delay in Init

**Location:** [motor_link.c:1011](../drivers/motors/motor_link.c#L1011)

```c
HAL_Delay(400);
```

**Recommendation:** Document why this delay is needed (motor driver cold boot?).

---

### 5.5 LOW: Frame Parser Buffer Size Not Configurable

**Location:** [motor_link_framing.h](../drivers/motors/motor_link_framing.h)

`FRAME_MAX_UNESCAPED_SIZE` is hardcoded. For different applications, this may need adjustment.

---

## 6. Positive Observations

### 6.1 Robust Binary Framing Protocol (v2)

The `motor_link_framing.c` implementation is excellent:
- SLIP-style byte stuffing for reliable framing
- CRC-32 with hardware acceleration on STM32H7
- Proper escape sequence handling
- Good error counters (sync_losses, crc_errors)

### 6.2 Dual-IMU Fusion with Health Monitoring

`StateEstimator.cpp` implements sophisticated sensor fusion:
- Automatic failover from BMI270 to ICM42688
- Configurable dwell times to prevent oscillation
- Vibration gating with hysteresis
- Angular disagreement detection

### 6.3 Motion Mode State Machine

`motion_modes.cpp` cleanly separates state transition logic:
- BALANCING -> FALLEN transitions with multiple trigger sources
- Clear reason logging for debugging
- Output struct for action flags (disable_motors, reset_pid)

### 6.4 Parameter Storage with Wear Leveling

`param_storage.c` uses append-only writes:
- Extends flash endurance by distributing writes
- Sequence numbers for recovery after power loss
- CRC validation on load

### 6.5 D-Cache Handling

Proper cache management throughout:
- DMA buffers in `.dma_buffer` section
- `SCB_InvalidateDCache_by_Addr` after DMA reads
- `SCB_CleanDCache_by_Addr` before DMA writes

---

## 7. Architecture Recommendations

### 7.1 Consider RTOS for Timing Guarantees

The cooperative multitasking model works but provides no timing guarantees. For a safety-critical balance controller, consider:
- FreeRTOS with priority-based preemption
- High-priority control task (1ms period)
- Lower-priority communication tasks

### 7.2 Add Telemetry for Control Loop Diagnostics

Current telemetry lacks:
- Control loop execution time
- Jitter measurements
- IMU sample age at time of use
- PID term breakdown (P, I, D components)

### 7.3 Implement Motor Driver Telemetry Parsing

Per MainControl.md spec, the following are recommended but not implemented:
- Bus voltage monitoring
- Estimated Uq
- Saturation flags

---

## 8. Code Quality Observations

### 8.1 Consistent Error Handling

Good pattern seen throughout:
```c
if (port == NULL || port->huart == NULL) {
    return false;
}
```

### 8.2 Clear Separation of Concerns

- `motion_control.cpp`: Orchestration
- `MotionController.cpp`: Control algorithm
- `StateEstimator.cpp`: Sensor fusion
- `motion_modes.cpp`: State machine

### 8.3 Appropriate Use of Volatile

Correctly used for ISR-shared variables:
```c
static volatile uint32_t s_pending_mask = 0U;
static volatile int8_t s_active_sensor = -1;
```

---

## 9. Testing Recommendations

| Area | Test Case |
|------|-----------|
| Flash Operation Safety | Attempt param save while balancing - verify rejection |
| Motor Init Timeout | Disconnect motor during boot - verify bounded retry |
| EKF Reset Behavior | Inject large theta error - verify recovery smoothness |
| IMU Failover | Disable BMI270 SPI - verify ICM42688 takeover |
| Control Loop Jitter | Measure control tick timestamps - verify < 10% jitter |
| Vibration Gating | Apply vibration source - verify gate_accel flag |

---

## 10. Summary of Action Items

### Must Fix (Before Production)
1. [x] ~~Add motion mode check before flash operations~~ **[FIXED]**
2. [x] ~~Fix race condition in IMU scheduler pending mask~~ **[FIXED]**
3. [x] ~~Add retry limits to motor link initialization~~ **[FIXED]**

### Should Fix
4. [ ] Move control loop to timer-based execution
5. [x] ~~Reset PID state when entering BALANCING mode~~ **[FIXED]**
6. [ ] Add bounds check to vibration RMS accumulator
7. [x] ~~Improve EKF reset strategy (partial reset vs full)~~ **[FIXED]**

### Nice to Have
8. [ ] Add control loop diagnostics to telemetry
9. [ ] Implement motor driver voltage/saturation telemetry
10. [ ] Clean up commented-out debug code

### Also Fixed
11. [x] ~~Remove obsolete CTS check~~ **[FIXED]**

---

## 11. Comparison with Previous Review (2026-01-02)

| Issue | Previous Status | Current Status |
|-------|-----------------|----------------|
| Encoder velocity not used by EKF | Open | **Still Open** (by design) |
| motion_modes_step placeholder | Fixed | Verified |
| Kill-switch after EKF update | Open | **Still Open** |
| Calibration validation for ARM | Fixed | Verified |
| Vibration window sample count | Fixed | Verified |
| Telemetry struct for EKF data | Implemented | Verified |

---

*Review performed on firmware commit: 86c09ce (RobustBinary protocol v2)*
*Reviewer: Claude Code Review Agent*
