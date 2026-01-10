# Code Review: Self-Balancing Robot Firmware (2026-01-05)

This comprehensive code review covers the STM32H723 firmware for the two-wheeled self-balancing robot, building upon the previous review from 2026-01-04.

---

## 1. Executive Summary

The firmware has made significant improvements since the last review. This deep analysis revealed new concerns across control systems, driver layers, and application logic. Critical safety issues have been addressed.

**Issue Status:**
| Severity | Original | Fixed | Remaining |
|----------|----------|-------|-----------|
| Critical | 2 | 2 | 0 |
| High Priority | 6 | 3 | 3 |
| Medium Priority | 14 | 0 | 14 |
| Low Priority | 8 | 0 | 8 |

**Fixed Issues:**
- ~~Race condition in control timer ISR/main thread synchronization~~ **[FIXED]**
- ~~Buffer overflow in motor link frame parser~~ **[FIXED]**
- ~~Missing link timeout detection in application layer~~ **[FIXED]**
- ~~SPI bus non-atomic error cleanup~~ **[FIXED]**
- ~~Integral windup edge cases in PID controller~~ **[FIXED]**
- ~~Missing state validation before EKF update~~ **[FIXED]**
- ~~ARM/DISARM state transition race condition~~ **[FIXED]**

**Remaining High Priority:**
- Flash erase disabling interrupts for too long


---

## 2. Critical Issues

### ~~2.1 CRITICAL: Race Condition in Control Timer Begin Cycle~~ [FIXED]

**Location:** [control_timer.c:78-99](../control/control_timer.c#L78-L99)

**Original Issue:** The `control_timer_begin_cycle()` function read `s_isr_timestamp` (set by ISR) without proper synchronization. The ISR could fire between reading the timestamp and clearing the pending flag, causing stale/inconsistent timestamp for latency calculation.

**Fix Applied:** Wrapped the timestamp read and flag clear in an atomic critical section:

```c
void control_timer_begin_cycle(void)
{
    /* Record start timestamp */
    s_cycle_start = DWT_CYCCNT;

    /* Atomically read ISR timestamp and clear pending flag.
     * This prevents race condition where ISR fires between reading
     * the timestamp and clearing the flag. */
    __disable_irq();
    uint32_t isr_ts = s_isr_timestamp;
    s_pending = false;
    __enable_irq();

    /* Calculate latency from ISR to now (outside critical section) */
    uint32_t latency_cycles = s_cycle_start - isr_ts;
    uint32_t latency_us = cycles_to_us(latency_cycles);

    s_diag.last_latency_us = latency_us;
    if (latency_us > s_diag.max_latency_us) {
        s_diag.max_latency_us = latency_us;
    }
}
```

This ensures correct latency measurements and prevents control loop timing jitter.

---

### ~~2.2 CRITICAL: Missing Heartbeat Timeout Detection~~ [FIXED]

**Location:** [app_main.c:330-337](../app/app_main.c#L330-L337)

**Original Issue:** The code defined `APP_HEARTBEAT_TIMEOUT_MS` (200ms) but never implemented the timeout check. The robot had no mechanism to detect if the command link was dead (no frames received), creating a safety hazard.

**Fix Applied:** Added link timeout detection (no frames) in `app_idle_tick()`:

```c
/* Heartbeat timeout detection: disarm robot if no commands received */
if ((now - s_last_cmd_ms) > APP_HEARTBEAT_TIMEOUT_MS) {
    motion_control_set_mode(MOTION_MODE_DISARMED);
#ifdef ENABLE_MOTORS
    motor_link_enable(false);
#endif
    led_status_set_flag(LED_STATUS_TELEM_FAILURE);
}
```

The robot now automatically disarms and disables motors if no command is received within 200ms, with visual LED indication of communication failure.

---

## 3. High Priority Issues

### ~~3.1 HIGH: Buffer Overflow in Frame Parser~~ [FIXED]

**Location:** [motor_link_framing.c:242-246](../Drivers/motors/motor_link_framing.c#L242-L246)

The frame parser buffers incoming data into `parser->buf[FRAME_MAX_UNESCAPED_SIZE]` (64 bytes). However, the bounds checking trusts the unescaped length field entirely:

```c
case FRAME_STATE_DATA:
    if (parser->pos < sizeof(parser->buf)) {
        parser->buf[parser->pos++] = byte;
    }
    // Line 246: Uses parser->expected_len which can be up to 255
```

**Issue:** A malformed frame with `expected_len > 64` causes the parser to silently drop bytes without proper error detection, potentially causing frame corruption.

**Impact:** Motor communication could be disrupted by malicious or corrupted frames from ESC.

**Status:** Fixed. Parser now rejects oversized lengths, resets on buffer overflow, and encoder refuses frames that exceed the parser buffer.

---

### ~~3.2 HIGH: SPI Bus Non-Atomic Cleanup on Error~~ [FIXED]

**Location:** [spi_bus.c:279-295](../Drivers/spi_bus.c#L279-L295)

**Original Issue:** While SPI bus acquire uses atomic LDREX/STREX, the error cleanup path was non-atomic. DMA callback could fire during error cleanup, causing busy flag to be out of sync with actual DMA state.

**Fix Applied:** Wrapped the error cleanup path in IRQ disable/enable to make it atomic:

```c
if (st != HAL_OK) {
    spi_bus_deassert_cs(dev);

    /* Atomically release bus to prevent race with DMA callback */
    __disable_irq();
    s_bus.busy = 0U;
    s_bus.active_dev = NULL;
    s_bus.tx = NULL;
    s_bus.rx = NULL;
    s_bus.len = 0U;
    s_bus.cb = NULL;
    s_bus.cb_ctx = NULL;
    __enable_irq();

    return (st == HAL_BUSY) ? SPI_BUS_BUSY : SPI_BUS_ERR;
}
```

This ensures the bus state cleanup is atomic and prevents potential deadlocks or corrupted IMU data.

---

### ~~3.3 HIGH: Integral Limit Edge Case with Small Ki~~ [FIXED]

**Location:** [MotionController.cpp:188-192, 236-240](../control/MotionController.cpp#L188-L192)

**Original Issue:** If `_velPid_Ki` was slightly above epsilon (e.g., 2e-6), division produced a huge limit (~500,000), allowing unbounded integral accumulation with poorly tuned gains.

**Fix Applied:** Changed comparison to `>=` and added explicit upper bound capping:

```cpp
/* Velocity PID integral limit */
float velIntegralLimit = kDefaultIntegralLimit;
if ((_velPid_Ki >= kGainEpsilon) && (_velPid_iMax > 0.0f)) {
    float computed = _velPid_iMax / _velPid_Ki;
    velIntegralLimit = (computed < kDefaultIntegralLimit) ? computed : kDefaultIntegralLimit;
}

/* Pitch PID integral limit */
float integralLimit = kDefaultIntegralLimit;
if (_pitchPid_Ki >= kGainEpsilon) {
    float computed = outputLimit / _pitchPid_Ki;
    integralLimit = (computed < kDefaultIntegralLimit) ? computed : kDefaultIntegralLimit;
}
```

This prevents huge integral limits with tiny Ki values, ensuring control stability even with poorly tuned gains.

---

### 3.4 HIGH: Flash Erase Disables Interrupts Too Long

**Location:** [param_storage.c:103-126](../Drivers/param_storage.c#L103-L126)

```c
static int param_erase_sector(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();  // Interrupts disabled for entire erase (100-2000ms!)
    status = HAL_FLASHEx_Erase(&erase, &sector_error);
    __set_PRIMASK(primask);
}
```

**Issue:** All interrupts disabled for flash erase duration (100-2000ms on STM32H7). This breaks real-time guarantees for IMU and motor handlers.

**Impact:** Robot could fall if flash save is triggered during balancing (though `param_storage_can_save()` check exists, edge cases remain).

**Status:** Risk accepted. Current behavior is retained despite the interrupt-disable window during flash erase.

---

### ~~3.5 HIGH: Missing State Validation Before EKF Update~~ [FIXED]

**Location:** [StateEstimator.cpp:338-340](../control/StateEstimator.cpp#L338-L340)

```c
float theta_acc = atan2f(-accel_body[0],
                         sqrtf(accel_body[1] * accel_body[1] +
                               accel_body[2] * accel_body[2]));
```

**Issue:** No validation that accelerometer data magnitude is reasonable before computing pitch angle. If both y and z are zero (bad sensor), the sqrt is zero but atan2 still returns a value.

**Impact:** Corrupted IMU data produces invalid pitch estimates, potentially causing falls.

**Status:** Fixed. `StateEstimator` sets `theta_acc = NAN` when accel magnitude is invalid; `BalancerEKF::step()` treats invalid `thetaAcc` as a skipped measurement (zeroes H row and substitutes prediction).

---

### ~~3.6 HIGH: ARM/DISARM State Transition Race Condition~~ [FIXED]

**Location:** [app_main.c:1395-1404, 1283-1293](../app/app_main.c#L1395-L1404)

Multiple code paths can transition robot to BALANCING mode:

```c
// Teleop handler (line 1395-1404)
if (arm_rise) {
    if (!motion_control_can_arm()) {
        APP_LOG_ERROR("Arm rejected");  // Logged but continues!
    } else {
        motion_control_set_mode(MOTION_MODE_BALANCING);
        motor_link_enable(true);  // Separate call - no atomicity
    }
}
```

**Issue:** Between `motion_control_can_arm()` and `motion_control_set_mode()`, system state could change. Motor enable is separate call after mode change.

**Impact:** Robot could enter BALANCING mode in unsafe state.

**Status:** Fixed. Arm/balance transitions now use a shared helper that re-checks readiness under a short IRQ-off critical section and performs mode + motor enable together.

---

## 4. Medium Priority Issues

### ~~4.1 MEDIUM: Volatile Variable Usage Inconsistencies~~ [FIXED]

**Location:** [imu_sched.c:50, 314](../Drivers/imu/imu_sched.c#L50)

```c
static volatile uint32_t s_pending_mask = 0U;  // Volatile
static int8_t s_active_sensor = -1;             // NOT volatile!
```

**Issue:** `s_active_sensor` is read from main loop and written from ISR but lacks volatile qualifier. Compiler could optimize away reads.

**Status:** Fixed. `s_active_sensor` is now declared `volatile` in `Drivers/imu/imu_sched.c`.

---

### ~~4.2 MEDIUM: Inconsistent dt Guard Values~~ [FIXED]

**Location:** Multiple files

Different fallback dt values when dt <= 0:
- MotionController.cpp:124: `dt = 1e-4f` (100 microseconds)
- MotionController.cpp:271: `dt = 1e-3f` (1 millisecond)
- BalancerEKF.cpp:114: `dt = 0.001f` (1 millisecond)

**Issue:** Inconsistent fallback values cause different control behavior depending on which function catches invalid dt.

**Status:** Fixed. Fallback `dt` is now derived from `control_rate_hz` and propagated through motion control, estimator, and EKF.

---

### ~~4.3 MEDIUM: Parameter Update Not Atomic~~ [FIXED]

**Location:** [app_main.c:1337-1345](../app/app_main.c#L1337-L1345)

```c
if (changed) {
    memcpy(&g_robot_params, &updated, sizeof(updated));  // Not protected
    motion_control_apply_params();
}
```

**Issue:** `motion_control_tick` running at 1kHz could read partially updated parameters during the memcpy.

**Status:** Fixed. Parameter update now uses a PRIMASK critical section around the memcpy; `motion_control_apply_params()` runs after interrupts are restored.

---

### 4.4 MEDIUM: NaN Propagation in EKF Inputs

**Location:** [StateEstimator.cpp:349-351](../control/StateEstimator.cpp#L349-L351)

```c
float v_enc_ekf = NAN;
float pos_enc = NAN;
bool ekf_ok = ekf_.step(theta_acc, v_enc_ekf, pos_enc, gyro_pitch, dt_s, theta_var);
```

**Issue:** NaN is used as sentinel value for "no measurement available". While EKF handles this, it's fragile and undocumented.

---

### ~~4.5 MEDIUM: Frame Parser State Not Fully Reset~~ [FIXED]

**Location:** [motor_link_framing.c:227-231](../Drivers/motors/motor_link_framing.c#L227-L231)

When invalid length is detected, parser returns to IDLE but buffer contents aren't cleared. Subsequent valid frame could collide with old data.

**Status:** Fixed. Parser now fully resets state/position on invalid length, overflow, invalid escape, and pop validation failures.

---

### ~~4.6 MEDIUM: Cache Coherency Incomplete~~ [FIXED]

**Location:** [spi_bus.c:272-276](../Drivers/spi_bus.c#L272-L276)

No explicit `__DMB()` between cache operations and DMA start. CPU could reorder these operations on STM32H7.

**Status:** Fixed. Added `__DMB()` after cache maintenance before DMA start and after cache invalidation in completion/abort paths.

---

### 4.7 MEDIUM: DWT Counter Overflow Handling

**Location:** [control_timer.c:99-101](../control/control_timer.c#L99-L101)

The DWT cycle counter is 32-bit and wraps every ~10 seconds at 400MHz. Timing calculations work due to unsigned arithmetic but are fragile.

---

### 4.8 MEDIUM: EKF Partial Reset Missing Cross-Covariance

**Location:** [BalancerEKF.cpp:39-60](../control/ekf/BalancerEKF.cpp#L39-L60)

After partial reset, only diagonal P elements are reset. Cross-covariance terms become inconsistent, potentially affecting filter optimality.

---

### 4.9 MEDIUM: Motor Enable Errors Not Checked

**Location:** [app_main.c:1402, 1423, 1429](../app/app_main.c#L1402)

Multiple calls to `motor_link_enable()` don't check return status:
```c
motor_link_enable(true);   // No return check!
```

---

### 4.10 MEDIUM: LED Status Flag Race Condition

**Location:** [led_status.c:162-170](../Drivers/led_status.c#L162-L170)

```c
void led_status_set_flag(led_status_flags_t flag) {
    s_status_flags |= (uint32_t)flag;  // Non-atomic RMW!
}
```

**Issue:** Read-modify-write on shared flag variable without synchronization.

---

### 4.11 MEDIUM: Timeout Wraparound Vulnerability

**Location:** [motor_link.c:668, spi_bus.c:312](../Drivers/motors/motor_link.c#L668)

```c
while ((HAL_GetTick() - start) < timeout_ms) {
```

**Issue:** 32-bit timer wraps every ~49 days. Wraparound during comparison causes incorrect timeout.

---

### 4.12 MEDIUM: Motor Manual Command NaN/Inf Not Checked

**Location:** [app_main.c:879-885](../app/app_main.c#L879-L885)

```c
s_motor_manual.left = intensity;  // Direct assignment, no NaN check
```

If intensity is NaN, subsequent comparisons don't work as intended.

---

### 4.13 MEDIUM: IMU Calibration Face Index UB Potential

**Location:** [app_main.c:1166](../app/app_main.c#L1166)

```c
state->valid_mask |= (uint8_t)(1U << face);  // If face >= 8, UB!
```

The check is present but if bypassed, undefined behavior results.

---

### 4.14 MEDIUM: applyVelocitySlew() Never Called

**Location:** [MotionController.cpp:268-288](../control/MotionController.cpp#L268-L288)

The `applyVelocitySlew()` function is defined but never invoked from `computeControl()`. Potentially important safety feature not being used.

---

## 5. Low Priority Issues

### 5.1 LOW: Dead Code Path

**Location:** [StateEstimator.cpp:397-402](../control/StateEstimator.cpp#L397-L402)

```c
bool StateEstimator::getLastWheelMechanicalAngles(...) const
{
    (void)angle_left;
    (void)angle_right;
    return false;  // Always returns false
}
```

---

### 5.2 LOW: Hardcoded GPIO Pins in Telemetry Logs

**Location:** [app_main.c:1686-1688](../app/app_main.c#L1686-L1688)

CTS/RTS pins hardcoded to GPIOA 0 and 1 in debug logging. Should use defined constants.

---

### 5.3 LOW: Debug Counter Overflow

**Location:** [app_main.c:662-672](../app/app_main.c#L662-L672)

```c
static uint32_t s_debug_reports = 0U;
++s_debug_reports;  // Increments forever, can overflow back to 0
```

---

### 5.4 LOW: Magic Numbers Lack Documentation

**Location:** [BalancerEKF.cpp:8-10](../control/ekf/BalancerEKF.cpp#L8-L10)

```c
constexpr int POST_RESET_DAMPING_STEPS = 10;
constexpr float DAMPING_R_MULTIPLIER = 5.0f;
constexpr float INNOV_GATE_R_MULTIPLIER = 100.0f;
```

Reasonable values but lacking justification in comments.

---

### 5.5 LOW: ACK Transmit Errors Ignored

**Location:** [app_main.c:613](../app/app_main.c#L613)

```c
HAL_UART_Transmit(...);  // Return value ignored
```

---

### 5.6 LOW: Parameter Load Fallback Not Explicit

**Location:** [app_main.c:198-206](../app/app_main.c#L198-L206)

If `param_storage_load()` fails with I/O error (not NOT_FOUND), code continues without explicit default initialization.

---

### 5.7 LOW: Sequence Counter Wrapping

**Location:** [app_main.c:167, 1571](../app/app_main.c#L167)

16-bit sequence counters wrap naturally. Depends on protocol design if this matters.

---

### 5.8 LOW: Measurement Variance Over-Inflation

**Location:** [BalancerEKF.cpp:206-214](../control/ekf/BalancerEKF.cpp#L206-L214)

With both inflation factors applied, measurement variance can be inflated by 500x. Filter becomes effectively unobservable in worst case.

---

## 6. Positive Observations

### 6.1 Improved Code Structure

Since last review:
- Named constants replace magic numbers in MotionController
- LED status state machine is well-implemented
- Vibration RMS bounds checking added
- Dual-IMU calibration check implemented

### 6.2 Good Error Handling Patterns

- Parameter storage has proper motion mode check before flash operations
- IMU scheduler has atomic pending mask handling
- Motor link has retry limits with exponential backoff

### 6.3 D-Cache Handling

Proper cache management throughout:
- DMA buffers in `.dma_buffer` section
- Cache clean/invalidate around DMA operations

### 6.4 Control Loop Timing

Timer-driven 1kHz control loop with DWT cycle counting provides:
- Deterministic control rate
- Latency/jitter measurements
- Overrun detection

---

## 7. Recommendations Summary

### Must Fix (Safety Critical)
1. [ ] Add IRQ protection in control_timer_begin_cycle()
2. [ ] Implement link timeout detection
3. [ ] Fix frame parser buffer bounds checking
4. [ ] Add atomicity to SPI bus error cleanup

### Should Fix (Stability)
5. [x] Validate accelerometer data before EKF update
6. [ ] Fix integral limit edge case with small Ki
7. [ ] Add volatile qualifier to s_active_sensor
8. [ ] Unify dt guard values across codebase
9. [ ] Add DMB before DMA operations
10. [ ] Check motor_link_enable() return values

### Nice to Have (Quality)
11. [ ] Remove or implement applyVelocitySlew()
12. [ ] Add NaN/Inf validation for motor commands
13. [ ] Document EKF magic numbers
14. [ ] Fix debug counter overflow
15. [ ] Remove dead code paths

---

## 8. Files Changed Since Last Review

Key files with modifications:
- `control/MotionController.cpp` - Named constants, integral limit fixes
- `control/StateEstimator.cpp` - Vibration RMS bounds check
- `control/motion_control.cpp` - Dual-IMU calibration
- `Drivers/led_status.c/h` - New LED status module
- `Drivers/motors/motor_link.c` - Direction type fix

---

## 9. Testing Recommendations

### Critical Path Tests
1. Simulate ISR firing during control_timer_begin_cycle()
2. Test link timeout with communication drop
3. Fuzz test frame parser with malformed packets
4. Test flash save during balancing (should fail safely)

### Stress Tests
1. Run for 49+ days to test timer wraparound
2. Inject NaN/Inf into motor commands
3. Test with corrupted IMU data
4. Test concurrent parameter updates

---

*Review performed on firmware commit: eb17093 (Fully working version)*
*Previous review: 2026-01-04 (CodeReview040126.md)*
*Reviewer: Claude Code Review Agent*
