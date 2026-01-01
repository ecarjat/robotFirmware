# SPI Sensor Scheduler Race Condition Review

This document analyzes potential race conditions in the IMU scheduler and SPI bus subsystem, and proposes solutions for each issue.

---

## Fix Status

| Issue | Status | Commit/Date |
|-------|--------|-------------|
| #1 s_running race | ✅ **FIXED** | Atomic LDREXB/STREXB in imu_sched.c |
| #2 s_active_sensor race | ✅ **FIXED** | IRQ protection in imu_sched_on_dma_done() |
| #3 s_dma_inflight race (ICM42688) | ✅ **FIXED** | Atomic LDREXB/STREXB in imu_icm42688.c |
| #3 s_dma_inflight race (BMI270) | ✅ **FIXED** | Atomic LDREXB/STREXB in imu_bmi270.c |
| #5 stale timeout state | ✅ **FIXED** | Atomic read of sensor+timestamp in imu_sched_tick() |
| #9 abort/finish race | ✅ **FIXED** | IRQ protection in spi_bus_finish() and spi_bus_abort() |
| #11 s_bus.busy race (NEW) | ✅ **FIXED** | Atomic LDREXB/STREXB in spi_bus.c |

---

## Table of Contents

1. [s_running Flag Race](#1-s_running-flag-race-in-imu_sched_run) ✅ FIXED
2. [s_active_sensor Read/Write Race](#2-s_active_sensor-readwrite-race) ✅ FIXED
3. [s_dma_inflight Race](#3-s_dma_inflight-race-between-kick-and-dma-callback) ✅ FIXED
4. [s_pending_mask Read Without Protection](#4-s_pending_mask-read-without-protection)
5. [Stale s_active_start_ms in Timeout Check](#5-stale-s_active_start_ms-in-timeout-check) ✅ FIXED
6. [spi_bus_is_busy() Check-Then-Act](#6-spi_bus_is_busy-check-then-act-pattern)
7. [s_last_served_ms Non-Atomic Update](#7-s_last_served_ms-non-atomic-update)
8. [Sample Storage Seqlock vs ISR Preemption](#8-sample-storage-seqlock-vs-isr-preemption)
9. [spi_bus_abort() vs spi_bus_finish() Race](#9-spi_bus_abort-vs-spi_bus_finish-race) ✅ FIXED
10. [s_irq_seen Increment Race](#10-s_irq_seen-increment-race)
11. [s_bus.busy Race in spi_bus_transfer_dma](#11-sbus_busy-race-in-spi_bus_transfer_dma) ✅ FIXED

---

## 1. `s_running` Flag Race in `imu_sched_run()` ✅ FIXED

### Location
[imu_sched.c:173-176](../Drivers/imu/imu_sched.c#L173-L176)

### Problem
```c
if (s_running)
{
    return;
}
s_running = 1U;
```

The check-then-set of `s_running` is not atomic. `imu_sched_run()` can be called from:
- Main loop via `imu_sched_tick()`
- DMA completion ISR via `spi_bus_idle_hook()` → `imu_sched_run()`

The ISR can preempt between lines 173-177, causing both contexts to pass the guard and potentially attempt concurrent DMA kicks.

### Severity
**Critical** - Could cause double DMA kicks corrupting SPI transfers.

### Solution Applied
```c
if (__LDREXB(&s_running) || __STREXB(1U, &s_running))
{
    return;
}
```

---

## 2. `s_active_sensor` Read/Write Race ✅ FIXED

### Location
- Write: [imu_sched.c:206](../Drivers/imu/imu_sched.c#L206)
- Read/Write: [imu_sched.c:157-159](../Drivers/imu/imu_sched.c#L157-L159)

### Problem
```c
// In imu_sched_on_dma_done() - runs in DMA ISR
if (s_active_sensor == (int8_t)sensor)
{
    s_active_sensor = -1;
}
```

```c
// In imu_sched_run() - can run in main or ISR context
s_active_sensor = (int8_t)selected;
```

`s_active_sensor` is written from `imu_sched_run()` and cleared in `imu_sched_on_dma_done()`. If `imu_sched_run()` sets a new sensor while the DMA callback is comparing, the state becomes inconsistent.

### Severity
**Medium** - Could cause incorrect sensor tracking, missed timeouts.

### Solution Applied
Protected the read-modify-write in `imu_sched_on_dma_done()`:

```c
void imu_sched_on_dma_done(imu_sched_sensor_t sensor, int status)
{
    if (sensor >= IMU_SCHED_SENSOR_COUNT)
    {
        return;
    }

    __disable_irq();
    s_last_served_ms[sensor] = HAL_GetTick();
    if (s_active_sensor == (int8_t)sensor)
    {
        s_active_sensor = -1;
    }
    __enable_irq();

    if (status != 0)
    {
        imu_sched_set_pending(sensor, HAL_GetTick());
    }
}
```

---

## 3. `s_dma_inflight` Race Between Kick and DMA Callback ✅ FIXED

### Location
- [imu_icm42688.c:185-193](../Drivers/imu/imu_icm42688.c#L185-L193)
- [imu_icm42688.c:168](../Drivers/imu/imu_icm42688.c#L168)
- [imu_bmi270.c:166-174](../Drivers/imu/imu_bmi270.c#L166-L174)

### Problem
```c
// icm_start_dma_read()
if (!s_init_ok || s_dma_inflight)  // check
{
    return false;
}
// ... gap where ISR can fire ...
int rc = spi_bus_transfer_dma(...);
if (rc == SPI_BUS_OK)
{
    s_dma_inflight = 1U;  // set
}
```

If a previous DMA completes between check and set, flag state is temporarily inconsistent.

### Severity
**Low** - The `spi_bus` layer has its own `busy` flag that provides real protection. This is defense-in-depth.

### Solution Applied
Used atomic test-and-set in both drivers:

```c
static bool icm_start_dma_read(void)
{
    if (!s_init_ok)
    {
        return false;
    }
    if (__LDREXB(&s_dma_inflight) || __STREXB(1U, &s_dma_inflight))
    {
        return false;
    }

    int rc = spi_bus_transfer_dma(&s_icm_spi, s_data_tx, s_data_rx,
                                   sizeof(s_data_tx), icm_dma_done, NULL);
    if (rc == SPI_BUS_OK)
    {
        return true;
    }
    s_dma_inflight = 0U;  // Clear on failure
    // ... error handling
    return false;
}
```

Same pattern applied to `bmi_start_dma_read()` in imu_bmi270.c.

---

## 4. `s_pending_mask` Read Without Protection

### Location
[imu_sched.c:179](../Drivers/imu/imu_sched.c#L179)

### Problem
```c
uint32_t pending = s_pending_mask;  // Unprotected read
```

An EXTI interrupt could set a new pending bit between this read and subsequent operations. The scheduler may miss this request until the next cycle.

### Severity
**Low** - Not data corruption, just potential latency (one tick delay).

### Proposed Solution
Read with interrupts disabled for consistency:

```c
__disable_irq();
uint32_t pending = s_pending_mask;
__enable_irq();

if (pending == 0U)
{
    s_running = 0U;
    return;
}
```

Or accept the latency as a design tradeoff (document it).

---

## 5. Stale `s_active_start_ms` in Timeout Check ✅ FIXED

### Location
- [imu_sched.c:238-251](../Drivers/imu/imu_sched.c#L238-L251)
- [imu_sched.c:207](../Drivers/imu/imu_sched.c#L207)

### Problem
```c
int8_t active = s_active_sensor;        // Read sensor index
// ... ISR can fire here, completing DMA and starting new transfer ...
uint32_t timeout_ms = s_timeout_ms[active];
// ...
if ((now_ms - s_active_start_ms) < timeout_ms)  // May use wrong start time
```

Between reading `s_active_sensor` and `s_active_start_ms`, the DMA could complete and a new transfer could start, causing inconsistent values to be used together.

### Severity
**High** - Could cause spurious aborts of valid transfers or missed timeouts.

### Solution Applied
Read all related state atomically:

```c
void imu_sched_tick(void)
{
    if (!imu_bus_is_ready())
    {
        return;
    }

    if (!spi_bus_is_busy())
    {
        imu_sched_run();
        return;
    }

    __disable_irq();
    int8_t active = s_active_sensor;
    uint32_t start_ms = s_active_start_ms;
    __enable_irq();

    if (active < 0 || active >= (int8_t)IMU_SCHED_SENSOR_COUNT)
    {
        return;
    }

    uint32_t timeout_ms = s_timeout_ms[active];
    if (timeout_ms == 0U)
    {
        return;
    }

    uint32_t now_ms = HAL_GetTick();
    if ((now_ms - start_ms) < timeout_ms)
    {
        return;
    }

    spi_bus_abort();
    imu_sched_run();
}
```

---

## 6. `spi_bus_is_busy()` Check-Then-Act Pattern

### Location
- [imu_sched.c:169](../Drivers/imu/imu_sched.c#L169)
- [imu_sched.c:232](../Drivers/imu/imu_sched.c#L232)

### Problem
```c
if (!imu_bus_is_ready() || spi_bus_is_busy())
{
    return;
}
// ... bus could become busy here via ISR ...
```

The `spi_bus_is_busy()` check can become stale if DMA completes and `spi_bus_idle_hook()` starts a new transfer before main thread continues.

### Severity
**Medium** - Mitigated by `s_running` flag (if fixed per issue #1) and SPI bus layer's own busy check.

### Proposed Solution
This is inherently a TOCTOU (time-of-check-time-of-use) issue. Mitigation:

1. Rely on the `s_running` flag to serialize scheduler entry (ensure issue #1 is fixed)
2. The `spi_bus_transfer_dma()` will return `SPI_BUS_BUSY` if bus is actually busy
3. Document that the check is advisory, not authoritative

```c
// Advisory check to avoid unnecessary work - actual protection is in spi_bus layer
if (!imu_bus_is_ready() || spi_bus_is_busy())
{
    return;
}
```

---

## 7. `s_last_served_ms[]` Non-Atomic Update

### Location
[imu_sched.c:156](../Drivers/imu/imu_sched.c#L156)

### Problem
```c
s_last_served_ms[sensor] = HAL_GetTick();  // Written in ISR
```

Read at [imu_sched.c:98](../Drivers/imu/imu_sched.c#L98):
```c
if (min_interval == 0U || (now_ms - s_last_served_ms[idx]) >= min_interval)
```

Potential torn read if main loop reads while ISR writes.

### Severity
**Very Low** - On ARM Cortex-M, 32-bit aligned accesses are atomic. Theoretical concern only.

### Proposed Solution
No change needed for ARM Cortex-M targets. For portability, use:

```c
// In imu_sched_on_dma_done()
__disable_irq();
s_last_served_ms[sensor] = HAL_GetTick();
__enable_irq();
```

Or document the ARM Cortex-M atomicity assumption.

---

## 8. Sample Storage Seqlock vs ISR Preemption

### Location
- Store: [imu_icm42688.c:133-138](../Drivers/imu/imu_icm42688.c#L133-L138)
- Read: [imu_icm42688.c:382-396](../Drivers/imu/imu_icm42688.c#L382-L396)

### Problem
The seqlock pattern assumes single-writer. If nested interrupts could trigger concurrent stores, the sequence would corrupt.

```c
static void icm_store_sample(const imu_icm42688_sample_t *sample)
{
    uint32_t seq = s_sample_seq;
    s_sample_seq = seq + 1U;  // Mark write in progress
    __DMB();
    s_latest_sample = *sample;
    __DMB();
    s_sample_seq = seq + 2U;  // Mark write complete
}
```

### Severity
**Low** - DMA callbacks run at same priority, so they don't nest. Only a concern if priorities are misconfigured.

### Proposed Solution
If concerned about nested interrupts, disable them during the store:

```c
static void icm_store_sample(const imu_icm42688_sample_t *sample)
{
    if (sample == NULL)
    {
        return;
    }

    __disable_irq();
    uint32_t seq = s_sample_seq;
    s_sample_seq = seq + 1U;
    __DMB();
    s_latest_sample = *sample;
    __DMB();
    s_sample_seq = seq + 2U;
    __enable_irq();
}
```

Or document that DMA callbacks must not be preempted by each other.

---

## 9. `spi_bus_abort()` vs `spi_bus_finish()` Race ✅ FIXED

### Location
- [spi_bus.c:363-393](../Drivers/spi_bus.c#L363-L393)
- [spi_bus.c:154-179](../Drivers/spi_bus.c#L154-L179)

### Problem
If `imu_sched_tick()` calls `spi_bus_abort()` from main loop while DMA ISR calls `spi_bus_finish()`:
- Both manipulate `s_bus.cb`, `s_bus.active_dev`, etc.
- Could cause double callback, use-after-clear, CS corruption

### Severity
**Critical** - Could corrupt SPI state machine, cause undefined behavior.

### Solution Applied
Added interrupt protection to both `spi_bus_abort()` and `spi_bus_finish()`:

**spi_bus_abort():**
```c
void spi_bus_abort(void)
{
    __disable_irq();
    if (!s_bus.busy || s_bus.hspi == NULL)
    {
        __enable_irq();
        return;
    }

    spi_bus_done_cb_t cb = s_bus.cb;
    void *cb_ctx = s_bus.cb_ctx;
    spi_bus_device_t *dev = s_bus.active_dev;
    uint8_t *rx = s_bus.rx;
    size_t len = s_bus.len;
    SPI_HandleTypeDef *hspi = s_bus.hspi;

    s_bus.cb = NULL;
    s_bus.cb_ctx = NULL;
    s_bus.active_dev = NULL;
    s_bus.tx = NULL;
    s_bus.rx = NULL;
    s_bus.len = 0U;
    s_bus.busy = 0U;
    __enable_irq();

    spi_bus_deassert_cs(dev);
    spi_bus_cache_invalidate(rx, len);

    (void)HAL_SPI_Abort(hspi);

    if (cb != NULL)
    {
        cb(cb_ctx, SPI_BUS_ERR);
    }
}
```

**spi_bus_finish():**
```c
static void spi_bus_finish(int status)
{
    __disable_irq();
    if (!s_bus.busy)
    {
        __enable_irq();
        return;  // Already handled by abort
    }

    spi_bus_done_cb_t cb = s_bus.cb;
    void *cb_ctx = s_bus.cb_ctx;
    spi_bus_device_t *dev = s_bus.active_dev;
    uint8_t *rx = s_bus.rx;
    size_t len = s_bus.len;

    s_bus.cb = NULL;
    s_bus.cb_ctx = NULL;
    s_bus.active_dev = NULL;
    s_bus.tx = NULL;
    s_bus.rx = NULL;
    s_bus.len = 0U;
    s_bus.busy = 0U;
    __enable_irq();

    spi_bus_deassert_cs(dev);
    spi_bus_cache_invalidate(rx, len);

    if (cb != NULL)
    {
        cb(cb_ctx, status);
    }

    spi_bus_idle_hook();
}
```

---

## 10. `s_irq_seen` Increment Race

### Location
- Write (ISR): [imu_icm42688.c:321](../Drivers/imu/imu_icm42688.c#L321)
- Read (main): [imu_icm42688.c:349](../Drivers/imu/imu_icm42688.c#L349)

### Problem
```c
// In ISR
s_irq_seen++;

// In poll (main loop)
if (imu_bus_is_ready() && s_irq_seen != 0U && s_irq_logged == 0U)
```

The increment and comparison could see inconsistent values on weakly-ordered architectures.

### Severity
**Very Low** - ARM Cortex-M is strongly ordered. `volatile` provides sufficient guarantees for this diagnostic counter.

### Proposed Solution
No change needed. The code is correct for ARM Cortex-M. For documentation:

```c
// s_irq_seen is volatile; on ARM Cortex-M, 32-bit accesses are atomic
// and strongly ordered, so no additional synchronization is needed
// for this diagnostic counter.
```

---

## 11. `s_bus.busy` Race in `spi_bus_transfer_dma()` ✅ FIXED

### Location
[spi_bus.c:240-250](../Drivers/spi_bus.c#L240-L250)

### Problem
```c
if (s_bus.busy)           // check
{
    return SPI_BUS_BUSY;
}
// ... ISR can complete DMA and call idle_hook which re-enters scheduler ...
s_bus.busy = 1U;          // set
```

The check-then-set of `s_bus.busy` is not atomic. This is the ultimate gatekeeper for SPI hardware access.

### Severity
**Critical** - Could cause double DMA starts corrupting SPI transfers.

### Solution Applied
```c
if (__LDREXB(&s_bus.busy) || __STREXB(1U, &s_bus.busy))
{
    return SPI_BUS_BUSY;
}

if (spi_bus_apply_config(dev) != SPI_BUS_OK)
{
    s_bus.busy = 0U;  // Clear on config failure
    return SPI_BUS_ERR;
}
```

Note: We also added `s_bus.busy = 0U` in the config failure path since the flag is now set before config is applied.

---

## Summary Table

| Issue | Severity | Fix Complexity | Priority | Status |
|-------|----------|----------------|----------|--------|
| #1 s_running race | Critical | Low | **P0** | ✅ FIXED |
| #11 s_bus.busy race | Critical | Low | **P0** | ✅ FIXED |
| #9 abort/finish race | Critical | Medium | **P0** | ✅ FIXED |
| #5 stale timeout state | High | Low | **P1** | ✅ FIXED |
| #2 s_active_sensor race | Medium | Low | **P1** | ✅ FIXED |
| #6 TOCTOU busy check | Medium | N/A (document) | **P2** | Mitigated by #1 fix |
| #3 s_dma_inflight race | Low | Low | **P2** | ✅ FIXED |
| #4 pending_mask read | Low | Low | **P3** | ⚠️ TODO |
| #8 seqlock nesting | Low | Low | **P3** | ⚠️ TODO |
| #7 last_served_ms atomicity | Very Low | N/A (ARM atomic) | **P4** | N/A |
| #10 irq_seen increment | Very Low | N/A (ARM atomic) | **P4** | N/A |

---

## Recommended Implementation Order

1. **Fix #1 and #9 first** - These are critical and could cause immediate failures
2. **Fix #5** - Timeout handling correctness is important for recovery
3. **Fix #2** - Improves state consistency
4. **Consider #3, #4, #6** - Defense in depth, low risk if skipped
5. **Skip #7, #8, #10** - Document ARM assumptions instead

---

## Testing Recommendations

After applying fixes:

1. **Stress test with high IRQ rate** - Configure both IMUs at maximum ODR
2. **Force timeout scenarios** - Inject DMA stalls to trigger abort path
3. **Concurrent access test** - Add artificial delays in scheduler to widen race windows
4. **ISR timing analysis** - Use logic analyzer to verify CS timing under load
