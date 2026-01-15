# Firmware Code Review

**Date:** January 11, 2026
**Project:** STM32H723 Self-Balancing Robot Controller Firmware
**Reviewer:** Claude Code
**Branch:** sdcard

---

## Executive Summary

This firmware implements a sophisticated self-balancing robot control system running on an STM32H723VGT6 microcontroller. The codebase demonstrates professional embedded development practices with well-structured modular architecture, proper DMA handling, and robust error handling. Key strengths include the dual-IMU redundancy system, async QSPI blackbox logging, and proper STM32H7 memory domain handling. Areas for improvement include documentation gaps, some magic numbers, and potential race conditions in a few areas.

**Overall Assessment:** Production-quality embedded firmware with solid architecture. Recommended for deployment with minor improvements.

---

## Architecture Overview

### Project Structure

```
firmware/
├── app/                    # Application layer (well-organized)
│   ├── control/            # Motion control & EKF
│   ├── drivers/            # Hardware abstraction
│   │   ├── imu/            # Sensor drivers
│   │   └── motors/         # Motor link protocol
│   ├── logging/            # Blackbox flight recorder
│   └── utils/              # Logging utilities
├── Core/                   # STM32CubeMX HAL setup
├── FATFS/                  # File system layer
├── Middlewares/            # USB & FatFS middleware
└── libs/                   # Third-party sensor APIs
```

### Key Architectural Decisions

1. **Super-loop with prioritized polling** - Control loop runs at highest priority when timer flag is set, followed by communication polling and background tasks
2. **DMA-based peripherals** - UART, SPI, SDMMC, and QSPI all use DMA for non-blocking I/O
3. **Dual-IMU redundancy** - BMI270 (primary) and ICM-42688 (secondary) with automatic failover
4. **Async QSPI logging** - 400Hz blackbox recording to W25Q64 flash with background writes
5. **Robust motor protocol** - CRC-32 framed binary protocol with byte stuffing

---

## Code Quality Analysis

### Strengths

#### 1. Memory Domain Handling (Excellent)
The firmware correctly handles STM32H7's complex memory architecture:

```c
// DMA buffers correctly placed in AXI SRAM (RAM_D1)
static uint8_t s_dump_buffer[DUMP_CHUNK_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));
```

The SDMMC IDMA cannot access DTCMRAM - this is correctly addressed in [sd_diskio.c](FATFS/Target/sd_diskio.c) and [fatfs.c](FATFS/App/fatfs.c) with clever CubeMX-safe workarounds.

#### 2. Cache Coherency (Excellent)
Proper D-Cache maintenance throughout:

```c
static void qspi_cache_clean(const void *addr, size_t len) {
  uintptr_t aligned_start = start & ~(uintptr_t)(32U - 1U);
  uintptr_t aligned_end = (end + (32U - 1U)) & ~(uintptr_t)(32U - 1U);
  SCB_CleanDCache_by_Addr((uint32_t *)aligned_start, ...);
}
```

#### 3. Dual-IMU Failover Logic (Excellent)
[StateEstimator.cpp](app/control/StateEstimator.cpp) implements sophisticated health monitoring:
- Gyro disagreement detection
- Accelerometer angle difference checks
- Vibration-based accel gating with hysteresis
- Timed switch-over with dwell periods

#### 4. Async QSPI Writes (Good)
[blackbox.c](app/logging/blackbox.c) implements a proper two-phase async write state machine:

```c
typedef enum {
  LOG_WRITE_IDLE = 0,
  LOG_WRITE_FIRST,
  LOG_WRITE_SECOND
} log_write_state_t;
```

This allows ring buffer wrap-around without blocking.

#### 5. Robust Motor Protocol (Good)
[motor_link.c](app/drivers/motors/motor_link.c) implements:
- CRC-32 protected frames
- Byte stuffing for framing
- Retry logic with configurable timeouts
- Telemetry staleness detection

### Areas for Improvement

#### ~~1. Magic Numbers (Medium Priority)~~

**RESOLVED:** All critical magic numbers have been addressed:
- `blackbox_dump.c:307` - Now uses `log_get_rate_hz()` instead of hardcoded 400 Hz for dump window calculation
- `blackbox_dump.c:122` - Now uses `log_get_rate_hz()` for metadata
- Timing constants extracted to config headers
- MotionController.cpp already uses well-named constants (`kGainEpsilon`, `kDefaultIntegralLimit`)

#### ~~2. Race Condition Risk in Blackbox Queue (Medium Priority)~~

**RESOLVED:** Implemented a proper lock-free SPSC (Single Producer Single Consumer) queue with:
- Memory barriers (`__DMB()`) instead of interrupt disabling
- Producer (`log_push_record`) only writes `s_queue_head`
- Consumer (`log_writer_tick`) only writes `s_queue_tail`
- Write barrier before publishing index updates
- Read barrier after reading index updates

#### ~~3. TODO Comments in Production Code (Low Priority)~~

**RESOLVED:** All production TODOs have been addressed:
- `blackbox_dump.c`: Now uses `log_get_rate_hz()` and `log_get_fields_mask()` from blackbox module
- `param_storage.c`: Replaced TODO with clear comment explaining version rejection behavior (defaults to parameter reset on version mismatch)

#### 4. Error Handling Inconsistency (Low Priority)

Some functions log errors and continue, others return false silently:

```c
// Good - logs error
if (qspi_w25q64_read_id(&manufacturer, &device)) {
  if (manufacturer == 0xEF && device == 0x4017) {
    s_initialized = true;
  }
}  // But what if it fails?

// Also good - logs and resets
if (res != FR_OK) {
  APP_LOG_ERROR("Dump open failed for %s (err=%d)", ...);
  dump_reset_context();
}
```

**Recommendation:** Ensure consistent error logging patterns across modules.

#### 5. Interrupt Priority Documentation (Low Priority)

The SDMMC interrupt priority override in [stm32h7xx_hal_msp.c](Core/Src/stm32h7xx_hal_msp.c) should have more documentation:

```c
/* Override SDMMC interrupt priority for DMA mode - priority 0 blocks callbacks */
HAL_NVIC_SetPriority(SDMMC1_IRQn, 6, 0);
```

**Recommendation:** Document why priority 6 was chosen and the relationship with FreeRTOS (if used) or other ISR priorities.

---

## Module-Specific Reviews

### app/app_main.c

**Quality: Good**

The main application loop is well-structured with clear priority ordering:

```c
/* 1. High Priority: Control Loop */
if (control_timer_pending()) {
  control_timer_begin_cycle();
  motion_control_tick(HAL_GetTick());
  control_timer_end_cycle();
}

/* 2. High Priority: Link Polling */
app_link_poll();
motor_link_poll();
imu_sched_tick();

/* 3. Background Tasks */
app_idle_tick();
```

**Issues:**
- The 3000ms startup delay lacks a constant definition
- Diagnostic logging is verbose (may want compile-time disable option)

### app/control/MotionController.cpp

**Quality: Excellent**

Well-implemented cascaded PID controller with:
- Proper anti-windup with conditional integration
- Derivative filtering
- Clean separation of velocity PID (outer) and pitch PID (inner)
- Yaw rate blending between gyro and encoder-derived rates

```cpp
float outputPreSat = p + Ki * state.integral + d;
bool saturatedHigh = outputPreSat >= outputLimit;
bool saturatedLow = outputPreSat <= -outputLimit;
bool integrationWouldHelp = (saturatedHigh && error < 0.0f) ||
                             (saturatedLow && error > 0.0f);

if (!saturatedHigh && !saturatedLow) {
  state.integral += error * dt;
} else if (integrationWouldHelp) {
  state.integral += error * dt;  // Back-calculation anti-windup
}
```

**Suggestion:** Consider adding integral preload for faster recovery from fallen state.

### app/control/StateEstimator.cpp

**Quality: Excellent**

Robust sensor fusion with:
- Per-sensor vibration monitoring using sliding window RMS
- IMU health comparison with configurable thresholds
- Hysteresis on vibration gating
- Timed IMU switchover with dwell period

The IMU rotation matrix handling is clean:

```cpp
void apply_rotation(const float rot[9], const float in[3], float out[3]) {
  out[0] = rot[0] * in[0] + rot[1] * in[1] + rot[2] * in[2];
  // ...
}
```

**Issues:**
- `getLastWheelMechanicalAngles()` returns false always (incomplete?)
- Some `NAN` comparisons should use `isnan()` consistently

### app/logging/blackbox.c

**Quality: Good**

Well-designed ring buffer logging with:
- Dual-slot metadata for power-fail safety
- Pre-erase ahead of write pointer
- Async write state machine

```c
/* Write to alternating slot */
uint32_t slot_addr = (meta.sequence % 2 == 0) ? LOG_META_SLOT0 : LOG_META_SLOT1;
```

**Issues:** None - metadata sequencing now correct with post-increment

### app/logging/blackbox_dump.c

**Quality: Good**

Recent improvements for sequential file numbering are solid:

```c
/* Scan SD card root for existing DUMP_*.BIN files */
res = f_opendir(&dir, "/");
if (res == FR_OK) {
  for (;;) {
    res = f_readdir(&dir, &fno);
    // Parse DUMP_NNNN.BIN pattern
  }
}
s_dump_ctx.last_dump_id = max_id + 1;
```

**Issues:**
- If SD card has >9999 dumps, filename generation will overflow the 4-digit format
- No verification that the newly generated filename doesn't already exist (edge case)

### app/drivers/motors/motor_link.c

**Quality: Good**

Robust UART-based motor protocol with:
- DMA RX with circular buffer
- Byte stuffing + CRC-32 framing
- Retry logic for configuration commands
- Telemetry staleness tracking

```c
bool left_stale = !s_left_telem.vel_valid ||
                  age_left > MOTOR_LINK_TELEM_STALE_MS;
if (left_stale) {
  s_left_telem.stale_count++;
}
```

**Issues:**
- Configuration retry loops could block for up to 2+ seconds during init
- `HAL_Delay(400)` between configuration steps lacks justification

### app/drivers/imu/imu_sched.c

**Quality: Excellent**

Clean SPI bus scheduling with:
- Round-robin + oldest-pending fallback
- Per-sensor minimum intervals
- Timeout detection and abort
- Lock-free running flag using `__LDREXB/__STREXB`

```c
if (__LDREXB(&s_running) || __STREXB(1U, &s_running)) {
  return;  // Another context already running
}
```

### app/drivers/qspi_w25q64.c

**Quality: Excellent**

Proper async QSPI flash driver with:
- Non-blocking state machine
- Page boundary handling
- Cache coherency
- Error callbacks

```c
static bool qspi_async_start_page(void) {
  size_t page_offset = s_async_addr % W25Q64_PAGE_SIZE;
  size_t chunk = W25Q64_PAGE_SIZE - page_offset;
  // Handle page-aligned writes
}
```

### FATFS/Target/sd_diskio.c

**Quality: Good (with workarounds)**

The CubeMX-safe DMA buffer workaround is clever:

```c
/* USER CODE BEGIN firstSection */
#define scratch _scratch_unused_dtcm
/* USER CODE END firstSection*/

/* USER CODE BEGIN enableScratchBuffer */
#define ENABLE_SCRATCH_BUFFER
#undef ALIGN_32BYTES
#define ALIGN_32BYTES(buf) buf __attribute__((section(".dma_buffer"), aligned(32)))
/* USER CODE END enableScratchBuffer */
```

This survives CubeMX regeneration while placing the scratch buffer in DMA-accessible RAM.

---

## Security Considerations

1. **No authentication on motor link protocol** - Acceptable for closed-loop control
2. **USB CDC input processed in ISR context** - Data is pushed to ring buffer (safe)
3. **Parameter storage has CRC validation** - Protects against corruption
4. **No flash encryption** - Acceptable for non-security-critical application

---

## Performance Analysis

### Control Loop Timing
- Target: 400 Hz (2.5 ms period)
- IMU data rate: 800-1000 Hz
- Motor telemetry: 500 Hz
- Profiling system in place for monitoring

### Memory Usage (from build output)
```
DTCMRAM:  19336 B / 128 KB (14.75%)
RAM_D1:  283520 B / 320 KB (86.52%)
FLASH:   165744 B / 640 KB (25.29%)
```

**Observations:**
- RAM_D1 usage is high (86%) due to DMA buffers
- Consider moving non-DMA data to DTCMRAM if space becomes critical
- FLASH has plenty of headroom for features

### DMA Buffer Allocations
- Blackbox write chunk: 4 KB
- Blackbox RAM queue: ~16 KB (configurable)
- SD scratch buffer: 512 bytes
- Motor link RX: 2x 1 KB
- Motor link TX: 2x 64 bytes

---

## Recommendations

### High Priority

1. ~~**Document memory requirements**~~ - **RESOLVED:** See [Memory.md](Memory.md) for comprehensive memory map and DMA buffer placement requirements

2. ~~**Add compile-time logging levels**~~ - **RESOLVED:** Compile-time log gating added with `APP_LOG_LEVEL` and presets for Debug/Release

3. **Fix incomplete function** - `StateEstimator::getLastWheelMechanicalAngles()` always returns false

### Medium Priority

4. ~~**Extract magic numbers**~~ - **RESOLVED:** Hardcoded 400 Hz replaced with `log_get_rate_hz()` in blackbox_dump.c; timing constants centralized in `app_config.h`

5. ~~**Add queue protection**~~ - **RESOLVED:** Implemented lock-free SPSC queue with memory barriers (`__DMB()`) in blackbox.c

6. ~~**Resolve TODOs**~~ - **RESOLVED:** Blackbox metadata now uses configured log fields mask and control-rate-derived log rate

### Low Priority

7. **Add file number overflow handling** - Handle case where dump ID exceeds 9999

8. **Add init retry backoff** - Motor link init retries should use exponential backoff

9. ~~**Document interrupt priorities**~~ - **RESOLVED:** - Create a table of all ISR priorities and their rationale

---

## Testing Recommendations

1. **Stress test blackbox logging** - Run at 400 Hz for 24+ hours, verify no data corruption

2. **Test IMU failover** - Physically disconnect primary IMU, verify switchover

3. **Test SD card hot-plug** - Verify graceful failure and recovery

4. **Test motor link recovery** - Disconnect UART mid-operation, verify error handling

5. **Test power-fail recovery** - Kill power during QSPI write, verify metadata recovery

---

## Conclusion

This firmware demonstrates mature embedded development practices suitable for a safety-relevant robotic application. The dual-IMU redundancy, proper DMA handling, and async logging architecture are particularly well-implemented. The recommended improvements are mostly documentation and edge-case handling rather than fundamental issues.

**Deployment Recommendation:** Approved with minor enhancements for production use.

---

*End of Code Review*
