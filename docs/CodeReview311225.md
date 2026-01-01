# Firmware Code Review - December 31, 2025

## Executive Summary

This document identifies race conditions and robustness issues in the STM32H7 robot controller firmware, with particular focus on the cold boot startup failure reported by the user.

**Primary Finding**: The root cause of the cold boot race condition is that **EXTI interrupts for IMU sensors are enabled before the IMU drivers are initialized**. During cold boot, sensors may generate spurious interrupts that interact with uninitialized state.

---

## Fix Status

| Issue | Status | Description |
|-------|--------|-------------|
| 1. EXTI before IMU init | **FIXED** | Disabled in USER CODE, re-enabled after init |
| 2. D-Cache not enabled | Open | Needs architectural decision |
| 3. SPI busy not atomic | **FIXED** | Added LDREX/STREX pattern |
| 4. Mixed sync primitives | Open | Low priority |
| 5. Flash ops unprotected | **FIXED** | Added interrupt disable |
| 6. Log overflow silent | Open | Low priority |
| 7. USB state race | **FIXED** | Simplified to USB-only logging |
| 8. Missing volatile | **FIXED** | Added volatile qualifiers |
| 9. No watchdog | **FIXED** | Implemented with checkpoint logging |
| 10. Error handler locks | Open | Recommended |
| 11. HAL_Delay in init | Open | Low risk |

---

## Critical Issues

### 1. EXTI Interrupts Enabled Before IMU Initialization (CRITICAL) - FIXED

**Location**: [main.c:180-194](Core/Src/main.c#L180-L194), [app_main.c:122-135](app/app_main.c#L122-L135)

**Problem**: CubeMX enables EXTI interrupts for IMU sensors in `MX_GPIO_Init()`, but IMU initialization happens much later in `app_init()`. On cold boot, sensors may generate spurious interrupts before the scheduler is ready.

**Fix Applied**:
1. Disable EXTI immediately after `MX_GPIO_Init()` in USER CODE section:
```c
/* USER CODE BEGIN 2 */
/*
 * EXTI Race Condition Fix:
 * CubeMX enables IMU EXTI interrupts in MX_GPIO_Init(), but IMU sensors
 * are not initialized until app_init(). On cold boot, sensors may be in
 * an undefined state and generate spurious interrupts.
 */
HAL_NVIC_DisableIRQ(ICM42688_INT1_EXTI_IRQn);
HAL_NVIC_DisableIRQ(BMI270_INT1_EXTI_IRQn);
HAL_NVIC_DisableIRQ(BMM150_INT1_EXTI_IRQn);
```

2. Re-enable after successful IMU init in `app_init()`:
```c
if (bmi_ok && icm_ok && bmm_ok) {
    __HAL_GPIO_EXTI_CLEAR_IT(ICM42688_INT1_Pin);
    __HAL_GPIO_EXTI_CLEAR_IT(BMI270_INT1_Pin);
    __HAL_GPIO_EXTI_CLEAR_IT(BMM150_INT1_Pin);

    HAL_NVIC_EnableIRQ(ICM42688_INT1_EXTI_IRQn);
    HAL_NVIC_EnableIRQ(BMI270_INT1_EXTI_IRQn);
    HAL_NVIC_EnableIRQ(BMM150_INT1_EXTI_IRQn);

    imu_bus_set_ready(1U);
    // ...
}
```

---

### 2. D-Cache Not Enabled But Cache Operations Used - OPEN

**Location**: [main.c:137](Core/Src/main.c#L137)

**Problem**: Only I-Cache is enabled, but code performs D-Cache operations. Since D-Cache is disabled, these calls are no-ops.

**Impact**: Without D-Cache:
- Performance penalty for memory-intensive operations
- DMA operations work correctly (no coherency issues)
- But if D-Cache is ever enabled without proper MPU configuration, corruption will occur

**Recommendation**: Either enable D-Cache with proper MPU regions for DMA buffers, or remove the cache management code to avoid confusion.

---

### 3. SPI Bus Busy Check Not Fully Atomic - FIXED

**Location**: [spi_bus.c:336-343](Drivers/spi_bus.c#L336-L343)

**Fix Applied**: Changed `spi_bus_transfer_blocking()` to use atomic LDREX/STREX:
```c
/*
 * Use atomic acquire to prevent race with DMA transfers.
 * Same pattern as spi_bus_transfer_dma() for consistency.
 */
if (__LDREXB(&s_bus.busy) || __STREXB(1U, &s_bus.busy))
{
    return SPI_BUS_BUSY;
}
```

Also added proper cleanup if `spi_bus_apply_config()` fails after acquiring the lock.

---

### 4. IMU Scheduler Mixed Synchronization Primitives - OPEN

**Location**: [imu_sched.c](Drivers/imu/imu_sched.c)

**Problem**: The scheduler uses both `__disable_irq()/__enable_irq()` and `__LDREXB()/__STREXB()`.

**Impact**: While currently functional, this pattern is error-prone for future modifications.

**Recommendation**: Standardize on one approach. For single-core STM32H7, `__disable_irq()/__enable_irq()` is simpler and deterministic.

---

### 5. Flash Operations Without Interrupt Protection - FIXED

**Location**: [param_storage.c:101-133](Drivers/param_storage.c#L101-L133), [param_storage.c:141-194](Drivers/param_storage.c#L141-L194)

**Fix Applied**: Added interrupt disable/restore around flash operations:
```c
/**
 * @brief Erase the parameter sector
 *
 * Interrupts are disabled during the erase operation because:
 * 1. STM32H7 flash operations block code execution from the same bank
 * 2. If ISRs reside in Bank 1 (common), they cannot execute during erase
 * 3. This prevents system hangs from missed interrupts or flash errors
 */
static int param_erase_sector(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // ... flash operations ...

    HAL_FLASH_Lock();
    __set_PRIMASK(primask);  // Restore previous IRQ state
    // ...
}
```

Same pattern applied to `param_program()`. Uses `__get_PRIMASK()`/`__set_PRIMASK()` to preserve caller's interrupt state.

---

## Moderate Issues

### 6. Log Buffer Overflow Silent - OPEN

**Location**: [log.c:72-76](utils/log.c#L72-L76)

**Problem**: When the log ring buffer is full, data is silently dropped.

**Recommendation**: Add a dropped message counter, periodically report drops.

---

### 7. USB State Check Race - FIXED

**Location**: [log.c](utils/log.c)

**Fix Applied**: Simplified logging to USB-only, removed UART fallback:
- Logs always go to ring buffer first
- USB transmission only attempted when device is configured
- Early boot logs are buffered and sent once USB enumerates
- Removed complex fallback logic that had race condition

```c
void app_log_printf(const char *fmt, ...)
{
    // ... format message ...

    /*
     * Always write to ring buffer. Early boot logs before USB enumeration
     * will be buffered and sent once USB is configured.
     */
    (void)app_log_ring_write((const uint8_t *)buffer, (uint16_t)bounded_len);
    app_log_usb_kick();
}

static void app_log_usb_kick(void)
{
    /*
     * Only attempt USB transmit if device is configured.
     * Early boot logs remain buffered until USB enumeration completes.
     */
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED)
    {
        return;
    }
    // ... transmit from ring buffer ...
}
```

---

### 8. Missing Volatile Qualifiers - FIXED

**Location**: [spi_bus.c:9-27](Drivers/spi_bus.c#L9-L27)

**Fix Applied**: Added volatile to all fields accessed from both thread and ISR context:
```c
/*
 * Fields accessed from both thread and ISR context are marked volatile.
 * hspi and config fields (cpol, cpha, etc.) are only modified during
 * init/config in thread context, so they don't need volatile.
 */
typedef struct {
    SPI_HandleTypeDef *hspi;
    spi_bus_device_t * volatile active_dev;
    const uint8_t * volatile tx;
    uint8_t * volatile rx;
    volatile size_t len;
    volatile spi_bus_done_cb_t cb;
    void * volatile cb_ctx;
    volatile uint8_t busy;
    uint32_t cpol;
    uint32_t cpha;
    uint32_t prescaler;
    uint32_t datasize;
} spi_bus_state_t;
```

---

## Robustness Recommendations

### 9. No Watchdog Timer - FIXED

**Problem**: No watchdog was configured. If the system hung, there was no automatic recovery.

**Fix Applied**: Implemented debug watchdog with checkpoint logging:
- IWDG configured via CubeMX (prescaler=256, reload=500 for ~4 second timeout)
- `debug_wdog` module stores checkpoints in backup SRAM (survives reset)
- On IWDG reset, logs the last checkpoint reached before the hang
- Checkpoints placed throughout initialization sequence for debugging

```c
// Usage in main.c
debug_wdog_init();
WDOG_CHECKPOINT(WDOG_CP_MAIN_START);
// ... initialization ...
WDOG_CHECKPOINT(WDOG_CP_BMI270_INIT_START);
imu_bmi270_init();
WDOG_CHECKPOINT(WDOG_CP_BMI270_INIT_DONE);

// On watchdog reset, logs:
// [WDOG] Reset detected! Last checkpoint: 0x0010 (BMI270_INIT_START)
```

---

### 10. Error Handler Locks System - OPEN

**Location**: [main.c:1272-1279](Core/Src/main.c#L1272-L1279)

**Problem**:
```c
void Error_Handler(void) {
    __disable_irq();
    while (1) {}  // System locked forever
}
```

**Recommendation**:
- Log the error source before disabling IRQs
- Store error info in retained RAM for post-mortem
- Consider a delayed reset instead of infinite loop
- Or trigger a watchdog reset

---

### 11. HAL_Delay in Sensor Init - OPEN

**Location**: Various IMU init functions

**Problem**: `HAL_Delay()` is used during sensor initialization. This relies on SysTick and would hang if called from ISR context.

**Impact**: Low risk since init is only called from thread context, but defensive programming suggests using a timeout-based approach or polling where possible.

---

## Summary Table

| Issue | Severity | Status | File | Type |
|-------|----------|--------|------|------|
| EXTI before IMU init | Critical | **FIXED** | main.c, app_main.c | Race Condition |
| D-Cache not enabled | Moderate | Open | main.c | Configuration |
| SPI busy check not atomic | Moderate | **FIXED** | spi_bus.c | Race Condition |
| Mixed sync primitives | Low | Open | imu_sched.c | Code Quality |
| Flash ops unprotected | Moderate | **FIXED** | param_storage.c | Interrupt Safety |
| Log overflow silent | Low | Open | log.c | Robustness |
| USB state race | Low | **FIXED** | log.c | Race Condition |
| Missing volatile | Low | **FIXED** | spi_bus.c | Code Quality |
| No watchdog | Moderate | **FIXED** | debug_wdog.c | Robustness |
| Error handler locks | Moderate | Open | main.c | Robustness |

---

## Files Modified

- [Core/Src/main.c](Core/Src/main.c) - Added EXTI disable in USER CODE section, watchdog init and checkpoints
- [app/app_main.c](app/app_main.c) - Added EXTI re-enable after IMU init, watchdog checkpoints and refresh
- [Drivers/spi_bus.c](Drivers/spi_bus.c) - Atomic busy check, volatile qualifiers
- [Drivers/param_storage.c](Drivers/param_storage.c) - Interrupt protection for flash ops
- [utils/log.c](utils/log.c) - Simplified to USB-only logging
- [Drivers/debug_wdog.h](Drivers/debug_wdog.h) - Debug watchdog header with checkpoint definitions
- [Drivers/debug_wdog.c](Drivers/debug_wdog.c) - Debug watchdog implementation using backup SRAM
- [CMakeLists.txt](CMakeLists.txt) - Added debug_wdog.c to build

## Files Reviewed

- [Core/Src/main.c](Core/Src/main.c) - Main entry point, peripheral init
- [app/app_main.c](app/app_main.c) - Application logic, startup
- [Drivers/imu/imu_exti.c](Drivers/imu/imu_exti.c) - EXTI callback
- [Drivers/imu/imu_sched.c](Drivers/imu/imu_sched.c) - IMU scheduler
- [Drivers/imu/imu_bus.c](Drivers/imu/imu_bus.c) - IMU bus ready flag
- [Drivers/imu/imu_bmi270.c](Drivers/imu/imu_bmi270.c) - BMI270 driver
- [Drivers/imu/imu_icm42688.c](Drivers/imu/imu_icm42688.c) - ICM42688 driver
- [Drivers/imu/imu_bmm150.c](Drivers/imu/imu_bmm150.c) - BMM150 driver
- [Drivers/spi_bus.c](Drivers/spi_bus.c) - SPI bus abstraction
- [Drivers/param_storage.c](Drivers/param_storage.c) - Flash parameter storage
- [USB_DEVICE/App/usbd_cdc_if.c](USB_DEVICE/App/usbd_cdc_if.c) - USB CDC interface
- [utils/log.c](utils/log.c) - Logging system
- [app/app_config.h](app/app_config.h) - Application configuration
