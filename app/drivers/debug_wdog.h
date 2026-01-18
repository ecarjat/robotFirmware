#ifndef DEBUG_WDOG_H
#define DEBUG_WDOG_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Debug Watchdog with Checkpoint Logging
 *
 * This module helps identify where the system hangs by:
 * 1. Storing checkpoints in backup SRAM (survives reset)
 * 2. Detecting IWDG reset on boot
 * 3. Reporting the last checkpoint before the hang
 *
 * Usage:
 *   debug_wdog_init();           // Call early in main(), checks for IWDG reset
 *   WDOG_CHECKPOINT(1);          // Mark progress with unique ID
 *   WDOG_CHECKPOINT(2);
 *   debug_wdog_refresh();        // Refresh watchdog periodically
 */

/* Checkpoint IDs - add more as needed */
#define WDOG_CP_MAIN_START           0x0001
#define WDOG_CP_CLOCK_CONFIG         0x0002
#define WDOG_CP_GPIO_INIT            0x0003
#define WDOG_CP_DMA_INIT             0x0004
#define WDOG_CP_USB_INIT             0x0005
#define WDOG_CP_SPI_INIT             0x0006
#define WDOG_CP_IWDG_INIT            0x0007
#define WDOG_CP_EXTI_DISABLE         0x0008
#define WDOG_CP_APP_MAIN_ENTER       0x0009
#define WDOG_CP_APP_INIT_START       0x000A
#define WDOG_CP_PARAM_INIT           0x000B
#define WDOG_CP_PARAM_LOAD           0x000C
#define WDOG_CP_IMU_SCHED_INIT       0x000D
#define WDOG_CP_BMI270_INIT_START    0x0010
#define WDOG_CP_BMI270_INIT_DONE     0x0011
#define WDOG_CP_ICM42688_INIT_START  0x0012
#define WDOG_CP_ICM42688_INIT_DONE   0x0013
#define WDOG_CP_BMM150_INIT_START    0x0014
#define WDOG_CP_BMM150_INIT_DONE     0x0015
#define WDOG_CP_EXTI_ENABLE          0x0016
#define WDOG_CP_IMU_BUS_READY        0x0017
#define WDOG_CP_IMU_SCHED_RUN        0x0018
#define WDOG_CP_MOTOR_INIT           0x0019
#define WDOG_CP_MUX_INIT             0x001A
#define WDOG_CP_LINK_START           0x001B
#define WDOG_CP_APP_INIT_DONE        0x001C
#define WDOG_CP_IDLE_LOOP            0x001D
#define WDOG_CP_SEND_TELEM           0x001E
#define WDOG_CP_SCHED_TICK           0x001F
#define WDOG_CP_ERROR_HANDLER        0x0020
#define WDOG_CP_PARAM_SCAN_START     0x0021
#define WDOG_CP_PARAM_SCAN_DONE      0x0022

/* Macro for setting checkpoint with file/line info */
#define WDOG_CHECKPOINT(id) debug_wdog_checkpoint((id), __LINE__)

/**
 * @brief Initialize debug watchdog
 *
 * Call this early in main(), before other initializations.
 * Checks if last reset was due to IWDG and logs the last checkpoint.
 */
void debug_wdog_init(void);

/**
 * @brief Set a checkpoint
 * @param checkpoint_id Unique ID for this location
 * @param line Source line number (use WDOG_CHECKPOINT macro)
 */
void debug_wdog_checkpoint(uint16_t checkpoint_id, uint16_t line);

/**
 * @brief Refresh the watchdog timer
 *
 * Call this periodically in the main loop and during long operations.
 */
void debug_wdog_refresh(void);

/**
 * @brief Check if last reset was due to watchdog
 * @return true if IWDG caused the reset
 */
bool debug_wdog_was_reset(void);

/**
 * @brief Get the last checkpoint before reset
 * @return Checkpoint ID, or 0 if none
 */
uint16_t debug_wdog_get_last_checkpoint(void);

/**
 * @brief Get checkpoint name string
 * @param checkpoint_id The checkpoint ID
 * @return Human-readable name
 */
const char* debug_wdog_checkpoint_name(uint16_t checkpoint_id);

typedef struct {
    uint32_t hfsr;
    uint32_t cfsr;
    uint32_t bfar;
    uint32_t mmfar;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
    uint32_t sp;
} debug_wdog_fault_t;

void debug_wdog_record_fault(uint32_t hfsr,
                             uint32_t cfsr,
                             uint32_t bfar,
                             uint32_t mmfar,
                             uint32_t lr,
                             uint32_t pc,
                             uint32_t psr,
                             uint32_t sp);

bool debug_wdog_get_fault(debug_wdog_fault_t *fault);

#endif /* DEBUG_WDOG_H */
