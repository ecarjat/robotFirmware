#include "debug_wdog.h"
#include "main.h"
#include <string.h>

/*
 * Debug Watchdog Implementation
 *
 * Uses Backup SRAM to store checkpoint data that survives resets.
 * On IWDG reset, reads back the last checkpoint to identify where
 * the system hung.
 *
 * Backup SRAM Layout (at BKPSRAM_BASE = 0x38800000):
 *   Offset 0x00: Magic value (0xDEADBEEF if valid)
 *   Offset 0x04: Last checkpoint ID
 *   Offset 0x06: Last checkpoint line number
 *   Offset 0x08: Checkpoint count (how many times checkpoint was called)
 */

#define BKPSRAM_BASE_ADDR  0x38800000UL
#define WDOG_MAGIC_VALUE   0xDEADBEEFUL
#define WDOG_FAULT_MAGIC   0xB16B00B5UL
#define WDOG_DCACHE_LINE_SIZE 32U

typedef struct {
    uint32_t magic;
    uint16_t checkpoint_id;
    uint16_t line;
    uint32_t count;
    uint32_t fault_magic;
    uint32_t fault_hfsr;
    uint32_t fault_cfsr;
    uint32_t fault_bfar;
    uint32_t fault_mmfar;
    uint32_t fault_lr;
    uint32_t fault_pc;
    uint32_t fault_psr;
    uint32_t fault_sp;
} wdog_backup_data_t;

#define WDOG_BACKUP ((volatile wdog_backup_data_t *)BKPSRAM_BASE_ADDR)

extern IWDG_HandleTypeDef hiwdg1;

static bool s_was_iwdg_reset = false;
static uint16_t s_last_checkpoint_id = 0;
static uint16_t s_last_checkpoint_line = 0;
static bool s_ready = false;

static void debug_wdog_cache_invalidate(void)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if ((SCB->CCR & SCB_CCR_DC_Msk) == 0U)
    {
        return;
    }
    uintptr_t start = ((uintptr_t)WDOG_BACKUP) & ~(uintptr_t)(WDOG_DCACHE_LINE_SIZE - 1U);
    uintptr_t end = (uintptr_t)WDOG_BACKUP + sizeof(*WDOG_BACKUP);
    if (end > start)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
    }
#endif
}

static void debug_wdog_cache_clean(void)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if ((SCB->CCR & SCB_CCR_DC_Msk) == 0U)
    {
        return;
    }
    uintptr_t start = ((uintptr_t)WDOG_BACKUP) & ~(uintptr_t)(WDOG_DCACHE_LINE_SIZE - 1U);
    uintptr_t end = (uintptr_t)WDOG_BACKUP + sizeof(*WDOG_BACKUP);
    if (end > start)
    {
        SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
    }
#endif
}

/**
 * @brief Enable access to Backup SRAM
 *
 * Must be called before reading/writing backup SRAM.
 * PWR clock is already enabled by HAL_Init().
 */
static void backup_sram_enable(void)
{
    /* Enable access to backup domain */
    HAL_PWR_EnableBkUpAccess();

    /* Enable Backup SRAM clock */
    __HAL_RCC_BKPRAM_CLK_ENABLE();

    /*
     * On STM32H7, BKPRAM is available immediately after clock enable.
     * No need to wait for a ready flag.
     */
}

void debug_wdog_init(void)
{
    /* Enable backup SRAM access */
    backup_sram_enable();
    debug_wdog_cache_invalidate();

    /* Check if last reset was due to IWDG */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST))
    {
        s_was_iwdg_reset = true;

        /* Read last checkpoint from backup SRAM if valid */
        if (WDOG_BACKUP->magic == WDOG_MAGIC_VALUE)
        {
            s_last_checkpoint_id = WDOG_BACKUP->checkpoint_id;
            s_last_checkpoint_line = WDOG_BACKUP->line;
        }
    }

    /* Clear all reset flags */
    __HAL_RCC_CLEAR_RESET_FLAGS();

    /* Initialize backup SRAM for new run */
    WDOG_BACKUP->magic = WDOG_MAGIC_VALUE;
    WDOG_BACKUP->checkpoint_id = 0;
    WDOG_BACKUP->line = 0;
    WDOG_BACKUP->count = 0;
    debug_wdog_cache_clean();
    s_ready = true;
}

void debug_wdog_checkpoint(uint16_t checkpoint_id, uint16_t line)
{
    /* Store checkpoint in backup SRAM */
    WDOG_BACKUP->checkpoint_id = checkpoint_id;
    WDOG_BACKUP->line = line;
    WDOG_BACKUP->count++;
    debug_wdog_cache_clean();

    /*
     * Refresh watchdog at each checkpoint to extend timeout.
     * Safety check: only refresh if IWDG is initialized (Instance != NULL).
     * This prevents undefined behavior if checkpoint is called before MX_IWDG1_Init().
     */
    if (hiwdg1.Instance != NULL)
    {
        HAL_IWDG_Refresh(&hiwdg1);
    }
}

void debug_wdog_refresh(void)
{
    /* Safety check: only refresh if IWDG is initialized */
    if (hiwdg1.Instance != NULL)
    {
        HAL_IWDG_Refresh(&hiwdg1);
    }
}

bool debug_wdog_was_reset(void)
{
    return s_was_iwdg_reset;
}

uint16_t debug_wdog_get_last_checkpoint(void)
{
    return s_last_checkpoint_id;
}

const char* debug_wdog_checkpoint_name(uint16_t checkpoint_id)
{
    switch (checkpoint_id)
    {
        case 0:                          return "NONE";
        case WDOG_CP_MAIN_START:         return "MAIN_START";
        case WDOG_CP_CLOCK_CONFIG:       return "CLOCK_CONFIG";
        case WDOG_CP_GPIO_INIT:          return "GPIO_INIT";
        case WDOG_CP_DMA_INIT:           return "DMA_INIT";
        case WDOG_CP_USB_INIT:           return "USB_INIT";
        case WDOG_CP_SPI_INIT:           return "SPI_INIT";
        case WDOG_CP_IWDG_INIT:          return "IWDG_INIT";
        case WDOG_CP_EXTI_DISABLE:       return "EXTI_DISABLE";
        case WDOG_CP_APP_MAIN_ENTER:     return "APP_MAIN_ENTER";
        case WDOG_CP_APP_INIT_START:     return "APP_INIT_START";
        case WDOG_CP_PARAM_INIT:         return "PARAM_INIT";
        case WDOG_CP_PARAM_LOAD:         return "PARAM_LOAD";
        case WDOG_CP_IMU_SCHED_INIT:     return "IMU_SCHED_INIT";
        case WDOG_CP_BMI270_INIT_START:  return "BMI270_INIT_START";
        case WDOG_CP_BMI270_INIT_DONE:   return "BMI270_INIT_DONE";
        case WDOG_CP_ICM42688_INIT_START: return "ICM42688_INIT_START";
        case WDOG_CP_ICM42688_INIT_DONE: return "ICM42688_INIT_DONE";
        case WDOG_CP_BMM150_INIT_START:  return "BMM150_INIT_START";
        case WDOG_CP_BMM150_INIT_DONE:   return "BMM150_INIT_DONE";
        case WDOG_CP_EXTI_ENABLE:        return "EXTI_ENABLE";
        case WDOG_CP_IMU_BUS_READY:      return "IMU_BUS_READY";
        case WDOG_CP_IMU_SCHED_RUN:      return "IMU_SCHED_RUN";
        case WDOG_CP_MOTOR_INIT:         return "MOTOR_INIT";
        case WDOG_CP_MUX_INIT:           return "MUX_INIT";
        case WDOG_CP_LINK_START:         return "LINK_START";
        case WDOG_CP_APP_INIT_DONE:      return "APP_INIT_DONE";
        case WDOG_CP_IDLE_LOOP:          return "IDLE_LOOP";
        case WDOG_CP_PARAM_SCAN_START:   return "PARAM_SCAN_START";
        case WDOG_CP_PARAM_SCAN_DONE:    return "PARAM_SCAN_DONE";
        case WDOG_CP_SCHED_TICK:         return "SCHEDULER TICK";
        case WDOG_CP_ERROR_HANDLER:      return "ERROR_HANDLER";
        default:                         return "UNKNOWN";
    }
}

void debug_wdog_record_fault(uint32_t hfsr,
                             uint32_t cfsr,
                             uint32_t bfar,
                             uint32_t mmfar,
                             uint32_t lr,
                             uint32_t pc,
                             uint32_t psr,
                             uint32_t sp)
{
    if (!s_ready)
    {
        return;
    }
    WDOG_BACKUP->fault_magic = WDOG_FAULT_MAGIC;
    WDOG_BACKUP->fault_hfsr = hfsr;
    WDOG_BACKUP->fault_cfsr = cfsr;
    WDOG_BACKUP->fault_bfar = bfar;
    WDOG_BACKUP->fault_mmfar = mmfar;
    WDOG_BACKUP->fault_lr = lr;
    WDOG_BACKUP->fault_pc = pc;
    WDOG_BACKUP->fault_psr = psr;
    WDOG_BACKUP->fault_sp = sp;
    debug_wdog_cache_clean();
}

bool debug_wdog_get_fault(debug_wdog_fault_t *fault)
{
    if (fault == NULL || !s_ready)
    {
        return false;
    }
    if (WDOG_BACKUP->fault_magic != WDOG_FAULT_MAGIC)
    {
        return false;
    }

    fault->hfsr = WDOG_BACKUP->fault_hfsr;
    fault->cfsr = WDOG_BACKUP->fault_cfsr;
    fault->bfar = WDOG_BACKUP->fault_bfar;
    fault->mmfar = WDOG_BACKUP->fault_mmfar;
    fault->lr = WDOG_BACKUP->fault_lr;
    fault->pc = WDOG_BACKUP->fault_pc;
    fault->psr = WDOG_BACKUP->fault_psr;
    fault->sp = WDOG_BACKUP->fault_sp;

    WDOG_BACKUP->fault_magic = 0U;
    return true;
}
