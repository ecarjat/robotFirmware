#include "system_reboot.h"
#include "stm32h7xx_hal.h"

/**
 * @brief Magic value to request bootloader entry (must match bl_config.h)
 */
#define REBOOT_MAGIC_BOOTLOADER  (0x424F4F54UL)  /* 'BOOT' */

void system_reboot_to_bootloader(void)
{
    /* Disable interrupts to prevent interference */
    __disable_irq();

    /* Enable access to RTC backup registers */
    HAL_PWR_EnableBkUpAccess();

    /* Write magic value to backup register 0 */
    RTC->BKP0R = REBOOT_MAGIC_BOOTLOADER;

    /* Ensure write completes before reset */
    __DSB();

    /* Trigger system reset */
    NVIC_SystemReset();

    /* Should never reach here */
    while (1) {}
}

void system_reboot(void)
{
    /* Disable interrupts */
    __disable_irq();

    /* Trigger system reset */
    NVIC_SystemReset();

    /* Should never reach here */
    while (1) {}
}
