#ifndef SYSTEM_REBOOT_H
#define SYSTEM_REBOOT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Reboot to bootloader for firmware update
 *
 * Writes a magic value to the RTC backup register and triggers a system reset.
 * The bootloader will detect this magic value and enter update mode instead
 * of jumping to the application.
 *
 * This function does not return.
 */
void system_reboot_to_bootloader(void);

/**
 * @brief Perform a normal system reset
 *
 * Triggers a system reset. The bootloader will check its normal boot
 * conditions and jump to the application if valid.
 *
 * This function does not return.
 */
void system_reboot(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_REBOOT_H */
