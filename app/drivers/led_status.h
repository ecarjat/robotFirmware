#ifndef LED_STATUS_H
#define LED_STATUS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * LED blink patterns (period in ms, duty cycle 50%)
 */
typedef enum {
    LED_PATTERN_OFF = 0,        /**< LED always off */
    LED_PATTERN_ON,             /**< LED always on (solid) */
    LED_PATTERN_SLOW,           /**< Slow blink: 1Hz (500ms on, 500ms off) */
    LED_PATTERN_FAST,           /**< Fast blink: 4Hz (125ms on, 125ms off) */
    LED_PATTERN_DOUBLE_FLASH,   /**< Double flash: 2 quick flashes, then pause */
} led_pattern_t;

/**
 * LED status flags for red LED conditions.
 * Multiple conditions can be active; highest priority is displayed.
 */
typedef enum {
    LED_STATUS_NONE = 0,
    LED_STATUS_MOTOR_SATURATED = (1 << 0),   /**< Motor output hitting limit */
    LED_STATUS_MOTOR_TIMEOUT = (1 << 1),     /**< Motor communication timeout */
    LED_STATUS_FAULT = (1 << 2),             /**< System fault */
    LED_STATUS_FALLEN = (1 << 3),            /**< Robot has fallen */
    LED_STATUS_TELEM_FAILURE = (1 << 4),     /**< Telemetry send failure (alternating R/G) */
    LED_STATUS_LOGGING_FAILURE = (1 << 5),   /**< Logging/QSPI failure */
} led_status_flags_t;

/**
 * Initialize the LED status module.
 * Call once at startup after GPIO init.
 */
void led_status_init(void);

/**
 * Update LED states based on current system status.
 * Call periodically from main loop (e.g., every 10-50ms).
 *
 * @param now_ms Current time in milliseconds (HAL_GetTick())
 */
void led_status_update(uint32_t now_ms);

/**
 * Set status flags that affect the red LED.
 * Flags are OR'd together; clear with led_status_clear_flag().
 *
 * @param flag Status flag to set
 */
void led_status_set_flag(led_status_flags_t flag);

/**
 * Clear a status flag.
 *
 * @param flag Status flag to clear
 */
void led_status_clear_flag(led_status_flags_t flag);

/**
 * Clear all status flags.
 */
void led_status_clear_all_flags(void);

/**
 * Get current status flags.
 *
 * @return Bitwise OR of all active flags
 */
uint32_t led_status_get_flags(void);

/**
 * Force a specific pattern on the green LED (overrides automatic mode).
 * Set to LED_PATTERN_OFF to return to automatic mode-based behavior.
 *
 * @param pattern Pattern to display
 */
void led_status_set_green_override(led_pattern_t pattern);

#ifdef __cplusplus
}
#endif

#endif /* LED_STATUS_H */
