#include "led_status.h"
#include "main.h"
#include "motion_modes.h"

/* Blink timing constants (in ms) */
#define SLOW_PERIOD_MS    1000U   /* 1Hz */
#define FAST_PERIOD_MS    250U    /* 4Hz */
#define DOUBLE_ON_MS      100U    /* Double flash: on duration */
#define DOUBLE_OFF_MS     100U    /* Double flash: gap between flashes */
#define DOUBLE_PAUSE_MS   600U    /* Double flash: pause after 2 flashes */
#define DOUBLE_TOTAL_MS   (2U * DOUBLE_ON_MS + 2U * DOUBLE_OFF_MS + DOUBLE_PAUSE_MS)
#define ALTERNATE_PERIOD_MS 500U  /* Alternating red/green: 2Hz */

/* State variables */
static uint32_t s_status_flags = 0U;
static uint32_t s_last_update_ms = 0U;
static led_pattern_t s_green_override = LED_PATTERN_OFF;

/* Pattern state tracking */
static uint32_t s_green_phase_start = 0U;
static uint32_t s_red_phase_start = 0U;
static uint32_t s_alternate_phase_start = 0U;

/**
 * Compute LED state (on/off) for a given pattern at current time.
 */
static bool led_pattern_is_on(led_pattern_t pattern, uint32_t now_ms, uint32_t *phase_start)
{
    uint32_t elapsed;

    switch (pattern) {
        case LED_PATTERN_OFF:
            return false;

        case LED_PATTERN_ON:
            return true;

        case LED_PATTERN_SLOW:
            elapsed = (now_ms - *phase_start) % SLOW_PERIOD_MS;
            return (elapsed < (SLOW_PERIOD_MS / 2U));

        case LED_PATTERN_FAST:
            elapsed = (now_ms - *phase_start) % FAST_PERIOD_MS;
            return (elapsed < (FAST_PERIOD_MS / 2U));

        case LED_PATTERN_DOUBLE_FLASH:
            elapsed = (now_ms - *phase_start) % DOUBLE_TOTAL_MS;
            /* First flash: 0 to DOUBLE_ON_MS */
            if (elapsed < DOUBLE_ON_MS) {
                return true;
            }
            elapsed -= DOUBLE_ON_MS;
            /* Gap: DOUBLE_OFF_MS */
            if (elapsed < DOUBLE_OFF_MS) {
                return false;
            }
            elapsed -= DOUBLE_OFF_MS;
            /* Second flash: DOUBLE_ON_MS */
            if (elapsed < DOUBLE_ON_MS) {
                return true;
            }
            /* Rest is pause */
            return false;

        default:
            return false;
    }
}

/**
 * Determine green LED pattern based on motion mode.
 */
static led_pattern_t led_get_green_pattern(void)
{
    if (s_green_override != LED_PATTERN_OFF) {
        return s_green_override;
    }

    motion_mode_t mode = motion_modes_get();

    switch (mode) {
        case MOTION_MODE_BALANCING:
            return LED_PATTERN_ON;       /* Solid green when balancing */

        case MOTION_MODE_DISARMED:
            return LED_PATTERN_SLOW;     /* Slow blink when disarmed/ready */

        case MOTION_MODE_FAULT:
        case MOTION_MODE_FALLEN:
            return LED_PATTERN_OFF;      /* Off when in fault/fallen */

        default:
            return LED_PATTERN_FAST;     /* Fast blink for unknown/init */
    }
}

/**
 * Determine red LED pattern based on status flags.
 * Priority: FAULT/FALLEN > MOTOR_TIMEOUT > MOTOR_SATURATED
 */
static led_pattern_t led_get_red_pattern(void)
{
    if (s_status_flags & (LED_STATUS_FAULT | LED_STATUS_FALLEN)) {
        return LED_PATTERN_ON;           /* Solid red for fault/fallen */
    }

    if (s_status_flags & LED_STATUS_MOTOR_TIMEOUT) {
        return LED_PATTERN_FAST;         /* Fast blink for motor timeout */
    }

    if (s_status_flags & LED_STATUS_MOTOR_SATURATED) {
        return LED_PATTERN_SLOW;         /* Slow blink for saturation */
    }

    return LED_PATTERN_OFF;              /* Off when healthy */
}

void led_status_init(void)
{
    s_status_flags = 0U;
    s_green_override = LED_PATTERN_OFF;
    s_last_update_ms = 0U;
    s_green_phase_start = 0U;
    s_red_phase_start = 0U;
    s_alternate_phase_start = 0U;

    /* Start with both LEDs off */
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
}

void led_status_update(uint32_t now_ms)
{
    bool green_on;
    bool red_on;

    /* Telemetry failure: alternating red/green (highest priority) */
    if (s_status_flags & LED_STATUS_TELEM_FAILURE) {
        uint32_t elapsed = (now_ms - s_alternate_phase_start) % ALTERNATE_PERIOD_MS;
        bool first_half = (elapsed < (ALTERNATE_PERIOD_MS / 2U));
        green_on = first_half;
        red_on = !first_half;
    } else {
        /* Normal operation: get patterns based on mode and flags */
        led_pattern_t green_pattern = led_get_green_pattern();
        led_pattern_t red_pattern = led_get_red_pattern();

        /* Compute LED states */
        green_on = led_pattern_is_on(green_pattern, now_ms, &s_green_phase_start);
        red_on = led_pattern_is_on(red_pattern, now_ms, &s_red_phase_start);
    }

    /* Update GPIO */
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,
                      green_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,
                      red_on ? GPIO_PIN_SET : GPIO_PIN_RESET);

    s_last_update_ms = now_ms;
}

void led_status_set_flag(led_status_flags_t flag)
{
    s_status_flags |= (uint32_t)flag;
}

void led_status_clear_flag(led_status_flags_t flag)
{
    s_status_flags &= ~(uint32_t)flag;
}

void led_status_clear_all_flags(void)
{
    s_status_flags = 0U;
}

uint32_t led_status_get_flags(void)
{
    return s_status_flags;
}

void led_status_set_green_override(led_pattern_t pattern)
{
    s_green_override = pattern;
}
