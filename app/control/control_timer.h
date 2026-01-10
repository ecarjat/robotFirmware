#ifndef CONTROL_CONTROL_TIMER_H
#define CONTROL_CONTROL_TIMER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Control loop timing diagnostics.
 * Tracks jitter, latency, and overrun statistics.
 */
typedef struct {
    uint32_t loop_count;          /**< Total control loop iterations */
    uint32_t overrun_count;       /**< Times loop took longer than period */
    uint32_t max_latency_us;      /**< Max time from ISR flag to loop start */
    uint32_t max_execution_us;    /**< Max control loop execution time */
    uint32_t last_latency_us;     /**< Most recent latency measurement */
    uint32_t last_execution_us;   /**< Most recent execution time */
    uint64_t sum_execution_us;    /**< Total execution time for average calculation */
} control_timing_diag_t;

/**
 * Initialize the control timer subsystem.
 * Must be called before control_timer_start().
 */
void control_timer_init(void);

/**
 * Start the control timer (TIM2).
 * After this call, control_timer_pending() will become true at the configured rate.
 */
void control_timer_start(void);

/**
 * Configure the control timer rate (Hz).
 * Safe to call before or after control_timer_start().
 */
void control_timer_set_rate_hz(float rate_hz);

/**
 * Stop the control timer.
 */
void control_timer_stop(void);

/**
 * Check if a control cycle is pending.
 * @return true if timer has fired and control loop should run
 */
bool control_timer_pending(void);

/**
 * Clear the pending flag and record loop start timestamp.
 * Call this at the beginning of the control loop.
 */
void control_timer_begin_cycle(void);

/**
 * Record loop end timestamp and update diagnostics.
 * Call this at the end of the control loop.
 */
void control_timer_end_cycle(void);

/**
 * Get timing diagnostics.
 * @param[out] diag Pointer to diagnostics structure to fill
 */
void control_timer_get_diag(control_timing_diag_t *diag);

/**
 * Reset timing diagnostics counters.
 */
void control_timer_reset_diag(void);

/**
 * Called from TIM2 period elapsed callback.
 * Sets the pending flag and records ISR timestamp.
 */
void control_timer_isr_callback(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_CONTROL_TIMER_H */
