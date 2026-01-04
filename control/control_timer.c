#include "control_timer.h"
#include "stm32h7xx_hal.h"

/* DWT cycle counter for high-resolution timing */
#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL_CYCCNTENA (1UL << 0)

/* CoreDebug registers for DWT enable */
#define CoreDebug_DEMCR (*(volatile uint32_t *)0xE000EDFC)
#define CoreDebug_DEMCR_TRCENA (1UL << 24)

/* External timer handle from main.c */
extern TIM_HandleTypeDef htim2;

/* Control period in microseconds (1ms = 1000us) */
#define CONTROL_PERIOD_US 1000

/* State variables - volatile for ISR access */
static volatile bool s_pending = false;
static volatile uint32_t s_isr_timestamp = 0;

/* Cycle start timestamp (set in begin_cycle) */
static uint32_t s_cycle_start = 0;

/* Diagnostics */
static control_timing_diag_t s_diag = {0};

/* CPU frequency for cycle-to-us conversion */
static uint32_t s_cpu_freq_mhz = 0;

/**
 * Convert DWT cycles to microseconds.
 */
static inline uint32_t cycles_to_us(uint32_t cycles)
{
    if (s_cpu_freq_mhz == 0) {
        return 0;
    }
    return cycles / s_cpu_freq_mhz;
}

void control_timer_init(void)
{
    /* Enable DWT cycle counter for precise timing */
    CoreDebug_DEMCR |= CoreDebug_DEMCR_TRCENA;
    DWT_CYCCNT = 0;
    DWT_CTRL |= DWT_CTRL_CYCCNTENA;

    /* Get CPU frequency for timing calculations */
    s_cpu_freq_mhz = HAL_RCC_GetHCLKFreq() / 1000000UL;

    /* Reset state */
    s_pending = false;
    s_isr_timestamp = 0;
    s_cycle_start = 0;

    control_timer_reset_diag();
}

void control_timer_start(void)
{
    /* Start TIM2 in interrupt mode */
    HAL_TIM_Base_Start_IT(&htim2);
}

void control_timer_stop(void)
{
    HAL_TIM_Base_Stop_IT(&htim2);
    s_pending = false;
}

bool control_timer_pending(void)
{
    return s_pending;
}

void control_timer_begin_cycle(void)
{
    /* Record start timestamp before clearing flag */
    s_cycle_start = DWT_CYCCNT;

    /* Calculate latency from ISR to now */
    uint32_t isr_ts = s_isr_timestamp;  /* Read once for consistency */
    uint32_t latency_cycles = s_cycle_start - isr_ts;
    uint32_t latency_us = cycles_to_us(latency_cycles);

    s_diag.last_latency_us = latency_us;
    if (latency_us > s_diag.max_latency_us) {
        s_diag.max_latency_us = latency_us;
    }

    /* Clear pending flag - must be after reading timestamp */
    s_pending = false;
}

void control_timer_end_cycle(void)
{
    uint32_t now = DWT_CYCCNT;
    uint32_t exec_cycles = now - s_cycle_start;
    uint32_t exec_us = cycles_to_us(exec_cycles);

    s_diag.last_execution_us = exec_us;
    s_diag.loop_count++;

    if (exec_us > s_diag.max_execution_us) {
        s_diag.max_execution_us = exec_us;
    }

    /* Check for overrun (loop took longer than period) */
    if (exec_us > CONTROL_PERIOD_US) {
        s_diag.overrun_count++;
    }
}

void control_timer_get_diag(control_timing_diag_t *diag)
{
    if (diag != NULL) {
        /* Disable interrupts for consistent read */
        __disable_irq();
        *diag = s_diag;
        __enable_irq();
    }
}

void control_timer_reset_diag(void)
{
    __disable_irq();
    s_diag.loop_count = 0;
    s_diag.overrun_count = 0;
    s_diag.max_latency_us = 0;
    s_diag.max_execution_us = 0;
    s_diag.last_latency_us = 0;
    s_diag.last_execution_us = 0;
    __enable_irq();
}

void control_timer_isr_callback(void)
{
    /* Record timestamp immediately */
    s_isr_timestamp = DWT_CYCCNT;

    /* Set pending flag - main loop will handle the control cycle */
    s_pending = true;
}
