#include "control_timer.h"
#include "config_control.h"
#include "app_log_macros.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_ll_rcc.h"
#include <math.h>
#include <stddef.h>

/* DWT cycle counter for high-resolution timing */
#ifdef CONTROL_TIMER_TEST
extern volatile uint32_t test_dwt_ctrl;
extern volatile uint32_t test_dwt_cyccnt;
extern volatile uint32_t test_coredebug_demcr;
#define DWT_CTRL    test_dwt_ctrl
#define DWT_CYCCNT  test_dwt_cyccnt
#define CoreDebug_DEMCR test_coredebug_demcr
#else
#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004)
#define CoreDebug_DEMCR (*(volatile uint32_t *)0xE000EDFC)
#endif
#define DWT_CTRL_CYCCNTENA (1UL << 0)

/* CoreDebug registers for DWT enable */
#define CoreDebug_DEMCR_TRCENA (1UL << 24)

/* External timer handle from main.c */
extern TIM_HandleTypeDef htim2;

/* Default control period in microseconds */
#define CONTROL_PERIOD_DEFAULT_US ((uint32_t)(1000000.0f / CONTROL_DEFAULT_HZ + 0.5f))

/* State variables - volatile for ISR access */
static volatile bool s_pending = false;
static volatile uint32_t s_isr_timestamp = 0;

/* Cycle start timestamp (set in begin_cycle) */
static uint32_t s_cycle_start = 0;

/* Diagnostics */
static control_timing_diag_t s_diag = {0};

/* CPU frequency for cycle-to-us conversion */
static uint32_t s_cpu_freq_mhz = 0;

/* Control period for overrun detection */
static uint32_t s_period_us = CONTROL_PERIOD_DEFAULT_US;

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

static uint32_t control_timer_tim2_clk_hz(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    if (LL_RCC_GetAPB1Prescaler() != LL_RCC_APB1_DIV_1) {
        pclk1 *= 2U;
    }
    return pclk1;
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
    s_period_us = CONTROL_PERIOD_DEFAULT_US;

    control_timer_reset_diag();
}

void control_timer_start(void)
{
    /* Start TIM2 in interrupt mode */
    HAL_TIM_Base_Start_IT(&htim2);
}

void control_timer_set_rate_hz(float rate_hz)
{
    if (!isfinite(rate_hz) || rate_hz <= 1e-3f) {
        rate_hz = CONTROL_DEFAULT_HZ;
    }

    uint32_t tim_clk_hz = control_timer_tim2_clk_hz();
    if (tim_clk_hz == 0U) {
        return;
    }

    const uint32_t target_tick_hz = 1000000U;
    uint64_t divider = (tim_clk_hz + (target_tick_hz / 2U)) / target_tick_hz;
    if (divider < 1U) {
        divider = 1U;
    }
    if (divider > 0x10000ULL) {
        divider = 0x10000ULL;
    }
    uint32_t prescaler = (uint32_t)(divider - 1U);
    uint32_t tick_hz = tim_clk_hz / (uint32_t)divider;

    double period_ticks = (double)tick_hz / (double)rate_hz;
    if (period_ticks < 1.0) {
        period_ticks = 1.0;
    }
    uint64_t period_plus = (uint64_t)(period_ticks + 0.5);
    if (period_plus < 1U) {
        period_plus = 1U;
    }
    if (period_plus > 0x100000000ULL) {
        period_plus = 0x100000000ULL;
    }
    uint32_t period = (uint32_t)(period_plus - 1U);

    double actual_rate = (double)tick_hz / (double)(period + 1U);
    if (actual_rate > 0.0) {
        s_period_us = (uint32_t)(1000000.0 / actual_rate + 0.5);
    } else {
        s_period_us = CONTROL_PERIOD_DEFAULT_US;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool was_running = (htim2.Instance->CR1 & TIM_CR1_CEN) != 0U;
    if (was_running) {
        HAL_TIM_Base_Stop_IT(&htim2);
    }
    __HAL_TIM_SET_PRESCALER(&htim2, prescaler);
    __HAL_TIM_SET_AUTORELOAD(&htim2, period);
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    htim2.Instance->EGR = TIM_EGR_UG;
    if (was_running) {
        HAL_TIM_Base_Start_IT(&htim2);
    }
    __set_PRIMASK(primask);

    htim2.Init.Prescaler = prescaler;
    htim2.Init.Period = period;

    APP_LOG_INFO("Control rate set: req=%.2f Hz actual=%.2f Hz period_us=%u", (double)rate_hz,
                 actual_rate, (unsigned)s_period_us);
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
    /* Record start timestamp */
    s_cycle_start = DWT_CYCCNT;

    /* Atomically read ISR timestamp and clear pending flag.
     * This prevents race condition where ISR fires between reading
     * the timestamp and clearing the flag. */
    __disable_irq();
    uint32_t isr_ts = s_isr_timestamp;
    s_pending = false;
    __enable_irq();

    /* Calculate latency from ISR to now (outside critical section) */
    uint32_t latency_cycles = s_cycle_start - isr_ts;
    uint32_t latency_us = cycles_to_us(latency_cycles);

    s_diag.last_latency_us = latency_us;
    if (latency_us > s_diag.max_latency_us) {
        s_diag.max_latency_us = latency_us;
    }
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
    s_diag.sum_execution_us += exec_us;

    /* Check for overrun (loop took longer than period) */
    if (exec_us > s_period_us) {
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
    s_diag.sum_execution_us = 0;
    __enable_irq();
}

uint32_t control_timer_time_to_deadline_us(void)
{
    if (s_period_us == 0 || s_cpu_freq_mhz == 0) {
        return 0;
    }

    uint32_t last_isr = s_isr_timestamp;
    if (last_isr == 0) {
        return 0;
    }

    uint32_t now = DWT_CYCCNT;
    uint32_t elapsed_us = cycles_to_us(now - last_isr);
    if (elapsed_us >= s_period_us) {
        return 0;
    }
    return s_period_us - elapsed_us;
}

void control_timer_isr_callback(void)
{
    /* Record timestamp immediately */
    s_isr_timestamp = DWT_CYCCNT;

    /* Set pending flag - main loop will handle the control cycle */
    s_pending = true;
}
