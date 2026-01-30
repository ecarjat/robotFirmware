#include <catch2/catch_test_macros.hpp>

extern "C" {
#include "control_timer.h"
}

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_ll_rcc.h"

extern uint32_t g_hal_pclk1_freq;
extern uint32_t g_hal_hclk_freq;
extern uint32_t g_hal_apb1_prescaler;
extern volatile uint32_t test_dwt_cyccnt;

namespace {
void reset_timer_state()
{
    g_hal_pclk1_freq = 100000000U;
    g_hal_hclk_freq = 200000000U;
    g_hal_apb1_prescaler = LL_RCC_APB1_DIV_1;

    htim2.Instance->CR1 = 0;
    htim2.Instance->PSC = 0;
    htim2.Instance->ARR = 0;
    htim2.Instance->CNT = 0;
    htim2.Instance->EGR = 0;
    htim2.Init.Prescaler = 0;
    htim2.Init.Period = 0;

    test_dwt_cyccnt = 0;
}
} // namespace

TEST_CASE("control_timer_set_rate_hz configures prescaler and period", "[control_timer]")
{
    reset_timer_state();
    control_timer_init();

    control_timer_set_rate_hz(400.0f);

    CHECK(htim2.Init.Prescaler == 99U);
    CHECK(htim2.Init.Period == 2499U);
    CHECK(htim2.Instance->PSC == 99U);
    CHECK(htim2.Instance->ARR == 2499U);
}

TEST_CASE("control_timer_pending toggles with ISR and begin_cycle", "[control_timer]")
{
    reset_timer_state();
    control_timer_init();

    CHECK_FALSE(control_timer_pending());

    test_dwt_cyccnt = 1000U;
    control_timer_isr_callback();
    CHECK(control_timer_pending());

    test_dwt_cyccnt = 2000U;
    control_timer_begin_cycle();
    CHECK_FALSE(control_timer_pending());
}

TEST_CASE("control_timer_time_to_deadline_us computes remaining time", "[control_timer]")
{
    reset_timer_state();
    control_timer_init();
    control_timer_set_rate_hz(400.0f);

    test_dwt_cyccnt = 1000000U;
    control_timer_isr_callback();

    test_dwt_cyccnt = 1000000U + 200000U; // 1 ms at 200 MHz
    const uint32_t remaining = control_timer_time_to_deadline_us();

    CHECK(remaining >= 1499U);
    CHECK(remaining <= 1501U);
}
