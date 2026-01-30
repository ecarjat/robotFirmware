#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_ll_rcc.h"

TIM_TypeDef g_tim2_instance = {0};
TIM_HandleTypeDef htim2 = {
    .Instance = &g_tim2_instance,
    .Init = {
        .Prescaler = 0,
        .Period = 0,
    },
};

uint32_t g_hal_pclk1_freq = 100000000U;
uint32_t g_hal_hclk_freq = 200000000U;
uint32_t g_hal_apb1_prescaler = LL_RCC_APB1_DIV_1;

volatile uint32_t test_dwt_ctrl = 0;
volatile uint32_t test_dwt_cyccnt = 0;
volatile uint32_t test_coredebug_demcr = 0;

uint32_t HAL_RCC_GetPCLK1Freq(void)
{
    return g_hal_pclk1_freq;
}

uint32_t HAL_RCC_GetHCLKFreq(void)
{
    return g_hal_hclk_freq;
}

uint32_t LL_RCC_GetAPB1Prescaler(void)
{
    return g_hal_apb1_prescaler;
}

void HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim)
{
    if (htim && htim->Instance) {
        htim->Instance->CR1 |= TIM_CR1_CEN;
    }
}

void HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim)
{
    if (htim && htim->Instance) {
        htim->Instance->CR1 &= ~TIM_CR1_CEN;
    }
}

uint32_t __get_PRIMASK(void)
{
    return 0;
}

void __set_PRIMASK(uint32_t primask)
{
    (void)primask;
}

void __disable_irq(void)
{
}

void __enable_irq(void)
{
}
