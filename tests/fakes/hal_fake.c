#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_hal_fdcan.h"
#include "main.h"
#include <string.h>

TIM_TypeDef g_tim2_instance = {0};
TIM_HandleTypeDef htim2 = {
    .Instance = &g_tim2_instance,
    .Init = {
        .Prescaler = 0,
        .Period = 0,
    },
};

GPIO_TypeDef g_gpioa = {0};
GPIO_TypeDef g_gpiod = {0};

uint8_t g_gpio_left_upper_state = GPIO_PIN_SET;
uint8_t g_gpio_left_lower_state = GPIO_PIN_SET;
uint8_t g_gpio_right_upper_state = GPIO_PIN_SET;
uint8_t g_gpio_right_lower_state = GPIO_PIN_SET;

FDCAN_HandleTypeDef hfdcan1 = {0};

enum { TEST_FDCAN_TX_MAX = 16 };
uint32_t g_fdcan_tx_count = 0;
FDCAN_TxHeaderTypeDef g_fdcan_tx_headers[TEST_FDCAN_TX_MAX];
uint8_t g_fdcan_tx_data[TEST_FDCAN_TX_MAX][8];

uint32_t g_hal_pclk1_freq = 100000000U;
uint32_t g_hal_hclk_freq = 200000000U;
uint32_t g_hal_apb1_prescaler = LL_RCC_APB1_DIV_1;
uint32_t g_hal_tick = 0U;

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

uint32_t HAL_GetTick(void)
{
    return g_hal_tick;
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

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    if (port == GPIOA && pin == LeftHipUpperLimit_Pin) {
        return g_gpio_left_upper_state ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    if (port == GPIOA && pin == LeftHipLowerLimit_Pin) {
        return g_gpio_left_lower_state ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    if (port == GPIOD && pin == RightHipUpperLimit_Pin) {
        return g_gpio_right_upper_state ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    if (port == GPIOD && pin == RightHipLowerLimit_Pin) {
        return g_gpio_right_lower_state ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    return GPIO_PIN_SET;
}

int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan,
                                 FDCAN_TxHeaderTypeDef *pTxHeader,
                                 const uint8_t *pData)
{
    (void)hfdcan;
    if (g_fdcan_tx_count < TEST_FDCAN_TX_MAX) {
        g_fdcan_tx_headers[g_fdcan_tx_count] = *pTxHeader;
        if (pData != NULL) {
            memcpy(g_fdcan_tx_data[g_fdcan_tx_count], pData, 8U);
        } else {
            memset(g_fdcan_tx_data[g_fdcan_tx_count], 0, 8U);
        }
    }
    g_fdcan_tx_count++;
    return HAL_OK;
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
