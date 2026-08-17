#ifndef TESTS_FAKES_STM32H7XX_HAL_H
#define TESTS_FAKES_STM32H7XX_HAL_H

#include <stdint.h>

typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;

typedef struct {
    volatile uint32_t dummy;
} GPIO_TypeDef;

extern GPIO_TypeDef g_gpioa;
#define GPIOA (&g_gpioa)

#define GPIO_PIN_0 0U
#define GPIO_PIN_1 1U

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t CNT;
    volatile uint32_t EGR;
} TIM_TypeDef;

typedef struct {
    uint32_t Prescaler;
    uint32_t Period;
} TIM_Base_InitTypeDef;

typedef struct {
    TIM_TypeDef *Instance;
    TIM_Base_InitTypeDef Init;
} TIM_HandleTypeDef;

typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1
} HAL_StatusTypeDef;

extern TIM_HandleTypeDef htim2;

uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t delay_ms);

extern uint32_t g_hal_delay_calls;
extern uint32_t g_hal_tick;

#ifndef __DMB
#define __DMB() do { } while (0)
#endif

void HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim);

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);

uint32_t __get_PRIMASK(void);
void __set_PRIMASK(uint32_t primask);
void __disable_irq(void);
void __enable_irq(void);

#endif /* TESTS_FAKES_STM32H7XX_HAL_H */
