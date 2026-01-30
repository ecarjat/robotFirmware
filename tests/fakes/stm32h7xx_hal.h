#ifndef TESTS_FAKES_STM32H7XX_HAL_H
#define TESTS_FAKES_STM32H7XX_HAL_H

#include <stdint.h>

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

extern TIM_HandleTypeDef htim2;

uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);

void HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim);

uint32_t __get_PRIMASK(void);
void __set_PRIMASK(uint32_t primask);
void __disable_irq(void);
void __enable_irq(void);

#endif /* TESTS_FAKES_STM32H7XX_HAL_H */
