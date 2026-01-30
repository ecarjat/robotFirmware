#ifndef TESTS_FAKES_STM32H7XX_HAL_TIM_H
#define TESTS_FAKES_STM32H7XX_HAL_TIM_H

#include "stm32h7xx_hal.h"

#define TIM_CR1_CEN (1UL << 0)
#define TIM_EGR_UG (1UL << 0)

#define __HAL_TIM_SET_PRESCALER(htim, prescaler) \
    do { (htim)->Instance->PSC = (prescaler); } while (0)

#define __HAL_TIM_SET_AUTORELOAD(htim, period) \
    do { (htim)->Instance->ARR = (period); } while (0)

#define __HAL_TIM_SET_COUNTER(htim, cnt) \
    do { (htim)->Instance->CNT = (cnt); } while (0)

#endif /* TESTS_FAKES_STM32H7XX_HAL_TIM_H */
