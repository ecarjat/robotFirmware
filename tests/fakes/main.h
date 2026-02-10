#ifndef TESTS_FAKES_MAIN_H
#define TESTS_FAKES_MAIN_H

#include <stdint.h>

#include "stm32h7xx_hal.h"

extern GPIO_TypeDef g_gpioa;
extern GPIO_TypeDef g_gpiod;

#define GPIOA (&g_gpioa)
#define GPIOD (&g_gpiod)

#define LeftHipUpperLimit_Pin 8U
#define LeftHipUpperLimit_GPIO_Port GPIOA
#define LeftHipLowerLimit_Pin 10U
#define LeftHipLowerLimit_GPIO_Port GPIOA
#define RightHipUpperLimit_Pin 3U
#define RightHipUpperLimit_GPIO_Port GPIOD
#define RightHipLowerLimit_Pin 5U
#define RightHipLowerLimit_GPIO_Port GPIOD

extern uint8_t g_gpio_left_upper_state;
extern uint8_t g_gpio_left_lower_state;
extern uint8_t g_gpio_right_upper_state;
extern uint8_t g_gpio_right_lower_state;

#endif /* TESTS_FAKES_MAIN_H */
