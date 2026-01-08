/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern ADC_HandleTypeDef hadc1;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BL_LED_Pin GPIO_PIN_3
#define BL_LED_GPIO_Port GPIOE
#define BTN2_Pin GPIO_PIN_4
#define BTN2_GPIO_Port GPIOE
#define BTN3_Pin GPIO_PIN_5
#define BTN3_GPIO_Port GPIOE
#define BTN5_Pin GPIO_PIN_6
#define BTN5_GPIO_Port GPIOE
#define SW2_Pin GPIO_PIN_13
#define SW2_GPIO_Port GPIOC
#define ICM42688_INT1_Pin GPIO_PIN_0
#define ICM42688_INT1_GPIO_Port GPIOC
#define ICM42688_INT1_EXTI_IRQn EXTI0_IRQn
#define BMI270_INT1_Pin GPIO_PIN_1
#define BMI270_INT1_GPIO_Port GPIOC
#define BMI270_INT1_EXTI_IRQn EXTI1_IRQn
#define BMM150_INT1_Pin GPIO_PIN_2
#define BMM150_INT1_GPIO_Port GPIOC
#define BMM150_INT1_EXTI_IRQn EXTI2_IRQn
#define ESP32_TX_Pin GPIO_PIN_2
#define ESP32_TX_GPIO_Port GPIOA
#define ESP32_RX_Pin GPIO_PIN_3
#define ESP32_RX_GPIO_Port GPIOA
#define BTN1_Pin GPIO_PIN_4
#define BTN1_GPIO_Port GPIOA
#define Voltage_Pin GPIO_PIN_4
#define Voltage_GPIO_Port GPIOC
#define LED_RED_Pin GPIO_PIN_0
#define LED_RED_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_1
#define LED_GREEN_GPIO_Port GPIOB
#define LIDAR2_RX_Pin GPIO_PIN_7
#define LIDAR2_RX_GPIO_Port GPIOE
#define LIDAR2_TX_Pin GPIO_PIN_8
#define LIDAR2_TX_GPIO_Port GPIOE
#define LCD_CS_Pin GPIO_PIN_11
#define LCD_CS_GPIO_Port GPIOE
#define LVCD_WS_RS_Pin GPIO_PIN_13
#define LVCD_WS_RS_GPIO_Port GPIOE
#define LIDAR_RX_Pin GPIO_PIN_12
#define LIDAR_RX_GPIO_Port GPIOB
#define LIDAR_TX_Pin GPIO_PIN_13
#define LIDAR_TX_GPIO_Port GPIOB
#define MOTOR1_TX_Pin GPIO_PIN_14
#define MOTOR1_TX_GPIO_Port GPIOB
#define MOTOR1_RX_Pin GPIO_PIN_15
#define MOTOR1_RX_GPIO_Port GPIOB
#define MOTOR3_TX_Pin GPIO_PIN_8
#define MOTOR3_TX_GPIO_Port GPIOD
#define MOTOR3_RX_Pin GPIO_PIN_9
#define MOTOR3_RX_GPIO_Port GPIOD
#define BMM150_CS_Pin GPIO_PIN_10
#define BMM150_CS_GPIO_Port GPIOD
#define BMI270_CS_Pin GPIO_PIN_14
#define BMI270_CS_GPIO_Port GPIOD
#define ICM42688_CS_Pin GPIO_PIN_15
#define ICM42688_CS_GPIO_Port GPIOD
#define MOTOR2_TX_Pin GPIO_PIN_6
#define MOTOR2_TX_GPIO_Port GPIOC
#define MOTOR2_RX_Pin GPIO_PIN_7
#define MOTOR2_RX_GPIO_Port GPIOC
#define MOTOR4_RX_Pin GPIO_PIN_0
#define MOTOR4_RX_GPIO_Port GPIOD
#define MOTOR4_TX_Pin GPIO_PIN_1
#define MOTOR4_TX_GPIO_Port GPIOD
#define MICROSD_SW_Pin GPIO_PIN_4
#define MICROSD_SW_GPIO_Port GPIOD
#define SPI_FLASH_CS_Pin GPIO_PIN_6
#define SPI_FLASH_CS_GPIO_Port GPIOD
#define SPIx_MOSI_Pin GPIO_PIN_7
#define SPIx_MOSI_GPIO_Port GPIOD
#define SPIx_SCK_Pin GPIO_PIN_3
#define SPIx_SCK_GPIO_Port GPIOB
#define SPIx_MISO_Pin GPIO_PIN_4
#define SPIx_MISO_GPIO_Port GPIOB
#define LIDAR3_RX_Pin GPIO_PIN_0
#define LIDAR3_RX_GPIO_Port GPIOE
#define LIDAR3_TX_Pin GPIO_PIN_1
#define LIDAR3_TX_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
