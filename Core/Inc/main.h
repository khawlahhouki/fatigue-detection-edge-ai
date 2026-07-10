/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_nucleo.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern EXTI_HandleTypeDef H_EXTI_11;
extern EXTI_HandleTypeDef H_EXTI_12;
extern EXTI_HandleTypeDef H_EXTI_13;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define PH0_MCU_Pin GPIO_PIN_0
#define PH0_MCU_GPIO_Port GPIOH
#define PH1_MCU_Pin GPIO_PIN_1
#define PH1_MCU_GPIO_Port GPIOH
#define INT_PPG_Pin GPIO_PIN_11
#define INT_PPG_GPIO_Port GPIOE
#define INT_PPG_EXTI_IRQn EXTI15_10_IRQn
#define DRDY_EMG_Pin GPIO_PIN_12
#define DRDY_EMG_GPIO_Port GPIOE
#define DRDY_EMG_EXTI_IRQn EXTI15_10_IRQn
#define INT1_IMU_Pin GPIO_PIN_13
#define INT1_IMU_GPIO_Port GPIOE
#define INT1_IMU_EXTI_IRQn EXTI15_10_IRQn
#define RESET_EMG_Pin GPIO_PIN_14
#define RESET_EMG_GPIO_Port GPIOE
#define START_EMG_Pin GPIO_PIN_15
#define START_EMG_GPIO_Port GPIOE
#define CS_IMU_Pin GPIO_PIN_10
#define CS_IMU_GPIO_Port GPIOD
#define CS_PPG_Pin GPIO_PIN_9
#define CS_PPG_GPIO_Port GPIOG
#define CS_EMG_Pin GPIO_PIN_10
#define CS_EMG_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
