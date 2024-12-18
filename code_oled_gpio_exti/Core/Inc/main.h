/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define SPEED_MODE_Pin GPIO_PIN_0
#define SPEED_MODE_GPIO_Port GPIOA
#define SPEED_MODE_EXTI_IRQn EXTI0_IRQn
#define EMERGENCY_STOP_Pin GPIO_PIN_1
#define EMERGENCY_STOP_GPIO_Port GPIOA
#define EMERGENCY_STOP_EXTI_IRQn EXTI1_IRQn
#define START_STOP_Pin GPIO_PIN_2
#define START_STOP_GPIO_Port GPIOA
#define START_STOP_EXTI_IRQn EXTI2_IRQn
#define TARE_Pin GPIO_PIN_3
#define TARE_GPIO_Port GPIOA
#define TARE_EXTI_IRQn EXTI3_IRQn
#define MOVEDW_Pin GPIO_PIN_4
#define MOVEDW_GPIO_Port GPIOA
#define MOVEDW_EXTI_IRQn EXTI4_IRQn
#define MOVEUP_Pin GPIO_PIN_5
#define MOVEUP_GPIO_Port GPIOA
#define MOVEUP_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */