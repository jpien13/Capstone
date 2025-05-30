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
#include "stm32l0xx_hal.h"

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
#define SEL_1_Pin GPIO_PIN_13
#define SEL_1_GPIO_Port GPIOC
#define TDet_1_Pin GPIO_PIN_0
#define TDet_1_GPIO_Port GPIOH
#define PDet_1_Pin GPIO_PIN_1
#define PDet_1_GPIO_Port GPIOH
#define Speaker_En_Pin GPIO_PIN_2
#define Speaker_En_GPIO_Port GPIOB
#define Vol_2_Pin GPIO_PIN_14
#define Vol_2_GPIO_Port GPIOB
#define Vol_1_Pin GPIO_PIN_15
#define Vol_1_GPIO_Port GPIOB
#define SEL_2_Pin GPIO_PIN_9
#define SEL_2_GPIO_Port GPIOA
#define TDet_2_Pin GPIO_PIN_11
#define TDet_2_GPIO_Port GPIOA
#define PDet_2_Pin GPIO_PIN_12
#define PDet_2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
