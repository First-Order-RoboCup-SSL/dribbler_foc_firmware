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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SO1_Pin GPIO_PIN_0
#define SO1_GPIO_Port GPIOC
#define SO2_Pin GPIO_PIN_1
#define SO2_GPIO_Port GPIOC
#define EN_GATE_Pin GPIO_PIN_12
#define EN_GATE_GPIO_Port GPIOB
#define INL_A_Pin GPIO_PIN_13
#define INL_A_GPIO_Port GPIOB
#define INL_B_Pin GPIO_PIN_14
#define INL_B_GPIO_Port GPIOB
#define INL_C_Pin GPIO_PIN_15
#define INL_C_GPIO_Port GPIOB
#define HALL_Z_Pin GPIO_PIN_9
#define HALL_Z_GPIO_Port GPIOC
#define INH_A_Pin GPIO_PIN_8
#define INH_A_GPIO_Port GPIOA
#define INH_B_Pin GPIO_PIN_9
#define INH_B_GPIO_Port GPIOA
#define INH_C_Pin GPIO_PIN_10
#define INH_C_GPIO_Port GPIOA
#define HALL_A_Pin GPIO_PIN_4
#define HALL_A_GPIO_Port GPIOB
#define HALL_B_Pin GPIO_PIN_5
#define HALL_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* External MCU start/enable input (drive high to spin, low/floating to stop) */
#define EXT_START_Pin       GPIO_PIN_0
#define EXT_START_GPIO_Port GPIOA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
