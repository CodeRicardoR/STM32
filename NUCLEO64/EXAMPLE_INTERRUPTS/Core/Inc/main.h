/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdbool.h"
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
#define FIL_0_Pin GPIO_PIN_0
#define FIL_0_GPIO_Port GPIOC
#define FIL_1_Pin GPIO_PIN_1
#define FIL_1_GPIO_Port GPIOC
#define FIL_2_Pin GPIO_PIN_2
#define FIL_2_GPIO_Port GPIOC
#define FIL_3_Pin GPIO_PIN_3
#define FIL_3_GPIO_Port GPIOC
#define COL_0_Pin GPIO_PIN_0
#define COL_0_GPIO_Port GPIOB
#define COL_0_EXTI_IRQn EXTI0_IRQn
#define COL_1_Pin GPIO_PIN_1
#define COL_1_GPIO_Port GPIOB
#define COL_1_EXTI_IRQn EXTI1_IRQn
#define COL_2_Pin GPIO_PIN_2
#define COL_2_GPIO_Port GPIOB
#define COL_2_EXTI_IRQn EXTI2_IRQn
#define LCD_D4_Pin GPIO_PIN_12
#define LCD_D4_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_13
#define LCD_D5_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_14
#define LCD_D6_GPIO_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_15
#define LCD_D7_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_6
#define LCD_RS_GPIO_Port GPIOC
#define LCD_ENA_Pin GPIO_PIN_7
#define LCD_ENA_GPIO_Port GPIOC
#define COL_3_Pin GPIO_PIN_3
#define COL_3_GPIO_Port GPIOB
#define COL_3_EXTI_IRQn EXTI3_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
