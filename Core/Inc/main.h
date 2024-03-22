/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define Rot_A_SW0_In_Pin GPIO_PIN_0
#define Rot_A_SW0_In_GPIO_Port GPIOC
#define Rot_A_SW1_In_Pin GPIO_PIN_1
#define Rot_A_SW1_In_GPIO_Port GPIOC
#define Rot_A_SW3_In_Pin GPIO_PIN_2
#define Rot_A_SW3_In_GPIO_Port GPIOC
#define Rot_A_SW2_In_Pin GPIO_PIN_3
#define Rot_A_SW2_In_GPIO_Port GPIOC
#define Rot_B_SW0_In_Pin GPIO_PIN_4
#define Rot_B_SW0_In_GPIO_Port GPIOC
#define Face_BTN3_In_Pin GPIO_PIN_12
#define Face_BTN3_In_GPIO_Port GPIOB
#define Face_BTN2_In_Pin GPIO_PIN_13
#define Face_BTN2_In_GPIO_Port GPIOB
#define Face_BTN5_In_Pin GPIO_PIN_14
#define Face_BTN5_In_GPIO_Port GPIOB
#define Up_Shift_In_Pin GPIO_PIN_15
#define Up_Shift_In_GPIO_Port GPIOB
#define Down_Shift_In_Pin GPIO_PIN_8
#define Down_Shift_In_GPIO_Port GPIOA
#define Face_BTN0_In_Pin GPIO_PIN_9
#define Face_BTN0_In_GPIO_Port GPIOA
#define Face_BTN1_In_Pin GPIO_PIN_10
#define Face_BTN1_In_GPIO_Port GPIOA
#define Face_BTN4_In_Pin GPIO_PIN_11
#define Face_BTN4_In_GPIO_Port GPIOA
#define HBEAT_LED_Pin GPIO_PIN_10
#define HBEAT_LED_GPIO_Port GPIOC
#define GSENSE_LED_Pin GPIO_PIN_11
#define GSENSE_LED_GPIO_Port GPIOC
#define Rot_B_SW1_In_Pin GPIO_PIN_7
#define Rot_B_SW1_In_GPIO_Port GPIOB
#define Rot_B_SW2_In_Pin GPIO_PIN_8
#define Rot_B_SW2_In_GPIO_Port GPIOB
#define Rot_B_SW3_In_Pin GPIO_PIN_9
#define Rot_B_SW3_In_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
