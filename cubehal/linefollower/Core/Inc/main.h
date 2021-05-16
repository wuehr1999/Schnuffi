/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include <stdbool.h>
#include <stm32f1xx_it.h>
#include "ctrl.h"
#include "queue.h"
#include "wire.h"
#include "vl53l0x.h"
#include "timestamp.h"
#include "parameters.h"
#include "weightedavg.h"
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
void Powertrain_Init();
void Powertrain_EnableMotors(int enableState);
void Powertrain_SetDutycycles(int left, int right);
void Powertrain_Break(bool left, bool right);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED0_Pin GPIO_PIN_13
#define LED0_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOC
#define XS4_Pin GPIO_PIN_15
#define XS4_GPIO_Port GPIOC
#define S0_Pin GPIO_PIN_0
#define S0_GPIO_Port GPIOA
#define S1_Pin GPIO_PIN_1
#define S1_GPIO_Port GPIOA
#define S2_Pin GPIO_PIN_2
#define S2_GPIO_Port GPIOA
#define S3_Pin GPIO_PIN_3
#define S3_GPIO_Port GPIOA
#define S4_Pin GPIO_PIN_4
#define S4_GPIO_Port GPIOA
#define S5_Pin GPIO_PIN_5
#define S5_GPIO_Port GPIOA
#define S6_Pin GPIO_PIN_6
#define S6_GPIO_Port GPIOA
#define S7_Pin GPIO_PIN_7
#define S7_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_0
#define PWM2_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_1
#define PWM1_GPIO_Port GPIOB
#define BTN0_Pin GPIO_PIN_12
#define BTN0_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_13
#define BTN1_GPIO_Port GPIOB
#define ENC0_Pin GPIO_PIN_14
#define ENC0_GPIO_Port GPIOB
#define ENC0_EXTI_IRQn EXTI15_10_IRQn
#define ENC1_Pin GPIO_PIN_15
#define ENC1_GPIO_Port GPIOB
#define ENC1_EXTI_IRQn EXTI15_10_IRQn
#define S8_Pin GPIO_PIN_8
#define S8_GPIO_Port GPIOA
#define XS0_Pin GPIO_PIN_9
#define XS0_GPIO_Port GPIOA
#define XS1_Pin GPIO_PIN_10
#define XS1_GPIO_Port GPIOA
#define XS2_Pin GPIO_PIN_11
#define XS2_GPIO_Port GPIOA
#define XS3_Pin GPIO_PIN_12
#define XS3_GPIO_Port GPIOA
#define TP1_Pin GPIO_PIN_15
#define TP1_GPIO_Port GPIOA
#define TP2_Pin GPIO_PIN_3
#define TP2_GPIO_Port GPIOB
#define PWM3_Pin GPIO_PIN_4
#define PWM3_GPIO_Port GPIOB
#define PWM4_Pin GPIO_PIN_5
#define PWM4_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
#define nSLEEP_Pin GPIO_PIN_8
#define nSLEEP_GPIO_Port GPIOB
#define nFAULT_Pin GPIO_PIN_9
#define nFAULT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
