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
void setFault(uint8_t flag);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MUX_5_Pin GPIO_PIN_0
#define MUX_5_GPIO_Port GPIOC
#define MUX_6_Pin GPIO_PIN_1
#define MUX_6_GPIO_Port GPIOC
#define MUX_7_Pin GPIO_PIN_2
#define MUX_7_GPIO_Port GPIOC
#define MUX_8_Pin GPIO_PIN_3
#define MUX_8_GPIO_Port GPIOC
#define MUX_9_Pin GPIO_PIN_0
#define MUX_9_GPIO_Port GPIOA
#define S0_Pin GPIO_PIN_1
#define S0_GPIO_Port GPIOA
#define S1_Pin GPIO_PIN_2
#define S1_GPIO_Port GPIOA
#define S2_Pin GPIO_PIN_3
#define S2_GPIO_Port GPIOA
#define S3_Pin GPIO_PIN_4
#define S3_GPIO_Port GPIOA
#define MUX_4_Pin GPIO_PIN_5
#define MUX_4_GPIO_Port GPIOA
#define MUX_3_Pin GPIO_PIN_6
#define MUX_3_GPIO_Port GPIOA
#define MUX_2_Pin GPIO_PIN_7
#define MUX_2_GPIO_Port GPIOA
#define MUX_1_Pin GPIO_PIN_4
#define MUX_1_GPIO_Port GPIOC
#define MUX_0_Pin GPIO_PIN_5
#define MUX_0_GPIO_Port GPIOC
#define Orange_LED_Pin GPIO_PIN_8
#define Orange_LED_GPIO_Port GPIOC
#define Green_LED_Pin GPIO_PIN_9
#define Green_LED_GPIO_Port GPIOC
#define Charge_Fault_Pin GPIO_PIN_3
#define Charge_Fault_GPIO_Port GPIOB
#define Discharge_Fault_Pin GPIO_PIN_4
#define Discharge_Fault_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
