/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
#define TS_ECU_ChargingStateTrigger_IDE (0U)
#define TS_ECU_ChargingStateTrigger_DLC (1U)
#define TS_ECU_ChargingStateTrigger_CANID (0x2BAU)

#define TS_ECU_SYNC_RX1_IDE (0U)
#define TS_ECU_SYNC_RX1_DLC (1U)
#define TS_ECU_SYNC_RX1_CANID (0x301U)

#define TS_ECU_SYNC_RX2_IDE (0U)
#define TS_ECU_SYNC_RX2_DLC (1U)
#define TS_ECU_SYNC_RX2_CANID (0x302U)
/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void TS_ECU_SYNC_RX1_FilterConfig(void);
void TS_ECU_SYNC_RX2_FilterConfig(void);
void TS_ECU_ChargingStateTrigFilterConfig(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

