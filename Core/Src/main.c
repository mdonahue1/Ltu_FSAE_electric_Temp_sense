/* USER CODE BEGIN Header */
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>

#include "temp_sense.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint32_t tempBuffer[MUX_BANK_COUNT];

CAN_RxHeaderTypeDef rxHeaderFIFO0;
uint8_t dataFIFO0[8] = { (0x00U) };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void     selectMuxChannel(uint8_t channel);
static Result_t sendCanMessage(uint32_t canStdId, CanData_t data, uint32_t dlc);
static void     setFaultLine(FaultLine_e faultLine);
static void     clearFaultLine(FaultLine_e faultLine);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(hcan);

  if (hcan->Instance == CAN1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeaderFIFO0, dataFIFO0);
    CanData_t data;
    memcpy(data.data8, dataFIFO0, sizeof(data.data8));
    handle_can_message(rxHeaderFIFO0.StdId, data, rxHeaderFIFO0.DLC);
  }

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_RxFifo0MsgPendingCallback could be implemented in the
            user file
   */
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  update_raw_temperatures((uint32_t *)tempBuffer);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  memset((void *) tempBuffer, 0, sizeof(tempBuffer));
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  TempSenseInit_t tempSenseInit = {
      .selectMuxCell = selectMuxChannel,
      .sendCanMessage = sendCanMessage,
      .setFault = setFaultLine,
      .clearFault = clearFaultLine
  };
  initialize_temp_sense(tempSenseInit);

  if (HAL_ADC_Start_DMA (&hadc1, (uint32_t*) tempBuffer, MUX_BANK_COUNT) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
  }

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  TS_ECU_ChargingStateTrigFilterConfig();
  TS_ECU_SYNC_RX1_FilterConfig();
  TS_ECU_SYNC_RX2_FilterConfig();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(150);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    loop();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void selectMuxChannel(uint8_t channel) {
  HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, (channel & 0b0001) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, (channel & 0b0010) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, (channel & 0b0100) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, (channel & 0b1000) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static Result_t sendCanMessage(uint32_t canStdId, CanData_t data, uint32_t dlc) {
  CAN_TxHeaderTypeDef txHeader = {
      .StdId = canStdId,
      .ExtId = (uint32_t)0,
      .RTR = CAN_RTR_DATA,
      .IDE = CAN_ID_STD,
      .DLC = dlc
  };
  uint32_t mailbox = 0;

  if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, data.data8, &mailbox) != HAL_OK) {
    return RESULT_FAILED;
  }

  uint32_t startTick = HAL_GetTick();
  while(HAL_CAN_IsTxMessagePending(&hcan1, mailbox)) {
    if ((HAL_GetTick() - startTick) > 5) {
      return RESULT_FAILED;
    }
  }

  return RESULT_OK;
}

void setFaultLine(FaultLine_e faultLine) {
  if (faultLine & DISCHARGE_TEMP_FAULT) {
    HAL_GPIO_WritePin(Discharge_Fault_GPIO_Port, Discharge_Fault_Pin, GPIO_PIN_SET);
  } else if (faultLine & CHARGE_TEMP_FAULT) {
    HAL_GPIO_WritePin(Charge_Fault_GPIO_Port, Charge_Fault_Pin, GPIO_PIN_SET);
  }
}

void clearFaultLine(FaultLine_e faultLine) {
  if (faultLine & DISCHARGE_TEMP_FAULT) {
    HAL_GPIO_WritePin(Discharge_Fault_GPIO_Port, Discharge_Fault_Pin, GPIO_PIN_RESET);
  } else if (faultLine & CHARGE_TEMP_FAULT) {
    HAL_GPIO_WritePin(Charge_Fault_GPIO_Port, Charge_Fault_Pin, GPIO_PIN_RESET);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  setFaultLine(CHARGE_TEMP_FAULT | DISCHARGE_TEMP_FAULT);
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
