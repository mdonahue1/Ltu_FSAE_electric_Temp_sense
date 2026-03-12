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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float temp_c;
  float volts;
} temp_point_t;

#define CAN_MESSAGES_PER_SEGMENT           3
#define CELLS_PER_CAN_MESSAGE              8

typedef union {
  uint64_t data64;
  uint8_t data8[8];
} CanDataBlock_t;

typedef union {
  uint8_t canTempArrays[CAN_MESSAGES_PER_SEGMENT][CELLS_PER_CAN_MESSAGE];
  uint8_t contiguousTempArray[CAN_MESSAGES_PER_SEGMENT * CELLS_PER_CAN_MESSAGE];
} CanSegmentTempData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MUX_PER_SEGMENT   2
#define SEGMENT_COUNT     5
#define MUX_BANK_COUNT		(MUX_PER_SEGMENT*SEGMENT_COUNT)
#define CELLS_PER_MUX			12

// Temp conversions
#define ADC_UNITS_TO_VOLTAGE(X)           ((X) * 3.3f / 4095.0f)
#define VOLTAGE_TO_ADC_UNITS(x)           ((x) * 4095.0f / 3.3f)

#define LOW_PASS_FILTER_ALPHA             0.3f
#define LOW_PASS_FILTER(new, old)         ((new)*(LOW_PASS_FILTER_ALPHA) + (old)*(1.0f - (LOW_PASS_FILTER_ALPHA)))

#define CELL_GROUND_SHORT_THRESHOLD       VOLTAGE_TO_ADC_UNITS(0.25f)
#define CELL_POWER_SHORT_THRESHOLD        VOLTAGE_TO_ADC_UNITS(3.00f)

#define CELL_DISCHARGE_MIN_TEMP_C         -20.0f
#define CELL_DISCHARGE_MAX_TEMP_C         60.0f
#define CELL_CHARGE_MIN_TEMP_C            0.0f
#define CELL_CHARGE_MAX_TEMP_C            45.0f

#define MAX_BAD_CELLS			6
#define MAX_BAD_READINGS  6

#define TEMP_SEGMENT_SYNC_MSGS_PER_SEGMENT          3
#define TEMP_SEGMENT_SYNC_CAN_ID_BASE               0x312
#define TEMP_SEGMENT_SYNC_CAN_ID_SEGMENT_OFFSET     0x010
#define TEMP_SEGMENT_SYNC_CAN_ID_MSG_OFFSET         0x001
#define TEMP_SEGMENT_SYNC_DEGREES_C_OFFSET          40

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t isCharging = 0;
uint8_t selectedCell = 0;
uint8_t dataReady = 0;

volatile uint32_t tempBuffer[MUX_BANK_COUNT];
uint32_t rawTempReadings[MUX_BANK_COUNT*CELLS_PER_MUX];
uint8_t badTempReadings[CELLS_PER_MUX];
float temperatures[MUX_BANK_COUNT*CELLS_PER_MUX];

CAN_RxHeaderTypeDef rxHeaderFIFO0;
uint8_t dataFIFO0[8] = { (0x00U) };
uint8_t summaryRefreshFlag = 0;
uint8_t segmentRefreshFlag = 0;

static const temp_point_t enepaq_table[] = {
  {-40, 2.44}, {-35, 2.42}, {-30, 2.40}, {-25, 2.38},
  {-20, 2.35}, {-15, 2.32}, {-10, 2.27}, {-5,  2.23},
  {  0, 2.17}, {  5, 2.11}, { 10, 2.05}, { 15, 1.99},
  { 20, 1.92}, { 25, 1.86}, { 30, 1.80}, { 35, 1.74},
  { 40, 1.68}, { 45, 1.63}, { 50, 1.59}, { 55, 1.55},
  { 60, 1.51}, { 65, 1.48}, { 70, 1.45}, { 75, 1.43},
  { 80, 1.40}, { 85, 1.38}, { 90, 1.37}, { 95, 1.35},
  {100, 1.34}, {105, 1.33}, {110, 1.32}, {115, 1.31},
  {120, 1.30}
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void     writeMuxSelector();
static float    voltageToTempC(float volts);
static void     writeSegmentTemperatureSummaryOverCan();
static void     writeSegmentTemperaturesOverCan();
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

    if(rxHeaderFIFO0.StdId == 0x301) {
      summaryRefreshFlag = 1;
    }

    if(rxHeaderFIFO0.StdId == 0x302) {
      segmentRefreshFlag |= (0b1 < dataFIFO0[0]);
    }

    // TODO Charging CAN message
  }

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_RxFifo0MsgPendingCallback could be implemented in the
            user file
   */
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	memcpy(rawTempReadings + MUX_BANK_COUNT*selectedCell, (const void *)tempBuffer, sizeof(tempBuffer));

	selectedCell = (selectedCell + 1) % CELLS_PER_MUX;
	writeMuxSelector();

	dataReady = 1;
}

void calculateTemperatures(uint8_t cellBank) {
  badTempReadings[cellBank] = 0;
	for(uint16_t i = cellBank*CELLS_PER_MUX; i < ((cellBank+1)*CELLS_PER_MUX); i++) {
		if (temperature < CELL_GROUND_SHORT_THRESHOLD) {
		  badTempReadings[cellBank]++;
		} else if (temperature > CELL_POWER_SHORT_THRESHOLD) {
		  badTempReadings[cellBank]++;
		} else {
      float voltage = ADC_UNITS_TO_VOLTAGE(rawTempReadings[i]);
      float temperature = voltageToTempC(voltage);
      temperatures[i] = LOW_PASS_FILTER(temperature, temperatures[i]);
		}
	}
}

void checkAndTriggerFaults() {
  float minTemperature = CELL_DISCHARGE_MIN_TEMP_C;
  float maxTemperature = CELL_DISCHARGE_MAX_TEMP_C;
  if (isCharging) {
    minTemperature = CELL_CHARGE_MIN_TEMP_C;
    maxTemperature = CELL_CHARGE_MAX_TEMP_C;
  }

	uint8_t badCellCount = 0;
	for (uint8_t i = 0; i < (MUX_BANK_COUNT * CELLS_PER_MUX); i++) {
		if (temperatures[i] < minTemperature) {
			badCellCount++;
		} else if (temperatures[i] > maxTemperature) {
			badCellCount++;
		}
	}

	uint8_t badReadings = 0;
	for (uint8_t i = 0; i < CELLS_PER_MUX; i++) {
	  badReadings += badTempReadings[i];
	}

	setFault((badCellCount > MAX_BAD_CELLS)
	      || (badReadings > MAX_BAD_READINGS));
}

static void writeSegmentTemperatureSummaryOverCan() {
  float minTemp = temperatures[0], maxTemp = temperatures[0];
  double tempSum = (double)temperatures[0];

  for(uint16_t )
}

static void writeSegmentTemperaturesOverCan() {
  uint8_t segmentsToRefresh = segmentRefreshFlag;
  for (uint8_t segment = 0; segment < SEGMENT_COUNT; segment++) {
    if ((segmentsToRefresh >> segment) & 0b1) continue;

    CanSegmentTempData_t tempData = { 0 };

    uint8_t temp_i = 0;
    for(uint8_t mux = 0; mux < 2; mux++) {
      uint8_t mux_i = mux*MUX_PER_SEGMENT + segment;

      for(uint8_t j = 0; j < CELLS_PER_MUX; j++) {
        tempData.contiguousTempArray[temp_i++] = ((uint8_t)temperatures[mux_i*CELLS_PER_MUX + j]) + TEMP_SEGMENT_SYNC_DEGREES_C_OFFSET;
      }
    }

    for(uint8_t msg_i = 0; msg_i < CAN_MESSAGES_PER_SEGMENT; msg_i++) {
      uint32_t mailbox;
      uint32_t msgId = TEMP_SEGMENT_SYNC_CAN_ID_BASE
                     + TEMP_SEGMENT_SYNC_CAN_ID_SEGMENT_OFFSET*segment
                     + TEMP_SEGMENT_SYNC_CAN_ID_MSG_OFFSET*msg_i;

      CAN_TxHeaderTypeDef txHeader = {
        .StdId = msgId, // Need to calculate ID for data
        .ExtId = 0x00,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8
      };

      if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, tempData.canTempArrays[msg_i], &mailbox) != HAL_OK) {
        Error_Handler();
      }

      uint32_t txPendingStart = HAL_GetTick();
      while(HAL_CAN_IsTxMessagePending(&hcan1, mailbox)) {
        if (HAL_GetTick() - txPendingStart > 10) { // 1-millisecond timeout per message
          Error_Handler();
        }
      }
    }

  }
}

static float voltageToTempC(float volts)
{
  int n = sizeof(enepaq_table) / sizeof(enepaq_table[0]);

  if (volts > enepaq_table[0].volts) return enepaq_table[0].temp_c;
  if (volts < enepaq_table[n - 1].volts) return enepaq_table[n - 1].temp_c;

  for (int i = 0; i < n - 1; i++)
  {
    float v1 = enepaq_table[i].volts;
    float v2 = enepaq_table[i + 1].volts;

    if ((v2 <= volts) && (volts <= v1))
    {
      float t1 = enepaq_table[i].temp_c;
      float t2 = enepaq_table[i + 1].temp_c;
      float frac = (volts - v1) / (v2 - v1);
      return t1 + frac * (t2 - t1);
    }
  }

  return CELL_OPEN_CIRCUIT;
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
  selectedCell = 0;
  memset((void *) tempBuffer, 0, sizeof(tempBuffer));
  memset((void *) rawTempReadings, 0, sizeof(rawTempReadings));
  memset((void *) badTempReadings, 0, sizeof(badTempReadings));
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
	  if (dataReady) {
		  int bank = (int)selectedCell - 1;
		  if (bank < 0) bank += CELLS_PER_MUX;

		  calculateTemperatures((uint8_t)bank);
		  checkAndTriggerFaults();

		  dataReady = 0;
	  }

	  if (segmentRefreshFlag != 0) {
	    writeSegmentTemperaturesOverCan();
	  }
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
void setFault(uint8_t flag) {
  if (flag) {
    HAL_GPIO_WritePin(Fault_line_GPIO_Port, Fault_line_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(Fault_line_GPIO_Port, Fault_line_Pin, GPIO_PIN_RESET);
  }
}

void writeMuxSelector() {
  HAL_GPIO_WritePin(S0_channel_GPIO_Port, S0_channel_Pin, (selectedCell & 0b0001) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(S1_channel_GPIO_Port, S1_channel_Pin, (selectedCell & 0b0010) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(S2_channel_GPIO_Port, S2_channel_Pin, (selectedCell & 0b0100) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(S3_channel_GPIO_Port, S3_channel_Pin, (selectedCell & 0b1000) ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
  setFault(1);
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
