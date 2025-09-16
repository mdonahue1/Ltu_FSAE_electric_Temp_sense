// Nucleo-F446RE + CD74HC4067 (1 mux): D3..D6 = PB3, PB5, PB4, PB10 (S0..S3)
// A0 (PA0/ADC1_IN0) reads; VDDA via factory VREFINT; UART2 @115200
// Features: throw-away sample after channel switch, open detection, single-point calibration, lock-to-channel mode
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>

/* ===== Optional single-point calibration =====
   Map your measured room volts to the table’s room volts.
   Example: if your room reads ~1.995 V but should be ~1.868 V, set ROOM_VOLTS_MEAS=1.995f. */
#define USE_CAL_SCALE     1
#define ROOM_VOLTS_MEAS   1.995f   // <-- put YOUR average room volts here
#define ROOM_VOLTS_TABLE  1.868f   // table volts near 24–25 °C
#define CAL_VSCALE        (ROOM_VOLTS_TABLE / ROOM_VOLTS_MEAS)

/* ===== Lock-to-channel logger =====
   Set to 255 to scan active[]; otherwise it locks and prints that channel at ~10 Hz (great for freezer/heat tests). */
#define LOCK_CHANNEL      255      // e.g., 13 to lock C13, or 255 to scan active[] list

/* ===== Open-channel threshold (mV) ===== */
#define OPEN_MV_THRESHOLD 3100     // >= this is considered "open" (prints ---)
#define AVG_SAMPLES       16

/* HAL module guards (for bare projects) */
#ifndef HAL_MODULE_ENABLED
#define HAL_MODULE_ENABLED
#endif
#ifndef HAL_RCC_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#endif
#ifndef HAL_GPIO_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#endif
#ifndef HAL_ADC_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#endif
#ifndef HAL_UART_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#endif

/* Handles */
ADC_HandleTypeDef  hadc1;
UART_HandleTypeDef huart2;

/* Protos */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void Error_Handler(void);

/* printf → UART2 */
int __io_putchar(int ch){ HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,HAL_MAX_DELAY); return ch; }

/* ---------- MUX pins: D3..D6 = PB3, PB5, PB4, PB10 ---------- */
#define MUX_S0_GPIO GPIOB
#define MUX_S0_PIN  GPIO_PIN_3   // D3 → S0 (LSB)
#define MUX_S1_GPIO GPIOB
#define MUX_S1_PIN  GPIO_PIN_5   // D4 → S1
#define MUX_S2_GPIO GPIOB
#define MUX_S2_PIN  GPIO_PIN_4   // D5 → S2
#define MUX_S3_GPIO GPIOB
#define MUX_S3_PIN  GPIO_PIN_10  // D6 → S3 (MSB)

static void MUX_GPIO_Init(void){
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef g={0};
  g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin  = MUX_S0_PIN|MUX_S1_PIN|MUX_S2_PIN|MUX_S3_PIN;
  HAL_GPIO_Init(GPIOB,&g);
  HAL_GPIO_WritePin(MUX_S0_GPIO, MUX_S0_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MUX_S1_GPIO, MUX_S1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MUX_S2_GPIO, MUX_S2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MUX_S3_GPIO, MUX_S3_PIN, GPIO_PIN_RESET);
}

static void MUX_Select(uint8_t ch){
  HAL_GPIO_WritePin(MUX_S0_GPIO, MUX_S0_PIN, (ch & 0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MUX_S1_GPIO, MUX_S1_PIN, (ch & 0x02)?GPIO_PIN_SET:GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MUX_S2_GPIO, MUX_S2_PIN, (ch & 0x04)?GPIO_PIN_SET:GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MUX_S3_GPIO, MUX_S3_PIN, (ch & 0x08)?GPIO_PIN_SET:GPIO_PIN_RESET);
  for(volatile int i=0;i<12000;i++) __NOP(); // ~250–350 µs settle @84 MHz
}

/* ---------- Energus volts↔°C points + linear interp ---------- */
typedef struct { float degC, volts; } tc_point_t;
static const tc_point_t temp_table[] = {
  { -40, 2.44f }, { -20, 2.35f }, {   0, 2.17f },
  {  15, 1.99f }, {  20, 1.92f }, {  25, 1.86f },
  {  30, 1.80f }, {  40, 1.68f }, {  50, 1.59f },
  {  60, 1.51f }, {  80, 1.40f }, { 100, 1.34f },
  { 120, 1.30f }
};
static const int NPTS = sizeof(temp_table)/sizeof(temp_table[0]);

static float volts_to_degC(float v){
  if (v >= temp_table[0].volts)      return temp_table[0].degC;
  if (v <= temp_table[NPTS-1].volts) return temp_table[NPTS-1].degC;
  for (int i=0;i<NPTS-1;i++){
    float vhi=temp_table[i].volts, vlo=temp_table[i+1].volts;
    if (v<=vhi && v>=vlo){
      float thi=temp_table[i].degC, tlo=temp_table[i+1].degC;
      float f=(vhi - v)/(vhi - vlo);
      return thi + f*(tlo - thi);
    }
  }
  return 25.0f;
}

/* ---------- ADC: VDDA via factory VREFINT calibration ---------- */
#define VREFINT_CAL_ADDR   ((uint16_t*) (0x1FFF7A2A)) // STM32F446
static inline void Enable_VREFINT(void){ ADC->CCR |= ADC_CCR_TSVREFE; }

typedef struct { uint16_t raw; uint16_t mv; } adc_sample_t;

static float Read_VDDA_Volts(void){
  ADC_ChannelConfTypeDef s={0};
  s.Channel = ADC_CHANNEL_VREFINT; s.Rank=1; s.SamplingTime=ADC_SAMPLETIME_480CYCLES;
  HAL_ADC_ConfigChannel(&hadc1,&s);
  HAL_ADC_Start(&hadc1); HAL_ADC_PollForConversion(&hadc1,10); (void)HAL_ADC_GetValue(&hadc1); HAL_ADC_Stop(&hadc1);
  uint32_t acc=0;
  for(int i=0;i<AVG_SAMPLES;i++){
    HAL_ADC_Start(&hadc1); HAL_ADC_PollForConversion(&hadc1,10);
    acc += (uint16_t)HAL_ADC_GetValue(&hadc1); HAL_ADC_Stop(&hadc1);
  }
  uint16_t vref_raw = acc/AVG_SAMPLES;
  uint16_t vref_cal = *VREFINT_CAL_ADDR; if (!vref_raw) return 3.30f;
  return 3.30f * ((float)vref_cal / (float)vref_raw);
}

/* throw-away one conversion on PA0 (reduces ghosting from prior channel) */
static void adc_throwaway_sample(void){
  ADC_ChannelConfTypeDef s={0};
  s.Channel=ADC_CHANNEL_0; s.Rank=1; s.SamplingTime=ADC_SAMPLETIME_480CYCLES;
  HAL_ADC_ConfigChannel(&hadc1,&s);
  HAL_ADC_Start(&hadc1); HAL_ADC_PollForConversion(&hadc1,10);
  (void)HAL_ADC_GetValue(&hadc1); HAL_ADC_Stop(&hadc1);
}

static adc_sample_t Read_A0_raw_mv(void){
  float vdda = Read_VDDA_Volts();
  uint32_t vdda_mv = (uint32_t)(vdda*1000.0f + 0.5f);

  ADC_ChannelConfTypeDef s={0};
  s.Channel=ADC_CHANNEL_0; s.Rank=1; s.SamplingTime=ADC_SAMPLETIME_480CYCLES;
  HAL_ADC_ConfigChannel(&hadc1,&s);

  uint32_t acc=0;
  for(int i=0;i<AVG_SAMPLES;i++){
    HAL_ADC_Start(&hadc1); HAL_ADC_PollForConversion(&hadc1,10);
    acc += (uint16_t)HAL_ADC_GetValue(&hadc1); HAL_ADC_Stop(&hadc1);
  }
  uint16_t raw = acc/AVG_SAMPLES;
  uint16_t mv  = (uint16_t)((raw * vdda_mv + 2047) / 4095);
  adc_sample_t out={raw,mv}; return out;
}

/* -------------------- MAIN -------------------- */
int main(void){
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();          // PA0 analog, PA2/PA3 AF7
  MX_USART2_UART_Init();   // 115200-8N1
  MX_ADC1_Init();          // ADC1 base
  Enable_VREFINT();        // once
  MUX_GPIO_Init();         // S0..S3

  printf("\r\n4067 → A0 (VREFINT-cal, throwaway sample, open detect)\r\n");

#if (LOCK_CHANNEL != 255)
  // 10 Hz logger on one channel (for freezer/heat gun tests)
  while (1) {
    MUX_Select(LOCK_CHANNEL);
    adc_throwaway_sample();
    adc_sample_t s = Read_A0_raw_mv();
    // open detect
    if (s.mv >= OPEN_MV_THRESHOLD) { printf("C%02u: --- (open)\r\n", LOCK_CHANNEL); }
    else {
      float v = s.mv / 1000.0f;
#if USE_CAL_SCALE
      v *= CAL_VSCALE;
#endif
      float tC = volts_to_degC(v);
      int t10 = (int)(tC*10.0f); int t10a = (t10<0)?-t10:t10;
      printf("C%02u: %u mV  %s%d.%01d C\r\n", LOCK_CHANNEL, s.mv, (t10<0)?"-":"", t10a/10, t10a%10);
    }
    HAL_Delay(100);
  }
#else
  // Scan only the channels you actually wired:
  const uint8_t active[] = { 11,12, 13, 14, 15 };  // <-- edit these to your 4 Cx inputs
  while (1) {
    for (unsigned i=0; i<sizeof(active); ++i) {
      uint8_t ch = active[i];
      MUX_Select(ch);
      adc_throwaway_sample();
      adc_sample_t s = Read_A0_raw_mv();

      if (s.mv >= OPEN_MV_THRESHOLD) {
        printf("C%02u: --- (open)\r\n", ch);
      } else {
        float v = s.mv / 1000.0f;
#if USE_CAL_SCALE
        v *= CAL_VSCALE;
#endif
        float tC = volts_to_degC(v);
        int t10 = (int)(tC*10.0f); int t10a=(t10<0)?-t10:t10;
        printf("C%02u: %u mV  %s%d.%01d C\r\n", ch, s.mv, (t10<0)?"-":"", t10a/10, t10a%10);
      }
      HAL_Delay(75);
    }
    HAL_Delay(250);
  }
#endif
}

/* -------------------- Init functions -------------------- */
void SystemClock_Config(void){
  RCC_OscInitTypeDef osc={0}; RCC_ClkInitTypeDef clk={0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  osc.OscillatorType=RCC_OSCILLATORTYPE_HSI; osc.HSIState=RCC_HSI_ON;
  osc.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;
  osc.PLL.PLLState=RCC_PLL_ON; osc.PLL.PLLSource=RCC_PLLSOURCE_HSI;
  osc.PLL.PLLM=16; osc.PLL.PLLN=336; osc.PLL.PLLP=RCC_PLLP_DIV4; osc.PLL.PLLQ=7;
  if(HAL_RCC_OscConfig(&osc)!=HAL_OK) Error_Handler();
  clk.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider=RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider=RCC_HCLK_DIV2; clk.APB2CLKDivider=RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2)!=HAL_OK) Error_Handler();
}

/* PA0 analog; PA2/PA3 AF7 (USART2) */
static void MX_GPIO_Init(void){
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef g={0};
  g.Pin=GPIO_PIN_0; g.Mode=GPIO_MODE_ANALOG; g.Pull=GPIO_NOPULL;   // A0
  HAL_GPIO_Init(GPIOA,&g);

  g.Pin=GPIO_PIN_2|GPIO_PIN_3; g.Mode=GPIO_MODE_AF_PP; g.Pull=GPIO_PULLUP;
  g.Speed=GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate=GPIO_AF7_USART2;  // USART2
  HAL_GPIO_Init(GPIOA,&g);
}

/* USART2 115200-8N1 (ST-Link VCP) */
static void MX_USART2_UART_Init(void){
  __HAL_RCC_USART2_CLK_ENABLE();
  huart2.Instance=USART2;
  huart2.Init.BaudRate=115200;
  huart2.Init.WordLength=UART_WORDLENGTH_8B;
  huart2.Init.StopBits=UART_STOPBITS_1;
  huart2.Init.Parity=UART_PARITY_NONE;
  huart2.Init.Mode=UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl=UART_HWCONTROL_NONE;
  huart2.Init.OverSampling=UART_OVERSAMPLING_16;
  if(HAL_UART_Init(&huart2)!=HAL_OK) Error_Handler();
}

/* ADC1 base init (we select channels per-read, incl. VREFINT) */
static void MX_ADC1_Init(void){
  __HAL_RCC_ADC1_CLK_ENABLE();
  hadc1.Instance=ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode          = DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  if(HAL_ADC_Init(&hadc1)!=HAL_OK) Error_Handler();
}

/* Trap */
static void Error_Handler(void){
  __disable_irq();
  while(1){}
}
