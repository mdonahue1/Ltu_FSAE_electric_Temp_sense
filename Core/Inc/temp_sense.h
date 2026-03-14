#ifndef _INC_TEMP_SENSE_H_
#define _INC_TEMP_SENSE_H_

#include <stdint.h>

#define SEGMENT_COUNT                                 5
#define MUX_BANK_COUNT                                10
#define CELLS_PER_MUX                                 12

#define ADC_UNITS                                     4095.0f
#define MAX_ADC_VOLTAGE                               3.3f
#define VOLTAGE_TO_ADC_UNITS(x)                       ((uint32_t)((x) * ADC_UNITS / MAX_ADC_VOLTAGE))
#define ADC_UNITS_TO_VOLTAGE(x)                       ((x) * MAX_ADC_VOLTAGE / ADC_UNITS)

#define CELL_TEMP_SHORT_TO_GROUND_THRESHOLD_ADC_UNITS VOLTAGE_TO_ADC_UNITS(0.2f)
#define CELL_TEMP_OPEN_CIRCUIT_THRESHOLD_ADC_UNITS    VOLTAGE_TO_ADC_UNITS(3.1f)
#define CELL_TEMP_MINIMUM_VALID_READINGS              ((uint8_t)(MUX_BANK_COUNT*CELLS_PER_MUX)*0.6f)

#define CELL_TEMP_ROOM_TEMPERATURE_VOLTAGE            1.86f

#define CELL_CHARGE_MIN_TEMP_VOLTAGE                  2.17f
#define CELL_CHARGE_MIN_TEMP_C                        0.0f 
#define CELL_CHARGE_MAX_TEMP_VOLTAGE                  1.58f
#define CELL_CHARGE_MAX_TEMP_C                        45.0f

#define CELL_DISCHARGE_MIN_TEMP_VOLTAGE               2.35f
#define CELL_DISCHARGE_MIN_TEMP_C                     -20.0f
#define CELL_DISCHARGE_MAX_TEMP_VOLTAGE               1.51f
#define CELL_DISCHARGE_MAX_TEMP_C                     60.0f

#define CAN_TEMP_C_OFFSET                             40

#define CAN_SYNC_SUMMARY_CAN_ID                       0x301
#define CAN_RESP_SUMMARY_CAN_ID                       0x311
#define CAN_RESP_SUMMARY_DLC                          3

#define CAN_SYNC_SEGMENT_CAN_ID                       0x302
#define CAN_RESP_SEGMENT_CAN_ID_BASE                  0x312
#define CAN_RESP_SEGMENT_CAN_ID_SEGMENT_OFFSET        0x010

#define RESULT_OK                                     0
#define RESULT_FAILED                                 -128

typedef enum {
  DISCHARGE_TEMP_FAULT = 0b01,
  CHARGE_TEMP_FAULT = 0b10
} FaultLine_e;

typedef union {
  uint64_t data64;
  uint8_t data8[8];
} CanData_t;

typedef int8_t Result_t;

typedef struct {
  float temp_c;
  float volts;
} temp_point_t;

typedef struct {
  void (*selectMuxCell)(uint8_t bank);
  Result_t (*sendCanMessage)(uint32_t canStdId, CanData_t data, uint8_t dlc);
  void (*setFault)(FaultLine_e faultLine);
  void (*clearFault)(FaultLine_e faultLine);
} TempSenseInit_t;

void initialize_temp_sense(TempSenseInit_t init);
void update_raw_temperatures(uint32_t rawTempData[MUX_BANK_COUNT]);
void handle_can_message(uint32_t canStdId, CanData_t data, uint8_t dlc);
void loop();

#endif // _INC_TEMP_SENSE_H_
