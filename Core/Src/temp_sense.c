#include "temp_sense.h"
#include <stdint.h>
#include <string.h>

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

TempSenseInit_t tempSense;

static uint8_t selectedCell;
static uint32_t rawTemperatureReadings[CELLS_PER_MUX][MUX_BANK_COUNT];
static uint8_t syncSegments;

static void checkForFaults();
static void sendTempSummaryOverCan();
static void sendSegmentTempsOverCan();

static void toggleFaultFromFlag(FaultLine_e faultLine, FaultLine_e flag);
static float voltageToTempC(float volts);

void initialize_temp_sense(TempSenseInit_t init) {
  tempSense = init;

  selectedCell = 0;
  tempSense.selectMuxCell(selectedCell);

  for(uint8_t cell = 0; cell < CELLS_PER_MUX; cell++) {
    for(uint8_t mux = 0; mux < MUX_BANK_COUNT; mux++) {
      rawTemperatureReadings[cell][mux] = VOLTAGE_TO_ADC_UNITS(CELL_TEMP_ROOM_TEMPERATURE_VOLTAGE);
    }
  }

  syncSegments = 0;
}

void update_raw_temperatures(uint32_t rawTempData[MUX_BANK_COUNT]) {
  memcpy(rawTemperatureReadings[selectedCell], rawTempData, sizeof(rawTemperatureReadings[0]));
  selectedCell = (selectedCell + 1) % CELLS_PER_MUX; 
  tempSense.selectMuxCell(selectedCell);
}

void handle_can_message(uint32_t canStdId, CanData_t canData, uint8_t dlc) {
  if (canStdId == CAN_SYNC_SUMMARY_CAN_ID) {
    syncSegments |= 0x1;
  } else {
    syncSegments |= (1 << canData.data8[0]);
  }
}

void loop() {
  checkForFaults();
  sendTempSummaryOverCan();
  sendSegmentTempsOverCan();
}

static void checkForFaults() {
  FaultLine_e faults = 0;
  for(uint8_t cell = 0; cell < CELLS_PER_MUX; cell++) 
  {
    for(uint8_t i = 0; i < MUX_BANK_COUNT; i++) {
      uint32_t temperature_AdcUnit = rawTemperatureReadings[cell][i];

      if (temperature_AdcUnit <= VOLTAGE_TO_ADC_UNITS(CELL_DISCHARGE_MAX_TEMP_VOLTAGE)) {
        faults |= DISCHARGE_TEMP_FAULT;
      } else if (temperature_AdcUnit >= VOLTAGE_TO_ADC_UNITS(CELL_DISCHARGE_MIN_TEMP_VOLTAGE)) {
        faults |= DISCHARGE_TEMP_FAULT;
      }

      if (temperature_AdcUnit <= VOLTAGE_TO_ADC_UNITS(CELL_CHARGE_MAX_TEMP_VOLTAGE)) {
        faults |= CHARGE_TEMP_FAULT;
      } else if (temperature_AdcUnit >= VOLTAGE_TO_ADC_UNITS(CELL_CHARGE_MIN_TEMP_VOLTAGE)) {
        faults |= CHARGE_TEMP_FAULT;
      }
    }
  }
  toggleFaultFromFlag(DISCHARGE_TEMP_FAULT, faults);
  toggleFaultFromFlag(CHARGE_TEMP_FAULT, faults);
}

static void sendTempSummaryOverCan() {
  if (syncSegments & 0x1) {
    syncSegments = syncSegments ^ 0x1;

    uint32_t highestTemp_AdcUnits = UINT32_MAX;
    uint32_t lowestTemp_AdcUnits = 0;
    uint64_t tempSum_AdcUnits = 0;

    for(uint8_t cell = 0; cell < CELLS_PER_MUX; cell++) {
      for(uint8_t mux = 0; mux < MUX_BANK_COUNT; mux++) {
        uint32_t temperature_AdcUnits = rawTemperatureReadings[cell][mux];
        tempSum_AdcUnits += rawTemperatureReadings[cell][mux];

        if (temperature_AdcUnits < highestTemp_AdcUnits) {
          highestTemp_AdcUnits = temperature_AdcUnits;
        }

        if (temperature_AdcUnits > lowestTemp_AdcUnits) {
          lowestTemp_AdcUnits = temperature_AdcUnits;
        }
      }
    }

    tempSum_AdcUnits = tempSum_AdcUnits / (uint64_t)(CELLS_PER_MUX * MUX_BANK_COUNT);

    CanData_t data = {
      .data8 = {
        (uint8_t)voltageToTempC(ADC_UNITS_TO_VOLTAGE(highestTemp_AdcUnits)) + CAN_TEMP_C_OFFSET,
        (uint8_t)voltageToTempC(ADC_UNITS_TO_VOLTAGE(lowestTemp_AdcUnits)) + CAN_TEMP_C_OFFSET,
        (uint8_t)voltageToTempC(ADC_UNITS_TO_VOLTAGE(tempSum_AdcUnits)) + CAN_TEMP_C_OFFSET,
      }
    };

    tempSense.sendCanMessage(CAN_RESP_SUMMARY_CAN_ID, data, CAN_RESP_SUMMARY_DLC);
  }
}

static void sendSegmentTempsOverCan() {
  if (syncSegments < 2) return;

  for(uint8_t segment = 1; segment <= SEGMENT_COUNT; segment++) {
    if ((syncSegments >> segment) & 0x1) 
    {
      syncSegments = syncSegments ^ (0x1 << segment);

      union {
        uint8_t temperatures[2][12];
        CanData_t canBuffers[3];
      } data;

      for(uint8_t i = 0; i < CELLS_PER_MUX; i++) {
        data.temperatures[0][i] = (uint8_t)voltageToTempC(ADC_UNITS_TO_VOLTAGE(rawTemperatureReadings[i][segment-1])) + CAN_TEMP_C_OFFSET;
        data.temperatures[1][i] = (uint8_t)voltageToTempC(ADC_UNITS_TO_VOLTAGE(rawTemperatureReadings[i][segment-1+5])) + CAN_TEMP_C_OFFSET;
      }

      tempSense.sendCanMessage(0x302 + segment*0x10, data.canBuffers[0], 8);
      tempSense.sendCanMessage(0x303 + segment*0x10, data.canBuffers[1], 8);
      tempSense.sendCanMessage(0x304 + segment*0x10, data.canBuffers[2], 8);
    }
  }
}

static void toggleFaultFromFlag(FaultLine_e faultLine, FaultLine_e flag) {
  if (flag & faultLine) {
    tempSense.setFault(faultLine);
  } else {
    tempSense.clearFault(faultLine);
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

  return 999.9f;
}
