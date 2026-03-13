#include "unity.h"

#include "temp_sense.h"
#include <string.h>

#define MAX_CAN_MESSAGES                                   4

typedef struct {
  uint32_t stdId;
  CanData_t data;
  uint8_t dlc;
} CanMessage_t;

static uint32_t selectedCell;
static FaultLine_e faults;
static CanMessage_t canMessages[MAX_CAN_MESSAGES];
static uint8_t receivedCanMessageCount;

static void selectMuxCell(uint8_t bank);
static Result_t sendCanMessage(uint32_t canStdId, CanData_t data, uint8_t dlc);
static void setFault(FaultLine_e faultLine);
static void clearFault(FaultLine_e faultLine);

/* ============================== Setup ============================== */
void setUp(void) {
  selectedCell = 0;
  faults = 0;
  receivedCanMessageCount = 0;

  TempSenseInit_t init = {
    .selectMuxCell = selectMuxCell,
    .sendCanMessage = sendCanMessage,
    .setFault = setFault,
    .clearFault = clearFault
  };

  initialize_temp_sense(init);
}

void tearDown(void) {
}

/* ============================== Utilities ============================== */
static void selectMuxCell(uint8_t bank) {
  selectedCell = bank;
}

static Result_t sendCanMessage(uint32_t canStdId, CanData_t data, uint8_t dlc) {
  if (receivedCanMessageCount == MAX_CAN_MESSAGES) return RESULT_FAILED;

  CanMessage_t message = {
    .stdId = canStdId,
    .data = data,
    .dlc = dlc
  };
  canMessages[receivedCanMessageCount++] = message;

  return RESULT_OK;
}

static void setFault(FaultLine_e faultLine) {
  faults = faults | faultLine;
}

static void clearFault(FaultLine_e faultLine) {
  faults = faults & (~faultLine);
}

/* ============================== Tests ============================== */
void test_initializeTempSense_selectFirstCellInMuxBank() {
  TEST_ASSERT_EQUAL(0, selectedCell);
}

TEST_CASE(1, 1)
TEST_CASE(11, 11)
TEST_CASE(12, 0)
void test_updateRawTemperatures_incrementsSelectedCell(uint8_t invocationCount, uint8_t expectedBank) {
  uint32_t rawTempReadings[MUX_BANK_COUNT];

  for(uint8_t i = 0; i < invocationCount; i++) {
    update_raw_temperatures(rawTempReadings);
  }

  TEST_ASSERT_EQUAL(expectedBank, selectedCell);
}

TEST_MATRIX([0, 9, 1], [0, 11, 1])
void test_loop_setsDischargeFaultWhenTemperaturesAboveDischargeThreshold(uint8_t muxBank, uint8_t cell)
{
  uint32_t rawTempReadings[MUX_BANK_COUNT];
  for(uint8_t i = 0; i < MUX_BANK_COUNT; i++) {
    rawTempReadings[i] = VOLTAGE_TO_ADC_UNITS(CELL_TEMP_ROOM_TEMPERATURE_VOLTAGE);
  }

  for(uint8_t i = 0; i < CELLS_PER_MUX; i++) {
    if (i == cell) {
      rawTempReadings[muxBank] = VOLTAGE_TO_ADC_UNITS(CELL_DISCHARGE_MAX_TEMP_VOLTAGE);
    }
    update_raw_temperatures(rawTempReadings);
    loop();
  }

  TEST_ASSERT_TRUE(faults & DISCHARGE_TEMP_FAULT);
}

TEST_MATRIX([0, 9, 1], [0, 11, 1])
void test_loop_doesNotTriggerDischargeIfTemperaturesAreJustBelowDischargeThreshold(uint8_t muxBank, uint8_t cell)
{
  uint32_t rawTempReadings[MUX_BANK_COUNT];
  for(uint8_t i = 0; i < MUX_BANK_COUNT; i++) {
    rawTempReadings[i] = VOLTAGE_TO_ADC_UNITS(CELL_TEMP_ROOM_TEMPERATURE_VOLTAGE);
  }

  for(uint8_t i = 0; i < CELLS_PER_MUX; i++) {
    if (i == cell) {
      rawTempReadings[muxBank] = (uint32_t)VOLTAGE_TO_ADC_UNITS(CELL_DISCHARGE_MAX_TEMP_VOLTAGE) + 1;
    }
    update_raw_temperatures(rawTempReadings);
    loop();
  }

  TEST_ASSERT_FALSE(faults & DISCHARGE_TEMP_FAULT);
}

TEST_MATRIX([0, 9, 1], [0, 11, 1])
void test_loop_shouldClearDischargeFaultWhenTemperatureFallsBelowDischargeThreshold(uint8_t muxBank, uint8_t cell) {
  uint32_t rawTempReadings[MUX_BANK_COUNT];
  for(uint8_t i = 0; i < MUX_BANK_COUNT; i++) {
    rawTempReadings[i] = VOLTAGE_TO_ADC_UNITS(CELL_TEMP_ROOM_TEMPERATURE_VOLTAGE);
  }

  for(uint8_t i = 0; i < CELLS_PER_MUX; i++) {
    if (i == cell) {
      rawTempReadings[muxBank] = VOLTAGE_TO_ADC_UNITS(CELL_DISCHARGE_MAX_TEMP_VOLTAGE);
    }
    update_raw_temperatures(rawTempReadings);
    loop();
  }

  for(uint8_t i = 0; i < MUX_BANK_COUNT; i++) {
    rawTempReadings[i] = VOLTAGE_TO_ADC_UNITS(CELL_TEMP_ROOM_TEMPERATURE_VOLTAGE);
  }

  for(uint8_t i = 0; i < CELLS_PER_MUX; i++) {
    update_raw_temperatures(rawTempReadings);
    loop();
  }

  TEST_ASSERT_FALSE(faults & DISCHARGE_TEMP_FAULT);
}

TEST_MATRIX([0, 9, 1], [0, 11, 1])
void test_loop_setsChargeFaultWhenTemperaturesAboveChargeThreshold(uint8_t muxBank, uint8_t cell)
{
  uint32_t rawTempReadings[MUX_BANK_COUNT];
  for(uint8_t i = 0; i < MUX_BANK_COUNT; i++) {
    rawTempReadings[i] = VOLTAGE_TO_ADC_UNITS(CELL_TEMP_ROOM_TEMPERATURE_VOLTAGE);
  }

  for(uint8_t i = 0; i < CELLS_PER_MUX; i++) {
    if (i == cell) {
      rawTempReadings[muxBank] = VOLTAGE_TO_ADC_UNITS(CELL_CHARGE_MAX_TEMP_VOLTAGE);
    }
    update_raw_temperatures(rawTempReadings);
    loop();
  }

  TEST_ASSERT_TRUE(faults & CHARGE_TEMP_FAULT);
}

TEST_MATRIX([0, 9, 1], [0, 11, 1])
void test_loop_doesNotTriggerChargeIfTemperaturesAreJustBelowChargeThreshold(uint8_t muxBank, uint8_t cell)
{
  uint32_t rawTempReadings[MUX_BANK_COUNT];
  for(uint8_t i = 0; i < MUX_BANK_COUNT; i++) {
    rawTempReadings[i] = VOLTAGE_TO_ADC_UNITS(CELL_TEMP_ROOM_TEMPERATURE_VOLTAGE);
  }

  for(uint8_t i = 0; i < CELLS_PER_MUX; i++) {
    if (i == cell) {
      rawTempReadings[muxBank] = (uint32_t)VOLTAGE_TO_ADC_UNITS(CELL_CHARGE_MAX_TEMP_VOLTAGE) + 1;
    }
    update_raw_temperatures(rawTempReadings);
    loop();
  }

  TEST_ASSERT_FALSE(faults & CHARGE_TEMP_FAULT);
}

void test_loop_doesNotSetFaultWhenTemperaturesAreWithinOperatingConditions() {
  uint32_t rawTempReadings[MUX_BANK_COUNT];
  for(uint8_t i = 0; i < MUX_BANK_COUNT; i++) {
    rawTempReadings[i] = VOLTAGE_TO_ADC_UNITS(CELL_TEMP_ROOM_TEMPERATURE_VOLTAGE);
  }

  for(uint8_t i = 0; i < CELLS_PER_MUX; i++) {
    update_raw_temperatures(rawTempReadings);
    loop();
  }

  TEST_ASSERT_FALSE(faults);
}

TEST_MATRIX([0, 9, 1], [0, 11, 1])
void test_loop_shouldClearChargeFaultWhenTemperatureFallsBelowChargeThreshold(uint8_t muxBank, uint8_t cell) {
  uint32_t rawTempReadings[MUX_BANK_COUNT];
  for(uint8_t i = 0; i < MUX_BANK_COUNT; i++) {
    rawTempReadings[i] = VOLTAGE_TO_ADC_UNITS(CELL_TEMP_ROOM_TEMPERATURE_VOLTAGE);
  }

  for(uint8_t i = 0; i < CELLS_PER_MUX; i++) {
    if (i == cell) {
      rawTempReadings[muxBank] = VOLTAGE_TO_ADC_UNITS(CELL_CHARGE_MAX_TEMP_VOLTAGE);
    }
    update_raw_temperatures(rawTempReadings);
    loop();
  }

  for(uint8_t i = 0; i < MUX_BANK_COUNT; i++) {
    rawTempReadings[i] = VOLTAGE_TO_ADC_UNITS(CELL_TEMP_ROOM_TEMPERATURE_VOLTAGE);
  }

  for(uint8_t i = 0; i < CELLS_PER_MUX; i++) {
    update_raw_temperatures(rawTempReadings);
    loop();
  }

  TEST_ASSERT_FALSE(faults & CHARGE_TEMP_FAULT);
}

TEST_CASE(1, 0x312)
TEST_CASE(2, 0x322)
TEST_CASE(3, 0x332)
TEST_CASE(4, 0x342)
TEST_CASE(5, 0x352)
void test_loop_shouldSendSegmentTemperaturesOverCanAfterReceivingSyncMessage(uint8_t segment, uint32_t canIdOffset) {
  CanData_t data = { 0 };
  data.data8[0] = segment;
  handle_can_message(CAN_SYNC_SEGMENT_CAN_ID, data, 1);

  loop();

  uint8_t expectedTemps[8] = { 65, 65, 65, 65, 65, 65, 65, 65 };
  TEST_ASSERT_EQUAL(3, receivedCanMessageCount);
  TEST_ASSERT_EQUAL_HEX32(canIdOffset, canMessages[0].stdId);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedTemps, canMessages[0].data.data8, 8);

  TEST_ASSERT_EQUAL_HEX32(canIdOffset + 1, canMessages[1].stdId);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedTemps, canMessages[1].data.data8, 8);

  TEST_ASSERT_EQUAL_HEX32(canIdOffset + 2, canMessages[2].stdId);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedTemps, canMessages[2].data.data8, 8);
}

TEST_CASE(CAN_SYNC_SEGMENT_CAN_ID)
void test_handleCanMessage_shouldNotSendSegmentSyncMessagesFromHandler(uint32_t canId) {
  CanData_t data = { 0 };
  data.data8[0] = 1;
  handle_can_message(canId, data, 1);

  TEST_ASSERT_EQUAL(0, receivedCanMessageCount);
}

TEST_CASE(1, 0x312)
TEST_CASE(2, 0x322)
TEST_CASE(3, 0x332)
TEST_CASE(4, 0x342)
TEST_CASE(5, 0x352)
void test_loop_shouldSendSegmentTemperaturesOnlyOnceUntilNextSyncMessage(uint8_t segment, uint32_t canIdOffset) {
  CanData_t data = { 0 };
  data.data8[0] = segment;
  handle_can_message(CAN_SYNC_SEGMENT_CAN_ID, data, 1);

  loop();
  receivedCanMessageCount = 0;
  loop();

  TEST_ASSERT_EQUAL(0, receivedCanMessageCount);
}

TEST_CASE(1, 0x312)
TEST_CASE(2, 0x322)
TEST_CASE(3, 0x332)
TEST_CASE(4, 0x342)
TEST_CASE(5, 0x352)
void test_loop_shouldCorrectlyMapSegmentTemperaturesToDegreesC(uint8_t segment, uint32_t canIdOffset) {
  temp_point_t muxTemps[MUX_BANK_COUNT] = {
    { 0.0f, 2.17f },    
    { 2.5f, 2.135f },   
    { 12.5f, 2.015f },   
    { 20.0f, 1.92f },    
    { 32.5f, 1.77f },    
    { 10.0f, 2.05f },    
    { 60.0f, 1.51f },    
    { 65.0f, 1.48f },    
    { 62.0f, 1.495f },   
    { 40.0f, 1.68f }     
  };

  uint32_t rawTempReadings[MUX_BANK_COUNT]; 
  for(uint8_t i = 0; i < MUX_BANK_COUNT; i++) {
    rawTempReadings[i] = VOLTAGE_TO_ADC_UNITS(muxTemps[i].volts);
  }

  for(uint8_t i = 0; i < CELLS_PER_MUX; i++) {
    update_raw_temperatures(rawTempReadings);
  }

  CanData_t data = { 0 };
  data.data8[0] = segment;
  handle_can_message(CAN_SYNC_SEGMENT_CAN_ID, data, 1);

  loop();

  typedef union {
    uint8_t canByteArrays[3][8];
    uint8_t testByteArray[2][12];
  } TempData_t;

  TempData_t expectedTempData;
  for(uint8_t i = 0; i < CELLS_PER_MUX; i++ ) {
    expectedTempData.testByteArray[0][i] = (uint8_t)(muxTemps[segment-1].temp_c) + 40;
    expectedTempData.testByteArray[1][i] = (uint8_t)(muxTemps[segment-1+5].temp_c) + 40;
  }

  TEST_ASSERT_EQUAL(3, receivedCanMessageCount);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedTempData.canByteArrays[0], canMessages[0].data.data8, 8);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedTempData.canByteArrays[1], canMessages[1].data.data8, 8);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedTempData.canByteArrays[2], canMessages[2].data.data8, 8);
}

void test_loop_sendsTemperatureSummaryWhenSyncMessageReceived() {
  temp_point_t muxTemps[MUX_BANK_COUNT] = {
    { 0.0f, 2.17f },    
    { 2.5f, 2.135f },   
    { 12.5f, 2.015f },   
    { 20.0f, 1.92f },    
    { 32.5f, 1.77f },    
    { 10.0f, 2.05f },    
    { 60.0f, 1.51f },    
    { 65.0f, 1.48f },    
    { 62.0f, 1.495f },   
    { 40.0f, 1.68f }     
  };

  uint32_t rawTempReadings[MUX_BANK_COUNT]; 
  for(uint8_t i = 0; i < MUX_BANK_COUNT; i++) {
    rawTempReadings[i] = VOLTAGE_TO_ADC_UNITS(muxTemps[i].volts);
  }

  for(uint8_t i = 0; i < CELLS_PER_MUX; i++) {
    update_raw_temperatures(rawTempReadings);
  }

  CanData_t data = { 0 };
  handle_can_message(CAN_SYNC_SUMMARY_CAN_ID, data, 0);

  loop();

  TEST_ASSERT_EQUAL(1, receivedCanMessageCount);

  uint8_t expectedDataLength = 3;
  CanData_t expectedData = {
    .data8 = {
      105, // Highest temp (60 C)
      40,  // Lowest Temp (0 C)
      68   // Average Temp (28 C)
    }
  };
  TEST_ASSERT_EQUAL(expectedDataLength, canMessages[0].dlc);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedData.data8, canMessages[0].data.data8, expectedDataLength);
}

// TODO Implement low pass filter for temperature readings
// TODO Trigger faults on lower threshold for temperatures
// TODO Send summary message
// TODO Discard bad readings from lowest/highest/avg temp readings in summary
