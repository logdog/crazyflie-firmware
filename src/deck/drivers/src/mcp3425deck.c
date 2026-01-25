#define DEBUG_MODULE "mcp3425Deck"

#include <stdbool.h>
#include "stm32fxxx.h"
#include <string.h>
#include <inttypes.h>

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "mcp3425.h"
#include "mcp3425deck.h"
#include "i2c_drv.h"

static uint8_t buffer[3];
static bool init = false;
static float vbat2Voltage = 7.4f;
static float voltageDividerRatio = 4.6f;

void mcp3425DeckInit() {
  DEBUG_PRINT("mcp3425Deck Init...\n");
  
  mcp3425Init(I2C1_DEV); // PB6 and PB7 i2c interface

  // put into continuous 12-bit mode
  if (!mcp3425EnableContinuous12Bit()) {
    DEBUG_PRINT("mcp3425Deck Init [FAIL]\n");
    return;
  }

  xTaskCreate(mcp3425DeckTask, "mcp3425Deck",
      configMINIMAL_STACK_SIZE, NULL,
      /*priority*/2, NULL);

  DEBUG_PRINT("mcp3425Deck Init [OK]\n");
  init = true;
}

static void mcp3425DeckTask(void* prm)
{
  TickType_t lastWakeTime = xTaskGetTickCount();

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(1000));
    readVoltage();
  }
}

bool mcp3425DeckTest() {
  return mcp3425Test();
}

bool readVoltage() {
    float adcMeasuredVoltage;
    if (!mcp3425ReadVoltage(&adcMeasuredVoltage)) {
      return false;
    }

    // 10k and 36k voltage divider
    vbat2Voltage = adcMeasuredVoltage * voltageDividerRatio;
    return true;
}

// Deck driver registration
static const DeckDriver mcp3425_deck = {
  .vid = 0x00,
  .pid = 0x00,
  .name = "mcp3425Deck",
  .usedGpio = DECK_USING_IO_1 | DECK_USING_IO_2 | DECK_USING_PA2 | DECK_USING_PA3,
  .init = mcp3425DeckInit,
  .test = mcp3425DeckTest,
};

DECK_DRIVER(mcp3425_deck);

PARAM_GROUP_START(mcp3425)
PARAM_ADD(PARAM_FLOAT, vdiv, &voltageDividerRatio)
PARAM_GROUP_STOP(mcp3425)

LOG_GROUP_START(mcp3425)
LOG_ADD(LOG_FLOAT, vbat2, &vbat2Voltage)
LOG_GROUP_STOP(mcp3425)