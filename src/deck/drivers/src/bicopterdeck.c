#define DEBUG_MODULE "BICOPTERDECK"

#include <stdbool.h>

/* ST includes */
#include "stm32fxxx.h"

#include <string.h>
#include <inttypes.h>
#include "motors.h"

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "extrx.h"
#include "flapperdeck.h"
#include "pm.h"
#include "autoconf.h"
#include "config.h"

// static uint16_t servo_MIN_us = 1000;
// static uint16_t servo_MAX_us = 2000;

#include "bicopterdeck.h"
#include "deck_analog.h"

// #define DEBUG_SERVO

// Add analog reading variables
static uint16_t pa6_raw = 0;
static uint16_t pa7_raw = 0;

static bool init = false;
void bicopterDeckInit()
{
  DEBUG_PRINT("bicopterDeckInit()\n");

  // Initialize ADC for analog readings
  adcInit();
  DEBUG_PRINT("ADC Init [OK]\n");
  init = true;

  xTaskCreate(bicopterDeckTask, BICOPTERDECK_TASK_NAME, BICOPTERDECK_TASK_STACKSIZE, NULL, BICOPTERDECK_TASK_PRI, NULL);
}

bool bicopterDeckTest() {
  return init;
}

#define SERVO_BUFFER_LENGTH 10
void bicopterDeckTask(void* arg)
{
  systemWaitStart();
  TickType_t xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();

  uint16_t pa6_measurements[SERVO_BUFFER_LENGTH] = { 0 };
  uint16_t pa7_measurements[SERVO_BUFFER_LENGTH] = { 0 };
  uint8_t i = 0;

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, M2T(1)); // 1 ms = 1kHz Hz

    // Read analog values from PA6 and PA7
    // and add them to the buffer
    pa6_measurements[i] = analogRead(DECK_GPIO_MISO);
    pa7_measurements[i] = analogRead(DECK_GPIO_MOSI);
    i++;
    i %= SERVO_BUFFER_LENGTH;

    // get running average
    float pa6_tmp = 0.0f;
    float pa7_tmp = 0.0f;
    for(uint8_t j = 0; j < SERVO_BUFFER_LENGTH; j++) {
      pa6_tmp += pa6_measurements[j];
      pa7_tmp += pa7_measurements[j];
    }
    pa6_raw = pa6_tmp / SERVO_BUFFER_LENGTH;
    pa7_raw = pa7_tmp / SERVO_BUFFER_LENGTH;

    // pa6_raw = analogRead(DECK_GPIO_MISO);
    // pa7_raw = analogRead(DECK_GPIO_MOSI);
  }
}

static const DeckDriver bicopter_deck = {
  .vid = 0x00,
  .pid = 0x00,
  .name = "bicopterDeck",

  .usedPeriph = 0,
  .usedGpio = DECK_USING_PA6 | DECK_USING_PA7,
  .requiredEstimator = StateEstimatorTypeKalman,

  .init = bicopterDeckInit,
  .test = bicopterDeckTest,
};

DECK_DRIVER(bicopter_deck);

LOG_GROUP_START(bideck)
// 0-4095 read these values
LOG_ADD(LOG_UINT16, pa6_raw, &pa6_raw)
LOG_ADD(LOG_UINT16, pa7_raw, &pa7_raw)
LOG_GROUP_STOP(bideck)