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

// void servo1MapInit(const MotorPerifDef* servoMapSelect)
// {
//   servo1Map = servoMapSelect;

//   GPIO_InitTypeDef GPIO_InitStructure;
//   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//   TIM_OCInitTypeDef  TIM_OCInitStructure;

//   //clock the servo pin and the timers
//   RCC_AHB1PeriphClockCmd(servo1Map->gpioPerif, ENABLE);
//   RCC_APB1PeriphClockCmd(servo1Map->timPerif, ENABLE);

//   //configure gpio for timer out
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//   GPIO_InitStructure.GPIO_Pin = servo1Map->gpioPin;
//   GPIO_Init(servo1Map->gpioPort, &GPIO_InitStructure);

//   //map timer to alternate function
//   GPIO_PinAFConfig(servo1Map->gpioPort, servo1Map->gpioPinSource, servo1Map->gpioAF);

//   //Timer configuration
//   TIM_TimeBaseStructure.TIM_Period = SERVO_PWM_PERIOD;
//   TIM_TimeBaseStructure.TIM_Prescaler = SERVO_PWM_PRESCALE;
//   TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//   TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//   TIM_TimeBaseInit(servo1Map->tim, &TIM_TimeBaseStructure);

//   // PWM channels configuration
//   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//   TIM_OCInitStructure.TIM_Pulse = 0;
//   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//   TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

//   // Configure OC1
//   servo1Map->ocInit(servo1Map->tim, &TIM_OCInitStructure);
//   servo1Map->preloadConfig(servo1Map->tim, TIM_OCPreload_Enable);


//   //Enable the timer PWM outputs
//   TIM_CtrlPWMOutputs(servo1Map->tim, ENABLE);
//   servo1Map->setCompare(servo1Map->tim, 0x00);

//   //Enable the timer
//   TIM_Cmd(servo1Map->tim, ENABLE);
// }

// void servo2MapInit(const MotorPerifDef* servoMapSelect)
// {
//   servo2Map = servoMapSelect;

//   GPIO_InitTypeDef GPIO_InitStructure;
//   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//   TIM_OCInitTypeDef  TIM_OCInitStructure;

//   //clock the servo pin and the timers
//   RCC_AHB1PeriphClockCmd(servo2Map->gpioPerif, ENABLE);
//   RCC_APB1PeriphClockCmd(servo2Map->timPerif, ENABLE);

//   //configure gpio for timer out
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//   GPIO_InitStructure.GPIO_Pin = servo2Map->gpioPin;
//   GPIO_Init(servo2Map->gpioPort, &GPIO_InitStructure);

//   //map timer to alternate function
//   GPIO_PinAFConfig(servo2Map->gpioPort, servo2Map->gpioPinSource, servo2Map->gpioAF);

//   //Timer configuration
//   TIM_TimeBaseStructure.TIM_Period = SERVO_PWM_PERIOD;
//   TIM_TimeBaseStructure.TIM_Prescaler = SERVO_PWM_PRESCALE;
//   TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//   TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//   TIM_TimeBaseInit(servo2Map->tim, &TIM_TimeBaseStructure);

//   // PWM channels configuration
//   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//   TIM_OCInitStructure.TIM_Pulse = 0;
//   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//   TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

//   // Configure OC1
//   servo2Map->ocInit(servo2Map->tim, &TIM_OCInitStructure);
//   servo2Map->preloadConfig(servo2Map->tim, TIM_OCPreload_Enable);


//   //Enable the timer PWM outputs
//   TIM_CtrlPWMOutputs(servo2Map->tim, ENABLE);
//   servo2Map->setCompare(servo2Map->tim, 0x00);

//   //Enable the timer
//   TIM_Cmd(servo2Map->tim, ENABLE);
// }

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
  }
}

#if defined(CONFIG_BICOPTER_NAME_REDCOPTER)
// left servo
// void servo1SetAngle(double angle)
// {
//   // set CCR register
//   // Duty% = CCR/ARR*100, so CCR = Duty%/100 * ARR

//   double pulse_length_us = 1500.0 + 10.85147 * angle;
//   double pulse_length_s = pulse_length_us / 1000000;
//   const uint32_t ccr_val = (uint32_t)(pulse_length_s * SERVO_PWM_PERIOD * SERVO_PWM_FREQUENCY_HZ + left_servo_trim);
//   servo1Map->setCompare(servo1Map->tim, ccr_val);
  
//   #ifdef DEBUG_SERVO
//     DEBUG_PRINT("Set Angle: %u deg, pulse width: %f us \n", angle, pulse_length_us);
//   #endif
// }

// // right servo
// void servo2SetAngle(double angle)
// {
//   // set CCR register
//   // Duty% = CCR/ARR*100, so CCR = Duty%/100 * ARR

//   double pulse_length_us = 1500.0 + 10.85147 * angle; // found using encoder for system ID
//   // double pulse_length_us = 1500.0 + angle; // found using encoder for system ID, this is a linear approximation
//   double pulse_length_s = pulse_length_us / 1000000;
//   const uint32_t ccr_val = (uint32_t)(pulse_length_s * SERVO_PWM_PERIOD * SERVO_PWM_FREQUENCY_HZ + right_servo_trim);
//   servo2Map->setCompare(servo2Map->tim, ccr_val);
  
//   #ifdef DEBUG_SERVO
//     DEBUG_PRINT("Set Angle: %u deg, pulse width: %f us \n", angle, pulse_length_us);
//   #endif
// }

// void servo1SetAngle(double angle)
// {
//   const uint32_t ccr_val = (uint32_t)(600 + left_servo_trim + angle*4);
//   servo1Map->setCompare(servo1Map->tim, ccr_val);
// }

// // right servo (angle is negative to account for its orientation)
// void servo2SetAngle(double angle)
// {
//   const uint32_t ccr_val = (uint32_t)(600 - right_servo_trim - angle*4);
//   servo2Map->setCompare(servo2Map->tim, ccr_val);
// }
#else  // MELONCOPTER
// void servo1SetAngle(double angle)
// {
//   const uint32_t ccr_val = (uint32_t)(600 + left_servo_trim + angle*4);
//   servo1Map->setCompare(servo1Map->tim, ccr_val);
// }

// // right servo (angle is negative to account for its orientation)
// void servo2SetAngle(double angle)
// {
//   const uint32_t ccr_val = (uint32_t)(600 - right_servo_trim - angle*4);
//   servo2Map->setCompare(servo2Map->tim, ccr_val);
// }
#endif


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