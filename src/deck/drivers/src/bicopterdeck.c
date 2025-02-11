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

static uint16_t servo_MIN_us = 1000;
static uint16_t servo_MAX_us = 2000;

#include "bicopterdeck.h"

// #define DEBUG_SERVO

static bool isInit1 = false;
static bool isInit2 = false;

const MotorPerifDef* servo1Map;
const MotorPerifDef* servo2Map;
extern const MotorPerifDef* servoMapIO1;
extern const MotorPerifDef* servoMapIO2;
extern const MotorPerifDef* servoMapIO3;
extern const MotorPerifDef* servoMapRX2;
extern const MotorPerifDef* servoMapTX2;
extern const MotorPerifDef* servoMapMOSI;

/* Public functions */
static uint8_t servo1_idle = 90;
static uint8_t servo2_idle = 90;
static uint8_t servo_range = 180; // in degrees

uint8_t s_servo1_angle = 90;
uint8_t s_servo2_angle = 90;

void servo1MapInit(const MotorPerifDef* servoMapSelect)
{
  servo1Map = servoMapSelect;

  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //clock the servo pin and the timers
  RCC_AHB1PeriphClockCmd(servo1Map->gpioPerif, ENABLE);
  RCC_APB1PeriphClockCmd(servo1Map->timPerif, ENABLE);

  //configure gpio for timer out
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Pin = servo1Map->gpioPin;
  GPIO_Init(servo1Map->gpioPort, &GPIO_InitStructure);

  //map timer to alternate function
  GPIO_PinAFConfig(servo1Map->gpioPort, servo1Map->gpioPinSource, servo1Map->gpioAF);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = SERVO_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = SERVO_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(servo1Map->tim, &TIM_TimeBaseStructure);

  // PWM channels configuration
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure OC1
  servo1Map->ocInit(servo1Map->tim, &TIM_OCInitStructure);
  servo1Map->preloadConfig(servo1Map->tim, TIM_OCPreload_Enable);


  //Enable the timer PWM outputs
  TIM_CtrlPWMOutputs(servo1Map->tim, ENABLE);
  servo1Map->setCompare(servo1Map->tim, 0x00);

  //Enable the timer
  TIM_Cmd(servo1Map->tim, ENABLE);
}

void servo2MapInit(const MotorPerifDef* servoMapSelect)
{
  servo2Map = servoMapSelect;

  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //clock the servo pin and the timers
  RCC_AHB1PeriphClockCmd(servo2Map->gpioPerif, ENABLE);
  RCC_APB1PeriphClockCmd(servo2Map->timPerif, ENABLE);

  //configure gpio for timer out
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Pin = servo2Map->gpioPin;
  GPIO_Init(servo2Map->gpioPort, &GPIO_InitStructure);

  //map timer to alternate function
  GPIO_PinAFConfig(servo2Map->gpioPort, servo2Map->gpioPinSource, servo2Map->gpioAF);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = SERVO_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = SERVO_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(servo2Map->tim, &TIM_TimeBaseStructure);

  // PWM channels configuration
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure OC1
  servo2Map->ocInit(servo2Map->tim, &TIM_OCInitStructure);
  servo2Map->preloadConfig(servo2Map->tim, TIM_OCPreload_Enable);


  //Enable the timer PWM outputs
  TIM_CtrlPWMOutputs(servo2Map->tim, ENABLE);
  servo2Map->setCompare(servo2Map->tim, 0x00);

  //Enable the timer
  TIM_Cmd(servo2Map->tim, ENABLE);
}

void servoInit()
{

  DEBUG_PRINT("servoInit()\n");

  if (!isInit1){
    // CONFIG_DECK_SERVO_USE_IO1
    servo1MapInit(servoMapIO1);
    DEBUG_PRINT("Init on IO1 [OK]\n");
    
    servo1SetAngle(saturateAngle(servo1_idle));
    // s_servo1_angle = servo1_idle;
    isInit1 = true;
  }

  if (!isInit2){
    // CONFIG_DECK_SERVO_USE_IO2
    servo2MapInit(servoMapIO2);
    DEBUG_PRINT("Init on IO2 [OK]\n");
    
    servo2SetAngle(saturateAngle(servo2_idle));
    // s_servo2_angle = servo2_idle;
    isInit2 = true;
  }

  xTaskCreate(bicopterDeckTask, BICOPTERDECK_TASK_NAME, BICOPTERDECK_TASK_STACKSIZE, NULL, BICOPTERDECK_TASK_PRI, NULL);
}

void bicopterDeckTask(void* arg)
{
  systemWaitStart();
  TickType_t xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, M2T(20)); // 20 ms = 50 Hz

    servo1SetAngle(saturateAngle(s_servo1_angle));
    servo2SetAngle(saturateAngle(s_servo2_angle));
  }
}

bool servoTest(void)
{
  return isInit1 && isInit2;
}

void servo1SetAngle(uint8_t angle)
{
  // set CCR register
  // Duty% = CCR/ARR*100, so CCR = Duty%/100 * ARR

  double pulse_length_us = (double)(angle) / servo_range * (servo_MAX_us - servo_MIN_us) + servo_MIN_us;
  double pulse_length_s = pulse_length_us / 1000000;
  const uint32_t ccr_val = (uint32_t)(pulse_length_s * SERVO_PWM_PERIOD * SERVO_PWM_FREQUENCY_HZ);
  servo1Map->setCompare(servo1Map->tim, ccr_val);
  
  #ifdef DEBUG_SERVO
    DEBUG_PRINT("Set Angle: %u deg, pulse width: %f us \n", angle, pulse_length_us);
  #endif
}

void servo2SetAngle(uint8_t angle)
{
  // set CCR register
  // Duty% = CCR/ARR*100, so CCR = Duty%/100 * ARR

  double pulse_length_us = (double)(angle) / servo_range * (servo_MAX_us - servo_MIN_us) + servo_MIN_us;
  double pulse_length_s = pulse_length_us / 1000000;
  const uint32_t ccr_val = (uint32_t)(pulse_length_s * SERVO_PWM_PERIOD * SERVO_PWM_FREQUENCY_HZ);
  servo2Map->setCompare(servo2Map->tim, ccr_val);
  
  #ifdef DEBUG_SERVO
    DEBUG_PRINT("Set Angle: %u deg, pulse width: %f us \n", angle, pulse_length_us);
  #endif
}

uint8_t saturateAngle(uint8_t angle)
{
  if (angle > servo_range) {
    return servo_range;
  }
  else if (angle < 0) {
    return 0;
  }
  else {
    return angle;
  }

}

static const DeckDriver bicopter_deck = {
  .vid = 0x00,
  .pid = 0x00,
  .name = "bicopterDeck",

  .usedPeriph = DECK_USING_TIMER4 | DECK_USING_TIMER3,
  .usedGpio = DECK_USING_IO_1 | DECK_USING_IO_2,

  .init = servoInit,
  .test = servoTest,
};

DECK_DRIVER(bicopter_deck);


// PARAM_GROUP_START(deck)

// PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcServo, &isInit)
// PARAM_GROUP_STOP(deck)

// /**
//  * "Servo" deck parameters
//  */
// PARAM_GROUP_START(servo)

// /**
//  * @brief PWM pulse width for minimal servo position (in microseconds)
//  */
// PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, servoMINus, &servo_MIN_us)
// /**
//  * @brief PWM pulse width for maximal servo position (in microseconds)
//  */
// PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, servoMAXus, &servo_MAX_us)
// /**
//  * @brief Servo range, i.e. angle between the min and max positions (in degrees)
//  */
// PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, servoRange, &servo_range)
// /**
//  * @brief Servo idle (startup) angular position (in degrees, min = 0, max = servoRange)
//  */
// PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, servoIdle, &servo_idle)
// /**
//  * @brief Servo angular position (in degrees, min = 0, max = servoRange)
//  */
// PARAM_ADD_WITH_CALLBACK(PARAM_UINT8 , servoAngle, &s_servo_angle, &servoAngleCallBack)

// PARAM_GROUP_STOP(servo)
