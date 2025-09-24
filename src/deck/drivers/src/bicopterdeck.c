#define DEBUG_MODULE "BICOPTERDECK"

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

#include "bicopterdeck.h"
// #include "deck_digital.h"

// Simple variables - no DMA buffers needed
#define BUFFER_SIZE 8  // Array size for averaging

// Arrays to store recent readings
static volatile uint16_t pa6_buffer[BUFFER_SIZE];
static volatile uint16_t pa7_buffer[BUFFER_SIZE];
static volatile uint8_t buffer_index = 0;

// Averaged output values  
static volatile uint16_t pa6_raw = 0;
static volatile uint16_t pa7_raw = 0;

static float leftAngle = 0.0f;
static float rightAngle = 0.0f;

bool init = false;

void bicopterDeckInit()
{
  DEBUG_PRINT("bicopterDeckInit() - Timer approach\n");
  
  // Initialize arrays with zeros
  for (int i = 0; i < BUFFER_SIZE; i++) {
    pa6_buffer[i] = 0;
    pa7_buffer[i] = 0;
  }
  
  setupGPIO();
  setupADC();
  setupTimerInterrupt();
  
  DEBUG_PRINT("Timer ADC Init [OK]\n");
  init = true;
}

void setupGPIO() {
  // Configure PA6 and PA7 as analog inputs
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void setupADC() {
  // Enable ADC2 clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
  
  // Configure ADC for single conversions
  ADC_InitTypeDef ADC_InitStructure;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;             // Single channel at a time
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;       // Single conversion
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;                // One channel at a time
  ADC_Init(ADC2, &ADC_InitStructure);
  
  // Enable ADC
  ADC_Cmd(ADC2, ENABLE);
}

void setupTimerInterrupt() {
  // Use Timer8 instead of Timer4
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE); // Note: TIM8 is on APB2

  // Debug: Check actual clock frequencies
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  
  DEBUG_PRINT("SYSCLK: %lu Hz\n", RCC_Clocks.SYSCLK_Frequency);
  DEBUG_PRINT("APB2: %lu Hz\n", RCC_Clocks.PCLK2_Frequency);  
  
  
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  // 168MHz (APB2) / 1kHz = 168000
  TIM_TimeBaseStructure.TIM_Period = 16799;  // 0 to 16799 = 16800 counts = 10kHz
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
  
  // Enable timer interrupt
  TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);


  // Configure NVIC for timer interrupt
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn; // TIM8 Update interrupt
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; // Lower priority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  // Start the timer
  TIM_Cmd(TIM8, ENABLE);
}

// Timer interrupt handler - change from TIM4 to TIM8
static bool led_state = false;
void TIM8_UP_TIM13_IRQHandler(void) {
  if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
    
    uint16_t pa6_reading, pa7_reading;
    
    // Read PA6
    ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 1, ADC_SampleTime_15Cycles);
    ADC_SoftwareStartConv(ADC2);
    while (!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC));  // Wait for conversion
    pa6_reading = ADC_GetConversionValue(ADC2);
    
    // Read PA7
    ADC_RegularChannelConfig(ADC2, ADC_Channel_7, 1, ADC_SampleTime_15Cycles);
    ADC_SoftwareStartConv(ADC2);
    while (!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC));  // Wait for conversion
    pa7_reading = ADC_GetConversionValue(ADC2);
    
    // Store in circular buffers
    pa6_buffer[buffer_index] = pa6_reading;
    pa7_buffer[buffer_index] = pa7_reading;

    digitalWrite(DECK_GPIO_IO3, led_state);
    led_state = !led_state;
    
    // Update buffer index (circular)
    buffer_index = (buffer_index + 1) % BUFFER_SIZE;
    
    calculateAverages();
  }
}

// Calculate moving averages from the arrays
void calculateAverages(void) {
  uint32_t pa6_sum = 0;
  uint32_t pa7_sum = 0;
  
  // Sum all values in the arrays
  for (int i = 0; i < BUFFER_SIZE; i++) {
    pa6_sum += pa6_buffer[i];
    pa7_sum += pa7_buffer[i];
  }
  
  pa6_raw = (uint16_t)(pa6_sum >> 3);
  pa7_raw = (uint16_t)(pa7_sum >> 3);

  leftAngle = 0.04583f * pa6_raw - 102.521;
  rightAngle = -0.04583f * pa7_raw + 102.521;
}

// Optional: Get current values
uint16_t bicopterDeckGetPA6(void) {
  return pa6_raw;
}

uint16_t bicopterDeckGetPA7(void) {
  return pa7_raw;
}

bool bicopterDeckTest() {
  return init;
}

// Deck driver registration
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

// Logging
LOG_GROUP_START(bideck)
LOG_ADD(LOG_UINT16, pa6_raw, &pa6_raw)
LOG_ADD(LOG_UINT16, pa7_raw, &pa7_raw)
LOG_ADD(LOG_FLOAT, leftAngle, &leftAngle)
LOG_ADD(LOG_FLOAT, rightAngle, &rightAngle)
LOG_GROUP_STOP(bideck)