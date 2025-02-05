/**
 *  _   _                     _           _     
 * | | | |                   | |         | |    
 * | |_| |_   _ _   _ _ __   | |     __ _| |__  
 * |  _  | | | | | | | '_ \  | |    / _` | '_ \ 
 * | | | | |_| | |_| | | | | | |___| (_| | |_) |
 * \_| |_/\__, |\__,_|_| |_| \_____/\__,_|_.__/ 
 *         __/ |                                
 *        |___/                                 
 * 
 * @file platform_bicopter.c
 * @author Logan Dihel
 * @brief Platform information for bicopter
 * @details This file was modified from platform_bolt.c
 */


#define DEBUG_MODULE "PLATFORM"

#include <string.h>

#include "platform.h"
#include "exti.h"
#include "nvic.h"
#include "debug.h"

static platformConfig_t configs[] = {
#ifdef CONFIG_SENSORS_BMI088_SPI
  {
    .deviceType = "CB11",
    .deviceTypeName = "Bicopter Bolt 1.1",
    .sensorImplementation = SensorImplementation_bmi088_spi_bmp3xx,
    .physicalLayoutAntennasAreClose = false,
    .motorMap = motorMapBolt11Brushless,
  }
#endif
};

const platformConfig_t* platformGetListOfConfigurations(int* nrOfConfigs) {
  *nrOfConfigs = sizeof(configs) / sizeof(platformConfig_t);
  return configs;
}

void platformInitHardware() {
  //Low level init: Clock and Interrupt controller
  nvicInit();

  //EXTI interrupts
  extiInit();
}

// Config functions ------------------------

const char* platformConfigGetPlatformName() {
  return "bicopter";
}
