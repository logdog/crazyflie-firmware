/**  Microchip Technology MCP3425A0T-E/CH
*    16-Bit Analog-to-Digital Converter with I2C Interface and On-Board Reference
*    Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/22072b.pdf
*/

#define DEBUG_MODULE "MCP3425"

#include <math.h>
#include "stm32fxxx.h"
#include "FreeRTOS.h"
#include "task.h"

// TA: Maybe not so good to bring in these dependencies...
#include "debug.h"
#include "eprintf.h"
#include "i2cdev.h"

#include "mcp3425.h"

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static uint8_t buffer[4];
static bool isInit;


/** Default constructor, uses default I2C address.
 * @see MCP3425_DEFAULT_ADDRESS
 */
void mcp3425Init(I2C_Dev *i2cPort)
{
  if (isInit)
    return;

  I2Cx = i2cPort;
  devAddr = MCP3425_DEFAULT_ADDRESS;

  isInit = true;
}

// enable continuous conversion mode with 12-bit resolution
// this is the default behavior of the device
bool mcp3425EnableContinuous12Bit() {
    uint8_t data[1] = {0x90};
    return i2cdevWrite(I2Cx, devAddr, 1, data);
}

// read the device (12-bit with LSB = 1 mV, assume positive voltage)
bool mcp3425ReadVoltage(float *voltage) {

  uint8_t data[1] = {0x90};
  if (i2cdevRead(I2Cx, devAddr, 4, buffer)) {
    // DEBUG_PRINT("read voltage: buffer = %x %x %x %x\n", buffer[0], buffer[1], buffer[2], buffer[3]);
    uint16_t data = ((uint16_t) (0x0F & buffer[0]) << 8 ) | (uint16_t) buffer[1];
    *voltage = data / 1000.0f;
    return true;
  }
  
  return false;
}

// do a test read, see if we get the ACK from the i2c device
bool mcp3425Test(void)
{
  if (!isInit)
    return false;

  if (!i2cdevRead(I2Cx, devAddr, 4, buffer)) {
    return false;
  }

  return true;
}
