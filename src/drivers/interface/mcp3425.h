#ifndef __MCP3425_H__
#define __MCP3425_H__

#include "i2cdev.h"

#define MCP3425_DEFAULT_ADDRESS     0x68    // default address for mcp3425: 0110 1000

/** Default constructor, uses default I2C address.
 * @see MCP3425_DEFAULT_ADDRESS
 */
void mcp3425Init(I2C_Dev *i2cPort);

// enable continuous conversion mode with 12-bit resolution
// this is the default behavior of the device
bool mcp3425EnableContinuous12Bit();

// read the device
bool mcp3425ReadVoltage(float *voltage);

// do a test read, and verify the configuration register
// has the correct value stored in it
bool mcp3425Test(void);

#endif // __MCP3425_H__