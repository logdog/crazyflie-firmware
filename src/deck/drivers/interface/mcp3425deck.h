#ifndef __MCP3425DECK_H__
#define __MCP3425DECK_H__

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "deck_core.h"

void mcp3425DeckInit();
void mcp3425DeckTask(void* prm);
bool mcp3425DeckTest();

bool readVoltage();

#endif /* __MCP3425DECK_H__ */
