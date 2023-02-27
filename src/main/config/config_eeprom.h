#pragma once

#include <stdint.h>
#include <stdbool.h>

// EEPROM 配置版本
#define EEPROM_CONF_VERSION 173

bool isEEPROMVersionValid(void);
bool isEEPROMStructureValid(void);
bool loadEEPROM(void);
void writeConfigToEEPROM(void);
uint16_t getEEPROMConfigSize(void);
size_t getEEPROMStorageSize(void);

