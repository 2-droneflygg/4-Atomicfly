#pragma once

#include "drivers/io_types.h"
#include "pg/pg.h"

// clockConfig值
#define MAX7456_CLOCK_CONFIG_HALF 0  // Force half clock
#define MAX7456_CLOCK_CONFIG_OC   1  // Half clock if OC
#define MAX7456_CLOCK_CONFIG_FULL 2  // Force full clock

/* --------------------------MAX7456配置结构体-------------------------- */	
typedef struct max7456Config_s {
    uint8_t clockConfig; // 基于设备类型和超时钟状态的SPI时钟(MAX7456_CLOCK_CONFIG_xxxx)
    ioTag_t csTag;
    uint8_t spiDevice;
    bool preInitOPU;
} max7456Config_t;
// 声明MAX7456配置结构体
PG_DECLARE(max7456Config_t, max7456Config);

