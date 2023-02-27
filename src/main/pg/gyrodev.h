#pragma once

#include <stdint.h>

#include "pg/pg.h"
#include "drivers/io_types.h"

/* ---------------------------陀螺仪设备配置结构体---------------------------- */	
typedef struct gyroDeviceConfig_s {
    int8_t index;
    uint8_t bustype;
    uint8_t spiBus;
    ioTag_t csnTag;
    uint8_t i2cBus;
    uint8_t i2cAddress;
    ioTag_t extiTag;
    uint8_t alignment;        			// 板对齐
} gyroDeviceConfig_t;
// 声明陀螺仪设备配置结构体
PG_DECLARE_ARRAY(gyroDeviceConfig_t, MAX_GYRODEV_COUNT, gyroDeviceConfig);

