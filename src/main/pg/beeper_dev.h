#pragma once

#include "drivers/io_types.h"

#include "pg/pg.h"
#include "drivers/io_types.h"

/* --------------------------蜂鸣器设备配置结构体-------------------------- */	
typedef struct beeperDevConfig_s {
    ioTag_t ioTag;				// 蜂鸣器引脚 
    uint8_t isInverted;			// 引脚开漏输出
    uint8_t isOpenDrain;		// 引脚推挽输出
    uint16_t frequency;			// 频率
} beeperDevConfig_t;
// 声明蜂鸣器设备配置结构体
PG_DECLARE(beeperDevConfig_t, beeperDevConfig);

