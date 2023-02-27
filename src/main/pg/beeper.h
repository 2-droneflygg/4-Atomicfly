#pragma once

#include "pg/pg.h"
#include "drivers/io_types.h"

/* --------------------------蜂鸣器配置结构体-------------------------- */	
typedef struct beeperConfig_s {
    uint32_t beeper_off_flags;     // 蜂鸣器关闭标志位
    uint8_t dshotBeaconTone;       // Dshot信标音调
    uint32_t dshotBeaconOffFlags;  // Dshot信标关闭标志位
} beeperConfig_t;
// 声明蜂鸣器配置结构体
PG_DECLARE(beeperConfig_t, beeperConfig);

