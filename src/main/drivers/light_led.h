#pragma once

#include "pg/pg.h"
#include "drivers/io_types.h"
#include "common/utils.h"

/* -----------------------------状态灯配置结构体------------------------------ */	
typedef struct statusLedConfig_s {
    ioTag_t ioTags;		// IO
    uint8_t inversion;  // 反转
} statusLedConfig_t;
// 声明状态灯配置结构体
PG_DECLARE(statusLedConfig_t, statusLedConfig);

// LED状态灯操作宏
#define LED0_TOGGLE              ledToggle(0)		// 反转
#define LED0_OFF                 ledSet(0, false)   // LED关闭
#define LED0_ON                  ledSet(0, true)	// LED开启

void ledInit(const statusLedConfig_t *statusLedConfig);
void ledToggle(int led);
void ledSet(int led, bool state);

