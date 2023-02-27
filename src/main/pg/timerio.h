#pragma once

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/dma_reqmap.h"
#include "drivers/io.h"

#ifdef USE_TIMER_MGMT

/* -----------------------------定时器引脚配置结构体--------------------------- */	
typedef struct timerIOConfig_s {
    ioTag_t ioTag;
    uint8_t index;
    int8_t dmaopt;
} timerIOConfig_t;
// 声明定时器引脚配置结构体
PG_DECLARE_ARRAY(timerIOConfig_t, MAX_TIMER_PINMAP_COUNT, timerIOConfig);
#endif

