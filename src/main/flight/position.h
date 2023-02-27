#pragma once

#include "common/time.h"

/* --------------------------位置配置结构体-------------------------- */	
typedef struct positionConfig_s {
    uint8_t altSource;
} positionConfig_t;
// 声明位置配置结构体
PG_DECLARE(positionConfig_t, positionConfig);

bool isAltitudeOffset(void);
void calculateEstimatedAltitude(timeUs_t currentTimeUs);
int32_t getEstimatedAltitudeCm(void);
int16_t getEstimatedVario(void);

