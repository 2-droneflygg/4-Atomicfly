#pragma once

#include "common/time.h"

#include "pg/motor.h"

#define DSHOT_MIN_THROTTLE       48			// DSHOT最小油门
#define DSHOT_MAX_THROTTLE     2047     	// DSHOT最大油门

/* --------------------------DSHOT协议结构体-------------------------- */	
typedef struct dshotProtocolControl_s {
    uint16_t value;							// 油门数据	
    bool requestTelemetry;					// 请求遥测位
} dshotProtocolControl_t;

void dshotInitEndpoints(const motorConfig_t *motorConfig, float outputLimit, float *outputLow, float *outputHigh, float *disarm);
float dshotConvertFromExternal(uint16_t externalValue);
uint16_t dshotConvertToExternal(float motorValue);
uint16_t prepareDshotPacket(dshotProtocolControl_t *pcb); 

