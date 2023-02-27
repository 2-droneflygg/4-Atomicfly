#pragma once

#include <stdint.h>

#include "pg/pg.h"

/* ---------------------------速率类型枚举---------------------------- */	
typedef enum {
    RATES_TYPE_DFLIGHT = 0,								// 速率类型Dflight
} ratesType_e;

/* --------------------------油门限制类型枚举------------------------- */	
typedef enum {
    THROTTLE_LIMIT_TYPE_OFF = 0,						// 关闭
    THROTTLE_LIMIT_TYPE_SCALE,							// 缩放
    THROTTLE_LIMIT_TYPE_CLIP,							// 截断
    THROTTLE_LIMIT_TYPE_COUNT   						// must be the last entry
} throttleLimitType_e;

/* ----------------------------TPA模式枚举---------------------------- */	
typedef enum {
    TPA_MODE_PD,
    TPA_MODE_D
} tpaMode_e;

/* --------------------------控制速率配置结构体------------------------ */
#define MAX_RATE_PROFILE_NAME_LENGTH 8u  			    // 最大速率配置文件名称长度
typedef struct controlRateConfig_s {
    uint8_t thrMid8;									// 油门中点
    uint8_t thrExpo8;									// 油门固定曲线值
    uint8_t rates_type;									// 速率类型
    uint8_t rcRates[3];									// RC速率
    uint8_t rcExpo[3];									// 固定曲线值
    uint8_t rates[3];									// 曲线角速度 
    uint8_t dynThrPID;									// 动态油门PID
    uint16_t tpa_breakpoint;                			// TPA起始点
    uint8_t throttle_limit_type;            			// 设置油门限制类型
    uint8_t throttle_limit_percent;         			// 油门限制百分比
    uint16_t rate_limit[3];                 			// 设置轴的最大速率
    uint8_t tpaMode;                       			    // 控制PD项TPA的影响
    char profileName[MAX_RATE_PROFILE_NAME_LENGTH + 1]; // 速率配置文件的描述性名称
} controlRateConfig_t;
// 声明控制速率配置结构体
PG_DECLARE_ARRAY(controlRateConfig_t, CONTROL_RATE_PROFILE_COUNT, controlRateProfiles);

extern controlRateConfig_t *currentControlRateProfile;

void loadControlRateProfile(void);
void changeControlRateProfile(uint8_t controlRateProfileIndex);
void copyControlRateProfile(const uint8_t dstControlRateProfileIndex, const uint8_t srcControlRateProfileIndex);

