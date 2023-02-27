#pragma once

#include "pg/pg.h"

#ifndef DEFAULT_FEATURES
#define DEFAULT_FEATURES 0
#endif
#ifndef DEFAULT_RX_FEATURE
#define DEFAULT_RX_FEATURE FEATURE_RX_SERIAL    // 默认串行接收机
#endif

/* --------------------------特性枚举----------------------- */	
// 掩码形式
typedef enum {
    FEATURE_RX_SERIAL = 1 << 3,     			// 串行接收机
    FEATURE_SOFTSERIAL = 1 << 6,	 			// 软件串口
    FEATURE_GPS = 1 << 7,						// GPS
    FEATURE_OSD = 1 << 18,						// OSD
    FEATURE_AIRMODE = 1 << 22,					// 空中模式
    FEATURE_ANTI_GRAVITY = 1 << 28,				// 反重力
    FEATURE_DYNAMIC_FILTER = 1 << 29,			// 动态低通滤波
} features_e;

/* -------------------------特性结构体-----------------------*/	
typedef struct featureConfig_s {
    uint32_t enabledFeatures;					// 使能特性
} featureConfig_t;
PG_DECLARE(featureConfig_t, featureConfig);

void featureInit(void);
bool featureIsEnabled(const uint32_t mask);
bool featureIsConfigured(const uint32_t mask);
void featureEnableImmediate(const uint32_t mask);
void featureDisableImmediate(const uint32_t mask);

