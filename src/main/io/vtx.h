#pragma once

#include <stdint.h>

#include "platform.h"
#include "common/time.h"
#include "pg/pg.h"

/* --------------------------vtx低功率解除枚举-------------------------- */	
typedef enum {
    VTX_LOW_POWER_DISARM_OFF = 0,
    VTX_LOW_POWER_DISARM_ALWAYS,
    VTX_LOW_POWER_DISARM_UNTIL_FIRST_ARM, // 设置低功率，直到第一次解锁
} vtxLowerPowerDisarm_e;

/* --------------------------vtx设置配置结构体-------------------------- */	
typedef struct vtxSettingsConfig_s {
    uint8_t band;           			  // 频组：1=A, 2=B, 3=E, 4=F(Airwaves/Fatshark), 5=Raceband
    uint8_t channel;        			  // 频点：1-8
    uint8_t power;          			  // 功率：0 = lowest
    uint16_t freq;          			  // 频带：为0，则设置“频率”为MHz
    uint16_t pitModeFreq;   			  // PIT模式
    uint8_t lowPowerDisarm; 			  // 低功率，直到第一次解锁
} vtxSettingsConfig_t;
// 声明vtx设置配置结构体
PG_DECLARE(vtxSettingsConfig_t, vtxSettingsConfig);

/* --------------------------vtx参数调度枚举-------------------------- */	
typedef enum {
    VTX_PARAM_POWER = 0,				  // 功率
    VTX_PARAM_BANDCHAN,					  // 频点
    VTX_PARAM_PITMODE,					  // PIT模式
    VTX_PARAM_CONFIRM,					  // SET
    VTX_PARAM_COUNT						  // 参数数量
} vtxScheduleParams_e;

void vtxInit(void);
void vtxUpdate(timeUs_t currentTimeUs);

