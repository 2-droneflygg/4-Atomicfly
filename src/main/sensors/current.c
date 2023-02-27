/*********************************************************************************
 提供电流计相关API。
*********************************************************************************/
#include "stdbool.h"
#include "stdint.h"
#include "string.h"

#include "platform.h"

#include "build/build_config.h"

#include "common/maths.h"
#include "common/utils.h"
#include "common/filter.h"

#include "drivers/adc.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/battery.h"

#include "current.h"

// ---------------------------------------------------------电流计访问
currentMeterADCState_t currentMeterADCState;

// ---------------------------------------------------------一阶巴特沃斯低通滤波器
static pt1Filter_t adciBatFilter;

// ---------------------------------------------------------电流计源名称
const char * const currentMeterSourceNames[CURRENT_METER_COUNT] = {
    "NONE", "ADC"
};
// ---------------------------------------------------------电流计ID
const uint8_t currentMeterIds[] = {
    CURRENT_METER_ID_BATTERY_1,
};
// ---------------------------------------------------------电流计支持数量
const uint8_t supportedCurrentMeterCount = ARRAYLEN(currentMeterIds);

// ---------------------------------------------------------相关宏定义
// 电流计缩放比例 - for Allegro ACS758LCB-100U (40mV/A)
#ifndef CURRENT_METER_SCALE_DEFAULT
#define CURRENT_METER_SCALE_DEFAULT 400   
#endif
// 电流计默认偏移
#ifndef CURRENT_METER_OFFSET_DEFAULT
#define CURRENT_METER_OFFSET_DEFAULT 0
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(currentSensorADCConfig_t, currentSensorADCConfig, PG_CURRENT_SENSOR_ADC_CONFIG, 0);
PG_RESET_TEMPLATE(currentSensorADCConfig_t, currentSensorADCConfig,
    .scale = CURRENT_METER_SCALE_DEFAULT,
    .offset = CURRENT_METER_OFFSET_DEFAULT,
);

//-------------------------------------------------------------------------------------电流计复位相关API

/**********************************************************************
函数名称：currentMeterADCToCentiamps
函数功能：电流计复位
函数形参：meter
函数返回值：None 
函数描述：None 
**********************************************************************/
void currentMeterReset(currentMeter_t *meter)
{
    meter->amperage = 0;
    meter->amperageLatest = 0;
}

//-------------------------------------------------------------------------------------电流计初始化相关API

/**********************************************************************
函数名称：currentMeterADCInit
函数功能：电流计ADC初始化
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void currentMeterADCInit(void)
{
    memset(&currentMeterADCState, 0, sizeof(currentMeterADCState_t));
    pt1FilterInit(&adciBatFilter, pt1FilterGain(GET_BATTERY_LPF_FREQUENCY(batteryConfig()->ibatLpfPeriod), HZ_TO_INTERVAL(50)));
}

//-------------------------------------------------------------------------------------电流计转换相关API

/**********************************************************************
函数名称：currentMeterADCToCentiamps
函数功能：电流计ADC转centiAmps
函数形参：src
函数返回值：centiAmps
函数描述：None 
**********************************************************************/
static int32_t currentMeterADCToCentiamps(const uint16_t src)
{
    const currentSensorADCConfig_t *config = currentSensorADCConfig();
	// 3.3V = ADC Vref（3300）
    int32_t millivolts = ((uint32_t)src * 3300) / 4096;
    // y=x/m+b m is scale in (mV/10A) and b is offset in (mA)
    int32_t centiAmps = (millivolts * 10000 / (int32_t)config->scale + (int32_t)config->offset) / 10;
    return centiAmps;
}

//-------------------------------------------------------------------------------------电流计刷新相关API

/**********************************************************************
函数名称：updateCurrentmAhDrawnState
函数功能：更新电流计mAh状态
函数形参：state，amperageLatest，lastUpdateAt
函数返回值：None 
函数描述：None 
**********************************************************************/
#if defined(USE_ADC) 
static void updateCurrentmAhDrawnState(currentMeterMAhDrawnState_t *state, int32_t amperageLatest, int32_t lastUpdateAt)
{
    state->mAhDrawnF = state->mAhDrawnF + (amperageLatest * lastUpdateAt / (100.0f * 1000 * 3600));
    state->mAhDrawn = state->mAhDrawnF;
}
#endif

/**********************************************************************
函数名称：currentMeterADCRefresh
函数功能：电流计ADC刷新
函数形参：lastUpdateAt
函数返回值：None 
函数描述：None 
**********************************************************************/
void currentMeterADCRefresh(int32_t lastUpdateAt)
{
#ifdef USE_ADC
	// 获取通道数据
    const uint16_t iBatSample = adcGetChannel(ADC_CURRENT);
	// 未滤波数据 - 转换
    currentMeterADCState.amperageLatest = currentMeterADCToCentiamps(iBatSample);
	// 滤波数据
    currentMeterADCState.amperage = currentMeterADCToCentiamps(pt1FilterApply(&adciBatFilter, iBatSample));
	// 更新电流计mAh状态
    updateCurrentmAhDrawnState(&currentMeterADCState.mahDrawnState, currentMeterADCState.amperageLatest, lastUpdateAt);
#endif
}

//-------------------------------------------------------------------------------------电流计读取相关API

/**********************************************************************
函数名称：currentMeterADCRead
函数功能：电流计ADC读取
函数形参：meter
函数返回值：None 
函数描述：None 
**********************************************************************/
void currentMeterADCRead(currentMeter_t *meter)
{
	// 未滤波数据
    meter->amperageLatest = currentMeterADCState.amperageLatest;
	// 滤波数据
    meter->amperage = currentMeterADCState.amperage;
}

/**********************************************************************
函数名称：currentMeterRead
函数功能：电流计读取
函数形参：id，meter
函数返回值：None
函数描述：None 
**********************************************************************/
void currentMeterRead(currentMeterId_e id, currentMeter_t *meter)
{
    if (id == CURRENT_METER_ID_BATTERY_1) {
		// 电流计ADC读取
        currentMeterADCRead(meter);
    }
    else {
		// 电流计复位
        currentMeterReset(meter);
    }
}

