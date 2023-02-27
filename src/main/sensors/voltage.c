/*********************************************************************************
 提供电压计相关API。
*********************************************************************************/
#include "stdbool.h"
#include "stdint.h"
#include "string.h"

#include "platform.h"

#include "build/build_config.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/adc.h"

#include "flight/pid.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/battery.h"

#include "voltage.h"

// ---------------------------------------------------------电压计访问
voltageMeterADCState_t voltageMeterADCStates[MAX_VOLTAGE_SENSOR_ADC];

// ---------------------------------------------------------电压计源名称
const char * const voltageMeterSourceNames[VOLTAGE_METER_COUNT] = {
    "NONE", "ADC"
};

// ---------------------------------------------------------电压计ADC通道
static const uint8_t voltageMeterAdcChannelMap[] = {
    ADC_BATTERY
};

// ---------------------------------------------------------相关宏定义
// 电压计默认比例
#ifndef VBAT_SCALE_DEFAULT
#define VBAT_SCALE_DEFAULT 110
#endif
// 电阻分频器R2(默认NAZE 10(K))
#ifndef VBAT_RESDIVVAL_DEFAULT
#define VBAT_RESDIVVAL_DEFAULT 10
#endif
// 比例乘法器(例如2.5:1的比例乘法器为4时，可以使用“100”代替比例中的“25”)来获得更好的精度
#ifndef VBAT_RESDIVMULTIPLIER_DEFAULT
#define VBAT_RESDIVMULTIPLIER_DEFAULT 1
#endif

PG_REGISTER_ARRAY_WITH_RESET_FN(voltageSensorADCConfig_t, MAX_VOLTAGE_SENSOR_ADC, voltageSensorADCConfig, PG_VOLTAGE_SENSOR_ADC_CONFIG, 0);
void pgResetFn_voltageSensorADCConfig(voltageSensorADCConfig_t *instance)
{
    for (int i = 0; i < MAX_VOLTAGE_SENSOR_ADC; i++) {
        RESET_CONFIG(voltageSensorADCConfig_t, &instance[i],
            .vbatscale = VBAT_SCALE_DEFAULT,
            .vbatresdivval = VBAT_RESDIVVAL_DEFAULT,
            .vbatresdivmultiplier = VBAT_RESDIVMULTIPLIER_DEFAULT,
        );
    }
}

//-------------------------------------------------------------------------------------电压计复位相关API

/**********************************************************************
函数名称：voltageMeterReset
函数功能：电压计复位
函数形参：meter
函数返回值：None 
函数描述：None 
**********************************************************************/
void voltageMeterReset(voltageMeter_t *meter)
{
    meter->displayFiltered = 0;
    meter->unfiltered = 0;
}

//-------------------------------------------------------------------------------------电压计初始化相关API

/**********************************************************************
函数名称：voltageMeterADCInit
函数功能：电压计ADC初始化 
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void voltageMeterADCInit(void)
{
    for (uint8_t i = 0; i < MAX_VOLTAGE_SENSOR_ADC && i < ARRAYLEN(voltageMeterAdcChannelMap); i++) { 
        voltageMeterADCState_t *state = &voltageMeterADCStates[i]; 
        memset(state, 0, sizeof(voltageMeterADCState_t)); 
        pt1FilterInit(&state->displayFilter, pt1FilterGain(GET_BATTERY_LPF_FREQUENCY(batteryConfig()->vbatDisplayLpfPeriod), HZ_TO_INTERVAL(SLOW_VOLTAGE_TASK_FREQ_HZ)));
    }
}

//-------------------------------------------------------------------------------------电压计转换相关API

/**********************************************************************
函数名称：voltageAdcToVoltage
函数功能：ADC电压模数转换
函数形参：src,config
函数返回值：result
函数描述：None 
**********************************************************************/
STATIC_UNIT_TESTED uint16_t voltageAdcToVoltage(const uint16_t src, const voltageSensorADCConfig_t *config)
{
	// 根据ADC读数计算电池电压
	// 结果是Vbatt在0.01V步长。3.3V = ADC Vref（3300）, 0xFFF = 12位ADC, 110 = 10:1分压器(10k:1k) * 100 for 0.01V
    return ((((uint32_t)src * config->vbatscale * 3300 / 10 + (0xFFF * 5)) / (0xFFF * config->vbatresdivval)) / config->vbatresdivmultiplier);
}

//-------------------------------------------------------------------------------------电压计刷新相关API

/**********************************************************************
函数名称：voltageMeterADCRefresh 
函数功能：电压计ADC刷新
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void voltageMeterADCRefresh(void)
{
	// 遍历所有电压计通道
    for (uint8_t i = 0; i < MAX_VOLTAGE_SENSOR_ADC && i < ARRAYLEN(voltageMeterAdcChannelMap); i++) {
        voltageMeterADCState_t *state = &voltageMeterADCStates[i];
#ifdef USE_ADC
        // 获取电压传感器ADC配置
        const voltageSensorADCConfig_t *config = voltageSensorADCConfig(i);

		// 获取通道
        uint8_t channel = voltageMeterAdcChannelMap[i];
		// 获取通道原始数据
        uint16_t rawSample = adcGetChannel(channel);
		// 进行显示低通滤波
        uint16_t filteredDisplaySample = pt1FilterApply(&state->displayFilter, rawSample);

        // ADC电压模数转换 - 显示滤波
        state->voltageDisplayFiltered = voltageAdcToVoltage(filteredDisplaySample, config);
		// ADC电压模数转换 - 无滤波
        state->voltageUnfiltered = voltageAdcToVoltage(rawSample, config);
#else
        UNUSED(voltageAdcToVoltage);
        state->voltageDisplayFiltered = 0;
        state->voltageUnfiltered = 0;
#endif
    }
}

/**********************************************************************
函数名称：voltageMeterADCRead
函数功能：电压计ADC读取
函数形参：adcChannel，voltageMeter 
函数返回值：None 
函数描述：None 
**********************************************************************/
void voltageMeterADCRead(voltageSensorADC_e adcChannel, voltageMeter_t *voltageMeter)
{
    voltageMeterADCState_t *state = &voltageMeterADCStates[adcChannel];
    voltageMeter->displayFiltered = state->voltageDisplayFiltered;
    voltageMeter->unfiltered = state->voltageUnfiltered;
}

/**********************************************************************
函数名称：voltageMeterRead
函数功能：电压计读取
函数形参：id，meter
函数返回值：None 
函数描述：None 
**********************************************************************/
void voltageMeterRead(voltageMeterId_e id, voltageMeter_t *meter)
{
    if (id == VOLTAGE_METER_ID_BATTERY_1) {
        voltageMeterADCRead(VOLTAGE_SENSOR_ADC_VBAT, meter);
    } else {
        voltageMeterReset(meter);
    }
}

