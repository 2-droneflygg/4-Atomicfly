/*********************************************************************************
 提供电池监控相关API。
*********************************************************************************/
#include "stdbool.h"
#include "stdint.h"
#include "math.h"

#include "platform.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/adc.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"

#include "flight/mixer.h"

#include "io/beeper.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/battery.h"

// ---------------------------------------------------------电池监控
// 电芯数量
uint8_t batteryCellCount; 				
// 警告电压
uint16_t batteryWarningVoltage;				
// 最小电压
uint16_t batteryCriticalVoltage;			
// 低电压截止
static lowVoltageCutoff_t lowVoltageCutoff; 

// ---------------------------------------------------------访问
static currentMeter_t currentMeter;
static voltageMeter_t voltageMeter;
// ---------------------------------------------------------状态函数
static batteryState_e batteryState;
static batteryState_e voltageState;

// ---------------------------------------------------------相关宏定义
// 电流计源
#ifndef DEFAULT_CURRENT_METER_SOURCE
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_NONE
#endif
// 电压计源
#ifndef DEFAULT_VOLTAGE_METER_SOURCE
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_NONE
#endif
// VBAT稳定最大值
#define VBAT_STABLE_MAX_DELTA 20    	
// LVC缓慢启动需要10秒
#define LVC_AFFECT_TIME 10000000 			

PG_REGISTER_WITH_RESET_TEMPLATE(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 3);
PG_RESET_TEMPLATE(batteryConfig_t, batteryConfig,
    // voltage
    .vbatmaxcellvoltage = 430,
    .vbatmincellvoltage = 330,
    .vbatwarningcellvoltage = 350,
    .vbatnotpresentcellvoltage = 300, // A cell below 3 will be ignored
    .voltageMeterSource = DEFAULT_VOLTAGE_METER_SOURCE,
    .lvcPercentage = 100, 			  // 默认关闭为100%
    // current
    .batteryCapacity = 0,
    .currentMeterSource = DEFAULT_CURRENT_METER_SOURCE,
    // cells
    .forceBatteryCellCount = 0,       // 0 will be ignored
    // warnings / alerts
    .useVBatAlerts = true,
    .useConsumptionAlerts = false,
    .consumptionWarningPercentage = 10,
    .vbathysteresis = 1,
    .vbatfullcellvoltage = 410,
    .vbatDisplayLpfPeriod = 30,
    .ibatLpfPeriod = 10,
    .vbatDurationForWarning = 0,
    .vbatDurationForCritical = 0,
);

//-------------------------------------------------------------------------------------电池监控初始化相关API

/**********************************************************************
函数名称：batteryInit
函数功能：电池初始化
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void batteryInit(void)
{
    // 初始化电池存在状态
    batteryState = BATTERY_INIT;
    batteryCellCount = 0;

    // 电压计初始化
    voltageState = BATTERY_INIT;
    batteryWarningVoltage = 0;
    batteryCriticalVoltage = 0;
    lowVoltageCutoff.enabled = false;
    lowVoltageCutoff.percentage = 100;
    lowVoltageCutoff.startTime = 0;

	// 电压计复位
    voltageMeterReset(&voltageMeter);
	// 判断电压计源
    switch (batteryConfig()->voltageMeterSource) {
        case VOLTAGE_METER_ADC:
            voltageMeterADCInit();
            break;
        default:
            break;
    }

    // 电流计复位
    currentMeterReset(&currentMeter);
	// 判断电流计源
    switch (batteryConfig()->currentMeterSource) {
        case CURRENT_METER_ADC:
            currentMeterADCInit();
            break;
        default:
            break;
    }

}

//-------------------------------------------------------------------------------------电压计更新相关API

/**********************************************************************
函数名称：batteryUpdateVoltage
函数功能：更新电池电压
函数形参：currentTimeUs
函数返回值：None 
函数描述：
	由调度器以任务形式调用.
**********************************************************************/
void batteryUpdateVoltage(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    switch (batteryConfig()->voltageMeterSource) {
        case VOLTAGE_METER_ADC:
			// 电压计ADC刷新
            voltageMeterADCRefresh();
			// 电压计ADC读取
            voltageMeterADCRead(VOLTAGE_SENSOR_ADC_VBAT, &voltageMeter);
            break;
        default:
        case VOLTAGE_METER_NONE:
			// 电压计复位
            voltageMeterReset(&voltageMeter);
            break;
    }
}

/**********************************************************************
函数名称：updateBatteryBeeperAlert
函数功能：更新电池电压警报
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
static void updateBatteryBeeperAlert(void)
{
    switch (getBatteryState()) {
        case BATTERY_WARNING:
            beeper(BEEPER_BAT_LOW);
            break;
        case BATTERY_CRITICAL:
            beeper(BEEPER_BAT_CRIT_LOW);
            break;
        case BATTERY_OK:
        case BATTERY_NOT_PRESENT:
        case BATTERY_INIT:
            break;
    }
}

//-------------------------------------------------------------------------------------电流计更新相关API

/**********************************************************************
函数名称：batteryUpdateCurrentMeter
函数功能：更新电池电流
函数形参：currentTimeUs
函数返回值：None 
函数描述：
	由调度器以任务形式调用.
**********************************************************************/
void batteryUpdateCurrentMeter(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    if (batteryCellCount == 0) {
		// 电流计复位
        currentMeterReset(&currentMeter);
        return;
    }
    static uint32_t ibatLastServiced = 0;
    const int32_t lastUpdateAt = cmp32(currentTimeUs, ibatLastServiced);
    ibatLastServiced = currentTimeUs;

    switch (batteryConfig()->currentMeterSource) {
        case CURRENT_METER_ADC:
			// 电流计ADC刷新
            currentMeterADCRefresh(lastUpdateAt);
			// 电流计ADC读取
            currentMeterADCRead(&currentMeter);
            break;
        default:
        case CURRENT_METER_NONE:
			// 电流计复位
            currentMeterReset(&currentMeter);
            break;
    }
}

//-------------------------------------------------------------------------------------电压计状态监测相关API

/**********************************************************************
函数名称：isVoltageStable
函数功能：电压是否稳定
函数形参：None 
函数返回值：状态
函数描述：使所有这些与显示的电压滤波无关
**********************************************************************/
static bool isVoltageStable(void)
{
    return ABS(voltageMeter.displayFiltered - voltageMeter.unfiltered) <= VBAT_STABLE_MAX_DELTA;
}

/**********************************************************************
函数名称：isVoltageFromBat
函数功能：是否来自BAT电压
函数形参：None 
函数返回值：状态
函数描述：None 
**********************************************************************/
static bool isVoltageFromBat(void)
{
    // 检测到USB电压或1s禁用电池
    return (voltageMeter.displayFiltered >= batteryConfig()->vbatnotpresentcellvoltage    // Above ~0V
        && voltageMeter.displayFiltered <= batteryConfig()->vbatmaxcellvoltage)           // 1s max cell voltage check
        || voltageMeter.displayFiltered > batteryConfig()->vbatnotpresentcellvoltage * 2; // USB voltage - 2s or more check
}

/**********************************************************************
函数名称：batteryUpdatePresence
函数功能：更新电池存在状态
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void batteryUpdatePresence(void)
{
	// 电池刚刚连接 - 计算电池，警告电压和复位状态
    if ((voltageState == BATTERY_NOT_PRESENT || voltageState == BATTERY_INIT) && isVoltageFromBat() && isVoltageStable()) {
        voltageState = BATTERY_OK;
		// 判断eeprom是否有电芯数据
        if (batteryConfig()->forceBatteryCellCount != 0) {
            batteryCellCount = batteryConfig()->forceBatteryCellCount;
        } else {
        	// 计算电池电芯数
            unsigned cells = (voltageMeter.displayFiltered / batteryConfig()->vbatmaxcellvoltage) + 1;
			// 最大8s
            if (cells > MAX_AUTO_DETECT_CELL_COUNT) {
                cells = MAX_AUTO_DETECT_CELL_COUNT;
            }
            batteryCellCount = cells;
            if (!ARMING_FLAG(ARMED)) {
                changePidProfileFromCellCount(batteryCellCount);
            }
        }
		// 更新警告电压
        batteryWarningVoltage = batteryCellCount * batteryConfig()->vbatwarningcellvoltage;
		// 更新最小电压
        batteryCriticalVoltage = batteryCellCount * batteryConfig()->vbatmincellvoltage;
		// 低电压截止百分比
        lowVoltageCutoff.percentage = 100;
		// 低电压起始时间
        lowVoltageCutoff.startTime = 0;
    } 
	// 电池已断开 -      电容放电可能需要一段时间
	else if (voltageState != BATTERY_NOT_PRESENT && isVoltageStable() && !isVoltageFromBat()) {
        voltageState = BATTERY_NOT_PRESENT;
        batteryCellCount = 0;
        batteryWarningVoltage = 0;
        batteryCriticalVoltage = 0;
    }
}

/**********************************************************************
函数名称：batteryUpdateVoltageState
函数功能：更新电池电压状态
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
static void batteryUpdateVoltageState(void)
{
    // 警报目前被BEEPER、OSD和其他子系统使用
    static uint32_t lastVoltageChangeMs;
    switch (voltageState) {
        case BATTERY_OK:
			// 电压小于警告迟滞电压
            if (voltageMeter.displayFiltered <= (batteryWarningVoltage - batteryConfig()->vbathysteresis)) {
				// 跳变时差 >= 持续时间临界点
                if (cmp32(millis(), lastVoltageChangeMs) >= batteryConfig()->vbatDurationForWarning * 100) {
                    voltageState = BATTERY_WARNING;
                }
            } else {
            	// 更新电压更改时间
                lastVoltageChangeMs = millis();
            }
            break;

        case BATTERY_WARNING:
			// 电压小于临界迟滞电压
            if (voltageMeter.displayFiltered <= (batteryCriticalVoltage - batteryConfig()->vbathysteresis)) {
				// 跳变时差 >= 持续时间临界点
                if (cmp32(millis(), lastVoltageChangeMs) >= batteryConfig()->vbatDurationForCritical * 100) {
                    voltageState = BATTERY_CRITICAL;
                }
            } else {
            	// 由警告 -> 正常电压
                if (voltageMeter.displayFiltered > batteryWarningVoltage) {
                    voltageState = BATTERY_OK;
                }
				// 更新电压更改时间
                lastVoltageChangeMs = millis();
            }
            break;

        case BATTERY_CRITICAL:
			// 由临界 -> 警告电压
            if (voltageMeter.displayFiltered > batteryCriticalVoltage) {
                voltageState = BATTERY_WARNING;
				// 更新电压更改时间
                lastVoltageChangeMs = millis();
            }
            break;

        default:
            break;
    }

}

/**********************************************************************
函数名称：batteryUpdateStates
函数功能：更新电池状态
函数形参：currentTimeUs
函数返回值：None 
函数描述：None 
**********************************************************************/
void batteryUpdateStates(timeUs_t currentTimeUs)
{
	// 更新电池电压状态
    batteryUpdateVoltageState();
	// 更新电池状态
    batteryState = voltageState;
}

/**********************************************************************
函数名称：batteryUpdateAlarms
函数功能：更新电池警报
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void batteryUpdateAlarms(void)
{
    if (batteryConfig()->useVBatAlerts) {
        updateBatteryBeeperAlert();
    }
}

//-------------------------------------------------------------------------------------电池信息获取相关API

/**********************************************************************
函数名称：getBatteryState
函数功能：获取电池状态
函数形参：None 
函数返回值：batteryState
函数描述：None 
**********************************************************************/
batteryState_e getBatteryState(void)
{
    return batteryState;
}

/**********************************************************************
函数名称：getBatteryVoltage
函数功能：获取电池电压
函数形参：None 
函数返回值：电池电压
函数描述：None 
**********************************************************************/
uint16_t getBatteryVoltage(void)
{
    return voltageMeter.displayFiltered;
}

/**********************************************************************
函数名称：getBatteryAverageCellVoltage
函数功能：获取电池平均电压
函数形参：None 
函数返回值：电池平均电压
函数描述：None 
**********************************************************************/
uint16_t getBatteryAverageCellVoltage(void)
{
    return voltageMeter.displayFiltered / batteryCellCount;
}

/**********************************************************************
函数名称：getLowVoltageCutoff
函数功能：获取低电压截止电压
函数形参：None 
函数返回值：lowVoltageCutoff
函数描述：None 
**********************************************************************/
const lowVoltageCutoff_t *getLowVoltageCutoff(void)
{
    return &lowVoltageCutoff;
}

/**********************************************************************
函数名称：getAmperage
函数功能：获取安培数
函数形参：None 
函数返回值：安培数
函数描述：None 
**********************************************************************/
int32_t getAmperage(void) {
    return currentMeter.amperage;
}

