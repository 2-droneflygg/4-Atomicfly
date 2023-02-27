#pragma once

#include "pg/pg.h"

#include "common/filter.h"
#include "common/time.h"
#include "sensors/current.h"
#include "sensors/voltage.h"

// TODO:使'cell full'电压用户可调
#define CELL_VOLTAGE_FULL_CV 420

#define VBAT_CELL_VOTAGE_RANGE_MIN 100       // 电池电压最小范围 
#define VBAT_CELL_VOTAGE_RANGE_MAX 500       // 电池电压最大范围

#define MAX_AUTO_DETECT_CELL_COUNT 8         // 最大检测电芯串数 - 8s

#define GET_BATTERY_LPF_FREQUENCY(period) (1 / (period / 10.0f)) // 获取电池低通滤波频率

enum {
    AUTO_PROFILE_CELL_COUNT_STAY = 0,        // 不管检测到电芯数数量如何，保持在这个配置文件上。如果没有其他配置文件匹配，则使用此配置文件(默认情况下，即自动配置文件切换关闭)
    AUTO_PROFILE_CELL_COUNT_CHANGE = -1,     // 如果存在匹配的单元格数，则始终切换到匹配的配置文件
};

/* --------------------------电池配置结构体-------------------------- */	
typedef struct batteryConfig_s {
    // 电压
    uint16_t vbatmaxcellvoltage;             // 单片电芯最大电压，自动检测0.01V单位的电池电压，默认为430 (4.30V)
    uint16_t vbatmincellvoltage;             // 单片电芯最小电压触发电池紧急报警，以0.01V为单位，默认为330 (3.30V)
    uint16_t vbatwarningcellvoltage;         // 单片电芯警告电压，触发电池警告报警，以0.01V为单位，默认为350 (3.50V)
    uint16_t vbatnotpresentcellvoltage;      // USB供电
    uint8_t lvcPercentage;                   // 触发lvc时的油门百分比
    voltageMeterSource_e voltageMeterSource; // 电压计源

    // 电流
    currentMeterSource_e currentMeterSource; // 电流计的来源，ADC，虚拟或ESC
    uint16_t batteryCapacity;                // 电池容量：mAh

    // 警报/警告
    bool useVBatAlerts;                      // 根据VBat读数发出警报
    bool useConsumptionAlerts;               // 根据总功耗发出警报
    uint8_t consumptionWarningPercentage;    // 应该触发电池警告的剩余电量百分比
    uint8_t vbathysteresis;                  // 警告迟滞量，默认1 = 0.1V

    uint16_t vbatfullcellvoltage;            // 电池被认为“满”时的电池电压为0.01V单位，默认为410 (4.1V)

    uint8_t forceBatteryCellCount;           // 电池中的电池单元数，如果有问题，用于覆盖自动检测的电池单元数。
    uint8_t vbatDisplayLpfPeriod;            // 电池电压显示低通滤波截止频率周期(0.1 s)
    uint8_t ibatLpfPeriod;                   // Ibat滤波器截止频率的周期(0.1 s)
    uint8_t vbatDurationForWarning;          // 周期电压必须在电池状态设置为BATTERY_WARNING之前维持(0.1秒)
    uint8_t vbatDurationForCritical;         // 周期电压必须在电池状态设置为BATTERY_CRIT之前维持(0.1秒内)
} batteryConfig_t;
// 声明电池配置结构体
PG_DECLARE(batteryConfig_t, batteryConfig);

/* -------------------------低电压截止结构体------------------------- */	
typedef struct lowVoltageCutoff_s {
    bool enabled;
    uint8_t percentage;
    timeUs_t startTime;
} lowVoltageCutoff_t;

/* --------------------------电池状态枚举---------------------------- */	
typedef enum {
    BATTERY_OK = 0,						     // OK		
    BATTERY_WARNING,						 // 警告
    BATTERY_CRITICAL,						 // 临界
    BATTERY_NOT_PRESENT,					 // 不存在
    BATTERY_INIT							 // 初始化
} batteryState_e;

void batteryInit(void);
void batteryUpdateVoltage(timeUs_t currentTimeUs);
void batteryUpdatePresence(void);
batteryState_e getBatteryState(void);
void batteryUpdateStates(timeUs_t currentTimeUs);
void batteryUpdateAlarms(void);
struct rxConfig_s;
uint16_t getBatteryVoltage(void);
uint16_t getBatteryAverageCellVoltage(void);
int32_t getAmperage(void);
void batteryUpdateCurrentMeter(timeUs_t currentTimeUs);
const lowVoltageCutoff_t *getLowVoltageCutoff(void);

