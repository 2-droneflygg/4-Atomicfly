#pragma once

#include "common/time.h"
#include "current_ids.h"


/* --------------------------电流计源枚举----------------------------- */	
typedef enum {
    CURRENT_METER_NONE = 0,
    CURRENT_METER_ADC,
    CURRENT_METER_COUNT
} currentMeterSource_e;
extern const char * const currentMeterSourceNames[CURRENT_METER_COUNT];

/* ------------------------电流计传感器枚举--------------------------- */	
typedef enum {
    CURRENT_SENSOR_VIRTUAL = 0,
    CURRENT_SENSOR_ADC
} currentSensor_e;

/* --------------------------电流计结构体----------------------------- */	
typedef struct currentMeter_s {
    int32_t amperage;           			// 电流传感器读出的电流，单位为厘安(1/100 A)
    int32_t amperageLatest;     			// 电流传感器读出的电流，单位为厘安培(1/100 A)(未滤波)
} currentMeter_t;

/* ----------------------电流计毫安/时状态结构体---------------------- */	
// WARNING - 不要混合使用CURRENT_SENSOR_*和CURRENT_METER_*，它们是独立的关注点
typedef struct currentMeterMAhDrawnState_s {
    int32_t mAhDrawn;           			// 电池从启动起就消耗了毫安小时
    float mAhDrawnF;
} currentMeterMAhDrawnState_t;

/* ------------------------电流计ADC状态结构体------------------------ */	
typedef struct currentMeterADCState_s {
    currentMeterMAhDrawnState_t mahDrawnState;
    int32_t amperage;           			// 电流传感器读出的电流，单位为厘安(1/100 A)
    int32_t amperageLatest;     			// 电流传感器读出的电流，单位为厘安培(1/100 A)(未滤波)
} currentMeterADCState_t;

/* ------------------------电流计ADC配置结构体------------------------ */	
typedef struct currentSensorADCConfig_s {
    int16_t scale;              			// 测量电流传感器输出电压到毫安. Value in mV/10A
    int16_t offset;             			// 电流传感器在mA中的偏移量
} currentSensorADCConfig_t;
// 声明ADC配置结构体
PG_DECLARE(currentSensorADCConfig_t, currentSensorADCConfig);

/* ------------------------电流计电压状态结构体------------------------ */	
typedef struct currentMeterVirtualState_s {
    currentMeterMAhDrawnState_t mahDrawnState;
    int32_t amperage;           			// 电流传感器读出的电流，单位为厘安(1/100 A)
} currentSensorVirtualState_t;

/* ------------------------电流计电压配置结构体------------------------ */	
typedef struct currentSensorVirtualConfig_s {
    int16_t scale;              			// 测量电流传感器输出电压到毫安，数值为1/10 mV/A
    uint16_t offset;            			// 电流传感器的偏移量，以毫伏为单位
} currentSensorVirtualConfig_t;
// 声明电流计电压配置
PG_DECLARE(currentSensorVirtualConfig_t, currentSensorVirtualConfig);

void currentMeterReset(currentMeter_t *meter);
void currentMeterADCInit(void);
void currentMeterADCRefresh(int32_t lastUpdateAt);
void currentMeterADCRead(currentMeter_t *meter);
void currentMeterVirtualInit(void);
void currentMeterVirtualRefresh(int32_t lastUpdateAt, bool armed, bool throttleLowAndMotorStop, int32_t throttleOffset);
void currentMeterVirtualRead(currentMeter_t *meter);
void currentMeterESCInit(void);
void currentMeterESCRefresh(int32_t lastUpdateAt);
void currentMeterESCReadCombined(currentMeter_t *meter);
void currentMeterESCReadMotor(uint8_t motorNumber, currentMeter_t *meter);
void currentMeterMSPInit(void);
void currentMeterMSPRefresh(timeUs_t currentTimeUs);
void currentMeterMSPRead(currentMeter_t *meter);
void currentMeterMSPSet(uint16_t amperage, uint16_t mAhDrawn);
extern const uint8_t supportedCurrentMeterCount;
extern const uint8_t currentMeterIds[];
void currentMeterRead(currentMeterId_e id, currentMeter_t *currentMeter);

