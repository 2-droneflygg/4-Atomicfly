#pragma once

#include "voltage_ids.h"


#define SLOW_VOLTAGE_TASK_FREQ_HZ 50
#define FAST_VOLTAGE_TASK_FREQ_HZ 200

#define VBAT_SCALE_MIN 0
#define VBAT_SCALE_MAX 255

#ifndef MAX_VOLTAGE_SENSOR_ADC
#define MAX_VOLTAGE_SENSOR_ADC 1 		    // VBAT
#endif

/* --------------------------电压表源枚举-------------------------- */	
typedef enum {
    VOLTAGE_METER_NONE = 0,
    VOLTAGE_METER_ADC,
    VOLTAGE_METER_COUNT
} voltageMeterSource_e;

/* ------------------------电压表ADC状态结构体--------------------- */	
typedef struct voltageMeterADCState_s {
    uint16_t voltageDisplayFiltered;         // 电池电压0.01V级(滤波后)
    uint16_t voltageUnfiltered;       		 // 电池电压0.01V级(未滤波)
    pt1Filter_t displayFilter;				 // 电池电压显示滤波
} voltageMeterADCState_t;

/* ----------------------------电压计结构体------------------------ */	
// WARNING - 不要混合使用电压表和电压传感器，它们是独立的问题
typedef struct voltageMeter_s {
    uint16_t displayFiltered;               // 显示滤波 - 电压为0.01V阶
    uint16_t unfiltered;                    // 无滤波 - 电压为0.01V阶
    bool lowVoltageCutoff;					// 截止电压
} voltageMeter_t;

/* --------------------------电压传感器ADC枚举--------------------- */	
typedef enum {
    VOLTAGE_SENSOR_ADC_VBAT = 0,
} voltageSensorADC_e; 					    

/* ------------------------电压传感器ADC配置结构体----------------- */	
typedef struct voltageSensorADCConfig_s {
    uint8_t vbatscale;                      // 电压计比例
    uint8_t vbatresdivval;                  // 电阻分频器R2(默认NAZE 10(K))
    uint8_t vbatresdivmultiplier;           // 比例乘法器(例如2.5:1的比例乘法器为4时，可以使用“100”代替比例中的“25”)来获得更好的精度
} voltageSensorADCConfig_t;
// 声明电压传感器ADC配置结构体
PG_DECLARE_ARRAY(voltageSensorADCConfig_t, MAX_VOLTAGE_SENSOR_ADC, voltageSensorADCConfig);

extern const char * const voltageMeterSourceNames[VOLTAGE_METER_COUNT];

void voltageMeterReset(voltageMeter_t *voltageMeter);
void voltageMeterADCInit(void);
void voltageMeterADCRefresh(void);
void voltageMeterADCRead(voltageSensorADC_e adcChannel, voltageMeter_t *voltageMeter);
void voltageMeterRead(voltageMeterId_e id, voltageMeter_t *voltageMeter);

