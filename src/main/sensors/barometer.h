#pragma once

#include "pg/pg.h"
#include "drivers/barometer/barometer.h"

#define BARO_SAMPLE_COUNT_MAX   48		    // 最大采样数

/* ---------------------------气压计传感器类型枚举----------------------------- */	
typedef enum {
    BARO_DEFAULT = 0,
    BARO_NONE = 1,
    BARO_BMP280 = 4,
} baroSensor_e;

/* ------------------------------气压计状态枚举-------------------------------- */	
typedef enum {
    BAROMETER_NEEDS_TEMPERATURE_READ = 0,
    BAROMETER_NEEDS_TEMPERATURE_SAMPLE,
    BAROMETER_NEEDS_PRESSURE_START,
    BAROMETER_NEEDS_PRESSURE_READ,
    BAROMETER_NEEDS_PRESSURE_SAMPLE,
    BAROMETER_NEEDS_TEMPERATURE_START
} barometerState_e;


/* -----------------------------气压计配置结构体------------------------------- */	
typedef struct barometerConfig_s {
    uint8_t baro_bustype;
    uint8_t baro_spi_device;
    ioTag_t baro_spi_csn;                   // 也用作BMP085的XCLR(正逻辑)
    uint8_t baro_i2c_device;
    uint8_t baro_i2c_address;
    uint8_t baro_hardware;                  // 气压计硬件
    uint8_t baro_sample_count;              // baro滤波器阵列的大小
    uint16_t baro_noise_lpf;                // 额外的低通径以减少气压噪声
    uint16_t baro_cf_vel;                   // 应用互补滤波器以保持基于baro速度的计算速度(即接近实际速度)
} barometerConfig_t;
// 声明气压计配置结构体
PG_DECLARE(barometerConfig_t, barometerConfig);

/* -------------------------------气压计结构体--------------------------------- */	
typedef struct baro_s {
    baroDev_t dev;						    // 气压计设备
    int32_t BaroAlt;						// 气压计高度
    int32_t baroTemperature;             	// 气压计温度
    int32_t baroPressure;                	// 气压计压力
} baro_t;
// 声明气压计结构体
extern baro_t baro;

void baroPreInit(void);
bool baroDetect(baroDev_t *dev, baroSensor_e baroHardwareToUse);
bool baroIsCalibrationComplete(void);
void baroStartCalibration(void);
void baroSetGroundLevel(void);
uint32_t baroUpdate(void);
bool isBaroReady(void);
int32_t baroCalculateAltitude(void);
void performBaroCalibrationCycle(void);

