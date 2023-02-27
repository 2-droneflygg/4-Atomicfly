#pragma once

#include "common/time.h"
#include "pg/pg.h"
#include "drivers/accgyro/accgyro.h"
#include "sensors/sensors.h"

/* ---------------------------使用/检测的加速度计类型枚举----------------------------- */	
typedef enum {
    ACC_DEFAULT,
    ACC_NONE,
    ACC_MPU6000,
} accelerationSensor_e;

/* ----------------------------------加速度计结构体----------------------------------- */	
typedef struct acc_s {
    accDev_t dev;
    uint16_t sampleRateHz;
    float accADC[XYZ_AXIS_COUNT];
    bool isAccelUpdatedAtLeastOnce;	// 加速度计是否至少更新一次
} acc_t;
extern acc_t acc;

/* ------------------------------rollAndPitch零偏结构体------------------------------ */	
typedef struct rollAndPitchTrims_s {
    int16_t roll;
    int16_t pitch;
} rollAndPitchTrims_t_def;

/* ------------------------------rollAndPitch零偏共用体------------------------------ */	
typedef union rollAndPitchTrims_u {
    int16_t raw[2];
    rollAndPitchTrims_t_def values;
} rollAndPitchTrims_t;

/* --------------------------------加速度计配置结构体--------------------------------- */	
#if defined(USE_ACC)
typedef struct accelerometerConfig_s {
    uint16_t acc_lpf_hz;                    // acc z轴上使用的低通滤波器的截止频率(Hz)
    uint8_t acc_hardware;                   // acc硬件
    bool acc_high_fsr;
    flightDynamicsTrims_t accZero;
    rollAndPitchTrims_t accelerometerTrims;
} accelerometerConfig_t;
// 声明加速度计配置结构体
PG_DECLARE(accelerometerConfig_t, accelerometerConfig);
#endif

extern uint16_t InflightcalibratingA;
extern bool AccInflightCalibrationMeasurementDone;
extern bool AccInflightCalibrationSavetoEEProm;
extern bool AccInflightCalibrationActive;
bool accInit(uint16_t accSampleRateHz);
bool accIsCalibrationComplete(void);
bool accHasBeenCalibrated(void);
void accStartCalibration(void);
void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims);
void accUpdate(timeUs_t currentTimeUs, rollAndPitchTrims_t *rollAndPitchTrims);
bool accGetAccumulationAverage(float *accumulation);
union flightDynamicsTrims_u;
void setAccelerationTrims(union flightDynamicsTrims_u *accelerationTrimsToUse);
void accInitFilters(void);

