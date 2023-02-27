/*********************************************************************************
 提供陀螺仪驱动相关信息。
*********************************************************************************/
#pragma once

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"
#include "sensors/boardalignment.h"
#include "drivers/exti.h"
#include "drivers/bus.h"
#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro_mpu.h"

/* -----------------------------陀螺仪硬件枚举------------------------------ */	
typedef enum {
    GYRO_NONE = 0,
    GYRO_DEFAULT,
    GYRO_MPU6000,
} gyroHardware_e;

/* --------------------------陀螺仪硬件滤波器枚举--------------------------- */	
typedef enum {
    GYRO_HARDWARE_LPF_NORMAL,
} gyroHardwareLpf_e;

/* ---------------------------陀螺仪更新速率枚举---------------------------- */	
typedef enum {
    GYRO_RATE_1_kHz,
    GYRO_RATE_1100_Hz,
    GYRO_RATE_3200_Hz,
    GYRO_RATE_6400_Hz,
    GYRO_RATE_8_kHz,
} gyroRateKHz_e;

/* ----------------------------陀螺仪设备结构体------------------------------ */	
typedef struct gyroDev_s {
    sensorGyroInitFuncPtr initFn;                             // 初始化函数
    sensorGyroReadFuncPtr readFn;                             // 读取3轴数据
    sensorGyroReadDataFuncPtr temperatureFn;                  // 读取温度
    extiCallbackRec_t exti;                                   // 外部中断
    busDevice_t bus;                                          // 总线
    float scale;                                              // 比例因子
    float gyroZero[XYZ_AXIS_COUNT];                           // 零偏数据
    float gyroADC[XYZ_AXIS_COUNT];                            // 校正后的陀螺数据
    int32_t gyroADCRawPrevious[XYZ_AXIS_COUNT];               // 上一次陀螺仪原始数据
    int16_t gyroADCRaw[XYZ_AXIS_COUNT];                       // 陀螺仪原始数据
    int16_t temperature;                                      // 温度数据
    mpuDetectionResult_t mpuDetectionResult;                  // 检测结果
    sensor_align_e gyroAlign;                                 // 陀螺仪对齐
    gyroRateKHz_e gyroRateKHz;                                // 陀螺仪更新速率
    bool dataReady;                                           // 数据就绪状态
    bool gyro_high_fsr;                                       // gyro_high_fsr
    uint8_t hardware_lpf;                                     // 硬件滤波
    uint8_t mpuDividerDrops;                                  // 分频
    ioTag_t mpuIntExtiTag;                                    // 外部中断线
    gyroHardware_e gyroHardware;                              // 陀螺仪硬件设备  
    fp_rotationMatrix_t rotationMatrix;                       // 旋转矩阵 - 默认板对齐
    uint16_t gyroSampleRateHz;                                // 陀螺仪采样速率
    uint16_t accSampleRateHz;                                 // 加速度计采样速率
} gyroDev_t;

/* ---------------------------加速度计设备结构体----------------------------- */	
typedef struct accDev_s {
    float acc_1G_rec;                                         // acc_1G_rec
    sensorAccInitFuncPtr initFn;                              // 初始化函数
    sensorAccReadFuncPtr readFn;                              // 读取3轴数据
    busDevice_t bus;                                          // 总线
    uint16_t acc_1G;                                          // acc_1G
    int16_t ADCRaw[XYZ_AXIS_COUNT];                           // 原始数据
    mpuDetectionResult_t mpuDetectionResult;                  // 检测结果
    sensor_align_e accAlign;                                  // 加速度计对齐
    bool dataReady;                                           // 数据就绪状态
    bool acc_high_fsr;                                        // acc_high_fsr
    uint8_t filler[2];                                        // 滤波
    fp_rotationMatrix_t rotationMatrix;                       // 旋转矩阵
} accDev_t;

