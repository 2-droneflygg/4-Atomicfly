#pragma once

#include "common/axis.h"
#include "common/filter.h"
#include "common/time.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus.h"
#include "drivers/sensor.h"

#include "flight/pid.h"

#include "pg/pg.h"

// 配置使用陀螺仪1
#define GYRO_CONFIG_USE_GYRO_1      0

// 滤波器截止的最大频率(nyquist极限8K最大采样)
#define FILTER_FREQUENCY_MAX 4000      					

#ifdef USE_YAW_SPIN_RECOVERY
// 反自旋阈值 - 最小
#define YAW_SPIN_RECOVERY_THRESHOLD_MIN 500         
// 反自旋阈值 - 最大
#define YAW_SPIN_RECOVERY_THRESHOLD_MAX 1950		    
#endif

/* --------------------------陀螺仪低通滤波器枚举--------------------------- */	
typedef union gyroLowpassFilter_u {
    pt1Filter_t pt1FilterState;							// PT1类
    biquadFilter_t biquadFilterState;					// biquad类
} gyroLowpassFilter_t;

/* ---------------------------陀螺仪检测标志枚举---------------------------- */	
typedef enum gyroDetectionFlags_e {
    GYRO_NONE_MASK = 0,									// 无
    GYRO_1_MASK = BIT(0),							    // GYRO_1
} gyroDetectionFlags_t;

/* ----------------------------陀螺仪校准结构体----------------------------- */	
typedef struct gyroCalibration_s {
    float sum[XYZ_AXIS_COUNT];							// sum
    stdev_t var[XYZ_AXIS_COUNT];						// var
    int32_t cyclesRemaining;							// 剩余周期
} gyroCalibration_t;

/* ---------------------------陀螺仪传感器结构体---------------------------- */	
typedef struct gyroSensor_s {
    gyroDev_t gyroDev;									// 陀螺仪设备
    gyroCalibration_t calibration;						// 校准
} gyroSensor_t;

/* ---------------------------陀螺仪低通滤波枚举---------------------------- */	
enum {
    FILTER_LOWPASS = 0,
    FILTER_LOWPASS2
};

/* ------------------------陀螺仪动态低通滤波类型枚举----------------------- */	
enum {
    DYN_LPF_NONE = 0,
    DYN_LPF_PT1
};

/* ----------------------------陀螺仪反自旋枚举----------------------------- */	
typedef enum {
    YAW_SPIN_RECOVERY_OFF,
    YAW_SPIN_RECOVERY_ON,
    YAW_SPIN_RECOVERY_AUTO
} yawSpinRecoveryMode_e;

/* ------------------------------陀螺仪结构体------------------------------- */	
typedef struct gyro_s {
    uint16_t sampleRateHz;								// 采样速率
    uint32_t targetLooptime;							// 目标循环时间
    uint32_t sampleLooptime;							// 采样循环时间
    float scale;									    // 缩放
    float gyroADC[XYZ_AXIS_COUNT];     					// 传感器的对齐、校准、缩放但未过滤的数据
    float gyroADCf[XYZ_AXIS_COUNT];    					// 滤波后的陀螺数据
    uint8_t sampleCount;               					// 陀螺传感器采样计数器
    float sampleSum[XYZ_AXIS_COUNT];   					// 采样的总样本
    bool downsampleFilterEnabled;      					// 如果为真，则使用陀螺低通2降低采样，否则使用平均值

    gyroSensor_t gyroSensor1;							// 陀螺仪传感器

    gyroDev_t *rawSensorDev;          					// 指向为DEBUG_GYRO_RAW提供原始数据的传感器的指针

    // 低通滤波器1
    filterApplyFnPtr lowpassFilterApplyFn;
    gyroLowpassFilter_t lowpassFilter[XYZ_AXIS_COUNT];

    // 低通滤波器2
    filterApplyFnPtr lowpass2FilterApplyFn;
    gyroLowpassFilter_t lowpass2Filter[XYZ_AXIS_COUNT];

    uint16_t accSampleRateHz;							// 加速度采样速率
    uint8_t gyroToUse;
    bool gyroHasOverflowProtection;

#ifdef USE_DYN_LPF
    uint8_t dynLpfFilter;
    uint16_t dynLpfMin;
    uint16_t dynLpfMax;
#endif
} gyro_t;
// 声明陀螺仪结构体
extern gyro_t gyro;
extern uint8_t activePidLoopDenom;

/* ----------------------------陀螺仪配置结构体----------------------------- */	
typedef struct gyroConfig_s {
	// 运动校准阈值 - 在初始化过程中移动飞行器会导致错误的陀螺偏移量
    uint8_t  gyroMovementCalibrationThreshold; 			
    // 陀螺DLPF设置
    uint8_t  gyro_hardware_lpf;                			

    uint8_t  gyro_high_fsr;
    uint8_t  gyro_to_use;

    uint16_t gyro_lowpass_hz;
    uint16_t gyro_lowpass2_hz;

    int16_t  gyro_offset_yaw;

    // 低通滤波器主要/次要
    uint8_t  gyro_lowpass_type;
    uint8_t  gyro_lowpass2_type;

    uint8_t  yaw_spin_recovery;
    int16_t  yaw_spin_threshold;
	// 陀螺校准时间为1/100秒
    uint16_t gyroCalibrationDuration;   	   

    uint16_t dyn_lpf_gyro_min_hz;
    uint16_t dyn_lpf_gyro_max_hz;
	// 在启动时应该尝试检测什么陀螺仪，第一次开机时自动设置。
    uint8_t gyrosDetected; 					   
} gyroConfig_t;
// 声明陀螺仪配置结构体
PG_DECLARE(gyroConfig_t, gyroConfig);

void GYRO_FILTER_FUNCTION(void);
void gyroUpdate(void);
void gyroFiltering(timeUs_t currentTimeUs);
bool gyroGetAccumulationAverage(float *accumulation);
void gyroStartCalibration(bool isFirstArmingCalibration);
bool isFirstArmingGyroCalibrationRunning(void);
bool gyroIsCalibrationComplete(void);
bool gyroOverflowDetected(void);
bool gyroYawSpinDetected(void);
uint16_t gyroAbsRateDps(int axis);
#ifdef USE_DYN_LPF
float dynThrottle(float throttle);
void dynLpfGyroUpdate(float throttle);
#endif
#ifdef USE_YAW_SPIN_RECOVERY
void initYawSpinRecovery(int maxYawRate);
#endif

