/*********************************************************************************
 提供陀螺仪初始化相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/config.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/accgyro/gyro_sync.h"

#include "fc/runtime_config.h"

#include "pg/gyrodev.h"

#include "sensors/gyro.h"
#include "sensors/sensors.h"

// ---------------------------------------------------------陀螺仪检测
static gyroDetectionFlags_t gyroDetectionFlags = GYRO_NONE_MASK;

// ---------------------------------------------------------激活陀螺仪
#define ACTIVE_GYRO (&gyro.gyroSensor1)

//-------------------------------------------------------------------------------------获取陀螺仪速率相关API
/**********************************************************************
函数名称：gyroRateDps
函数功能：陀螺速率Dps
函数形参：axis
函数返回值：速率Dps
函数描述：None 
**********************************************************************/
int16_t gyroRateDps(int axis)
{
    return lrintf(gyro.gyroADCf[axis] / ACTIVE_GYRO->gyroDev.scale);
}

//-------------------------------------------------------------------------------------陀螺仪滤波器初始化相关API

/**********************************************************************
函数名称：gyroInitLowpassFilterLpf
函数功能：陀螺仪初始化低通滤波器
函数形参：slot，type，lpfHz，looptime
函数返回值：ret
函数描述：None 
**********************************************************************/
static bool gyroInitLowpassFilterLpf(int slot, int type, uint16_t lpfHz, uint32_t looptime)
{
    filterApplyFnPtr *lowpassFilterApplyFn;
    gyroLowpassFilter_t *lowpassFilter = NULL;

    switch (slot) {
    case FILTER_LOWPASS:
        lowpassFilterApplyFn = &gyro.lowpassFilterApplyFn;
        lowpassFilter = gyro.lowpassFilter;
        break;

    case FILTER_LOWPASS2:
        lowpassFilterApplyFn = &gyro.lowpass2FilterApplyFn;
        lowpassFilter = gyro.lowpass2Filter;
        break;

    default:
        return false;
    }

    bool ret = false;

    // 建立一些共同常数
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / looptime;
    const float gyroDt = looptime * 1e-6f;

    // 增益可以稍后计算，因为它是特定于pt1/bqrcf2/fkf分支
    const float gain = pt1FilterGain(lpfHz, gyroDt);

	// 在检查有效的截止和过滤之前，将指针解引用为null类型。阳性的案件将被推翻。
    *lowpassFilterApplyFn = nullFilterApply;

    // 如果已指定低通截止，且小于奈奎斯特频率
    if (lpfHz && lpfHz <= gyroFrequencyNyquist) {
        switch (type) {
        case FILTER_PT1:
            *lowpassFilterApplyFn = (filterApplyFnPtr) pt1FilterApply;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterInit(&lowpassFilter[axis].pt1FilterState, gain);
            }
            ret = true;
            break;
        }
    }
    return ret;
}

/**********************************************************************
函数名称：dynLpfFilterInit
函数功能：动态低通滤波器初始化
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
#ifdef USE_DYN_LPF
static void dynLpfFilterInit()
{
    if (gyroConfig()->dyn_lpf_gyro_min_hz > 0) {
        switch (gyroConfig()->gyro_lowpass_type) {
        case FILTER_PT1:
            gyro.dynLpfFilter = DYN_LPF_PT1;
            break;
        default:
            gyro.dynLpfFilter = DYN_LPF_NONE;
            break;
        }
    } else {
        gyro.dynLpfFilter = DYN_LPF_NONE;
    }
    gyro.dynLpfMin = gyroConfig()->dyn_lpf_gyro_min_hz;
    gyro.dynLpfMax = gyroConfig()->dyn_lpf_gyro_max_hz;
}
#endif

/**********************************************************************
函数名称：gyroInitFilters
函数功能：陀螺仪滤波器初始化
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void gyroInitFilters(void)
{
    uint16_t gyro_lowpass_hz = gyroConfig()->gyro_lowpass_hz;

#ifdef USE_DYN_LPF
    if (gyroConfig()->dyn_lpf_gyro_min_hz > 0) {
        gyro_lowpass_hz = gyroConfig()->dyn_lpf_gyro_min_hz;
    }
#endif

	// 初始化陀螺仪低通滤波器1
    gyroInitLowpassFilterLpf(
      FILTER_LOWPASS,
      gyroConfig()->gyro_lowpass_type,
      gyro_lowpass_hz,
      gyro.targetLooptime
    );

	// 初始化陀螺仪低通滤波器2
    gyro.downsampleFilterEnabled = gyroInitLowpassFilterLpf(
      FILTER_LOWPASS2,
      gyroConfig()->gyro_lowpass2_type,
      gyroConfig()->gyro_lowpass2_hz,
      gyro.sampleLooptime
    );

	// 动态滤波器初始化
#ifdef USE_DYN_LPF
    dynLpfFilterInit();
#endif
}

//-------------------------------------------------------------------------------------设置目标陀螺仪循环时间相关API

/**********************************************************************
函数名称：gyroSetTargetLooptime
函数功能：设置目标陀螺仪循环时间
函数形参：pidDenom
函数返回值：None 
函数描述：None 
**********************************************************************/
void gyroSetTargetLooptime(uint8_t pidDenom)
{
    activePidLoopDenom = pidDenom;
    if (gyro.sampleRateHz) {
		// 陀螺仪采样循环时间 = 1000000 / 8000 = 125us（8K）
        gyro.sampleLooptime = 1e6 / gyro.sampleRateHz;
		// 陀螺仪采样目标时间 = 1000000 / 8000 = 125us（8K）
        gyro.targetLooptime = activePidLoopDenom * 1e6 / gyro.sampleRateHz;
    } else {
        gyro.sampleLooptime = 0;
        gyro.targetLooptime = 0;
    }
}

//-------------------------------------------------------------------------------------获取陀螺仪传感器总线相关API

/**********************************************************************
函数名称：gyroSensorBus
函数功能：获取陀螺仪传感器总线
函数形参：None 
函数返回值：ACTIVE_GYRO->gyroDev.bus
函数描述：None 
**********************************************************************/
const busDevice_t *gyroSensorBus(void)
{
    return &ACTIVE_GYRO->gyroDev.bus;
}

//-------------------------------------------------------------------------------------陀螺仪检测相关API

/**********************************************************************
函数名称：gyroDetect
函数功能：陀螺仪检测
函数形参：dev
函数返回值：gyroHardware
函数描述：None 
**********************************************************************/
STATIC_UNIT_TESTED gyroHardware_e gyroDetect(gyroDev_t *dev)
{
    gyroHardware_e gyroHardware = GYRO_DEFAULT;

    switch (gyroHardware) {
	    case GYRO_DEFAULT:
	        FALLTHROUGH;

#ifdef USE_GYRO_SPI_MPU6000
	    case GYRO_MPU6000:
	        if (mpu6000SpiGyroDetect(dev)) {
	            gyroHardware = GYRO_MPU6000;
	            break;
	        }
	        FALLTHROUGH;
#endif
	    default:
	        gyroHardware = GYRO_NONE;
    }

    if (gyroHardware != GYRO_NONE) {
        sensorsSet(SENSOR_GYRO);
    }
	
    return gyroHardware;
}

/**********************************************************************
函数名称：gyroDetectSensor
函数功能：陀螺仪传感器检测
函数形参：gyroSensor，config
函数返回值：gyroHardware
函数描述：None 
**********************************************************************/
static bool gyroDetectSensor(gyroSensor_t *gyroSensor, const gyroDeviceConfig_t *config)
{
#if defined(USE_GYRO_SPI_MPU6000) 
    bool gyroFound = mpuDetect(&gyroSensor->gyroDev, config);
#if !defined(USE_FAKE_GYRO)        // 允许求助于伪造的accgyro，如果定义
    if (!gyroFound) {
        return false;
    }
#else
    UNUSED(gyroFound);
#endif
#else
    UNUSED(config);
#endif
    const gyroHardware_e gyroHardware = gyroDetect(&gyroSensor->gyroDev);
    gyroSensor->gyroDev.gyroHardware = gyroHardware;

    return gyroHardware != GYRO_NONE;
}

/**********************************************************************
函数名称：gyroMpuDetectionResult
函数功能：获取陀螺仪检测结果
函数形参：None 
函数返回值：ACTIVE_GYRO->gyroDev.mpuDetectionResult
函数描述：None 
**********************************************************************/
const mpuDetectionResult_t *gyroMpuDetectionResult(void)
{
    return &ACTIVE_GYRO->gyroDev.mpuDetectionResult;
}

//-------------------------------------------------------------------------------------陀螺仪初始化相关API

/**********************************************************************
函数名称：gyroPreInitSensor
函数功能：陀螺仪传感器预初始化
函数形参：config
函数返回值：None 
函数描述：None 
**********************************************************************/
static void gyroPreInitSensor(const gyroDeviceConfig_t *config)
{
#if defined(USE_GYRO_SPI_MPU6000) 
    mpuPreInit(config);
#else
    UNUSED(config);
#endif
}

/**********************************************************************
函数名称：gyroPreInit
函数功能：陀螺仪预初始化
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void gyroPreInit(void)
{
    gyroPreInitSensor(gyroDeviceConfig(0));
}

/**********************************************************************
函数名称：gyroInitSensor
函数功能：陀螺仪传感器初始化
函数形参：gyroSensor，config
函数返回值：None 
函数描述：None 
**********************************************************************/
void gyroInitSensor(gyroSensor_t *gyroSensor, const gyroDeviceConfig_t *config)
{
    gyroSensor->gyroDev.gyro_high_fsr = gyroConfig()->gyro_high_fsr;
    gyroSensor->gyroDev.gyroAlign = config->alignment;
    gyroSensor->gyroDev.mpuIntExtiTag = config->extiTag;
    gyroSensor->gyroDev.hardware_lpf = gyroConfig()->gyro_hardware_lpf;

    // 获取陀螺仪采样速率
    gyroSensor->gyroDev.gyroSampleRateHz = gyroSetSampleRate(&gyroSensor->gyroDev);
    gyroSensor->gyroDev.initFn(&gyroSensor->gyroDev);

    switch (gyroSensor->gyroDev.gyroHardware) {
	    case GYRO_NONE:    								
	    case GYRO_DEFAULT:
	    case GYRO_MPU6000:
	    default:
        break;
    }
}

/**********************************************************************
函数名称：gyroInit
函数功能：陀螺仪初始化
函数形参：None 
函数返回值：状态
函数描述：None 
**********************************************************************/
bool gyroInit(void)
{
    gyro.gyroHasOverflowProtection = true;

    gyroDetectionFlags = GYRO_NONE_MASK;
	// 陀螺仪扫描
    uint8_t gyrosToScan = gyroConfig()->gyrosDetected;
	// 陀螺仪使用 - 使用陀螺仪1
    gyro.gyroToUse = gyroConfig()->gyro_to_use;

	// 陀螺仪扫描
    if ((!gyrosToScan || (gyrosToScan & GYRO_1_MASK)) && gyroDetectSensor(&gyro.gyroSensor1, gyroDeviceConfig(0))) {
        gyroDetectionFlags |= GYRO_1_MASK;
    }
    if (gyroDetectionFlags == GYRO_NONE_MASK) {
        return false;
    }

	// EEPROM写入需求 - 写入陀螺仪设备
    bool eepromWriteRequired = false;
    if (!gyrosToScan) {
        gyroConfigMutable()->gyrosDetected = gyroDetectionFlags;
        eepromWriteRequired = true;
    }
    if (eepromWriteRequired) {
        writeEEPROM();
    }

	// 陀螺仪传感器初始化
    if (gyro.gyroToUse == GYRO_CONFIG_USE_GYRO_1) {
		// 陀螺仪传感器初始化
        gyroInitSensor(&gyro.gyroSensor1, gyroDeviceConfig(0));
		// 陀螺仪在线
        detectedSensors[SENSOR_INDEX_GYRO] = gyro.gyroSensor1.gyroDev.gyroHardware;
    }

	// 陀螺仪缩放
    gyro.scale = gyro.gyroSensor1.gyroDev.scale;
	// 原始陀螺仪设备
    gyro.rawSensorDev = &gyro.gyroSensor1.gyroDev;

	// 设置陀螺仪/加速度计采样速率
    if (gyro.rawSensorDev) {
		// 由gyroSetSampleRate设置
        gyro.sampleRateHz = gyro.rawSensorDev->gyroSampleRateHz;    
		// 由gyroSetSampleRate设置
        gyro.accSampleRateHz = gyro.rawSensorDev->accSampleRateHz;   
    } else {
        gyro.sampleRateHz = 0;
        gyro.accSampleRateHz = 0;
    }

    return true;
}

