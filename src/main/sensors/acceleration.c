/*********************************************************************************
 提供加速度计相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_ACC
#include "common/axis.h"
#include "common/filter.h"
#include "common/utils.h"

#include "config/config_reset.h"
#include "config/feature.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"

#include "drivers/bus_spi.h"

#include "config/config.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"

#include "pg/gyrodev.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/sensors.h"

#include "acceleration.h"

// ---------------------------------------------------------加速度计访问
FAST_RAM_ZERO_INIT acc_t acc;    				    		  

// ---------------------------------------------------------加速度计校准
static flightDynamicsTrims_t *accelerationTrims;
// ---------------------------------------------------------加速度计校准周期
#define CALIBRATING_ACC_CYCLES   400	
// ---------------------------------------------------------校准在每个周期递减直到0，然后进入正常模式
static uint16_t calibratingA = 0;   			  
// ---------------------------------------------------------累积测量
static float accumulatedMeasurements[XYZ_AXIS_COUNT]; 
// ---------------------------------------------------------累积测量数量
static int accumulatedMeasurementCount;		

// ---------------------------------------------------------低通滤波
static uint16_t accLpfCutHz = 0;
static biquadFilter_t accFilter[XYZ_AXIS_COUNT];

PG_REGISTER_WITH_RESET_FN(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 2);
void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims)
{
    RESET_CONFIG_2(rollAndPitchTrims_t, rollAndPitchTrims,
        .values.roll = 0,
        .values.pitch = 0,
    );
}
static void resetFlightDynamicsTrims(flightDynamicsTrims_t *accZero)
{
    accZero->values.roll = 0;
    accZero->values.pitch = 0;
    accZero->values.yaw = 0;
    accZero->values.calibrationCompleted = 0;
}
void pgResetFn_accelerometerConfig(accelerometerConfig_t *instance)
{
    RESET_CONFIG_2(accelerometerConfig_t, instance,
        .acc_lpf_hz = 10,
        .acc_hardware = ACC_DEFAULT,
        .acc_high_fsr = false,
    );
    resetRollAndPitchTrims(&instance->accelerometerTrims);
    resetFlightDynamicsTrims(&instance->accZero);
}	

//-------------------------------------------------------------------------------------复位相关API

/**********************************************************************
函数名称：accResetRollAndPitchTrims
函数功能：复位横滚俯仰零偏
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
void accResetRollAndPitchTrims(void)
{
    resetRollAndPitchTrims(&accelerometerConfigMutable()->accelerometerTrims);
}

//-------------------------------------------------------------------------------------加速度计校准相关API

/**********************************************************************
函数名称：setConfigCalibrationCompleted
函数功能：设置配置 - 校准完成
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
static void setConfigCalibrationCompleted(void)
{
    accelerometerConfigMutable()->accZero.values.calibrationCompleted = 1;
}

/**********************************************************************
函数名称：accHasBeenCalibrated
函数功能：获取加速度计是否校准
函数形参：None  
函数返回值：accZero.values.calibrationCompleted
函数描述：None 
**********************************************************************/
bool accHasBeenCalibrated(void)
{
    return accelerometerConfig()->accZero.values.calibrationCompleted;
}

/**********************************************************************
函数名称：accStartCalibration
函数功能：加速度计开始校准
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void accStartCalibration(void)
{
    calibratingA = CALIBRATING_ACC_CYCLES;
}

/**********************************************************************
函数名称：accIsCalibrationComplete
函数功能：获取加速度计是否校准完成
函数形参：None 
函数返回值：状态
函数描述：None 
**********************************************************************/
bool accIsCalibrationComplete(void)
{
    return calibratingA == 0;
}

/**********************************************************************
函数名称：isOnFinalAccelerationCalibrationCycle
函数功能：获取是否在最后的加速度校准周期
函数形参：None 
函数返回值：状态
函数描述：None 
**********************************************************************/
static bool isOnFinalAccelerationCalibrationCycle(void)
{
    return calibratingA == 1;
}

/**********************************************************************
函数名称：isOnFinalAccelerationCalibrationCycle
函数功能：获取是否在第一个加速校准周期
函数形参：None 
函数返回值：状态
函数描述：None 
**********************************************************************/
static bool isOnFirstAccelerationCalibrationCycle(void)
{
    return calibratingA == CALIBRATING_ACC_CYCLES;
}

/**********************************************************************
函数名称：performAcclerationCalibration
函数功能：执行加速度校准
函数形参：rollAndPitchTrims
函数返回值：None 
函数描述：None 
**********************************************************************/
static void performAcclerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims)
{
    static int32_t a[3];

	// 遍历三轴
    for (int axis = 0; axis < 3; axis++) {
        // 在第一个周期复位所有轴
        if (isOnFirstAccelerationCalibrationCycle()) {
            a[axis] = 0;
        }
        // 采集加速度数据
        a[axis] += acc.accADC[axis];
        // 重置全局变量以防止其他代码使用未校准的数据
        acc.accADC[axis] = 0;
        accelerationTrims->raw[axis] = 0;
    }

	// 判断是否在最后一个校准周期
    if (isOnFinalAccelerationCalibrationCycle()) {
        // 计算平均值，将Z向下平移acc_1G，并在校准结束时将值存储在EEPROM中
        accelerationTrims->raw[X] = (a[X] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationTrims->raw[Y] = (a[Y] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationTrims->raw[Z] = (a[Z] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - acc.dev.acc_1G;
        resetRollAndPitchTrims(rollAndPitchTrims);
		// 设置配置校准完成
        setConfigCalibrationCompleted();
		// 保存配置和通知
        saveConfigAndNotify();
    }
	// 校准周期递减
    calibratingA--;
}

/**********************************************************************
函数名称：applyAccelerationTrims
函数功能：应用加速度校准零偏
函数形参：accelerationTrims
函数返回值：None 
函数描述：
	加速度数据减零偏.
**********************************************************************/
static void applyAccelerationTrims(const flightDynamicsTrims_t *accelerationTrims)
{
    acc.accADC[X] -= accelerationTrims->raw[X];
    acc.accADC[Y] -= accelerationTrims->raw[Y];
    acc.accADC[Z] -= accelerationTrims->raw[Z];
}

/**********************************************************************
函数名称：setAccelerationTrims
函数功能：设置加速度计零偏 - 从FLASH校准后的数据设置
函数形参：accelerationTrimsToUse
函数返回值：None 
函数描述：None 
**********************************************************************/
void setAccelerationTrims(flightDynamicsTrims_t *accelerationTrimsToUse)
{
    accelerationTrims = accelerationTrimsToUse;
}

//-------------------------------------------------------------------------------------加速度计检测相关API

/**********************************************************************
函数名称：accDetect
函数功能：加速度计检测
函数形参：dev，ccHardwareToUse 
函数返回值：加速度计是否存在
函数描述：None 
**********************************************************************/
bool accDetect(accDev_t *dev, accelerationSensor_e accHardwareToUse)
{
	// ---------------------------加速度计检测
    accelerationSensor_e accHardware = ACC_NONE;
retry:
    switch (accHardwareToUse) {
		// 无加速度计
	    case ACC_DEFAULT:	 
	        FALLTHROUGH;
#ifdef USE_ACC_SPI_MPU6000	 
		// MPU6000
	    case ACC_MPU6000:
	        if (mpu6000SpiAccDetect(dev)) {
	            accHardware = ACC_MPU6000;
	            break;
	        }
	        FALLTHROUGH;
#endif
		// 其他
    	default:			 
		    case ACC_NONE:   
				// 失能加速度计
		        accHardware = ACC_NONE;
		        break;
	}

    // ---------------------------检查错误或加速度计真的缺失
    if (accHardware == ACC_NONE && accHardwareToUse != ACC_DEFAULT && accHardwareToUse != ACC_NONE) {
        // 不存在的强制传感器 - 重新检测
        accHardwareToUse = ACC_DEFAULT;
        goto retry;
    }
 	// ---------------------------未检测到加速度计
    if (accHardware == ACC_NONE) {
        return false;
    }
	// ---------------------------加速度计在线
    detectedSensors[SENSOR_INDEX_ACC] = accHardware;
	// ---------------------------使能加速度计传感器
    sensorsSet(SENSOR_ACC);
    return true;
}

//-------------------------------------------------------------------------------------加速度计初始化相关API

/**********************************************************************
函数名称：accInitFilters
函数功能：加速度计滤波初始化
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void accInitFilters(void)
{
	// 只设置低通截止频率，如果ACC采样率检测到其他过滤器初始化没有定义(采样率 = 0)
    accLpfCutHz = (acc.sampleRateHz) ? accelerometerConfig()->acc_lpf_hz : 0;
    if (accLpfCutHz) {
		// 1*10^6 / 1000 = 1000
        const uint32_t accSampleTimeUs = 1e6 / acc.sampleRateHz;
		// 遍历3轴
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&accFilter[axis], accLpfCutHz, accSampleTimeUs);
        }
    }
}

/**********************************************************************
函数名称：accInit
函数功能：加速度计初始化
函数形参：加速度计采样速率
函数返回值：None 
函数描述：None 
**********************************************************************/
bool accInit(uint16_t accSampleRateHz)
{
    memset(&acc, 0, sizeof(acc));
    // 复制常见的陀螺mpu设置
    acc.dev.bus = *gyroSensorBus();
    acc.dev.mpuDetectionResult = *gyroMpuDetectionResult();
    acc.dev.acc_high_fsr = accelerometerConfig()->acc_high_fsr;

	// 加速度计板对齐
    sensor_align_e alignment = gyroDeviceConfig(0)->alignment;
    acc.dev.accAlign = alignment;
	
	// 加速度计检测
    if (!accDetect(&acc.dev, accelerometerConfig()->acc_hardware)) {
        return false;
    }
	// set default
    acc.dev.acc_1G = 256;
	// driver initialisation
    acc.dev.initFn(&acc.dev);  
    acc.dev.acc_1G_rec = 1.0f / acc.dev.acc_1G;

    acc.sampleRateHz = accSampleRateHz;
	// 初始化加速度计滤波
    accInitFilters();
    return true;
}

//-------------------------------------------------------------------------------------加速度计数据更新相关API

/**********************************************************************
函数名称：accUpdate
函数功能：加速度计更新
函数形参：currentTimeUs，rollAndPitchTrims
函数返回值：None 
函数描述：None 
**********************************************************************/
void accUpdate(timeUs_t currentTimeUs, rollAndPitchTrims_t *rollAndPitchTrims)
{
    UNUSED(currentTimeUs);

    if (!acc.dev.readFn(&acc.dev)) {
        return;
    }
	// 加速度计已更新
    acc.isAccelUpdatedAtLeastOnce = true;

	// 获取三轴原始加速度
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        acc.accADC[axis] = acc.dev.ADCRaw[axis];
    }

	// 滤波 - 二阶巴特沃斯低通滤波
    if (accLpfCutHz) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            acc.accADC[axis] = biquadFilterApply(&accFilter[axis], acc.accADC[axis]);
        }
    }

	// 加速度计数据板对齐
    alignSensorViaRotation(acc.accADC, acc.dev.accAlign);

	// 加速度计是否校准完成
    if (!accIsCalibrationComplete()) {
		// 执行加速度校准
        performAcclerationCalibration(rollAndPitchTrims);
    }

	// 应用加速度计校准值
    applyAccelerationTrims(accelerationTrims);

	// 加速度计测量累加
    ++accumulatedMeasurementCount;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
		// 获取三轴加速度测量值
        accumulatedMeasurements[axis] += acc.accADC[axis];
    }
}

/**********************************************************************
函数名称：accGetAccumulationAverage
函数功能：获取加速度计累积平均值
函数形参：accumulationAverage
函数返回值：是否有有加速度计数据积累
函数描述：None 
**********************************************************************/
bool accGetAccumulationAverage(float *accumulationAverage)
{
	// 如果有陀螺数据积累，计算平均速率将产生相同的旋转
    if (accumulatedMeasurementCount > 0) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = accumulatedMeasurements[axis] / accumulatedMeasurementCount;
            accumulatedMeasurements[axis] = 0.0f;
        }
        accumulatedMeasurementCount = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}
#endif

