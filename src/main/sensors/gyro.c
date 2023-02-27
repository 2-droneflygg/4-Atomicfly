/*********************************************************************************
 提供陀螺仪相关API。
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

#include "config/feature.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/gyrodev.h"

#include "drivers/bus_spi.h"
#include "drivers/io.h"

#include "config/config.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"
#include "io/statusindicator.h"

#include "scheduler/scheduler.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"

// ---------------------------------------------------------陀螺仪访问
FAST_RAM_ZERO_INIT gyro_t gyro;

// ---------------------------------------------------------反自旋
#ifdef USE_YAW_SPIN_RECOVERY
// 使能状态
static FAST_RAM_ZERO_INIT bool yawSpinRecoveryEnabled;
// 阈值
static FAST_RAM_ZERO_INIT int yawSpinRecoveryThreshold;
// 检测状态
static FAST_RAM_ZERO_INIT bool yawSpinDetected;
// 时间
static FAST_RAM_ZERO_INIT timeUs_t yawSpinTimeUs;
#endif

// ---------------------------------------------------------累积测量
static FAST_RAM_ZERO_INIT float accumulatedMeasurements[XYZ_AXIS_COUNT];	
// ---------------------------------------------------------上一个陀螺仪数据
static FAST_RAM_ZERO_INIT float gyroPrevious[XYZ_AXIS_COUNT];				
// ---------------------------------------------------------累积测量数量
static FAST_RAM_ZERO_INIT int accumulatedMeasurementCount;				

// ---------------------------------------------------------PID进程分母项
FAST_RAM uint8_t activePidLoopDenom = 1;								

// ---------------------------------------------------------首次解锁是否已校准陀螺仪
static bool firstArmingCalibrationWasStarted = false;					

// ---------------------------------------------------------默认使用陀螺仪1
#ifndef GYRO_CONFIG_USE_GYRO_DEFAULT
#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1				  
#endif

PG_REGISTER_WITH_RESET_FN(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 8);
void pgResetFn_gyroConfig(gyroConfig_t *gyroConfig)
{
    gyroConfig->gyroCalibrationDuration = 125;          // 陀螺仪校准持续时间 - 125/1000000*10000 = 1.25秒
    gyroConfig->gyroMovementCalibrationThreshold = 48;  // 运动校准阈值
    gyroConfig->gyro_hardware_lpf = GYRO_HARDWARE_LPF_NORMAL;
    gyroConfig->gyro_lowpass_type = FILTER_PT1;			// 陀螺仪低通滤波器1类型
    gyroConfig->gyro_lowpass_hz = 200;  				// 陀螺仪低通滤波器1截止频率 [Hz] - 不启用，使用动态低通滤波器
    gyroConfig->gyro_lowpass2_type = FILTER_PT1;		// 螺仪低通滤波器2类型
    gyroConfig->gyro_lowpass2_hz = 188;					// 陀螺仪低通滤波器2截止频率 [Hz]
    gyroConfig->gyro_high_fsr = false;
    gyroConfig->gyro_to_use = GYRO_CONFIG_USE_GYRO_DEFAULT;
    gyroConfig->gyro_offset_yaw = 0;
    gyroConfig->yaw_spin_recovery = YAW_SPIN_RECOVERY_AUTO;
    gyroConfig->yaw_spin_threshold = 1950;
    gyroConfig->dyn_lpf_gyro_min_hz = 150;			    // 陀螺仪低通滤波器1动态最低截止频率 [Hz]
    gyroConfig->dyn_lpf_gyro_max_hz = 375;			    // 陀螺仪低通滤波器1动态最高截止频率 [Hz]
}

//-------------------------------------------------------------------------------------陀螺仪校准部分API

/**********************************************************************
函数名称：isOnFinalGyroCalibrationCycle
函数功能：陀螺仪是否校准完毕
函数形参：None 
函数返回值：状态
函数描述：None 
**********************************************************************/
bool isGyroSensorCalibrationComplete(const gyroSensor_t *gyroSensor)
{
    return gyroSensor->calibration.cyclesRemaining == 0;
}

/**********************************************************************
函数名称：gyroIsCalibrationComplete
函数功能：获取陀螺仪设备是否校准完毕
函数形参：None 
函数返回值：状态
函数描述：None 
**********************************************************************/
bool gyroIsCalibrationComplete(void)
{
    switch (gyro.gyroToUse) {
        default:
        case GYRO_CONFIG_USE_GYRO_1: {
            return isGyroSensorCalibrationComplete(&gyro.gyroSensor1);
        }
    }
}

/**********************************************************************
函数名称：gyroCalculateCalibratingCycles
函数功能：计算陀螺仪校准周期
函数形参：None 
函数返回值：陀螺仪校准周期
函数描述：None 
**********************************************************************/
static int32_t gyroCalculateCalibratingCycles(void)
{
	// 校准周期 = 125 * 10000 / 125 = 10000 
	// 125 / 1000000 * 10000 = 1.25s
    return (gyroConfig()->gyroCalibrationDuration * 10000) / gyro.sampleLooptime;
}

/**********************************************************************
函数名称：isOnFirstGyroCalibrationCycle
函数功能：是否在第一个陀螺仪校准周期
函数形参：gyroCalibration
函数返回值：状态
函数描述：None 
**********************************************************************/
static bool isOnFirstGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
	// 周期为递减
    return gyroCalibration->cyclesRemaining == gyroCalculateCalibratingCycles();
}

/**********************************************************************
函数名称：isOnFinalGyroCalibrationCycle
函数功能：是否在最后一个陀螺仪校准周期
函数形参：gyroCalibration
函数返回值：result
函数描述：None 
**********************************************************************/
static bool isOnFinalGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->cyclesRemaining == 1;
}

/**********************************************************************
函数名称：gyroSetCalibrationCycles
函数功能：设置陀螺仪校准周期
函数形参：gyroSensor
函数返回值：None 
函数描述：None 
**********************************************************************/
static void gyroSetCalibrationCycles(gyroSensor_t *gyroSensor)
{
    gyroSensor->calibration.cyclesRemaining = gyroCalculateCalibratingCycles();
}

/**********************************************************************
函数名称：gyroStartCalibration
函数功能：陀螺仪开始校准
函数形参：isFirstArmingCalibration
函数返回值：None 
函数描述：None 
**********************************************************************/
void gyroStartCalibration(bool isFirstArmingCalibration)
{
	// 判断是否已校准陀螺仪
    if (isFirstArmingCalibration && firstArmingCalibrationWasStarted) {
        return;
    }

	// 设置陀螺仪校准周期
    gyroSetCalibrationCycles(&gyro.gyroSensor1);

	// 陀螺仪正在校准
    if (isFirstArmingCalibration) {
        firstArmingCalibrationWasStarted = true;
    }
}

/**********************************************************************
函数名称：isFirstArmingGyroCalibrationRunning
函数功能：获取首次解锁陀螺仪是否正在校准
函数形参：None 
函数返回值：状态
函数描述：None 
**********************************************************************/
bool isFirstArmingGyroCalibrationRunning(void)
{
    return firstArmingCalibrationWasStarted && !gyroIsCalibrationComplete();
}

/**********************************************************************
函数名称：performGyroCalibration
函数功能：执行陀螺仪校准
函数形参：gyroSensor，运动校准阈值
函数返回值：None  
函数描述：None 
**********************************************************************/
STATIC_UNIT_TESTED void performGyroCalibration(gyroSensor_t *gyroSensor, uint8_t gyroMovementCalibrationThreshold)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // 检测是否在第一个陀螺仪校准周期 - 校准开始时进行相关复位
        if (isOnFirstGyroCalibrationCycle(&gyroSensor->calibration)) {
            gyroSensor->calibration.sum[axis] = 0.0f;
            devClear(&gyroSensor->calibration.var[axis]);
            gyroSensor->gyroDev.gyroZero[axis] = 0.0f;
        }

        // 对陀螺仪数据进行求和
        gyroSensor->calibration.sum[axis] += gyroSensor->gyroDev.gyroADCRaw[axis];
        devPush(&gyroSensor->calibration.var[axis], gyroSensor->gyroDev.gyroADCRaw[axis]);

		// 检测是否在最后一个陀螺仪校准周期
        if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
			// 标准偏差
            const float stddev = devStandardDeviation(&gyroSensor->calibration.var[axis]);

            // 判断是否超过运动校准阈值 - 为防止错误的陀螺仪零偏重新设置校准周期
            if (gyroMovementCalibrationThreshold && stddev > gyroMovementCalibrationThreshold) {
                gyroSetCalibrationCycles(gyroSensor);
                return;
            }

			// 计算陀螺仪零偏 - 均值计算
            gyroSensor->gyroDev.gyroZero[axis] = gyroSensor->calibration.sum[axis] / gyroCalculateCalibratingCycles();
            if (axis == Z) {
              	gyroSensor->gyroDev.gyroZero[axis] -= ((float)gyroConfig()->gyro_offset_yaw / 100);
            }
        }
    }

	// 检测是否在最后一个陀螺仪校准周期 - 陀螺仪校准完成提示
    if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
        if (!firstArmingCalibrationWasStarted || (getArmingDisableFlags() & ~ARMING_DISABLED_CALIBRATING) == 0) {
            beeper(BEEPER_GYRO_CALIBRATED);
        }
    }

	// 校准周期递减
    --gyroSensor->calibration.cyclesRemaining;
}

//-------------------------------------------------------------------------------------反自旋部分API

#ifdef USE_YAW_SPIN_RECOVERY
/**********************************************************************
函数名称：handleYawSpin
函数功能：处理反自旋
函数形参：currentTimeUs
函数返回值：None  
函数描述：None 
**********************************************************************/
static void handleYawSpin(timeUs_t currentTimeUs)
{
    const float yawSpinResetRate = yawSpinRecoveryThreshold - 100.0f;
    if (fabsf(gyro.gyroADCf[Z]) < yawSpinResetRate) {
        // 连续20ms
        if (cmpTimeUs(currentTimeUs, yawSpinTimeUs) > 20000) {
            yawSpinDetected = false;
        }
    } else {
        // 重置偏航旋转时间
        yawSpinTimeUs = currentTimeUs;
    }
}

/**********************************************************************
函数名称：checkForYawSpin
函数功能：检查反自旋
函数形参：currentTimeUs
函数返回值：None  
函数描述：None 
**********************************************************************/
static void checkForYawSpin(timeUs_t currentTimeUs)
{
    if (yawSpinDetected) {
		// 处理反自旋
        handleYawSpin(currentTimeUs);
    } else {
        // 检查偏航轴是否自旋
         if (abs((int)gyro.gyroADCf[Z]) > yawSpinRecoveryThreshold) {
            yawSpinDetected = true;
            yawSpinTimeUs = currentTimeUs;
        }
    }
}

/**********************************************************************
函数名称：gyroYawSpinDetected
函数功能：陀螺仪反自旋检测
函数形参：None 
函数返回值：yawSpinDetected
函数描述：None 
**********************************************************************/
bool gyroYawSpinDetected(void)
{
    return yawSpinDetected;
}

/**********************************************************************
函数名称：initYawSpinRecovery
函数功能：反自旋初始化
函数形参：maxYawRate
函数返回值：None  
函数描述：None 
**********************************************************************/
void initYawSpinRecovery(int maxYawRate)
{
    bool enabledFlag;
    int threshold;

    switch (gyroConfig()->yaw_spin_recovery) {	
	    case YAW_SPIN_RECOVERY_ON:
	        enabledFlag = true;
	        threshold = gyroConfig()->yaw_spin_threshold;
	        break;
	    case YAW_SPIN_RECOVERY_AUTO:
	        enabledFlag = true;
			// 允许25%或最低200dps超调公差
	        const int overshootAllowance = MAX(maxYawRate / 4, 200); 
	        threshold = constrain(maxYawRate + overshootAllowance, YAW_SPIN_RECOVERY_THRESHOLD_MIN, YAW_SPIN_RECOVERY_THRESHOLD_MAX);
	        break;
	    case YAW_SPIN_RECOVERY_OFF:
	    default:
	        enabledFlag = false;
	        threshold = YAW_SPIN_RECOVERY_THRESHOLD_MAX;
	        break;
    }
    yawSpinRecoveryEnabled = enabledFlag;
    yawSpinRecoveryThreshold = threshold;
}
#endif // USE_YAW_SPIN_RECOVERY

//-------------------------------------------------------------------------------------陀螺仪部分API

/**********************************************************************
函数名称：gyroUpdateSensor
函数功能：更新陀螺仪传感器
函数形参：gyroSensor
函数返回值：None  
函数描述：None 
**********************************************************************/
static void gyroUpdateSensor(gyroSensor_t *gyroSensor)
{
    if (!gyroSensor->gyroDev.readFn(&gyroSensor->gyroDev)) {
        return;
    }
    gyroSensor->gyroDev.dataReady = false;

	// 陀螺仪是否校准完毕
    if (isGyroSensorCalibrationComplete(gyroSensor)) {
        // 将16位陀螺数据移到32位变量中以避免计算中溢出 - 原始数据 - 零偏
        gyroSensor->gyroDev.gyroADC[X] = gyroSensor->gyroDev.gyroADCRaw[X] - gyroSensor->gyroDev.gyroZero[X];
        gyroSensor->gyroDev.gyroADC[Y] = gyroSensor->gyroDev.gyroADCRaw[Y] - gyroSensor->gyroDev.gyroZero[Y];
        gyroSensor->gyroDev.gyroADC[Z] = gyroSensor->gyroDev.gyroADCRaw[Z] - gyroSensor->gyroDev.gyroZero[Z];

    	// 陀螺仪数据板对齐
        alignSensorViaRotation(gyroSensor->gyroDev.gyroADC, gyroSensor->gyroDev.gyroAlign);
    } else {
    	// 执行陀螺仪校准
        performGyroCalibration(gyroSensor, gyroConfig()->gyroMovementCalibrationThreshold);
    }
}

/**********************************************************************
函数名称：gyroUpdate
函数功能：陀螺仪更新
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
void gyroUpdate(void)
{
    switch (gyro.gyroToUse) {
	    case GYRO_CONFIG_USE_GYRO_1:
			// 更新陀螺仪传感器
	        gyroUpdateSensor(&gyro.gyroSensor1);
			// 判断陀螺仪是否校准完毕
	        if (isGyroSensorCalibrationComplete(&gyro.gyroSensor1)) {
	            gyro.gyroADC[X] = gyro.gyroSensor1.gyroDev.gyroADC[X] * gyro.gyroSensor1.gyroDev.scale;
	            gyro.gyroADC[Y] = gyro.gyroSensor1.gyroDev.gyroADC[Y] * gyro.gyroSensor1.gyroDev.scale;
	            gyro.gyroADC[Z] = gyro.gyroSensor1.gyroDev.gyroADC[Z] * gyro.gyroSensor1.gyroDev.scale;
	        }
	        break;
    }

    if (gyro.downsampleFilterEnabled) {
        // 采用陀螺低通滤波器2进行采样
        gyro.sampleSum[X] = gyro.lowpass2FilterApplyFn((filter_t *)&gyro.lowpass2Filter[X], gyro.gyroADC[X]);
        gyro.sampleSum[Y] = gyro.lowpass2FilterApplyFn((filter_t *)&gyro.lowpass2Filter[Y], gyro.gyroADC[Y]);
        gyro.sampleSum[Z] = gyro.lowpass2FilterApplyFn((filter_t *)&gyro.lowpass2Filter[Z], gyro.gyroADC[Z]);
    } else {
        // 使用简单的平均采样
        gyro.sampleSum[X] += gyro.gyroADC[X];
        gyro.sampleSum[Y] += gyro.gyroADC[Y];
        gyro.sampleSum[Z] += gyro.gyroADC[Z];
        gyro.sampleCount++;
    }
}

/**********************************************************************
函数名称：gyroFiltering
函数功能：陀螺仪滤波
函数形参：currentTimeUs
函数返回值：None 
函数描述：None 
**********************************************************************/
void gyroFiltering(timeUs_t currentTimeUs)
{
	// 陀螺仪滤波 - gyro.gyroADCf[axis]
	GYRO_FILTER_FUNCTION();
	// 检测反自旋状态
#ifdef USE_YAW_SPIN_RECOVERY
    if (yawSpinRecoveryEnabled) {
		// 检查偏航反自旋
        checkForYawSpin(currentTimeUs);
    }
#endif
	// 累积测量
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // 采用梯形法则进行积分，避免偏置
        accumulatedMeasurements[axis] += 0.5f * (gyroPrevious[axis] + gyro.gyroADCf[axis]) * gyro.targetLooptime;
        gyroPrevious[axis] = gyro.gyroADCf[axis];
    }
	// 累积测量数
    accumulatedMeasurementCount++;
}

/**********************************************************************
函数名称：gyroGetAccumulationAverage
函数功能：获取陀螺仪数据累积平均值
函数形参：accumulationAverage
函数返回值：是否有陀螺仪数据累积平均值
函数描述：None 
**********************************************************************/
bool gyroGetAccumulationAverage(float *accumulationAverage)
{
	// 判断陀螺仪数据是否有累积
    if (accumulatedMeasurementCount) {
        // 计算累计测量时间
        const timeUs_t accumulatedMeasurementTimeUs = accumulatedMeasurementCount * gyro.targetLooptime;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
			// 对累积测量求平均值
            accumulationAverage[axis] = accumulatedMeasurements[axis] / accumulatedMeasurementTimeUs;
			// 复位累积测量
            accumulatedMeasurements[axis] = 0.0f;
        }
		// 复位累积测量数
        accumulatedMeasurementCount = 0;
        return true;
    } else {
		// 如果陀螺仪数据没有累积则复位累积平均值数据
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}

/**********************************************************************
函数名称：gyroAbsRateDps
函数功能：陀螺仪速率绝对值
函数形参：axis
函数返回值：陀螺仪RateDps绝对值
函数描述：None 
**********************************************************************/
uint16_t gyroAbsRateDps(int axis)
{
    return fabsf(gyro.gyroADCf[axis]);
}

//-------------------------------------------------------------------------------------陀螺仪低通滤波部分API

/**********************************************************************
函数名称：GYRO_FILTER_FUNCTION
函数功能：陀螺仪滤波功能函数
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
void GYRO_FILTER_FUNCTION(void)
{
	// 遍历RPY
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // 对单个陀螺采样进行取样
        float gyroADCf = 0;
        if (gyro.downsampleFilterEnabled) {
            // 采用陀螺低通滤波器2进行采样
            gyroADCf = gyro.sampleSum[axis];
        } else {
            // 使用简单平均值进行下采样
            if (gyro.sampleCount) {
                gyroADCf = gyro.sampleSum[axis] / gyro.sampleCount;
            }
            gyro.sampleSum[axis] = 0;
        }

        // 应用低通滤波器
        gyroADCf = gyro.lowpassFilterApplyFn((filter_t *)&gyro.lowpassFilter[axis], gyroADCf);
        gyro.gyroADCf[axis] = gyroADCf;
    }
    gyro.sampleCount = 0;
}

#ifdef USE_DYN_LPF
/**********************************************************************
函数名称：dynLpfGyroUpdate
函数功能：陀螺仪动态低通滤波更新
函数形参：throttle
函数返回值：None  
函数描述：None 
**********************************************************************/
void dynLpfGyroUpdate(float throttle)
{
    if (gyro.dynLpfFilter != DYN_LPF_NONE) {
        const unsigned int cutoffFreq = fmax(dynThrottle(throttle) * gyro.dynLpfMax, gyro.dynLpfMin);
		// PT1类型
        if (gyro.dynLpfFilter == DYN_LPF_PT1) {                
            const float gyroDt = gyro.targetLooptime * 1e-6f;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&gyro.lowpassFilter[axis].pt1FilterState, pt1FilterGain(cutoffFreq, gyroDt));
            }
        }
    }
}

/**********************************************************************
函数名称：dynThrottle
函数功能：动态油门
函数形参：throttle
函数返回值：dynThrottle
函数描述：None 
**********************************************************************/
float dynThrottle(float throttle) {
    return throttle * (1 - (throttle * throttle) / 3.0f) * 1.5f;
}
#endif

