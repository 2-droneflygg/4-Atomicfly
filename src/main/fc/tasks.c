/*********************************************************************************
 提供任务注册以及相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include "platform.h"

#include "cms/cms.h"

#include "common/utils.h"

#include "config/feature.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/vtx_common.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/beeper.h"
#include "io/gps.h"
#include "io/serial.h"
#include "io/vtx.h"
#include "io/vtx_smartaudio.h"
#include "io/msp.h"

#include "osd/osd.h"

#include "pg/rx.h"
#include "pg/motor.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "scheduler/scheduler.h"

#include "tasks.h"

//-------------------------------------------------------------------------------------相关任务函数

/**********************************************************************
函数名称：taskHandleSerial
函数功能：串行数据处理任务函数
函数形参：当前时间节拍
函数返回值：None
函数描述：
    由调度器以任务形式调用.
**********************************************************************/
static void taskHandleSerial(timeUs_t currentTimeUs)
{
	// 未使用该变量
    UNUSED(currentTimeUs);
	// 根据ARMED标志位判断处理跳过非MSP数据还是评估非MSP数据
    bool evaluateMspData = ARMING_FLAG(ARMED) ? MSP_SKIP_NON_MSP_DATA : MSP_EVALUATE_NON_MSP_DATA;
    // 串行MSP命令处理
    mspSerialProcess(evaluateMspData,mspFcProcessCommand);
}

/**********************************************************************
函数名称：taskBatteryAlerts
函数功能：电池警报任务函数
函数形参：当前时间节拍
函数返回值：None
函数描述：
	由调度器以任务形式调用.
**********************************************************************/
static void taskBatteryAlerts(timeUs_t currentTimeUs)
{
	// 未解锁且电池刚刚连接或断开 - 更新电池存在状态
    if (!ARMING_FLAG(ARMED)) {
        batteryUpdatePresence();
    }
	// 更新电池状态
    batteryUpdateStates(currentTimeUs);
	// 更新电池警报
    batteryUpdateAlarms();
}

/**********************************************************************
函数名称：taskUpdateAccelerometer
函数功能：更新加速度数据任务函数
函数形参：当前时间节拍
函数返回值：None
函数描述：
	由调度器以任务形式调用.
**********************************************************************/
#ifdef USE_ACC
static void taskUpdateAccelerometer(timeUs_t currentTimeUs)
{
	// 加速度计更新
    accUpdate(currentTimeUs, &accelerometerConfigMutable()->accelerometerTrims);
}
#endif

/**********************************************************************
函数名称：taskUpdateRxMain
函数功能：更新RC数据任务函数
函数形参：当前时间节拍
函数返回值：None
函数描述：
	由调度器以任务形式调用.
**********************************************************************/
static void taskUpdateRxMain(timeUs_t currentTimeUs)
{
	// Rx进程 - 获取rcData
    if (!processRx(currentTimeUs)) {
        return;
    }
    // 更新RC命令 - 获取rcCommand
    updateRcCommands();
	// 更新解锁状态
    updateArmingStatus();
}

/**********************************************************************
函数名称：taskUpdateBaro
函数功能：更新气压计数据任务函数
函数形参：当前时间节拍
函数返回值：None
函数描述：
	由调度器以任务形式调用.
**********************************************************************/
#ifdef USE_BARO
static void taskUpdateBaro(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (sensors(SENSOR_BARO)) {
		// 气压计更新 - 返回睡眠时间
        const uint32_t newDeadline = baroUpdate();
		// 任务周期设置为气压计睡眠时间
        if (newDeadline != 0) {
			// 重新设置任务周期
            rescheduleTask(TASK_SELF, newDeadline);
        }
    }
}
#endif

/**********************************************************************
函数名称：taskUpdateRangefinder
函数功能：高度计算任务函数
函数形参：当前时间节拍
函数返回值：None
函数描述：
	由调度器以任务形式调用.
**********************************************************************/
#if defined(USE_BARO) || defined(USE_GPS)
static void taskCalculateAltitude(timeUs_t currentTimeUs)
{
	// 计算高度和垂直速度
    calculateEstimatedAltitude(currentTimeUs);
}
#endif 

//-------------------------------------------------------------------------------------任务注册

/**********************************************************************
 任务注册格式宏定义：
**********************************************************************/
// 注册格式：检查任务函数，任务函数，执行周期，静态优先级 
#define DEFINE_TASK(checkFuncParam, taskFuncParam, desiredPeriodParam, staticPriorityParam) {  \
    .checkFunc = checkFuncParam, \
    .taskFunc = taskFuncParam, \
    .desiredPeriodUs = desiredPeriodParam, \
    .staticPriority = staticPriorityParam \
}

/**********************************************************************
 任务信息注册：
**********************************************************************/
task_t tasks[TASK_COUNT] = {
	// 系统负载计算任务 - 计算系统负载百分比
    [TASK_SYSTEM] = DEFINE_TASK(NULL, taskSystemLoad, TASK_PERIOD_HZ(10), TASK_PRIORITY_MEDIUM_HIGH),
    
	// USB任务
	[TASK_SERIAL] = DEFINE_TASK(NULL, taskHandleSerial, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOW),  

	// 电池警报任务 - 更新电池状态 && 电压警报
    [TASK_BATTERY_ALERTS] = DEFINE_TASK(NULL, taskBatteryAlerts, TASK_PERIOD_HZ(5), TASK_PRIORITY_MEDIUM),
    // 电压计任务 - 更新电池电压 
    [TASK_BATTERY_VOLTAGE] = DEFINE_TASK(NULL, batteryUpdateVoltage, TASK_PERIOD_HZ(SLOW_VOLTAGE_TASK_FREQ_HZ), TASK_PRIORITY_MEDIUM), 
	// 电流计任务 - 更新电池电流
	[TASK_BATTERY_CURRENT] = DEFINE_TASK(NULL, batteryUpdateCurrentMeter, TASK_PERIOD_HZ(50), TASK_PRIORITY_MEDIUM),

	// 陀螺仪任务 - 获取陀螺仪数据
    [TASK_GYRO] = DEFINE_TASK(NULL, taskGyroSample, TASK_GYROPID_DESIRED_PERIOD, TASK_PRIORITY_REALTIME), 
    // 滤波任务 - 陀螺仪数据滤波
    [TASK_FILTER] = DEFINE_TASK(NULL, taskFiltering, TASK_GYROPID_DESIRED_PERIOD, TASK_PRIORITY_REALTIME),
    // PID任务 - RC命令处理->PID控制器->混控输出
    [TASK_PID] = DEFINE_TASK(NULL, taskMainPidLoop, TASK_GYROPID_DESIRED_PERIOD, TASK_PRIORITY_REALTIME),
	
#ifdef USE_ACC
	// 加速度计任务 - 获取加速度数据 && 加速度校准
    [TASK_ACCEL] = DEFINE_TASK(NULL, taskUpdateAccelerometer, TASK_PERIOD_HZ(1000), TASK_PRIORITY_MEDIUM),
    // IMU任务 - 获取欧拉角
    [TASK_ATTITUDE] = DEFINE_TASK(NULL, imuUpdateAttitude, TASK_PERIOD_HZ(100), TASK_PRIORITY_MEDIUM),
#endif
	
	// RX任务(检查任务检测RX更新事件，如果基于事件的调度不起作用，则返回到周期调度) - rcData->rcCommand->更新解锁状态 
    [TASK_RX] = DEFINE_TASK(rxUpdateCheck, taskUpdateRxMain, TASK_PERIOD_HZ(33), TASK_PRIORITY_HIGH),         

#ifdef USE_BEEPER
	// 蜂鸣器任务 - 报警提示
    [TASK_BEEPER] = DEFINE_TASK(NULL, beeperUpdate, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOW),
#endif

#ifdef USE_GPS
	// GPS任务 - 获取GPS数据（如果运行在115200波特率(115字节/周期 < 256字节缓冲区)，需要防止缓冲区溢出）
    [TASK_GPS] = DEFINE_TASK(NULL, gpsUpdate, TASK_PERIOD_HZ(100), TASK_PRIORITY_MEDIUM), 			 		  
#endif

#ifdef USE_MAG
	// 磁力计任务 - 获取磁力计数据 && 磁力计校准
    [TASK_COMPASS] = DEFINE_TASK(NULL, compassUpdate,TASK_PERIOD_HZ(10), TASK_PRIORITY_LOW),
#endif

#ifdef USE_BARO
	// 气压计任务 - 获取气压计数据
    [TASK_BARO] = DEFINE_TASK(NULL, taskUpdateBaro, TASK_PERIOD_HZ(20), TASK_PRIORITY_LOW),
#endif

#if defined(USE_BARO) || defined(USE_GPS)
	// 高度任务 - 计算高度和垂直速度 && 气压计校准
    [TASK_ALTITUDE] = DEFINE_TASK(NULL, taskCalculateAltitude, TASK_PERIOD_HZ(40), TASK_PRIORITY_LOW),
#endif

#ifdef USE_OSD
	// OSD任务 - 视频字符信息叠加
    [TASK_OSD] = DEFINE_TASK(NULL, osdUpdate, TASK_PERIOD_HZ(60), TASK_PRIORITY_LOW),
#endif

#ifdef USE_CMS
	// OSD菜单任务 - CMS菜单管理
    [TASK_CMS] = DEFINE_TASK(NULL, cmsHandler, TASK_PERIOD_HZ(20), TASK_PRIORITY_LOW),
#endif

#ifdef USE_VTX_CONTROL
	// 图传控制任务 - 控制图传频组、频点、功率、PIT模式
    [TASK_VTXCTRL] = DEFINE_TASK(NULL, vtxUpdate, TASK_PERIOD_HZ(5), TASK_PRIORITY_IDLE),
#endif
};

//-------------------------------------------------------------------------------------任务初始化

/**********************************************************************
函数名称：tasksInit
函数功能：任务初始化
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void tasksInit(void)
{
// -----------------------------------------------------------------任务调度器初始化
    schedulerInit();

// -----------------------------------------------------------------USB任务
    setTaskEnabled(TASK_SERIAL, true);
    rescheduleTask(TASK_SERIAL, TASK_PERIOD_HZ(serialConfig()->serial_update_rate_hz));
	
// -----------------------------------------------------------------电池电压检测任务
    const bool useBatteryVoltage = batteryConfig()->voltageMeterSource != VOLTAGE_METER_NONE;
    setTaskEnabled(TASK_BATTERY_VOLTAGE, useBatteryVoltage);

// -----------------------------------------------------------------电池电流计任务
    const bool useBatteryCurrent = batteryConfig()->currentMeterSource != CURRENT_METER_NONE;
    setTaskEnabled(TASK_BATTERY_CURRENT, useBatteryCurrent);

// -----------------------------------------------------------------电池电压警报任务
    const bool useBatteryAlerts = batteryConfig()->useVBatAlerts || batteryConfig()->useConsumptionAlerts || featureIsEnabled(FEATURE_OSD);
    setTaskEnabled(TASK_BATTERY_ALERTS, (useBatteryVoltage || useBatteryCurrent) && useBatteryAlerts);

// -----------------------------------------------------------------陀螺仪、滤波、PID任务
    if (sensors(SENSOR_GYRO)) {
        rescheduleTask(TASK_GYRO, gyro.sampleLooptime);    // 8K（125）
        rescheduleTask(TASK_FILTER, gyro.targetLooptime);  // 8K（125）
        rescheduleTask(TASK_PID, gyro.targetLooptime);     // 8K（125）
        setTaskEnabled(TASK_GYRO, true);
        setTaskEnabled(TASK_FILTER, true);
        setTaskEnabled(TASK_PID, true);
		// 使能陀螺仪任务调度 - gyroEnabled
        schedulerEnableGyro();
    }

// -----------------------------------------------------------------加速度计、IMU任务
#if defined(USE_ACC)
    if (sensors(SENSOR_ACC) && acc.sampleRateHz) {
        setTaskEnabled(TASK_ACCEL, true);
        rescheduleTask(TASK_ACCEL, TASK_PERIOD_HZ(acc.sampleRateHz));
        setTaskEnabled(TASK_ATTITUDE, true);	
    }
#endif

// -----------------------------------------------------------------接收机任务
    setTaskEnabled(TASK_RX, true);

// -----------------------------------------------------------------蜂鸣器任务
#ifdef USE_BEEPER
    setTaskEnabled(TASK_BEEPER, true);
#endif

// -----------------------------------------------------------------GPS任务
#ifdef USE_GPS
    setTaskEnabled(TASK_GPS, featureIsEnabled(FEATURE_GPS));
#endif

// -----------------------------------------------------------------磁力计任务
#ifdef USE_MAG
    setTaskEnabled(TASK_COMPASS, sensors(SENSOR_MAG));
#endif

// -----------------------------------------------------------------气压计任务
#ifdef USE_BARO
    setTaskEnabled(TASK_BARO, sensors(SENSOR_BARO));
#endif

// -----------------------------------------------------------------高度计算任务
#if defined(USE_BARO) || defined(USE_GPS)
    setTaskEnabled(TASK_ALTITUDE, sensors(SENSOR_BARO) || featureIsEnabled(FEATURE_GPS));
#endif

// -----------------------------------------------------------------OSD任务
#ifdef USE_OSD
    setTaskEnabled(TASK_OSD, featureIsEnabled(FEATURE_OSD) && osdInitialized());
#endif

// -----------------------------------------------------------------CMS OSD菜单管理任务
#ifdef USE_CMS
    setTaskEnabled(TASK_CMS, featureIsEnabled(FEATURE_OSD));
#endif

// -----------------------------------------------------------------图传控制任务
#ifdef USE_VTX_CONTROL
#if defined(USE_VTX_SMARTAUDIO)
    setTaskEnabled(TASK_VTXCTRL, true);
#endif
#endif
}

//-------------------------------------------------------------------------------------获取任务信息API

/**********************************************************************
函数名称：getTask
函数功能：获取任务信息结构体
函数形参：任务ID
函数返回值：返回任务结构体指针
函数描述：None
**********************************************************************/
task_t *getTask(unsigned taskId)
{
    return &tasks[taskId];
}

