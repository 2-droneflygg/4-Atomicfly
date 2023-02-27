/*********************************************************************************
 提供RC摇杆控制相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <math.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/feature.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "flight/failsafe.h"

#include "io/beeper.h"
#include "io/gps.h"
#include "io/vtx_control.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "rc_controls.h"

// -----------------------------------------------------------------油门行程为[1000;2000]，横摇/俯仰/偏航行程为[-500;+500]
// 存储RC命令 - rcdata->rcCommand
float rcCommand[4];          

PG_REGISTER_WITH_RESET_TEMPLATE(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);
PG_RESET_TEMPLATE(rcControlsConfig_t, rcControlsConfig,
    .deadband = 0,                       // 俯仰横滚死区区间
    .yaw_deadband = 4,                   // 偏航死区区间
    .alt_hold_deadband = 40,
    .alt_hold_fast_change = 1,
    .yaw_control_reversed = false,
);

PG_REGISTER_WITH_RESET_TEMPLATE(armingConfig_t, armingConfig, PG_ARMING_CONFIG, 1);
PG_RESET_TEMPLATE(armingConfig_t, armingConfig,
    .gyro_cal_on_first_arm = 0,          // TODO - Cleanup retarded arm support
    .auto_disarm_delay = 5
);


/**********************************************************************
函数名称：calculateThrottleStatus
函数功能：计算油门状态
函数形参：None  
函数返回值：返回油门状态
函数描述：None 
**********************************************************************/
throttleStatus_e calculateThrottleStatus(void)
{
	// 正常飞行模式
	if (rcData[THROTTLE] < rxConfig()->mincheck) {
        return THROTTLE_LOW;
    }
    return THROTTLE_HIGH;
}

/**********************************************************************
函数名称：getRcStickDeflection
函数功能：根据摇杆位置进行功能控制
函数形参：None  
函数返回值：None  
函数描述：RPYT通道
**********************************************************************/
// 摇杆延时时间
#define STICK_DELAY_MS      50
void processRcStickPositions()
{
    // RC延迟
    static int16_t rcDelayMs;
    // 保存每一步的摇杆命令
    static uint8_t rcSticks;
    static uint8_t rcDisarmTicks;
    static bool doNotRepeat;

    // --------------------------------------检查摇杆位置
    uint8_t stTmp = 0;
    for (int i = 0; i < 4; i++) {
        stTmp >>= 2;
        if (rcData[i] > rxConfig()->mincheck) {
            stTmp |= 0x80;  // 检查最小
        }
        if (rcData[i] < rxConfig()->maxcheck) {
            stTmp |= 0x40;  // 检查最大
        }
    }
    if (stTmp == rcSticks) {
        if (rcDelayMs <= INT16_MAX - (getTaskDeltaTimeUs(TASK_SELF) / 1000)) {
            rcDelayMs += getTaskDeltaTimeUs(TASK_SELF) / 1000;
        }
    } else {
        rcDelayMs = 0;
        doNotRepeat = false;
    }
    rcSticks = stTmp;


	// --------------------------------------解锁飞行器
    if (IS_RC_MODE_ACTIVE(BOXARM)) {
        rcDisarmTicks = 0;
        // 尝试解锁
        tryArm();
    } 
	// --------------------------------------锁定飞行器
	else {
		// 复位尝试解锁
        resetTryingToArm();
        // 复位解锁禁用
        resetArmingDisabled();
        if (ARMING_FLAG(ARMED) && rxIsReceivingSignal() && !failsafeIsActive()) {
            rcDisarmTicks++;
            if (rcDisarmTicks > 3) {
                disarm(DISARM_REASON_SWITCH);
            }
        }
    }

	// --------------------------------------解锁等原因后不执行下面的操作 - 直接return
    if (ARMING_FLAG(ARMED) || doNotRepeat || rcDelayMs <= STICK_DELAY_MS || (getArmingDisableFlags() & (ARMING_DISABLED_RUNAWAY_TAKEOFF | ARMING_DISABLED_CRASH_DETECTED))) {
        return;
    }
    doNotRepeat = true;

    // --------------------------------------未解锁时的摇杆快捷功能
    // 陀螺仪校准+复位家的位置+复位地面高度
    if (rcSticks == THR_LO + YAW_HI + PIT_LO + ROL_LO) {
        // GYRO calibration
        gyroStartCalibration(false);

#ifdef USE_GPS
        if (featureIsEnabled(FEATURE_GPS)) {
            GPS_reset_home_position();
        }
#endif

#ifdef USE_BARO
        if (sensors(SENSOR_BARO)) {
            baroSetGroundLevel();
        }
#endif
        return;
    }
	// 保存配置和通知
    if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_HI) {
        saveConfigAndNotify();
    }
}

/**********************************************************************
函数名称：rcControlsInit
函数功能：RC控制初始化
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
void rcControlsInit(void)
{
	// 分析模式激活条件
    analyzeModeActivationConditions();
}

