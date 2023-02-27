/**********************************************************************
 系统相关核心功能API函数：
**********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "cms/cms.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/light_led.h"
#include "drivers/motor.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"

#include "io/beeper.h"
#include "io/gps.h"
#include "io/serial.h"
#include "io/statusindicator.h"
#include "io/vtx_control.h"

#include "osd/osd.h"

#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "core.h"

/* ------------------------------解锁延迟枚举------------------------------ */	
enum {
    ARMING_DELAYED_DISARMED = 0,
    ARMING_DELAYED_NORMAL = 1,
    ARMING_DELAYED_CRASHFLIP = 2,
};
	
// ---------------------------------------------------------尝试解锁
static int tryingToArm = ARMING_DELAYED_DISARMED;	 

// ---------------------------------------------------------当“解锁无怠速”被启用并且auto_disarm_delay不为零时自动解除时间
static timeUs_t disarmAt;    					 

// ---------------------------------------------------------上一个解锁禁用原因
static int lastArmingDisabledReason = 0;			  
// ---------------------------------------------------------上一个解锁禁用时间
static timeUs_t lastDisarmTimeUs;					 

// ---------------------------------------------------------PID更新次数
static FAST_RAM_ZERO_INIT uint8_t pidUpdateCounter;	   

// ---------------------------------------------------------反乌龟激活状态
static bool flipOverAfterCrashActive = false;		   

// ---------------------------------------------------------失控起飞
#ifdef USE_RUNAWAY_TAKEOFF
#define RUNAWAY_TAKEOFF_PIDSUM_THRESHOLD         600   	// 触发所需的pidSum阈值
#define RUNAWAY_TAKEOFF_ACTIVATE_DELAY           75000 	// (75ms) pidSum超过阈值触发的时间(以微秒为单位)
#define RUNAWAY_TAKEOFF_DEACTIVATE_STICK_PERCENT 15    	// 15% -在钝化阶段，最小摇杆偏斜
#define RUNAWAY_TAKEOFF_DEACTIVATE_PIDSUM_LIMIT  100   	// 10.0% - 失活阶段的pidSum限制
#define RUNAWAY_TAKEOFF_GYRO_LIMIT_RP            15    	// 滚转/俯仰15度/秒阈值限制，在调试架测试中防止触发
#define RUNAWAY_TAKEOFF_GYRO_LIMIT_YAW           50    	// 偏航50度/秒阈值，防止在无支柱的支架测试中触发
#define RUNAWAY_TAKEOFF_HIGH_THROTTLE_PERCENT    75    	// 加速取消激活的高油门限制(一半的取消激活延迟)
#endif
#ifdef USE_RUNAWAY_TAKEOFF
static timeUs_t runawayTakeoffDeactivateUs = 0;
static timeUs_t runawayTakeoffAccumulatedUs = 0;
static bool runawayTakeoffCheckDisabled = false;
static timeUs_t runawayTakeoffTriggerUs = 0;
static bool runawayTakeoffTemporarilyDisabled = false;
#endif

// ---------------------------------------------------------磁力计保持
#if defined(USE_GPS) || defined(USE_MAG)
int16_t magHold;
#endif

//-------------------------------------------------------------------------------------传感器校准相关API

/**********************************************************************
函数名称：isCalibrating
函数功能：校获传感器取准状态
函数形参：None
函数返回值：校准状态
函数描述：None
**********************************************************************/
static bool isCalibrating(void)
{
    return (sensors(SENSOR_GYRO) && !gyroIsCalibrationComplete())
#ifdef USE_ACC
        || (sensors(SENSOR_ACC) && !accIsCalibrationComplete())
#endif
#ifdef USE_BARO
        || (sensors(SENSOR_BARO) && !baroIsCalibrationComplete())
#endif
#ifdef USE_MAG
        || (sensors(SENSOR_MAG) && !compassIsCalibrationComplete())
#endif
        ;
}

/**********************************************************************
函数名称：accNeedsCalibration
函数功能：获取加速度计是否有校准需求
函数形参：None
函数返回值：状态
函数描述：None
**********************************************************************/
#ifdef USE_ACC
static bool accNeedsCalibration(void)
{
    if (sensors(SENSOR_ACC)) {
        // 检查ACC是否已经校准
        if (accHasBeenCalibrated()) {
            return false;
        }
		// 已经确定有一个未检测到的ACC未校准检查是否有东西在使用ACC会因缺少校准而受到影响
        // 检查任何使用ACC的配置模式
        if (isModeActivationConditionPresent(BOXANGLE) ||
            isModeActivationConditionPresent(BOXGPSRESCUE)) {
            return true;
        }
#ifdef USE_OSD
        // 检查是否有需要ACC的已启用OSD元素
        if (featureIsEnabled(FEATURE_OSD)) {
            if (osdNeedsAccelerometer()) {
                return true;
            }
        }
#endif
#ifdef USE_GPS_RESCUE
        // 检查失控保护是否使用GPS救援
        if (failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE) {
            return true;
        }
#endif
    }
    return false;
}
#endif

//-------------------------------------------------------------------------------------解锁相关API

/**********************************************************************
函数名称：resetArmingDisabled
函数功能：复位解锁禁用
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void resetArmingDisabled(void)
{
    lastArmingDisabledReason = 0;
}

/**********************************************************************
函数名称：updateArmingStatus
函数功能：更新解锁状态
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void updateArmingStatus(void)
{
	// 解锁状态
    if (ARMING_FLAG(ARMED)) {
		// LED常亮
        LED0_ON;
    } 
	// 未解锁状态
	else {
        // 检查电源开启解锁的宽限时间是否已过
        if ((getArmingDisableFlags() & ARMING_DISABLED_BOOT_GRACE_TIME) && (millis() >= systemConfig()->powerOnArmingGraceTime * 1000)
#ifdef USE_DSHOT
			// 需要防止解锁，直到它可以发送DSHOT命令，否则，如果初始解锁是在崩溃翻转电机方向命令，可能不会被发送
            && (!isMotorProtocolDshot() || dshotCommandsAreEnabled(DSHOT_CMD_TYPE_INLINE))
#endif
        ) {
            // 如果是，取消设置ARMING_DISABLED_BOOT_GRACE_TIME解锁禁用标志
            unsetArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);
        }
        // 复位反乌龟激活状态
        flipOverAfterCrashActive = false;
        // 如果开关被用于禁止解锁，那么当RX链路从故障中恢复时，检查它是否默认为on
        if (true) {
            static bool hadRx = false;
            const bool haveRx = rxIsReceivingSignal();

            const bool justGotRxBack = !hadRx && haveRx;

            if (justGotRxBack && IS_RC_MODE_ACTIVE(BOXARM)) {
                // 如果RX刚刚开始再次接收信号，解锁开关是打开的，应用禁止解锁
                setArmingDisabled(ARMING_DISABLED_BAD_RX_RECOVERY);
            } else if (haveRx && !IS_RC_MODE_ACTIVE(BOXARM)) {
                // 如果RX信号正常，解锁开关关闭，解除禁止解锁
                unsetArmingDisabled(ARMING_DISABLED_BAD_RX_RECOVERY);
            }

            hadRx = haveRx;
        }
		// 失控保护模式检测
        if (IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
            setArmingDisabled(ARMING_DISABLED_BOXFAILSAFE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_BOXFAILSAFE);
        }
		// 解锁低油门检测
        if (calculateThrottleStatus() != THROTTLE_LOW) {
            setArmingDisabled(ARMING_DISABLED_THROTTLE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_THROTTLE);
        }
		// 判断IMU正确性并且未开启反乌龟模式
        if (!isUpright() && !IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH)) {
            setArmingDisabled(ARMING_DISABLED_ANGLE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_ANGLE);
        }
		// 系统负载百分比检测
        if (getAverageSystemLoadPercent() > LOAD_PERCENTAGE_ONE) {
            setArmingDisabled(ARMING_DISABLED_LOAD);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_LOAD);
        }
		// 传感器校准状态检测
        if (isCalibrating()) {
            setArmingDisabled(ARMING_DISABLED_CALIBRATING);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_CALIBRATING);
        }
		// GPS救援模式是否配置
#ifdef USE_GPS_RESCUE
        if (gpsRescueIsConfigured()) {
            if (gpsRescueConfig()->allowArmingWithoutFix || STATE(GPS_FIX) || ARMING_FLAG(WAS_EVER_ARMED) || IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH)) {
                unsetArmingDisabled(ARMING_DISABLED_GPS);
            } else {
                setArmingDisabled(ARMING_DISABLED_GPS);
            }
            if (IS_RC_MODE_ACTIVE(BOXGPSRESCUE)) {
                setArmingDisabled(ARMING_DISABLED_RESC);
            } else {
                unsetArmingDisabled(ARMING_DISABLED_RESC);
            }
        }
#endif
		// 加速度校准检测
#ifdef USE_ACC
        if (accNeedsCalibration()) {
            setArmingDisabled(ARMING_DISABLED_ACC_CALIBRATION);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_ACC_CALIBRATION);
        }
#endif
		// 电机协议配置检测
        if (!isMotorProtocolEnabled()) {
            setArmingDisabled(ARMING_DISABLED_MOTOR_PROTOCOL);
        }
		// BOXARM模式是否被激活
        if (true) {
            if (!IS_RC_MODE_ACTIVE(BOXARM)) {
#ifdef USE_RUNAWAY_TAKEOFF
                unsetArmingDisabled(ARMING_DISABLED_RUNAWAY_TAKEOFF);
#endif
                unsetArmingDisabled(ARMING_DISABLED_CRASH_DETECTED);
            }

            // 如果我们要校准陀螺仪first_arm,忽略ARMING_DISABLED_CALIBRATING
            bool ignoreGyro = armingConfig()->gyro_cal_on_first_arm
                && !(getArmingDisableFlags() & ~(ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_CALIBRATING));
			
            // 如果禁止解锁是无效的，解锁开关是on
            if (isArmingDisabled() && !ignoreGyro  && IS_RC_MODE_ACTIVE(BOXARM)) {
                setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            } else if (!IS_RC_MODE_ACTIVE(BOXARM)) {
                unsetArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            }
        }
		// 获取解锁失能标志位状态
        if (isArmingDisabled()) {
            warningLedFlash();
        } else {
        	// 警告LED失能
            warningLedDisable();
        }
		// 警告LED更新
        warningLedUpdate();
    }
}

/**********************************************************************
函数名称：disarm
函数功能：禁止解锁
函数形参：飞行日志禁止解锁原因
函数返回值：None
函数描述：None
**********************************************************************/
void disarm(flightLogDisarmReason_e reason)
{
	// 解锁状态
    if (ARMING_FLAG(ARMED)) {
        if (!flipOverAfterCrashActive) {
            ENABLE_ARMING_FLAG(WAS_EVER_ARMED);
        }
        DISABLE_ARMING_FLAG(ARMED);
        lastDisarmTimeUs = micros();
#ifdef USE_OSD
        if (flipOverAfterCrashActive) {
            osdSuppressStats(true);
        }
#endif
        UNUSED(reason);
        BEEP_OFF;
#ifdef USE_DSHOT
        if (isMotorProtocolDshot() && flipOverAfterCrashActive ) {
            dshotCommandWrite(ALL_MOTORS, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_NORMAL, DSHOT_CMD_TYPE_INLINE);
        }
#endif
        flipOverAfterCrashActive = false;
        // 如果arming_disabled_runaway_TAKEOFF被设置，播放它的哔哔模式代替
        if (!(getArmingDisableFlags() & (ARMING_DISABLED_RUNAWAY_TAKEOFF | ARMING_DISABLED_CRASH_DETECTED))) {
            beeper(BEEPER_DISARMING);      // 发出解除beeper
        }
    }
}

/**********************************************************************
函数名称：tryArm
函数功能：尝试解锁
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void tryArm(void)
{
	// 判断陀螺仪是否校准
    if (armingConfig()->gyro_cal_on_first_arm) {
        gyroStartCalibration(true);
    }

	// 更新解锁状态 - 设置解锁禁用
    updateArmingStatus();

	// ------------------------------------------------解锁禁用未开启
    if (!isArmingDisabled()) {
		// 如果已解锁 - 则直接return
        if (ARMING_FLAG(ARMED)) {
            return;
        }

        const timeUs_t currentTimeUs = micros();

		// 反乌龟相关
#ifdef USE_DSHOT
		// Dshot信标命令时差 < DSHOT信标延迟
        if (currentTimeUs - getLastDshotBeaconCommandTimeUs() < DSHOT_BEACON_GUARD_DELAY_US) {
            if (tryingToArm == ARMING_DELAYED_DISARMED) {
				// 判断是否激活反乌龟模式
                if (IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH)) {
                    tryingToArm = ARMING_DELAYED_CRASHFLIP;
                } else {
                    tryingToArm = ARMING_DELAYED_NORMAL;
                }
            }
            return;
        }
        if (isMotorProtocolDshot() && isModeActivationConditionPresent(BOXFLIPOVERAFTERCRASH)) {
            if (!(IS_RC_MODE_ACTIVE(BOXFLIPOVERAFTERCRASH) || (tryingToArm == ARMING_DELAYED_CRASHFLIP))) {
                flipOverAfterCrashActive = false;
				// 写Dshot命令 - 内联发送
                dshotCommandWrite(ALL_MOTORS, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_NORMAL, DSHOT_CMD_TYPE_INLINE);
            } else {
                flipOverAfterCrashActive = true;
#ifdef USE_RUNAWAY_TAKEOFF
                runawayTakeoffCheckDisabled = false;
#endif			
				// 写Dshot命令 - 内联发送
                dshotCommandWrite(ALL_MOTORS, getMotorCount(), DSHOT_CMD_SPIN_DIRECTION_REVERSED, DSHOT_CMD_TYPE_INLINE);
            }
        }
#endif

#ifdef USE_OSD
		// 关闭osd抑制统计数据
        osdSuppressStats(false);
#endif
		// 使能解锁标志位
        ENABLE_ARMING_FLAG(ARMED);

		// 复位尝试解锁
        resetTryingToArm();

		// 启动解除超时，将延长油门非零
        disarmAt = currentTimeUs + armingConfig()->auto_disarm_delay * 1e6;    
        lastArmingDisabledReason = 0;

#ifdef USE_GPS
		// 更新家的位置
        GPS_reset_home_position();

        // 哔哔声表示解锁
        if (featureIsEnabled(FEATURE_GPS)) {
            if (STATE(GPS_FIX) && gpsSol.numSat >= 5) {
                beeper(BEEPER_ARMING_GPS_FIX);
            } else {
                beeper(BEEPER_ARMING_GPS_NO_FIX);
            }
        } else {
            beeper(BEEPER_ARMING);
        }
#else
        beeper(BEEPER_ARMING);
#endif

#ifdef USE_RUNAWAY_TAKEOFF
        runawayTakeoffDeactivateUs = 0;
        runawayTakeoffAccumulatedUs = 0;
        runawayTakeoffTriggerUs = 0;
#endif
    }
	// ------------------------------------------------解锁禁用开启
	else {
	   // 复位尝试解锁
       resetTryingToArm();
	    // 首次解锁陀螺仪是否正在校准
        if (!isFirstArmingGyroCalibrationRunning()) {
			// 获取禁止解锁理由
			// ffs函数：查找一个整数中的第一个置位值(也就是bit为1的位)
			// 比如 0,1,2,4,8,16,32,64
			//      0,1,2,3,4, 5, 6, 7
            int armingDisabledReason = ffs(getArmingDisableFlags());
			// 判断前后两次理由是否一致
            if (lastArmingDisabledReason != armingDisabledReason) {
				// 不一致 - 更新理由
                lastArmingDisabledReason = armingDisabledReason;
				// 蜂鸣器警告哔哔声
                beeperWarningBeeps(armingDisabledReason);
            }
        }
    }
}

/**********************************************************************
函数名称：getLastDisarmTimeUs
函数功能：获取最后禁止解锁时间
函数形参：None
函数返回值：lastDisarmTimeUs
函数描述：None
**********************************************************************/
timeUs_t getLastDisarmTimeUs(void)
{
    return lastDisarmTimeUs;
}

/**********************************************************************
函数名称：getLastDisarmTimeUs
函数功能：获取试图解锁状态
函数形参：None
函数返回值：tryingToArm != ARMING_DELAYED_DISARMED
函数描述：None
**********************************************************************/
bool isTryingToArm()
{
    return (tryingToArm != ARMING_DELAYED_DISARMED);
}

/**********************************************************************
函数名称：resetTryingToArm
函数功能：复位尝试解锁
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void resetTryingToArm()
{
    tryingToArm = ARMING_DELAYED_DISARMED;
}

//-------------------------------------------------------------------------------------MagHold相关API

/**********************************************************************
函数名称：updateMagHold
函数功能：更新磁力计保持
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
#if defined(USE_GPS) || defined(USE_MAG)
static void updateMagHold(void)
{
    if (fabsf(rcCommand[YAW]) < 15 && FLIGHT_MODE(MAG_MODE)) {
        int16_t dif = DECIDEGREES_TO_DEGREES(attitude.values.yaw) - magHold;
        if (dif <= -180)
            dif += 360;
        if (dif >= +180)
            dif -= 360;
        dif *= -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
		// 判断IMU正确性
        if (isUpright()) {
            rcCommand[YAW] -= dif * currentPidProfile->pid[PID_MAG].P / 30;    // 18 deg
        }
    } else
        magHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
}
#endif

//-------------------------------------------------------------------------------------R/P/Y杆偏差相关API

/**********************************************************************
函数名称：areSticksActive
函数功能：确定R/P/Y杆偏差是否超过限制
函数形参：None
函数返回值：超过限制true，未超过限制false
函数描述：None
**********************************************************************/
#if defined(USE_RUNAWAY_TAKEOFF) || defined(USE_GPS_RESCUE)
bool areSticksActive(uint8_t stickPercentLimit)
{
    for (int axis = FD_ROLL; axis <= FD_YAW; axis ++) {
        const uint8_t deadband = axis == FD_YAW ? rcControlsConfig()->yaw_deadband : rcControlsConfig()->deadband;
        uint8_t stickPercent = 0;
        if ((rcData[axis] >= PWM_RANGE_MAX) || (rcData[axis] <= PWM_RANGE_MIN)) {
            stickPercent = 100;
        } else {
            if (rcData[axis] > (rxConfig()->midrc + deadband)) {
                stickPercent = ((rcData[axis] - rxConfig()->midrc - deadband) * 100) / (PWM_RANGE_MAX - rxConfig()->midrc - deadband);
            } else if (rcData[axis] < (rxConfig()->midrc - deadband)) {
                stickPercent = ((rxConfig()->midrc - deadband - rcData[axis]) * 100) / (rxConfig()->midrc - deadband - PWM_RANGE_MIN);
            }
        }
        if (stickPercent >= stickPercentLimit) {
            return true;
        }
    }
    return false;
}
#endif

//-------------------------------------------------------------------------------------反乌龟相关API

/**********************************************************************
函数名称：isFlipOverAfterCrashActive
函数功能：获取反乌龟激活状态
函数形参：None
函数返回值：flipOverAfterCrashActive
函数描述：None
**********************************************************************/
bool isFlipOverAfterCrashActive(void)
{
    return flipOverAfterCrashActive;
}

//-------------------------------------------------------------------------------------失控起飞相关API

/**********************************************************************
函数名称：runawayTakeoffTemporaryDisable
函数功能：允许暂时禁用失控起飞
函数形参：None
函数返回值：true
函数描述：None
**********************************************************************/
#ifdef USE_RUNAWAY_TAKEOFF
void runawayTakeoffTemporaryDisable(uint8_t disableFlag)
{
    runawayTakeoffTemporarilyDisabled = disableFlag;
}
#endif


//-------------------------------------------------------------------------------------油门相关API

/**********************************************************************
函数名称：calculateThrottlePercent
函数功能：计算油门百分比
函数形参：None
函数返回值：油门百分比
函数描述：None
**********************************************************************/
int8_t calculateThrottlePercent(void)
{
    uint8_t ret = 0;
	// 对油门数据进行约束
    int channelData = constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX);
	// 计算百分比并约束
    ret = constrain(((channelData - rxConfig()->mincheck) * 100) / (PWM_RANGE_MAX - rxConfig()->mincheck), 0, 100);
    return ret;
}

/**********************************************************************
函数名称：calculateThrottlePercentAbs
函数功能：计算油门百分比绝对值
函数形参：None
函数返回值：ABS(calculateThrottlePercent())
函数描述：None
**********************************************************************/
uint8_t calculateThrottlePercentAbs(void)
{
    return ABS(calculateThrottlePercent());
}

/**********************************************************************
函数名称：isAirmodeActivated
函数功能：获取空中模式激活状态
函数形参：None
函数返回值：airmodeIsActivated
函数描述：None
**********************************************************************/
static bool airmodeIsActivated;
bool isAirmodeActivated()
{
    return airmodeIsActivated;
}

//-------------------------------------------------------------------------------------RX相关API

/**********************************************************************
函数名称：processRx
函数功能：Rx进程
函数形参：currentTimeUs
函数返回值：true
函数描述：	
	由taskUpdateRxMain主RX进程调用.
**********************************************************************/
bool processRx(timeUs_t currentTimeUs)
{
    static bool armedBeeperOn = false;								// 解锁蜂鸣器开启状态

    timeDelta_t frameAgeUs;											// 帧年龄（us）
    timeDelta_t frameDeltaUs = rxGetFrameDelta(&frameAgeUs);		// 帧增量（us）

	// 计算Rx通道和更新故障保护（未失控则得到rcData） - 如果不需要处理rx数据直接返回false
    if (!calculateRxChannelsAndUpdateFailsafe(currentTimeUs)) {
        return false;
    }

	// 更新Rc刷新率
    updateRcRefreshRate(currentTimeUs);

	// 更新RSSI
    updateRSSI(currentTimeUs);

	// 判断是否开启失控保护监控
    if (currentTimeUs > FAILSAFE_POWER_ON_DELAY_US && !failsafeIsMonitoring()) {
		// 如果未监控则失控保护开始监控
        failsafeStartMonitoring();
    }

	// 更新失控保护状态 - 需要确保失控保护状态开启，否则直接return
    failsafeUpdateState();

	// 获取油门状态
    const throttleStatus_e throttleStatus = calculateThrottleStatus();
	// 获取油门百分比绝对值
    const uint8_t throttlePercent = calculateThrottlePercentAbs();

	// 空中模式使能 && 解锁
    if (airmodeIsEnabled() && ARMING_FLAG(ARMED)) {
		// 油门 >= Airmode激活阈值
        if (throttlePercent >= rxConfig()->airModeActivateThreshold) {
			// 置位空中模式状态
            airmodeIsActivated = true; 
        }
    } else {
    	// 复位空中模式状态
        airmodeIsActivated = false;
    }

	// 低油门 && 空中模式未激活 - I项应该防止累积，以免在地面上缠绕
    if (throttleStatus == THROTTLE_LOW && !airmodeIsActivated) {
		// 设置 PID I项复位
        pidSetItermReset(true);
    } else {
        pidSetItermReset(false);
    }

#ifdef USE_RUNAWAY_TAKEOFF
	// 如果启用runaway_takeoff_prevention，则累积该油门时间
	// 高于runaway_takeoff_deactivate_throttle，同时任何R/P/Y杆偏转，至少runaway_takeoff_stick_percent %，同时所有轴上的pidSum保持低。
	// 一旦累计时间超过runaway_takeoff_deactivate_delay，则禁用防止电池的剩余部分。
    if (ARMING_FLAG(ARMED)
        && pidConfig()->runaway_takeoff_prevention
        && !runawayTakeoffCheckDisabled
        && !flipOverAfterCrashActive
        && !runawayTakeoffTemporarilyDisabled) {
		// 确定是否在飞行中：
		// -马达在运转
		// -油门超过runaway_takeoff_deactivate_throttle_percent
		// -摇杆是激活的，并且有大于runaway_takeoff_deactivate_stick_percent的偏差
		// -所有轴上的pidSum小于runaway_takeoff_deactivate_pidlimit
        bool inStableFlight = false;
		// AIRmode开启 || 不在低油门
        if (airmodeIsEnabled() || (throttleStatus != THROTTLE_LOW)) { 
            const uint8_t lowThrottleLimit = pidConfig()->runaway_takeoff_deactivate_throttle;
            const uint8_t midThrottleLimit = constrain(lowThrottleLimit * 2, lowThrottleLimit * 2, RUNAWAY_TAKEOFF_HIGH_THROTTLE_PERCENT);
            if ((((throttlePercent >= lowThrottleLimit) && areSticksActive(RUNAWAY_TAKEOFF_DEACTIVATE_STICK_PERCENT)) || (throttlePercent >= midThrottleLimit))
                && (fabsf(pidData[FD_PITCH].Sum) < RUNAWAY_TAKEOFF_DEACTIVATE_PIDSUM_LIMIT)
                && (fabsf(pidData[FD_ROLL].Sum) < RUNAWAY_TAKEOFF_DEACTIVATE_PIDSUM_LIMIT)
                && (fabsf(pidData[FD_YAW].Sum) < RUNAWAY_TAKEOFF_DEACTIVATE_PIDSUM_LIMIT)) {
                // 正在飞行中
                inStableFlight = true;
                if (runawayTakeoffDeactivateUs == 0) {
                    runawayTakeoffDeactivateUs = currentTimeUs;
                }
            }
        }

        // 如果正在飞行，那么累积时间，当它超过runaway_takeoff_deactivate_delay毫秒时取消激活
        if (inStableFlight) {
            if (runawayTakeoffDeactivateUs == 0) {
                runawayTakeoffDeactivateUs = currentTimeUs;
            }
            uint16_t deactivateDelay = pidConfig()->runaway_takeoff_deactivate_delay;
            // 在高油门水平减少50%的取消激活延迟
            if (throttlePercent >= RUNAWAY_TAKEOFF_HIGH_THROTTLE_PERCENT) {
                deactivateDelay = deactivateDelay / 2;
            }
            if ((cmpTimeUs(currentTimeUs, runawayTakeoffDeactivateUs) + runawayTakeoffAccumulatedUs) > deactivateDelay * 1000) {
                runawayTakeoffCheckDisabled = true;
            }

        } 
		// 未在飞行中
		else {
            if (runawayTakeoffDeactivateUs != 0) {
                runawayTakeoffAccumulatedUs += cmpTimeUs(currentTimeUs, runawayTakeoffDeactivateUs);
            }
            runawayTakeoffDeactivateUs = 0;
        }

    } 
#endif

	// 当解锁和电机不旋转，做哔哔声，然后禁止解锁，混控表约束电机命令，因此检查油门状态就足够
    const timeUs_t autoDisarmDelayUs = armingConfig()->auto_disarm_delay * 1e6;
    if (ARMING_FLAG(ARMED) && !airmodeIsEnabled() && !FLIGHT_MODE(GPS_RESCUE_MODE))  // 当GPS救援激活时禁用自动禁止解锁
	{
        // 解锁通过AUX开关;低油门时哔声
        if (throttleStatus == THROTTLE_LOW) {
            beeper(BEEPER_ARMED);
            armedBeeperOn = true;
        } else if (armedBeeperOn) {
            beeperSilence();
            armedBeeperOn = false;
        }
        
    } else {
        disarmAt = currentTimeUs + autoDisarmDelayUs;  // 延长自动禁止解锁时间
    }

	// 根据摇杆位置进行功能控制 - 解锁/锁定、摇杆快捷功能
    if ( true
#ifdef USE_CMS
        && !cmsInMenu
#endif
        ) {
        processRcStickPositions();
    }

	// 更新激活模式
    updateActivatedModes();

#ifdef USE_DSHOT
    // 当crash flip模式激活时启用beep warning
    if (flipOverAfterCrashActive) {
        beeper(BEEPER_CRASH_FLIP_MODE);
    }
#endif

	// 无振荡转换到自稳模式
    if ((IS_RC_MODE_ACTIVE(BOXANGLE) || failsafeIsActive()) && (sensors(SENSOR_ACC))) {
        if (!FLIGHT_MODE(ANGLE_MODE)) {
            ENABLE_FLIGHT_MODE(ANGLE_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(ANGLE_MODE); // 失控保护支持
    }
		
	// GPS救援模式
#ifdef USE_GPS_RESCUE
    if (ARMING_FLAG(ARMED) && (IS_RC_MODE_ACTIVE(BOXGPSRESCUE) || (failsafeIsActive() && failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE))) {
        if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
            ENABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
    }
#endif

	// 自稳模式
    if (FLIGHT_MODE(ANGLE_MODE)) {
        // 在自稳模式下，增加姿态任务的频率以减少漂移
        rescheduleTask(TASK_ATTITUDE, TASK_PERIOD_HZ(500));
    } else {
        rescheduleTask(TASK_ATTITUDE, TASK_PERIOD_HZ(100));
    }

	// 更新图传激活通道
#ifdef USE_VTX_CONTROL
    vtxUpdateActivatedChannel();
#endif

	// 设置PID反重力状态 - 是否激活
    pidSetAntiGravityState(featureIsEnabled(FEATURE_ANTI_GRAVITY));

    return true;
}

/**********************************************************************
函数名称：subTaskRcCommand
函数功能：RC命令子任务
函数形参：当前节拍时间
函数返回值：true
函数描述：None
**********************************************************************/
static void subTaskRcCommand(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
	// RC命令进程
    processRcCommand();
}

//-------------------------------------------------------------------------------------陀螺仪相关API

/**********************************************************************
函数名称：taskGyroSample
函数功能：陀螺仪采样子任务
函数形参：当前节拍时间
函数返回值：true
函数描述：
	由调度器以任务形式调用.
**********************************************************************/
void taskGyroSample(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
	// 陀螺仪更新
    gyroUpdate();
	// 保证陀螺仪测量值和PID循环一致
    if (pidUpdateCounter % activePidLoopDenom == 0) {
        pidUpdateCounter = 0;
    }
	// PID更新计数
    pidUpdateCounter++;
}


/**********************************************************************
函数名称：taskFiltering
函数功能：滤波任务
函数形参：当前时间节拍
函数返回值：None
函数描述：
	由调度器以任务形式调用.
**********************************************************************/
void taskFiltering(timeUs_t currentTimeUs)
{
	// 陀螺仪滤波
    gyroFiltering(currentTimeUs);
}

/**********************************************************************
函数名称：gyroFilterReady
函数功能：获取陀螺仪滤波就绪状态
函数形参：None
函数返回值：就绪状态
函数描述：None
**********************************************************************/
bool gyroFilterReady(void)
{
    if (pidUpdateCounter % activePidLoopDenom == 0) {
        return true;
    } else {
        return false;
    }
}

//-------------------------------------------------------------------------------------PID控制器相关API

/**********************************************************************
函数名称：subTaskPidController
函数功能：PID控制器子任务
函数形参：当前节拍时间
函数返回值：true
函数描述：None
**********************************************************************/
static void subTaskPidController(timeUs_t currentTimeUs)
{
    uint32_t startTime = 0;
    // ---------PID控制器
    pidController(currentPidProfile, currentTimeUs);

	// ---------检查失控起飞检测是否激活(anti-taz) - 仅在上电首次起飞时在失控事件期间进行干预，不会对正常飞行产生任何影响
	// 如果激活，为了安全，将禁止解锁
#ifdef USE_RUNAWAY_TAKEOFF
    if (ARMING_FLAG(ARMED)															// 已解锁
        && pidConfig()->runaway_takeoff_prevention								    // 防止失控起飞启用
        && !runawayTakeoffCheckDisabled												// 防止失控起飞未被失能
        && !flipOverAfterCrashActive												// 反乌龟模式未激活上
        && !runawayTakeoffTemporarilyDisabled										// 防止失控起飞未被禁用
        && !FLIGHT_MODE(GPS_RESCUE_MODE)  											// GPS救援未激活，关闭失控起飞触发
        && (airmodeIsEnabled() || (calculateThrottleStatus() != THROTTLE_LOW))) { // 空中模式开启 || 未在低油门

		// 判断三轴pidSum和三轴角速度是否有超过失控起飞阈值
        if (((fabsf(pidData[FD_PITCH].Sum) >= RUNAWAY_TAKEOFF_PIDSUM_THRESHOLD)
            || (fabsf(pidData[FD_ROLL].Sum) >= RUNAWAY_TAKEOFF_PIDSUM_THRESHOLD)
            || (fabsf(pidData[FD_YAW].Sum) >= RUNAWAY_TAKEOFF_PIDSUM_THRESHOLD))
            && ((gyroAbsRateDps(FD_PITCH) > RUNAWAY_TAKEOFF_GYRO_LIMIT_RP)
            || (gyroAbsRateDps(FD_ROLL) > RUNAWAY_TAKEOFF_GYRO_LIMIT_RP)
            || (gyroAbsRateDps(FD_YAW) > RUNAWAY_TAKEOFF_GYRO_LIMIT_YAW))) {

			// 判断失控起飞触发时间
            if (runawayTakeoffTriggerUs == 0) {
                runawayTakeoffTriggerUs = currentTimeUs + RUNAWAY_TAKEOFF_ACTIVATE_DELAY;
            } else if (currentTimeUs > runawayTakeoffTriggerUs) {
            	// 锁定飞行器并禁止解锁
                setArmingDisabled(ARMING_DISABLED_RUNAWAY_TAKEOFF);
                disarm(DISARM_REASON_RUNAWAY_TAKEOFF);
            }
        } else {
            runawayTakeoffTriggerUs = 0;
        }
    } else {
        runawayTakeoffTriggerUs = 0;
    }
#endif
}

/**********************************************************************
函数名称：subTaskPidSubprocesses
函数功能：PID子进程子任务
函数形参：当前节拍时间
函数返回值：true
函数描述：None
**********************************************************************/
static void subTaskPidSubprocesses(timeUs_t currentTimeUs)
{
#if defined(USE_GPS) || defined(USE_MAG)
	// 判断是否开启GPS和磁力计
    if (sensors(SENSOR_GPS) || sensors(SENSOR_MAG)) {
		// 更新磁力计保持
        updateMagHold();
    }
#endif
    UNUSED(currentTimeUs);
}

/**********************************************************************
函数名称：pidLoopReady
函数功能：获取pid循环就绪状态
函数形参：None
函数返回值：就绪状态
函数描述：None
**********************************************************************/
bool pidLoopReady(void)
{
	// 如activePidLoopDenom = 1，则pidUpdateCounter = 1
    if ((pidUpdateCounter % activePidLoopDenom) == (activePidLoopDenom / 2)) {
        return true;
    }
    return false;
}

//-------------------------------------------------------------------------------------电机相关API

/**********************************************************************
函数名称：subTaskMotorUpdate
函数功能：电机更新子任务
函数形参：当前节拍时间
函数返回值：true
函数描述：None
**********************************************************************/
static void subTaskMotorUpdate(timeUs_t currentTimeUs)
{
	// 混控表
    mixTable(currentTimeUs);
	// 将motor数据写入所有电机
    writeMotors();
}

//-------------------------------------------------------------------------------------PidLoop相关API

/**********************************************************************
函数名称：taskMainPidLoop
函数功能：PID环回控制函数
函数形参：当前时间节拍
函数返回值：None
函数描述：
	由调度器以任务形式调用.
**********************************************************************/
void taskMainPidLoop(timeUs_t currentTimeUs)
{
	// RC命令子任务 - RC命令处理
    subTaskRcCommand(currentTimeUs);
	// PID控制器子任务 - PID控制器 && 失控起飞检测
    subTaskPidController(currentTimeUs);
	// 电机更新子任务 - 混控
    subTaskMotorUpdate(currentTimeUs);
	// PID子进程子任务
    subTaskPidSubprocesses(currentTimeUs);
}


