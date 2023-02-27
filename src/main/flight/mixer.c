/**********************************************************************
混控：(每个电机混合俯仰、偏航、横滚)
							---> 机头 <---
							   m3     m2
								 \   /
								  \ /
								  / \
							<--- /   \ --->
							   m4     m1
								 机尾
**********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"

#include "pg/motor.h"
#include "pg/rx.h"
#include "pg/pg_ids.h"

#include "drivers/dshot.h"
#include "drivers/motor.h"
#include "drivers/time.h"
#include "drivers/io.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/core.h"
#include "fc/rc.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/gps_rescue.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

// ---------------------------------------------------------电机数量
static FAST_RAM_ZERO_INIT uint8_t motorCount;	 			
// ---------------------------------------------------------电机混控范围
static FAST_RAM_ZERO_INIT float motorMixRange;   		
// ---------------------------------------------------------电机输出范围
FAST_RAM_ZERO_INIT float motorOutputHigh, motorOutputLow;	

// ---------------------------------------------------------电机命令存储
float FAST_RAM_ZERO_INIT motor[MAX_SUPPORTED_MOTORS];	
// ---------------------------------------------------------电机锁定
float motor_disarmed[MAX_SUPPORTED_MOTORS];			
// ---------------------------------------------------------锁定电机输出
static FAST_RAM_ZERO_INIT float disarmMotorOutput;		

// ---------------------------------------------------------油门动态低通滤波
// 油门动态低通滤波步距
#define DYN_LPF_THROTTLE_STEPS           100				
// 油门动态低通滤波延迟 - 更新间隔至少5毫秒
#define DYN_LPF_THROTTLE_UPDATE_DELAY_US 5000 	
// ---------------------------------------------------------RC命令油门角
static FAST_RAM_ZERO_INIT float rcCommandThrottleRange;	
// ---------------------------------------------------------油门角修正
static FAST_RAM_ZERO_INIT int throttleAngleCorrection;		

// ---------------------------------------------------------油门值
static FAST_RAM_ZERO_INIT float throttle = 0;          
// ---------------------------------------------------------混控油门值
static FAST_RAM_ZERO_INIT float mixerThrottle = 0;			
// ---------------------------------------------------------电机输出最小值
static FAST_RAM_ZERO_INIT float motorOutputMin;				
// ---------------------------------------------------------电机范围最小值
static FAST_RAM_ZERO_INIT float motorRangeMin;			
// ---------------------------------------------------------电机范围最大值
static FAST_RAM_ZERO_INIT float motorRangeMax;				
// ---------------------------------------------------------电机输出范围
static FAST_RAM_ZERO_INIT float motorOutputRange;			
// ---------------------------------------------------------电机输出混控
static FAST_RAM_ZERO_INIT int8_t motorOutputMixSign;		
 			
PG_REGISTER_WITH_RESET_TEMPLATE(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
PG_RESET_TEMPLATE(mixerConfig_t, mixerConfig,
    .mixerMode = DEFAULT_MIXER,       		  	 			// 默认混控模式 - 正X四轴
    .yaw_motors_reversed = false,							// 偏航不反转
    .crashflip_motor_percent = 0,
    .crashflip_expo = 35
);
PG_REGISTER_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer, PG_MOTOR_MIXER, 0);
static motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];     // 当前混控
mixerMode_e currentMixerMode;					 			// 当前混控模式	 
static const motorMixer_t mixerQuadX[] = {					// QuadX混控表
    { 1.0f, -1.0f,  1.0f, -1.0f },            	 			// 右后
    { 1.0f, -1.0f, -1.0f,  1.0f },            	 			// 右前
    { 1.0f,  1.0f,  1.0f,  1.0f },            	 			// 左前
    { 1.0f,  1.0f, -1.0f, -1.0f },            	 			// 左后
};

//-------------------------------------------------------------------------------------初始化相关API

/**********************************************************************
函数名称：mixerResetDisarmedMotors
函数功能：混控复位禁用马达
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void mixerResetDisarmedMotors(void)
{
    // 设置锁定的马达值
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motor_disarmed[i] = disarmMotorOutput;
    }
}

/**********************************************************************
函数名称：mixerConfigureOutput
函数功能：混控输出配置
函数形参：None
函数返回值：None
函数描述：
	由系统初始化init调用。
**********************************************************************/
void mixerConfigureOutput(void)
{
    motorCount = QUAD_MOTOR_COUNT;
	// 应用混控表
    for (int i = 0; i < motorCount; i++) {
        currentMixer[i] = mixerQuadX[i];
    }
	// 设置锁定的马达值
    mixerResetDisarmedMotors();
}

/**********************************************************************
函数名称：initEscEndpoints
函数功能：初始化电调结束点
函数形参：None  
函数返回值：None
函数描述：
	所有PWM电机缩放到标准PWM范围1000-2000。
	DSHOT缩放是对实际DSHOT范围进行的。
	由mixerInit（混控初始化）或changePidProfile（PID配置文件）调用。
**********************************************************************/
void initEscEndpoints(void)
{
    float motorOutputLimit = 1.0f;
    if (currentPidProfile->motor_output_limit < 100) {
		// 电机输出限制
        motorOutputLimit = currentPidProfile->motor_output_limit / 100.0f;
    }
	// 初始化电机结束点
    motorInitEndpoints(motorConfig(), motorOutputLimit, &motorOutputLow, &motorOutputHigh, &disarmMotorOutput);
	// RC油门命令范围
    rcCommandThrottleRange = PWM_RANGE_MAX - PWM_RANGE_MIN;
}

/**********************************************************************
函数名称：mixerInit
函数功能：混控初始化
函数形参：混控模式
函数返回值：None
函数描述：
	由系统初始化init调用。
**********************************************************************/
void mixerInit(mixerMode_e mixerMode)
{
    currentMixerMode = mixerMode;
	// 初始化电调结束点
    initEscEndpoints();
}

//-------------------------------------------------------------------------------------获取混控参数相关API

/**********************************************************************
函数名称：getMotorCount
函数功能：获取电机数量
函数形参：None  
函数返回值：motorCount
函数描述：None 
**********************************************************************/
uint8_t getMotorCount(void)
{
    return motorCount;
}

/**********************************************************************
函数名称：getMotorMixRange
函数功能：获取电机混控范围
函数形参：None  
函数返回值：motorMixRange
函数描述：None 
**********************************************************************/
float getMotorMixRange(void)
{
    return motorMixRange;
}

/**********************************************************************
函数名称：areMotorsRunning
函数功能：获取电机是否在运行
函数形参：None  
函数返回值：电机运行状态
函数描述：None 
**********************************************************************/
bool areMotorsRunning(void)
{
    bool motorsRunning = false;
    if (ARMING_FLAG(ARMED)) {
        motorsRunning = true;
    } else {
        for (int i = 0; i < motorCount; i++) {
            if (motor_disarmed[i] != disarmMotorOutput) {
                motorsRunning = true;
                break;
            }
        }
    }
    return motorsRunning;
}

//-------------------------------------------------------------------------------------电机控制相关API

/**********************************************************************
函数名称：writeMotors
函数功能：将motor数据写入所有电机
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void writeMotors(void)
{
    motorWriteAll(motor);
}

/**********************************************************************
函数名称：writeAllMotors
函数功能：将命令写入所有电机
函数形参：mc
函数返回值：None
函数描述：None
**********************************************************************/
static void writeAllMotors(int16_t mc)
{
    // 向所有电机发送命令
    for (int i = 0; i < motorCount; i++) {
        motor[i] = mc;
    }
    writeMotors();
}

/**********************************************************************
函数名称：stopMotors
函数功能：停止电机
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void stopMotors(void)
{
    writeAllMotors(disarmMotorOutput);
	// 延时给ESC一个反应的机会
    delay(50); 
}

/**********************************************************************
函数名称：applyFlipOverAfterCrashModeToMotors
函数功能：反乌龟模式下对电机进行翻转
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
// 反乌龟死区
#define CRASH_FLIP_DEADBAND 20								
#define CRASH_FLIP_STICK_MINF 0.15f
static void applyFlipOverAfterCrashModeToMotors(void)
{
	// ------------------------------------------------已解锁
    if (ARMING_FLAG(ARMED)) {
        const float flipPowerFactor = 1.0f - mixerConfig()->crashflip_expo / 100.0f;
		// 获取摇杆偏转绝对值
        const float stickDeflectionPitchAbs = getRcDeflectionAbs(FD_PITCH);
        const float stickDeflectionRollAbs = getRcDeflectionAbs(FD_ROLL);
        const float stickDeflectionYawAbs = getRcDeflectionAbs(FD_YAW);
		// 计算摇杆偏转固定曲线值
        const float stickDeflectionPitchExpo = flipPowerFactor * stickDeflectionPitchAbs + power3(stickDeflectionPitchAbs) * (1 - flipPowerFactor);
        const float stickDeflectionRollExpo = flipPowerFactor * stickDeflectionRollAbs + power3(stickDeflectionRollAbs) * (1 - flipPowerFactor);
        const float stickDeflectionYawExpo = flipPowerFactor * stickDeflectionYawAbs + power3(stickDeflectionYawAbs) * (1 - flipPowerFactor);
		// 计算符号
        float signPitch = getRcDeflection(FD_PITCH) < 0 ? 1 : -1;
        float signRoll = getRcDeflection(FD_ROLL) < 0 ? 1 : -1;
        float signYaw = (getRcDeflection(FD_YAW) < 0 ? 1 : -1) * (mixerConfig()->yaw_motors_reversed ? 1 : -1);

		// 摇杆偏转长度
        float stickDeflectionLength = sqrtf(sq(stickDeflectionPitchAbs) + sq(stickDeflectionRollAbs));
		// 摇杆偏转曲线长度
        float stickDeflectionExpoLength = sqrtf(sq(stickDeflectionPitchExpo) + sq(stickDeflectionRollExpo));

        if (stickDeflectionYawAbs > MAX(stickDeflectionPitchAbs, stickDeflectionRollAbs)) {
            // 如果偏航是主导，禁用俯仰和横摇
            stickDeflectionLength = stickDeflectionYawAbs;
            stickDeflectionExpoLength = stickDeflectionYawExpo;
            signRoll = 0;
            signPitch = 0;
        } else {
            // 如果俯仰/滚转占主导地位，禁止偏航
            signYaw = 0;
        }

        const float cosPhi = (stickDeflectionLength > 0) ? (stickDeflectionPitchAbs + stickDeflectionRollAbs) / (sqrtf(2.0f) * stickDeflectionLength) : 0;
		// cos(PI/6.0f)
        const float cosThreshold = sqrtf(3.0f)/2.0f; 
        if (cosPhi < cosThreshold) {
            // 如果不是对角线，则强制强制滚转或俯仰
            if (stickDeflectionRollAbs > stickDeflectionPitchAbs) {
                signPitch = 0;
            } else {
                signRoll = 0;
            }
        }

        // 应用合理的摇杆死区
        const float crashFlipStickMinExpo = flipPowerFactor * CRASH_FLIP_STICK_MINF + power3(CRASH_FLIP_STICK_MINF) * (1 - flipPowerFactor);
        const float flipStickRange = 1.0f - crashFlipStickMinExpo;
        const float flipPower = MAX(0.0f, stickDeflectionExpoLength - crashFlipStickMinExpo) / flipStickRange;

		// 遍历所有电机
        for (int i = 0; i < motorCount; ++i) {
            float motorOutputNormalised =
                signPitch*currentMixer[i].pitch +
                signRoll*currentMixer[i].roll +
                signYaw*currentMixer[i].yaw;

            if (motorOutputNormalised < 0) {
                if (mixerConfig()->crashflip_motor_percent > 0) {
                    motorOutputNormalised = -motorOutputNormalised * (float)mixerConfig()->crashflip_motor_percent / 100.0f;
                } else {
                    motorOutputNormalised = 0;
                }
            }
            motorOutputNormalised = MIN(1.0f, flipPower * motorOutputNormalised);
            float motorOutput = motorOutputMin + motorOutputNormalised * motorOutputRange;

            // 添加一点到motorOutputMin，这样当摇杆居中时，飞行器不会旋转
            motorOutput = (motorOutput < motorOutputMin + CRASH_FLIP_DEADBAND) ? disarmMotorOutput : (motorOutput - CRASH_FLIP_DEADBAND);

            motor[i] = motorOutput;
        }
    } 
	// ------------------------------------------------已锁定
	else {
		// 电机状态应用锁定值
        for (int i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

//-------------------------------------------------------------------------------------混控计算相关API

/**********************************************************************
函数名称：calculateThrottleAndCurrentMotorEndpoints
函数功能：计算油门和当前电机的终点 - 计算油门范围
函数形参：currentTimeUs
函数返回值：None
函数描述：
	由mixTable（混控表）调用。
**********************************************************************/
static void calculateThrottleAndCurrentMotorEndpoints(timeUs_t currentTimeUs)
{ 
	// 电机最小范围
    static float motorRangeMinIncrease = 0;	
	// 当前油门输入范围
    float currentThrottleInputRange = 0;					

	// 油门 = 油门命令[1000,2000] - 1000 -> [0,1000]
    throttle = rcCommand[THROTTLE] - PWM_RANGE_MIN;

	// 应用电机最低输出
    float appliedMotorOutputLow = motorOutputLow;
	// 电机最大范围
    motorRangeMax = motorOutputHigh;
	// 当前油门输入范围
    currentThrottleInputRange = rcCommandThrottleRange;
    motorRangeMin = appliedMotorOutputLow + motorRangeMinIncrease * (motorOutputHigh - appliedMotorOutputLow);
    motorOutputMin = motorRangeMin;
    motorOutputRange = motorRangeMax - motorRangeMin;
    motorOutputMixSign = 1;

    throttle = constrainf(throttle / currentThrottleInputRange, 0.0f, 1.0f);
}

/**********************************************************************
函数名称：applyThrottleLimit
函数功能：应用油门限制
函数形参：油门
函数返回值：限制后的油门
函数描述：
	由mixTable（混控表）调用。
**********************************************************************/
static float applyThrottleLimit(float throttle)
{
    if (currentControlRateProfile->throttle_limit_percent < 100) {
        const float throttleLimitFactor = currentControlRateProfile->throttle_limit_percent / 100.0f;
        switch (currentControlRateProfile->throttle_limit_type) {
            case THROTTLE_LIMIT_TYPE_SCALE:
                return throttle * throttleLimitFactor;
            case THROTTLE_LIMIT_TYPE_CLIP:
                return MIN(throttle, throttleLimitFactor);
        }
    }
    return throttle;
}

/**********************************************************************
函数名称：updateDynLpfCutoffs
函数功能：更新动态低通滤波截止频率
函数形参：当前时间节拍，油门
函数返回值：None
函数描述：
	由mixTable（混控表）调用。
**********************************************************************/
#ifdef USE_DYN_LPF
static void updateDynLpfCutoffs(timeUs_t currentTimeUs, float throttle)
{
    static timeUs_t lastDynLpfUpdateUs = 0;
	// 允许一个初始零油门设置过滤器截止
    static int dynLpfPreviousQuantizedThrottle = -1;  		

    if (cmpTimeUs(currentTimeUs, lastDynLpfUpdateUs) >= DYN_LPF_THROTTLE_UPDATE_DELAY_US) {
		// 量化油门可以减少滤波器更新的次数
        const int quantizedThrottle = lrintf(throttle * DYN_LPF_THROTTLE_STEPS); 
        if (quantizedThrottle != dynLpfPreviousQuantizedThrottle) {
            // 动态低通滤波油门
            const float dynLpfThrottle = (float)quantizedThrottle / DYN_LPF_THROTTLE_STEPS;
			// 陀螺仪动态低通滤波更新
            dynLpfGyroUpdate(dynLpfThrottle);
			// D项动态低通滤波更新
            dynLpfDTermUpdate(dynLpfThrottle);
            dynLpfPreviousQuantizedThrottle = quantizedThrottle;
            lastDynLpfUpdateUs = currentTimeUs;
        }
    }
}
#endif

/**********************************************************************
函数名称：applyMixToMotors
函数功能：应用电机混控 - 得到motor数据
函数形参：motorMix，motorMixer_t
函数返回值：None
函数描述：
	由mixTable（混控表）调用。
**********************************************************************/
static void applyMixToMotors(float motorMix[MAX_SUPPORTED_MOTORS], motorMixer_t *activeMixer)
{
	// ---------------------------------------------------------1.遍历所有电机 - 更新电机油门控制
    for (int i = 0; i < motorCount; i++) {
		// -------------------------------(1)计算电机输出
        float motorOutput = motorOutputMixSign * motorMix[i] + throttle * activeMixer[i].throttle;
		// -------------------------------(2)应用油门线性化
#ifdef USE_THRUST_LINEARIZATION
        motorOutput = pidApplyThrustLinearization(motorOutput);
#endif
        motorOutput = motorOutputMin + motorOutputRange * motorOutput;
		// -------------------------------(3)失控保护
        if (failsafeIsActive()) {
			// 失控保护已激活
#ifdef USE_DSHOT
			// 判断电机Dshot协议是否使能
            if (isMotorProtocolDshot()) {
				// 防止进入特殊的预留范围
                motorOutput = (motorOutput < motorRangeMin) ? disarmMotorOutput : motorOutput; 
            }
#endif
            motorOutput = constrain(motorOutput, disarmMotorOutput, motorRangeMax);
        } else {
			// 失控保护未激活
            motorOutput = constrain(motorOutput, motorRangeMin, motorRangeMax);
        }
		// -------------------------------(4)获取电机输出->motor[i]
        motor[i] = motorOutput;
    }

    // ---------------------------------------------------------2.检测锁定状态
    if (!ARMING_FLAG(ARMED)) {
		// 电机状态应用锁定值
        for (int i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

//-------------------------------------------------------------------------------------混控表API[最终的电机控制]

/**********************************************************************
函数名称：mixTable
函数功能：混控表
函数形参：当前时间节拍，vbatPidCompensation
函数返回值：None
函数描述：None
**********************************************************************/
void mixTable(timeUs_t currentTimeUs)
{
    // ---------------------------------------------------计算油门和当前电机的终点 - 求出油门范围
    calculateThrottleAndCurrentMotorEndpoints(currentTimeUs);

	// ---------------------------------------------------检测反乌龟是否激活
    if (isFlipOverAfterCrashActive()) {
		// 反乌龟模式下对电机进行翻转
        applyFlipOverAfterCrashModeToMotors();
        return;
    }

	// ---------------------------------------------------获取激活的混控
    motorMixer_t * activeMixer = &currentMixer[0];
	
    // ---------------------------------------------------缩放俯仰横滚轴PID_SUM - 混控电机转速使用
    const float scaledAxisPidRoll =
        constrainf(pidData[FD_ROLL].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;
    const float scaledAxisPidPitch =
        constrainf(pidData[FD_PITCH].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;
    
	// ---------------------------------------------------反自旋检测
	uint16_t yawPidSumLimit = currentPidProfile->pidSumLimitYaw;
#ifdef USE_YAW_SPIN_RECOVERY
    const bool yawSpinDetected = gyroYawSpinDetected();
    if (yawSpinDetected) {
		// 设置为偏航旋转恢复时的最大限制，以防止限制电机的权力
        yawPidSumLimit = PIDSUM_LIMIT_MAX;   
    }
#endif 
	// ---------------------------------------------------缩放偏航轴PID_SUM
    float scaledAxisPidYaw = constrainf(pidData[FD_YAW].Sum, -yawPidSumLimit, yawPidSumLimit) / PID_MIXER_SCALING;
	// ---------------------------------------------------判断偏航是否反转
    if (!mixerConfig()->yaw_motors_reversed) {
        scaledAxisPidYaw = -scaledAxisPidYaw;
    }

    // ---------------------------------------------------基于油门限制类型应用油门限制百分比缩放或限制油门
    if (currentControlRateProfile->throttle_limit_type != THROTTLE_LIMIT_TYPE_OFF) {
		// 应用油门限制
        throttle = applyThrottleLimit(throttle);
    }
	// ---------------------------------------------------获取空中模式使能状态
    const bool airmodeEnabled = airmodeIsEnabled();
	// ---------------------------------------------------当airmode未激活 && 触发反自旋 - 油门限制50%
#ifdef USE_YAW_SPIN_RECOVERY
    if (yawSpinDetected && !airmodeEnabled) {
        throttle = 0.5f;   
    }
#endif 

    // ---------------------------------------------------混控 - 混合横滚/俯仰/偏航控制
    float motorMix[MAX_SUPPORTED_MOTORS];
    float motorMixMax = 0, motorMixMin = 0;
	// 遍历所有电机
    for (int i = 0; i < motorCount; i++) {
		// 混合控制
        float mix =
            scaledAxisPidRoll  * activeMixer[i].roll +
            scaledAxisPidPitch * activeMixer[i].pitch +
            scaledAxisPidYaw   * activeMixer[i].yaw; 

		// 更新混控极值
        if (mix > motorMixMax) {
            motorMixMax = mix;
        } else if (mix < motorMixMin) {
            motorMixMin = mix;
        }
        motorMix[i] = mix;
    }

	// ---------------------------------------------------更新反重力油门滤波
    pidUpdateAntiGravityThrottleFilter(throttle);

	// ---------------------------------------------------更新油门动态低通滤波截止频率
#ifdef USE_DYN_LPF
    updateDynLpfCutoffs(currentTimeUs, throttle);
#endif
	// ---------------------------------------------------油门线性化PID补偿 - 通过反向补偿推力线性化
#ifdef USE_THRUST_LINEARIZATION
    throttle = pidCompensateThrustLinearization(throttle);
#endif
	// ---------------------------------------------------油门增压
#if defined(USE_THROTTLE_BOOST)
    if (throttleBoost > 0.0f) {
        const float throttleHpf = throttle - pt1FilterApply(&throttleLpf, throttle);
        throttle = constrainf(throttle + throttleBoost * throttleHpf, 0.0f, 1.0f);
    }
#endif
	// ---------------------------------------------------如果GPS救援模式激活,获取GPS救援模式油门覆盖当前油门
#ifdef USE_GPS_RESCUE
    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        throttle = gpsRescueGetThrottle();
    }
#endif
	// ---------------------------------------------------获取混控油门为当前油门
    mixerThrottle = throttle;
	// 计算电机混控范围
    motorMixRange = motorMixMax - motorMixMin;
    if (motorMixRange > 1.0f) {
        for (int i = 0; i < motorCount; i++) {
            motorMix[i] /= motorMixRange;
        }
        // 当airmode启用时，通过设置偏移到中心来获得最大的校正
        if (airmodeEnabled) {
            throttle = 0.5f;
        }
    } else {
    	// 只有在开启空中模式时才会自动调整油门，空中模式逻辑在高油门时总是激活的
        if (airmodeEnabled || throttle > 0.5f) {  
            throttle = constrainf(throttle, -motorMixMin, 1.0f - motorMixMax);
        }
    }

    // ---------------------------------------------------应用电机混控 - 得到motor数据
    applyMixToMotors(motorMix, activeMixer);
}

