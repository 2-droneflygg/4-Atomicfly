/**********************************************************************
GPS救援：
	旨在在紧急情况下（例如视频或无线电链路丢失）自动将四轴驱动带回。
	唯一的目的是使四轴重新回到可控范围内，以便可以尽快重新控制。
	这并不意味着是可靠的“返回原位”模式。
 注意：
	信号恢复时飞行员应尽快恢复控制，不要依靠GPS救援飞行。
 激活方法：
	可以使用OSD菜单（FEATURES -> FAILSAFE）激活。
	如果在受限或受限的空域（例如室内或树丛中）飞行，
	则OSD菜单设置还可方便地在故障保护现场禁用GPS救援。
	确保在适当的时候重新启用。
**********************************************************************/
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_GPS_RESCUE
#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "io/gps.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"

#include "gps_rescue.h"

/* --------------------------GPS救援健全枚举-------------------------- */	
typedef enum {
    RESCUE_SANITY_OFF = 0,
    RESCUE_SANITY_ON,
    RESCUE_SANITY_FS_ONLY
} gpsRescueSanity_e;

/* --------------------------GPS救援阶段枚举-------------------------- */	
typedef enum {
    RESCUE_IDLE,								// 救援闲置
    RESCUE_INITIALIZE,							// 救援初始化
    RESCUE_ATTAIN_ALT,							// 爬升到设定高度		
    RESCUE_CROSSTRACK,							// 轨迹
    RESCUE_LANDING_APPROACH,					// 接近着陆
    RESCUE_LANDING,								// 着陆
    RESCUE_ABORT,								// 救援中止
    RESCUE_COMPLETE								// 救援完成
} rescuePhase_e;

/* ------------------------GPS救援失控保护状态枚举-------------------- */	
typedef enum {
    RESCUE_HEALTHY,			    				// 健康
    RESCUE_FLYAWAY,
    RESCUE_GPSLOST,								// GPS丢失
    RESCUE_LOWSATS,								// 卫星数量低
    RESCUE_CRASH_FLIP_DETECTED,					// 检测到救援碰撞翻转
    RESCUE_STALLED,								// 救援停滞不前
    RESCUE_TOO_CLOSE							// 距离太近
} rescueFailureState_e;

/* ---------------------------GPS救援目标结构体----------------------- */	
typedef struct {
    int32_t targetAltitudeCm;   				// 目标高度
    int32_t targetGroundspeed;					// 目标速度
    uint8_t minAngleDeg;						// 最小角度
    uint8_t maxAngleDeg;						// 最大角度
    bool crosstrack;							// 偏航距离
} rescueIntent_s;

/* -------------------------GPS救援传感器数据结构体------------------- */	
typedef struct {
    int32_t maxAltitudeCm;
    int32_t currentAltitudeCm;
    uint16_t distanceToHomeM;
    uint16_t maxDistanceToHomeM;
    int16_t directionToHome;
    uint16_t groundSpeed;
    uint8_t numSat;
    float zVelocity;       						// Up/down movement in cm/s
    float zVelocityAvg;    						// Up/down average in cm/s
    float accMagnitude;
    float accMagnitudeAvg;
    bool healthy;
} rescueSensorData_s;

/* --------------------------GPS救援健全标志结构体-------------------- */	
typedef struct {
    bool bumpDetection;
    bool convergenceDetection;
} rescueSanityFlags;

/* ---------------------------GPS救援状态结构体----------------------- */	
typedef struct {
    rescuePhase_e phase;						// 救援阶段			
    rescueFailureState_e failure;				// 失控保护状态
    rescueSensorData_s sensor;					// GPS救援传感器数据	
    rescueIntent_s intent;						// GPS救援目标
    bool isFailsafe;							// 失控保护状态
    bool isAvailable;							// 可用状态
} rescueState_s;

/* -------------------------GPS救援高度模式结构体--------------------- */	
typedef enum {
    MAX_ALT,									// 最大高度（飞行过程中记录的最大高度+ 15m）
    FIXED_ALT,									// 固定高度
    CURRENT_ALT									// 当前高度（将返回以保持读取的高度不变（不建议））
} altitudeMode_e;

/* --------------------------GPS救援油门PID结构体--------------------- */	
typedef struct {
    float Kp;
    float Ki;
    float Kd;
} throttle_s;

// ---------------------------------------------------------GPS救援油门
throttle_s throttle;
// ---------------------------------------------------------救援油门
static uint16_t rescueThrottle;
// ---------------------------------------------------------悬停油门
uint16_t hoverThrottle = 0;
// ---------------------------------------------------------平均油门
float averageThrottle = 0.0;
// ---------------------------------------------------------油门采样
uint32_t throttleSamples = 0;
// ---------------------------------------------------------救援角度
int32_t gpsRescueAngle[ANGLE_INDEX_COUNT] = {0, 0};
// ---------------------------------------------------------救援偏航
static float rescueYaw;
// ---------------------------------------------------------高度误差
float altitudeError = 0.0;
// ---------------------------------------------------------磁力计失能
bool magForceDisable = false;
// ---------------------------------------------------------GPS数据状态
static bool newGPSData = false;
// ---------------------------------------------------------救援状态
rescueState_s rescueState;

// ---------------------------------------------------------相关宏定义
// 最大偏航速率[deg/sec]
#define GPS_RESCUE_MAX_YAW_RATE         180 	
// 当误差小于这个角度时，测量命令的偏航速率
#define GPS_RESCUE_RATE_SCALE_DEGREES    45 	
// 从离家的距离开始降低速度
#define GPS_RESCUE_SLOWDOWN_DISTANCE_M  200 	
// 允许的最小下降距离
#define GPS_RESCUE_MIN_DESCENT_DIST_M    30 
// 开始下降z速度的高度阈值
#define GPS_RESCUE_ZVELOCITY_THRESHOLD  300 	
// 着陆阶段下降速度
#define GPS_RESCUE_LANDING_ZVELOCITY     80 	
// z速度误差100cm/s后复位I项
#define GPS_RESCUE_ITERM_WINDUP         100 	
// 允许最大 iterm 值
#define GPS_RESCUE_MAX_ITERM_ACC        250.0f  
// 四轴飞行器开始降低下降速度的高度
#define GPS_RESCUE_SLOWDOWN_ALT         500 	
// 最后着陆阶段的最小速度
#define GPS_RESCUE_MINIMUM_ZVELOCITY     50 	
// 高度之后，四轴飞行器增加地面探测灵敏度
#define GPS_RESCUE_ALMOST_LANDING_ALT   100 	
// pid scaler for P term
#define GPS_RESCUE_THROTTLE_P_SCALE 0.0003125f 
// pid scaler for I term
#define GPS_RESCUE_THROTTLE_I_SCALE 0.1f       
// pid scaler for D term
#define GPS_RESCUE_THROTTLE_D_SCALE 0.0003125f  
// 是否使用磁力计
#ifdef USE_MAG
#define GPS_RESCUE_USE_MAG              true
#else
#define GPS_RESCUE_USE_MAG              false
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 1);
PG_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig,
    .angle = 32,								// 角度
    .initialAltitudeM = 100,					// 初始高度
    .descentDistanceM = 50,						// 下降距离
    .rescueGroundspeed = 2000,					// 地面速度（厘米/秒）-> 0.02km/s -> 0.02*3600=72km/h
    .throttleP = 150,							// 油门P
    .throttleI = 20,							// 油门I
    .throttleD = 50,							// 油门D
    .velP = 80,									// 速率P
    .velI = 20,									// 速率I
    .velD = 15,									// 速率D
    .yawP = 40,									// 偏航P
    .throttleMin = 1100,						// 油门最小值
    .throttleMax = 1600,						// 油门最大值
    .throttleHover = 1300,						// 悬停油门
    .sanityChecks = RESCUE_SANITY_ON,			// 开启安全检查
    .minSats = 6,								// 最小星数
    .minRescueDth = 100,            			// 最小距离
    .allowArmingWithoutFix = true,	 			// 允许低于定位星数解锁
    .useMag = GPS_RESCUE_USE_MAG,
    .targetLandingAltitudeM = 5,
    .targetLandingDistanceM = 10,
    .altitudeMode = FIXED_ALT,       			// 设定高度
    .ascendRate = 500,
    .descendRate = 150,
);

/**********************************************************************
函数名称：rescueNewGpsData
函数功能：救援模式新数据 - 如果有新的GPS数据适用的话,更新主航向
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
void rescueNewGpsData(void)
{
    newGPSData = true;
}

/**********************************************************************
函数名称：rescueStart
函数功能：救援开始
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
static void rescueStart()
{
    rescueState.phase = RESCUE_INITIALIZE;
}

/**********************************************************************
函数名称：rescueStop
函数功能：救援停止
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
static void rescueStop()
{
    rescueState.phase = RESCUE_IDLE;
}

/**********************************************************************
函数名称：idleTasks
函数功能：不管GPS救援模式是否启用，都需要运行
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
static void idleTasks()
{
    // 当未解锁时，不要计算任何空闲任务值
    if (!ARMING_FLAG(ARMED)) {
        rescueState.sensor.maxAltitudeCm = 0;
        rescueState.sensor.maxDistanceToHomeM = 0;
        return;
    }

    // 如果还没有应用适当的高度偏移，不要更新任何救援飞行数据
    if (!isAltitudeOffset()) {
        return;
    }

	// 重置GPS救援俯仰和横滚角度
    gpsRescueAngle[AI_PITCH] = 0;
    gpsRescueAngle[AI_ROLL] = 0;

    // 存储在RTH期间最大高度，这样就知道飞回最小高度
    rescueState.sensor.maxAltitudeCm = MAX(rescueState.sensor.currentAltitudeCm, rescueState.sensor.maxAltitudeCm);
    // 储存正常飞行时到家的最大距离，这样就知道是否发生飞行
    rescueState.sensor.maxDistanceToHomeM = MAX(rescueState.sensor.distanceToHomeM, rescueState.sensor.maxDistanceToHomeM);
	// 救援模式油门
    rescueThrottle = rcCommand[THROTTLE];
	
	// 设置盘旋油门的默认值 - GPS救援油门处理应该考虑min_check作为激活油门从min_check到PWM_RANGE_MAX
	// 目前正在为此进行调整在gpsRescueGetThrottle()，但它将更好地处理在这里
    const float ct = getCosTiltAngle();
    // 5 ~ 45度倾斜
    if (ct > 0.5 && ct < 0.96 && throttleSamples < 1E6 && rescueThrottle > 1070) {    
        // 只在加速度较低时采样
        uint16_t adjustedThrottle = 1000 + (rescueThrottle - PWM_RANGE_MIN) * ct;
        if (throttleSamples == 0) {
            averageThrottle = adjustedThrottle;
        } else {
            averageThrottle += (adjustedThrottle - averageThrottle) / (throttleSamples + 1);
        }
        hoverThrottle = lrintf(averageThrottle);
        throttleSamples++;
    }
}

/**********************************************************************
函数名称：setBearing
函数功能：设置方位
函数形参：None  
函数返回值：None  
函数描述：非常类似于betaflight/cleanflight的maghold功能
**********************************************************************/
static void setBearing(int16_t desiredHeading)
{
    float errorAngle = (attitude.values.yaw / 10.0f) - desiredHeading;

    // 确定最有效的旋转方向
    if (errorAngle <= -180) {
        errorAngle += 360;
    } else if (errorAngle > 180) {
        errorAngle -= 360;
    }

    errorAngle *= -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
	
	// 根据超出的最大限制计算所需的偏航率
	// 错误窗口，然后将请求的速率向下缩放
	// 当错误接近0时的窗口。
    rescueYaw = -constrainf(errorAngle / GPS_RESCUE_RATE_SCALE_DEGREES * GPS_RESCUE_MAX_YAW_RATE, -GPS_RESCUE_MAX_YAW_RATE, GPS_RESCUE_MAX_YAW_RATE);
}

/**********************************************************************
函数名称：rescueAttainPosition
函数功能：救援高度位置控制
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
static void rescueAttainPosition()
{
    // 速度和高度控制器内部变量
    static float previousSpeedError = 0;
    static int16_t speedIntegral = 0;
    int zVelocityError;
    static int previousZVelocityError = 0;
    static float zVelocityIntegral = 0;
    static float scalingRate = 0;
    static int16_t altitudeAdjustment = 0;

	// 每次启动GPS救援时初始化内部变量
    if (rescueState.phase == RESCUE_INITIALIZE) {
        previousSpeedError = 0;
        speedIntegral = 0;
        previousZVelocityError = 0;
        zVelocityIntegral = 0;
        altitudeAdjustment = 0;
    }

	// 设置方位
    if (rescueState.intent.crosstrack) {
        setBearing(rescueState.sensor.directionToHome);
    }

	// 如果不为GPS新数据，不要进行下面的计算
    if (!newGPSData) {
        return;
    }

    // ----------------------------------------------------------------速度控制器 - AI_PITCH(只需要控制俯仰，横滚为0即可)
    // 计算速度误差
    const int16_t speedError = (rescueState.intent.targetGroundspeed - rescueState.sensor.groundSpeed) / 100;
    const int16_t speedDerivative = speedError - previousSpeedError;

    speedIntegral = constrain(speedIntegral + speedError, -100, 100);

    previousSpeedError = speedError;

    const int16_t angleAdjustment =  gpsRescueConfig()->velP * speedError + (gpsRescueConfig()->velI * speedIntegral) / 100 +  gpsRescueConfig()->velD * speedDerivative;

    gpsRescueAngle[AI_PITCH] = constrain(gpsRescueAngle[AI_PITCH] + MIN(angleAdjustment, 80), rescueState.intent.minAngleDeg * 100, rescueState.intent.maxAngleDeg * 100);

    const float ct = cos(DECIDEGREES_TO_RADIANS(gpsRescueAngle[AI_PITCH] / 10));

	// ----------------------------------------------------------------高度控制器 - rescueThrottle
    const int16_t altitudeError = rescueState.intent.targetAltitudeCm - rescueState.sensor.currentAltitudeCm;

    // ------------------------------------------P component
    if (ABS(altitudeError) > 0 && ABS(altitudeError) < GPS_RESCUE_ZVELOCITY_THRESHOLD) {
        scalingRate = (float)altitudeError / GPS_RESCUE_ZVELOCITY_THRESHOLD;
    } else {
        scalingRate = 1;
    }
    if (altitudeError > 0) {
        zVelocityError = gpsRescueConfig()->ascendRate * scalingRate - rescueState.sensor.zVelocity;
    } else if (altitudeError < 0) {
        if (rescueState.sensor.currentAltitudeCm <= GPS_RESCUE_SLOWDOWN_ALT) {
            const int16_t rescueLandingDescendVel = MAX(GPS_RESCUE_LANDING_ZVELOCITY * rescueState.sensor.currentAltitudeCm / GPS_RESCUE_SLOWDOWN_ALT, GPS_RESCUE_MINIMUM_ZVELOCITY);
            zVelocityError = -rescueLandingDescendVel - rescueState.sensor.zVelocity;
        } else {
            zVelocityError = -gpsRescueConfig()->descendRate * scalingRate - rescueState.sensor.zVelocity;
        }
    } else {
        zVelocityError = 0;
    }

    // ------------------------------------------I component
    if (ABS(zVelocityError) < GPS_RESCUE_ITERM_WINDUP) {
        zVelocityIntegral = constrainf(zVelocityIntegral + zVelocityError / 100.0f, -GPS_RESCUE_MAX_ITERM_ACC, GPS_RESCUE_MAX_ITERM_ACC);
    } else {
        zVelocityIntegral = 0;
    }

    // ------------------------------------------D component
    const int zVelocityDerivative = zVelocityError - previousZVelocityError;
    previousZVelocityError = zVelocityError;
    const int16_t hoverAdjustment = (hoverThrottle - 1000) / ct;
    altitudeAdjustment = constrain(altitudeAdjustment + (throttle.Kp * zVelocityError + throttle.Ki * zVelocityIntegral + throttle.Kd * zVelocityDerivative),
                                    gpsRescueConfig()->throttleMin - 1000 - hoverAdjustment, gpsRescueConfig()->throttleMax - 1000 - hoverAdjustment);

	// ----------------------------------------------------------------计算返航油门
    rescueThrottle = constrain(1000 + altitudeAdjustment + hoverAdjustment, gpsRescueConfig()->throttleMin, gpsRescueConfig()->throttleMax);
}

/**********************************************************************
函数名称：performSanityChecks
函数功能：进行健康检查
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
static void performSanityChecks()
{
    static uint32_t previousTimeUs = 0; 	  // 上次停止/低sat是检查过的
    static int8_t secondsStalled = 0;         // 停滞不前的运动检测
    static uint16_t lastDistanceToHomeM = 0;  // 飞行检测
    static int8_t secondsFlyingAway = 0;	
    static int8_t secondsLowSats = 0; 		  // 最小星数检测

    const uint32_t currentTimeUs = micros();

    if (rescueState.phase == RESCUE_IDLE) {
		// GPS救援处于空闲状态
        rescueState.failure = RESCUE_HEALTHY;
        return;
    } else if (rescueState.phase == RESCUE_INITIALIZE) {
        // 每次启动GPS救援初始化内部变量
        previousTimeUs = currentTimeUs;
        secondsStalled = 10; 				
        lastDistanceToHomeM = rescueState.sensor.distanceToHomeM;
        secondsFlyingAway = 0;
        secondsLowSats = 5;  				
        return;
    }

    // 在这些项目都完全测试之前不要中止
    if (rescueState.failure != RESCUE_HEALTHY) {
        if (gpsRescueConfig()->sanityChecks == RESCUE_SANITY_ON
            || (gpsRescueConfig()->sanityChecks == RESCUE_SANITY_FS_ONLY && rescueState.isFailsafe == true)) {
            rescueState.phase = RESCUE_ABORT;
        }
    }

    // 检查崩溃恢复模式是否激活，如果激活就解除
    if (crashRecoveryModeActive()) {
		// 检测到救援碰撞翻转
        rescueState.failure = RESCUE_CRASH_FLIP_DETECTED;
    }

    // 检查GPS通讯是否正常
    if (!rescueState.sensor.healthy) {
        rescueState.failure = RESCUE_GPSLOST;
    }

    //  应该以低刷新率运行的内容(如flyaway检测等)
    //  以约1hz的频率运行
    const uint32_t dTime = currentTimeUs - previousTimeUs;
    if (dTime < 1000000) { 				      // 1hz
        return;
    }

    previousTimeUs = currentTimeUs;

    if (rescueState.phase == RESCUE_CROSSTRACK) {
        secondsStalled = constrain(secondsStalled + ((rescueState.sensor.groundSpeed < 150) ? 1 : -1), 0, 20);

        if (secondsStalled == 20) {
            rescueState.failure = RESCUE_STALLED;
        }

        secondsFlyingAway = constrain(secondsFlyingAway + ((lastDistanceToHomeM < rescueState.sensor.distanceToHomeM) ? 1 : -1), 0, 10);
        lastDistanceToHomeM = rescueState.sensor.distanceToHomeM;

        if (secondsFlyingAway == 10) {
#ifdef USE_MAG
            // 如果磁力计并且没有被禁用，必须假定它是健康的并且已经在imu.c使用过
            if (sensors(SENSOR_MAG) && gpsRescueConfig()->useMag && !magForceDisable) {
                // 禁用mag后再试一次
                magForceDisable = true;
                secondsFlyingAway = 0;
            } else
#endif
            {
                rescueState.failure = RESCUE_FLYAWAY;
            }
        }
    }

    secondsLowSats = constrain(secondsLowSats + ((rescueState.sensor.numSat < gpsRescueConfig()->minSats) ? 1 : -1), 0, 10);

    if (secondsLowSats == 10) {
		// 卫星数量低
        rescueState.failure = RESCUE_LOWSATS;
    }
}

/**********************************************************************
函数名称：sensorUpdate
函数功能：传感器更新
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
static void sensorUpdate()
{
    rescueState.sensor.currentAltitudeCm = getEstimatedAltitudeCm();
    rescueState.sensor.healthy = gpsIsHealthy();

    // 计算高度速度
    static uint32_t previousTimeUs;
    static int32_t previousAltitudeCm;

    const uint32_t currentTimeUs = micros();
    const float dTime = currentTimeUs - previousTimeUs;

	// 计算最小公分母的速度
    if (newGPSData) { 
        rescueState.sensor.distanceToHomeM = GPS_distanceToHome;
        rescueState.sensor.directionToHome = GPS_directionToHome;
        rescueState.sensor.numSat = gpsSol.numSat;
        rescueState.sensor.groundSpeed = gpsSol.groundSpeed;

        rescueState.sensor.zVelocity = (rescueState.sensor.currentAltitudeCm - previousAltitudeCm) * 1000000.0f / dTime;
        rescueState.sensor.zVelocityAvg = 0.8f * rescueState.sensor.zVelocityAvg + rescueState.sensor.zVelocity * 0.2f;

        rescueState.sensor.accMagnitude = (float) sqrtf(sq(acc.accADC[Z]) + sq(acc.accADC[X]) + sq(acc.accADC[Y])) * acc.dev.acc_1G_rec;
        rescueState.sensor.accMagnitudeAvg = (rescueState.sensor.accMagnitudeAvg * 0.8f) + (rescueState.sensor.accMagnitude * 0.2f);

        previousAltitudeCm = rescueState.sensor.currentAltitudeCm;
        previousTimeUs = currentTimeUs;
    }
}

/**********************************************************************
函数名称：checkGPSRescueIsAvailable
函数功能：这个函数检查以下条件来确定GPS救援是否可用
函数形参：None  
函数返回值：GPS救援是否可用
函数描述：
	 1.传感器正常-正在接收GPS数据
	 2.GPS有一个有效的定位
	 3.GPS卫星数量低于GPS救援的最小配置
	 注意:这个函数不考虑到homepoint的距离等(gps_rescue_min_dth)也独立于gps_rescue_sanity_checks配置
**********************************************************************/
static bool checkGPSRescueIsAvailable(void)
{
	// Last time LowSat was checked
    static uint32_t previousTimeUs = 0; 
    const uint32_t currentTimeUs = micros();
	// Minimum sat detection
    static int8_t secondsLowSats = 0;   
    static bool lowsats = false;
    static bool noGPSfix = false;
    bool result = true;

    if (!gpsIsHealthy() || !STATE(GPS_FIX_HOME)) {
        return false;
    }

    //  应该以低刷新率>> ~1hz运行的东西
    const uint32_t dTime = currentTimeUs - previousTimeUs;
    if (dTime < 1000000) { //1hz
        if (noGPSfix || lowsats) {
            result = false;
        }
        return result;
    }

    previousTimeUs = currentTimeUs;

    if (!STATE(GPS_FIX)) {
        result = false;
        noGPSfix = true;
    } else {
        noGPSfix = false;
    }

    secondsLowSats = constrain(secondsLowSats + ((gpsSol.numSat < gpsRescueConfig()->minSats) ? 1 : -1), 0, 2);
    if (secondsLowSats == 2) {
        lowsats = true;
        result = false;
    } else {
        lowsats = false;
    }

    return result;
}

/**********************************************************************
函数名称：updateGPSRescueState
函数功能：确定所处的阶段，确定是否满足了所有的条件以进入下一个阶段
函数形参：None  
函数返回值：None  
函数描述：
	由gpsUpdate()调用。
**********************************************************************/
void updateGPSRescueState(void)
{
    static uint16_t newDescentDistanceM;
    static float_t lineSlope;
    static float_t lineOffsetM;
    static int32_t newSpeed;
    static int32_t newAltitude;
    float magnitudeTrigger;

	// 当前未处于GPS救援模式
    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
		// 救援停止 - GPS救援空闲状态
        rescueStop();
    } 
	// 当前处于GPS救援模式
	else if (FLIGHT_MODE(GPS_RESCUE_MODE) && rescueState.phase == RESCUE_IDLE) {
    	// 救援开始
        rescueStart();			
		// 救援高度位置控制并设置方向 - 偏航角速度在calculateSetpointRate中获取并输入到PID
        rescueAttainPosition(); 
		// 进行健康检查
        performSanityChecks(); 
    }

	// 失控保护激活状态
    rescueState.isFailsafe = failsafeIsActive();

	// 传感器更新
    sensorUpdate();

	// 确定GPS救援是否可用
    rescueState.isAvailable = checkGPSRescueIsAvailable();

	// 执行GPS救援模式阶段
    switch (rescueState.phase) {
	    case RESCUE_IDLE:							// -----------------------空闲状态
	        idleTasks();
	        break;
	    case RESCUE_INITIALIZE:						// -----------------------初始化状态
	        if (hoverThrottle == 0) { // 还没有实际的油门数据，让我们使用默认值。
	            hoverThrottle = gpsRescueConfig()->throttleHover;
	        }

	        throttle.Kp = gpsRescueConfig()->throttleP * GPS_RESCUE_THROTTLE_P_SCALE;
	        throttle.Ki = gpsRescueConfig()->throttleI * GPS_RESCUE_THROTTLE_I_SCALE;
	        throttle.Kd = gpsRescueConfig()->throttleD * GPS_RESCUE_THROTTLE_D_SCALE;

	        if (!STATE(GPS_FIX_HOME)) {
	            setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
	            disarm(DISARM_REASON_GPS_RESCUE);
	        }

	        // 最小距离检查
	        if (rescueState.sensor.distanceToHomeM < gpsRescueConfig()->minRescueDth) {
	            rescueState.failure = RESCUE_TOO_CLOSE;
	            // 当距离太近时，不要让救援模式作为自动防故障装置
	            if (rescueState.isFailsafe) {
					// 锁定并禁止解锁
	                setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
	                disarm(DISARM_REASON_GPS_RESCUE);
	            }
	            // 当未处于故障保护模式时:将其留给完整性检查设置
	        }

	        newSpeed = gpsRescueConfig()->rescueGroundspeed;
	        // 如果到家的实际距离较低，设置新的下降距离
	        if (rescueState.sensor.distanceToHomeM < gpsRescueConfig()->descentDistanceM) {
	            newDescentDistanceM = MAX(rescueState.sensor.distanceToHomeM - 5, GPS_RESCUE_MIN_DESCENT_DIST_M);
	        } else {
	            newDescentDistanceM = gpsRescueConfig()->descentDistanceM;
	        }

	        switch (gpsRescueConfig()->altitudeMode) {
	            case FIXED_ALT:
	                newAltitude = gpsRescueConfig()->initialAltitudeM * 100;
	                break;
	            case CURRENT_ALT:
	                newAltitude = rescueState.sensor.currentAltitudeCm;
	                break;
	            case MAX_ALT:
	            default:
	                newAltitude = MAX(gpsRescueConfig()->initialAltitudeM * 100, rescueState.sensor.maxAltitudeCm + 1500);
	                break;
	        }

	        // 计算在救援着陆时需要的两点直线方程的角系数和偏移量
	        lineSlope = ((float)gpsRescueConfig()->initialAltitudeM - gpsRescueConfig()->targetLandingAltitudeM) / (newDescentDistanceM - gpsRescueConfig()->targetLandingDistanceM);
	        lineOffsetM = gpsRescueConfig()->initialAltitudeM - lineSlope * newDescentDistanceM;

	        rescueState.phase = RESCUE_ATTAIN_ALT;
	        FALLTHROUGH;
	    case RESCUE_ATTAIN_ALT:						// -----------------------爬升到设定高度		
	        // 尽快以低速到达安全高度
	        if (ABS(rescueState.intent.targetAltitudeCm - rescueState.sensor.currentAltitudeCm) < 1000) {
	            rescueState.phase = RESCUE_CROSSTRACK;
	        }
	        rescueState.intent.targetGroundspeed = 500;
	        rescueState.intent.targetAltitudeCm = newAltitude;
	        rescueState.intent.crosstrack = true;
	        rescueState.intent.minAngleDeg = 10;
	        rescueState.intent.maxAngleDeg = 15;
	        break;
	    case RESCUE_CROSSTRACK:						// -----------------------执行飞行轨迹
	    	// 判断是否接近着陆
	        if (rescueState.sensor.distanceToHomeM <= newDescentDistanceM) {
	            rescueState.phase = RESCUE_LANDING_APPROACH;
	        }

			// 可以假设在这一点上处于或高于RTH高度，所以需要尝试并指向home和倾斜，同时保持高度
	        rescueState.intent.targetGroundspeed = gpsRescueConfig()->rescueGroundspeed;
	        rescueState.intent.targetAltitudeCm = newAltitude;
	        rescueState.intent.crosstrack = true;
	        rescueState.intent.minAngleDeg = 15;
	        rescueState.intent.maxAngleDeg = gpsRescueConfig()->angle;
	        break;
	    case RESCUE_LANDING_APPROACH:			    // -----------------------接近着陆
	        // 如果当前距家距离小于着陆距离并且当前高度小于着陆高度 -> 着陆
	        if (rescueState.sensor.distanceToHomeM <= gpsRescueConfig()->targetLandingDistanceM && rescueState.sensor.currentAltitudeCm <= gpsRescueConfig()->targetLandingAltitudeM * 100) {
	            rescueState.phase = RESCUE_LANDING;
	        }

	        // 只允许新高度和新速度等于或低于当前值(防止超调时抛物线运动)
	        const int32_t newAlt = MAX((lineSlope * rescueState.sensor.distanceToHomeM + lineOffsetM) * 100, 0);

	        // 当到终点的距离小于或等于GPS_RESCUE_SLOWDOWN_DISTANCE_M时，开始按比例减小四轴飞行器的速度
	        if (rescueState.sensor.distanceToHomeM <= GPS_RESCUE_SLOWDOWN_DISTANCE_M) {
	            newSpeed = gpsRescueConfig()->rescueGroundspeed * rescueState.sensor.distanceToHomeM / GPS_RESCUE_SLOWDOWN_DISTANCE_M;
	        }

	        rescueState.intent.targetAltitudeCm = constrain(newAlt, 100, rescueState.intent.targetAltitudeCm);
	        rescueState.intent.targetGroundspeed = constrain(newSpeed, 100, rescueState.intent.targetGroundspeed);
	        rescueState.intent.crosstrack = true;
	        rescueState.intent.minAngleDeg = 10;
	        rescueState.intent.maxAngleDeg = gpsRescueConfig()->angle;
	        break;
	    case RESCUE_LANDING:					    // -----------------------着陆
			// 已经到达起飞点（家），需要轻轻着陆
			// 不要让目标高度再上升，所以如果超飞，不会在抛物线轨道上移动
			// 如果加速度检测到的震级超过平均水平的150%，说明接触到地面，锁定飞行器
	        if (rescueState.sensor.currentAltitudeCm < GPS_RESCUE_ALMOST_LANDING_ALT) {
	            magnitudeTrigger = rescueState.sensor.accMagnitudeAvg * 1.2;
	        } else {
	            magnitudeTrigger = rescueState.sensor.accMagnitudeAvg * 1.5;
	        }
	        if (rescueState.sensor.accMagnitude > magnitudeTrigger) {
	            setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
	            disarm(DISARM_REASON_GPS_RESCUE);
	            rescueState.phase = RESCUE_COMPLETE;
	        }
	        rescueState.intent.targetGroundspeed = 0;
	        rescueState.intent.targetAltitudeCm = 0;
	        rescueState.intent.crosstrack = true;
	        rescueState.intent.minAngleDeg = 0;
	        rescueState.intent.maxAngleDeg = 15;
	        break;
	    case RESCUE_COMPLETE:			 		    // -----------------------救援完成
	    	// 救援停止
	        rescueStop();
	        break;	
	    case RESCUE_ABORT:							// -----------------------救援中止
	    	// 设置解锁禁用
	        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
			// 禁止解锁
	        disarm(DISARM_REASON_GPS_RESCUE);
			// 救援停止
	        rescueStop();
	        break;
	    default:
	        break;
    }

	// 进行健康检查
    performSanityChecks();

	// 高度位置控制（速度：AI_PITCH，高度：rescueThrottle）和方向控制（YAW）
    if (rescueState.phase != RESCUE_IDLE) {
        rescueAttainPosition();
    }

    newGPSData = false;
}

/**********************************************************************
函数名称：gpsRescueGetYawRate
函数功能：获取救援模式偏航角速度 - 救援模式开启时作为RC命令输入到PID控制器
函数形参：None  
函数返回值：rescueYaw
函数描述：
	calculateSetpointRate -> processRcCommand 
		-> subTaskRcCommand(RC命令子任务) -> taskMainPidLoop。
**********************************************************************/
float gpsRescueGetYawRate(void)
{
    return rescueYaw;
}

/**********************************************************************
函数名称：gpsRescueGetThrottle
函数功能：获取GPS救援模式油门 - 救援模式开启时获取并覆盖到混控器
函数形参：None  
函数返回值：commandedThrottle
函数描述：
	由mixTable调用。
**********************************************************************/
float gpsRescueGetThrottle(void)
{
	// 计算所需的命令油门比例从0.0-1.0在混控器中使用
	// 需要补偿min_check，因为油门值由gps救援设置基于原始rcCommand值
    float commandedThrottle = scaleRangef(rescueThrottle, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);
    commandedThrottle = constrainf(commandedThrottle, 0.0f, 1.0f);
    return commandedThrottle;
}

/**********************************************************************
函数名称：gpsRescueIsConfigured
函数功能：获取GPS救援模式是否配置
函数形参：None  
函数返回值：是否配置
函数描述：None 
**********************************************************************/
bool gpsRescueIsConfigured(void)
{
    return failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE || isModeActivationConditionPresent(BOXGPSRESCUE);
}

/**********************************************************************
函数名称：gpsRescueIsAvailable
函数功能：获取GPS救援模式是否可用
函数形参：None  
函数返回值：rescueState.isAvailable
函数描述：None 
**********************************************************************/
bool gpsRescueIsAvailable(void)
{
    return rescueState.isAvailable;
}

/**********************************************************************
函数名称：gpsRescueIsDisabled
函数功能：获取GPS救援模式是否禁用
函数形参：None  
函数返回值：是否禁用
函数描述：None 
**********************************************************************/
bool gpsRescueIsDisabled(void)
{
    return (!STATE(GPS_FIX_HOME));
}

/**********************************************************************
函数名称：gpsRescueDisableMag
函数功能：GPS救援模式禁用磁力计
函数形参：None  
函数返回值：GPS救援模式禁用磁力计
函数描述：None 
**********************************************************************/
#ifdef USE_MAG
bool gpsRescueDisableMag(void)
{
    return ((!gpsRescueConfig()->useMag || magForceDisable) && (rescueState.phase >= RESCUE_INITIALIZE) && (rescueState.phase <= RESCUE_LANDING));
}
#endif
#endif

