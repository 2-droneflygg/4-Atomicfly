/**********************************************************************
 提供一系列高度&&垂直速度相关API。
**********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

#include "platform.h"

#include "common/maths.h"

#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

/* --------------------------高度获取源枚举-------------------------- */	
typedef enum {
    DEFAULT = 0,									   // 主GPS,气压计互补
    BARO_ONLY,										   // 气压计
    GPS_ONLY			                               // GPS
} altSource_e;

PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 1);
PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .altSource = DEFAULT,                              // 高度源类型
);

static int32_t estimatedAltitudeCm = 0;                // in cm
#define BARO_UPDATE_FREQUENCY_40HZ (1000 * 25)		   // BARO更新频率40HZ

/**********************************************************************
函数名称：calculateEstimatedVario
函数功能：计算垂直速度
函数形参：baroAlt，dTime
函数返回值：垂直速度
函数描述：None
**********************************************************************/
#ifdef USE_VARIO
// 垂直速度： cm/s
static int16_t estimatedVario = 0;                     
int16_t calculateEstimatedVario(int32_t baroAlt, const uint32_t dTime) {
    static float vel = 0;
    static int32_t lastBaroAlt = 0;
    int32_t baroVel = 0;

	// 速度 = 距离 / 时间
    baroVel = (baroAlt - lastBaroAlt) * 1000000.0f / dTime;
    lastBaroAlt = baroAlt;

	// 限制约束
    baroVel = constrain(baroVel, -1500.0f, 1500.0f);
	// 应用死区
    baroVel = applyDeadband(baroVel, 10.0f);

    vel = vel * CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_cf_vel) + baroVel * (1.0f - CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_cf_vel));
    int32_t vel_tmp = lrintf(vel);
	// 应用死区
    vel_tmp = applyDeadband(vel_tmp, 5.0f);
	// 约束限制
    return constrain(vel_tmp, SHRT_MIN, SHRT_MAX);
}
#endif

#if defined(USE_BARO) || defined(USE_GPS)
/**********************************************************************
函数名称：calculateEstimatedAltitude
函数功能：计算高度和垂直速度
函数形参：当前时间节拍
函数返回值：None
函数描述：None
**********************************************************************/
// 高度偏移设置
static bool altitudeOffsetSet = false;
void calculateEstimatedAltitude(timeUs_t currentTimeUs)
{
	// 时间、高度偏移变量
    static timeUs_t previousTimeUs = 0;
    static int32_t baroAltOffset = 0;
    static int32_t gpsAltOffset = 0;

	// 计算时差
    const uint32_t dTime = currentTimeUs - previousTimeUs;
    if (dTime < BARO_UPDATE_FREQUENCY_40HZ) {
        return;
    }
    previousTimeUs = currentTimeUs;

	// 高度值缓冲缓冲变量
    int32_t baroAlt = 0;
    int32_t gpsAlt = 0;

	// GPS垂直速度缓冲变量
#if defined(USE_GPS) && defined(USE_VARIO)
    int16_t gpsVertSpeed = 0;
#endif

    float gpsTrust = 0.3;   	// GSP信赖值
    bool haveBaroAlt = false;
    bool haveGpsAlt = false;

	// ------------------------获取气压计高度
#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
		// 判断气压计是否校准
        if (!baroIsCalibrationComplete()) {
			// 未校准 - 执行气压校准循环
            performBaroCalibrationCycle();
        } else {
        	// 已校准 - 获取气压计高度
            baroAlt = baroCalculateAltitude();
            haveBaroAlt = true;
        }
    }
#endif

	// ------------------------获取GPS高度和垂直速度
#ifdef USE_GPS
    if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
		// 获取GPS高度
        gpsAlt = gpsSol.llh.altCm;
#ifdef USE_VARIO
		// 获取GPS垂直速度
        gpsVertSpeed = GPS_verticalSpeedInCmS;
#endif
        haveGpsAlt = true;

        if (gpsSol.hdop != 0) {
            gpsTrust = 100.0 / gpsSol.hdop;
        }
        // 如果有gps，使用至少10%的气压计互补
        gpsTrust = MIN(gpsTrust, 0.9f);
    }
#endif

	// ------------------------判断是否解锁并且偏移值是否被设置
    if (ARMING_FLAG(ARMED) && !altitudeOffsetSet) {
        baroAltOffset = baroAlt;
        gpsAltOffset = gpsAlt;
        altitudeOffsetSet = true;
    } else if (!ARMING_FLAG(ARMED) && altitudeOffsetSet) {
        altitudeOffsetSet = false;
    }
	
	// ------------------------减去偏移量
    baroAlt -= baroAltOffset;
    gpsAlt -= gpsAltOffset;

	// ------------------------1.已经获得GPS高度&&气压计高度&&高度源默认
    if (haveGpsAlt && haveBaroAlt && positionConfig()->altSource == DEFAULT) {
        if (ARMING_FLAG(ARMED)) {
			// 解锁 - GPS数据和气压计数据互补
            estimatedAltitudeCm = gpsAlt * gpsTrust + baroAlt * (1 - gpsTrust);
        } else {
        	// 解锁前显示绝对高度
            estimatedAltitudeCm = gpsAlt; 	
        }
#ifdef USE_VARIO
        // ----------计算垂直速度，气压计是一个更好的高度源，所以忽略GPS的垂直速度
        estimatedVario = calculateEstimatedVario(baroAlt, dTime);
#endif
    } 
	// ------------------------2.使用GPS高度
	else if (haveGpsAlt && (positionConfig()->altSource == GPS_ONLY || positionConfig()->altSource == DEFAULT )) {
        estimatedAltitudeCm = gpsAlt;
#if defined(USE_VARIO) && defined(USE_GPS)
        estimatedVario = gpsVertSpeed;
#endif
    } 
	// ------------------------3.使用气压计高度
	else if (haveBaroAlt && (positionConfig()->altSource == BARO_ONLY || positionConfig()->altSource == DEFAULT)) {
        estimatedAltitudeCm = baroAlt;
#ifdef USE_VARIO
        estimatedVario = calculateEstimatedVario(baroAlt, dTime);
#endif
    }
}

/**********************************************************************
函数名称：isAltitudeOffset
函数功能：获取高度偏移设置状态
函数形参：None
函数返回值：状态
函数描述：None
**********************************************************************/
bool isAltitudeOffset(void)
{
    return altitudeOffsetSet;
}
#endif

/**********************************************************************
函数名称：isAltitudeOffset
函数功能：获取估计高度[cm]
函数形参：None
函数返回值：estimatedAltitudeCm
函数描述：None
**********************************************************************/
int32_t getEstimatedAltitudeCm(void)
{
    return estimatedAltitudeCm;
}

/**********************************************************************
函数名称：getEstimatedVario
函数功能：获取垂直速度[cm/s]
函数形参：None
函数返回值：垂直速度，如果未启用垂直速度表功能则返回0
函数描述：None
**********************************************************************/
int16_t getEstimatedVario(void)
{
#ifdef USE_VARIO
    return estimatedVario;
#else
    return 0;
#endif
}

