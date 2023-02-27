#pragma once

#include "common/axis.h"

#include "pg/pg.h"

/* --------------------------GPS救援配置结构体-------------------------- */	
typedef struct gpsRescue_s {
    uint16_t angle;                           // 角度
    uint16_t initialAltitudeM;                // 初始海报[米]
    uint16_t descentDistanceM;                // 下降距离[米]
    uint16_t rescueGroundspeed;               // 地面速度[厘米/秒]
    uint16_t throttleP, throttleI, throttleD; // 油门PID
    uint16_t yawP;							  // 偏航P
    uint16_t throttleMin;					  // 最小油门
    uint16_t throttleMax;					  // 最大油门
    uint16_t throttleHover;					  // 悬停油门
    uint16_t velP, velI, velD;				  // 速率 PID
    uint8_t minSats;						  // 最小卫星数
    uint16_t minRescueDth; 					  // meters
    uint8_t sanityChecks;					  // 安全检查
    uint8_t allowArmingWithoutFix;
    uint8_t useMag;
    uint16_t targetLandingAltitudeM; 		  // meters
    uint16_t targetLandingDistanceM; 		  // meters
    uint8_t altitudeMode;
    uint16_t ascendRate;
    uint16_t descendRate;
} gpsRescueConfig_t;
// 声明GPS救援配置结构体
PG_DECLARE(gpsRescueConfig_t, gpsRescueConfig);

extern int32_t gpsRescueAngle[ANGLE_INDEX_COUNT]; // 注意:角度的单位是CENTI DEGREES
void updateGPSRescueState(void);
void rescueNewGpsData(void);
float gpsRescueGetYawRate(void);
float gpsRescueGetThrottle(void);
bool gpsRescueIsConfigured(void);
bool gpsRescueIsAvailable(void);
bool gpsRescueIsDisabled(void);
bool gpsRescueDisableMag(void);

