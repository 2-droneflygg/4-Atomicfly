#pragma once

#include "common/axis.h"
#include "common/time.h"
#include "common/maths.h"
#include "pg/pg.h"

/* --------------------------四元数结构体-------------------------- */	
typedef struct {
    float w,x,y,z;
} quaternion;
// 数据初始化定义
#define QUATERNION_INITIALIZE  {.w=1, .x=0, .y=0,.z=0}

/* -------------------------四元数乘积结构体----------------------- */	
// 求解方向余弦矩阵时加速运算
typedef struct {
    float ww,wx,wy,wz,xx,xy,xz,yy,yz,zz;
} quaternionProducts;
// 数据初始化定义
#define QUATERNION_PRODUCTS_INITIALIZE  {.ww=1, .wx=0, .wy=0, .wz=0, .xx=0, .xy=0, .xz=0, .yy=0, .yz=0, .zz=0}

/* -------------------------姿态欧拉角共用体----------------------- */	
typedef union {
    int16_t raw[XYZ_AXIS_COUNT];
    struct {
        // 绝对倾角倾角以0.1度为倍数180度= 1800度
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } values;
} attitudeEulerAngles_t;
// 数据初始化定义
#define EULER_INITIALIZE  { { 0, 0, 0 } }

/* ---------------------------IMU配置结构体------------------------- */	
typedef struct imuConfig_s {
    uint16_t dcm_kp;                        // 误差补偿PI控制器比例增益(x 10000)
    uint16_t dcm_ki;                        // 误差补偿PI控制器积分增益(x 10000)
    uint8_t small_angle;					// 最大解锁角度
} imuConfig_t;
// 声明IMU配置结构体
PG_DECLARE(imuConfig_t, imuConfig);

/* -------------------------IMU运行配置结构体----------------------- */	
typedef struct imuRuntimeConfig_s {
    float dcm_ki;							// 标准误差补偿PI控制器比例增益
    float dcm_kp;							// 标准误差补偿PI控制器积分增益
} imuRuntimeConfig_t;

extern attitudeEulerAngles_t attitude;
extern int32_t accSum[XYZ_AXIS_COUNT];
extern float accAverage[XYZ_AXIS_COUNT];
void imuConfigure(void);
float getCosTiltAngle(void);
void imuUpdateAttitude(timeUs_t currentTimeUs);
void imuInit(void);
bool shouldInitializeGPSHeading(void);
bool isUpright(void);

