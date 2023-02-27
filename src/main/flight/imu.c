/**********************************************************************
 惯性导航单元 (IMU) - [采用Mahony互补滤波进行姿态解算]
	 四个概念：“地理”坐标系、“机体”坐标系、换算公式、换算公式系数。
		(1) 地理坐标系：东、北、天，以下简称地理。
			 在这个坐标系里有重力永远是（0,0,1g），地磁永远是（0,1,x）[不关心地磁的垂直]两个三维向量。
		(2) 机体坐标系：以下简称机体，上面有陀螺、加计、电子罗盘传感器，三个三维向量。
		(3) 换算公式：以下简称公式，公式就是描述机体姿态的表达方法，
			 一般都是用以地理为基准，从地理换算到机体的公式，有四元数、欧拉角、方向余弦矩阵。
		(4) 换算公式的系数：以下简称系数，四元数的q0123、欧拉角的ROLL/PITCH/YAW、余弦矩阵的9个数。
			 系数就是描述机体姿态的表达方法的具体数值。
		(5) 姿态，其实就是公式+系数的组合，一般经常用人容易理解的公式“欧拉角”表示，系数就是横滚xx度俯仰xx度航向xx度。
**********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/axis.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

// ---------------------------------------------------------积分极限(以度/秒为单位）
// 旋转速率
#define SPIN_RATE_LIMIT  20
// 250ms -姿态复位前解锁后的陀螺静周期
#define ATTITUDE_RESET_QUIET_TIME 250000   	
// 15度/秒-陀螺静周期的极限
#define ATTITUDE_RESET_GYRO_LIMIT 15      
// 姿态重置时使用的误差补偿Kp值
#define ATTITUDE_RESET_KP_GAIN    25.0   
// 500ms -等待姿态在高增益时收敛的时间
#define ATTITUDE_RESET_ACTIVE_TIME 500000  	
// 500厘米/秒的GPS航向的最小地面速度被认为是有效的
#define GPS_COG_MIN_GROUNDSPEED 500        		

// ---------------------------------------------------------acc
int32_t accSum[XYZ_AXIS_COUNT];
float accAverage[XYZ_AXIS_COUNT];

// ---------------------------------------------------------Cos(解锁最大角度)
static float smallAngleCosZ = 0;

// ---------------------------------------------------------IMU运行配置
static imuRuntimeConfig_t imuRuntimeConfig;

// ---------------------------------------------------------旋转矩阵
STATIC_UNIT_TESTED float rMat[3][3];

// ---------------------------------------------------------姿态是否计算完成
STATIC_UNIT_TESTED bool attitudeIsEstablished = false;

// ---------------------------------------------------------机体坐标系相对于地理坐标系的四元数
// 四元数
STATIC_UNIT_TESTED quaternion q = QUATERNION_INITIALIZE;
// 四元数乘积
STATIC_UNIT_TESTED quaternionProducts qP = QUATERNION_PRODUCTS_INITIALIZE;

// ---------------------------------------------------------绝对倾角倾角以0.1度为倍数180度= 1800度
attitudeEulerAngles_t attitude = EULER_INITIALIZE;

PG_REGISTER_WITH_RESET_TEMPLATE(imuConfig_t, imuConfig, PG_IMU_CONFIG, 1);
PG_RESET_TEMPLATE(imuConfig_t, imuConfig,
    .dcm_kp = 2500,                // 1.0 * 10000
    .dcm_ki = 0,                   // 0.003 * 10000
    .small_angle = 180,            // 最大解锁角度
);

/**********************************************************************
函数名称：imuQuaternionComputeProducts
函数功能：计算四元数平方
函数形参：quat,quatProd
函数返回值：None  
函数描述：
	求解方向余弦矩阵时加速运算。
**********************************************************************/
static void imuQuaternionComputeProducts(quaternion *quat, quaternionProducts *quatProd)
{
    quatProd->ww = quat->w * quat->w;
    quatProd->wx = quat->w * quat->x;
    quatProd->wy = quat->w * quat->y;
    quatProd->wz = quat->w * quat->z;
    quatProd->xx = quat->x * quat->x;
    quatProd->xy = quat->x * quat->y;
    quatProd->xz = quat->x * quat->z;
    quatProd->yy = quat->y * quat->y;
    quatProd->yz = quat->y * quat->z;
    quatProd->zz = quat->z * quat->z;
}

/**********************************************************************
函数名称：imuComputeRotationMatrix
函数功能：计算旋转矩阵
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
STATIC_UNIT_TESTED void imuComputeRotationMatrix(void)
{
	// 预先计算四元数平方 - 求解方向余弦矩阵时加速运算
    imuQuaternionComputeProducts(&q, &qP);
	// 计算旋转矩阵
    rMat[0][0] = 1.0f - 2.0f * qP.yy - 2.0f * qP.zz;
    rMat[0][1] = 2.0f * (qP.xy + -qP.wz);
    rMat[0][2] = 2.0f * (qP.xz - -qP.wy);

    rMat[1][0] = 2.0f * (qP.xy - -qP.wz);
    rMat[1][1] = 1.0f - 2.0f * qP.xx - 2.0f * qP.zz;
    rMat[1][2] = 2.0f * (qP.yz + -qP.wx);

    rMat[2][0] = 2.0f * (qP.xz + -qP.wy);
    rMat[2][1] = 2.0f * (qP.yz - -qP.wx);
    rMat[2][2] = 1.0f - 2.0f * qP.xx - 2.0f * qP.yy;
}

/**********************************************************************
函数名称：getCosTiltAngle
函数功能：得到Cos倾斜角
函数形参：None 
函数返回值：Cos倾斜角
函数描述：None 
**********************************************************************/
float getCosTiltAngle(void)
{
    return rMat[2][2];
}

/**********************************************************************
函数名称：imuConfigure
函数功能：IMU配置
函数形参：throttle_correction_angle，throttle_correction_value
函数返回值：None 
函数描述：None 
**********************************************************************/
void imuConfigure(void)
{
	// 计算姿态解锁误差补偿增益
    imuRuntimeConfig.dcm_kp = imuConfig()->dcm_kp / 10000.0f;
    imuRuntimeConfig.dcm_ki = imuConfig()->dcm_ki / 10000.0f;
	// 计算解锁最大角度 - 角度转弧度
    smallAngleCosZ = cos_approx(degreesToRadians(imuConfig()->small_angle));
}

/**********************************************************************
函数名称：isUpright
函数功能：获取IMU正确性
函数形参：None 
函数返回值：IMU正确性
函数描述：None 
**********************************************************************/
bool isUpright(void)
{
#ifdef USE_ACC
    return !sensors(SENSOR_ACC) || (attitudeIsEstablished && getCosTiltAngle() > smallAngleCosZ);
#else
    return true;
#endif
}

/**********************************************************************
函数名称：imuInit
函数功能：IMU初始化
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void imuInit(void)
{
	// 计算旋转矩阵
    imuComputeRotationMatrix();
}

/**********************************************************************
函数名称：shouldInitializeGPSHeading
函数功能：初始化GPS航向
函数形参：None 
函数返回值：初始化成功与否
函数描述：None 
**********************************************************************/
bool shouldInitializeGPSHeading()
{
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        return true;
    }
    return false;
}

#if defined(USE_ACC)
/**********************************************************************
函数名称：invSqrt
函数功能：计算平方根的倒数
函数形参：数据
函数返回值：结果
函数描述：None 
**********************************************************************/
static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

/**********************************************************************
函数名称：imuComputeQuaternionFromRPY
函数功能：IMU从RPY计算四元数
函数形参：四元数,初始化横滚,初始化俯仰,初始化偏航
函数返回值：None 
函数描述：None 
**********************************************************************/
#if defined(USE_GPS)
static void imuComputeQuaternionFromRPY(quaternionProducts *quatProd, int16_t initialRoll, int16_t initialPitch, int16_t initialYaw)
{
    if (initialRoll > 1800) {
        initialRoll -= 3600;
    }
    if (initialPitch > 1800) {
        initialPitch -= 3600;
    }
    if (initialYaw > 1800) {
        initialYaw -= 3600;
    }

    const float cosRoll = cos_approx(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);
    const float sinRoll = sin_approx(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);

    const float cosPitch = cos_approx(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);
    const float sinPitch = sin_approx(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);

    const float cosYaw = cos_approx(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);
    const float sinYaw = sin_approx(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);

    const float q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    const float q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    const float q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    const float q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

    quatProd->xx = sq(q1);
    quatProd->yy = sq(q2);
    quatProd->zz = sq(q3);

    quatProd->xy = q1 * q2;
    quatProd->xz = q1 * q3;
    quatProd->yz = q2 * q3;

    quatProd->wx = q0 * q1;
    quatProd->wy = q0 * q2;
    quatProd->wz = q0 * q3;

	// 计算旋转矩阵
    imuComputeRotationMatrix();

	// 姿态计算完成
    attitudeIsEstablished = true;
}
#endif

/**********************************************************************
函数名称：imuUpdateEulerAngles
函数功能：获取IMU加速度计健康状态
函数形参：accAverage
函数返回值：状态
函数描述：
	判断加速度计的读数模长是否在0.90g - 1.10g范围。
**********************************************************************/
static bool imuIsAccelerometerHealthy(float *accAverage)
{
    float accMagnitudeSq = 0;
    for (int axis = 0; axis < 3; axis++) {
        const float a = accAverage[axis];
        accMagnitudeSq += a * a;
    }

    accMagnitudeSq = accMagnitudeSq * sq(acc.dev.acc_1G_rec);

    // 接受加速度读数范围只有0.9g - 1.1g
    return (0.81f < accMagnitudeSq) && (accMagnitudeSq < 1.21f);
}

/**********************************************************************
函数名称：imuCalcKpGain
函数功能：计算要姿态解算误差补偿Kp增益（当解锁时，增益为imuRuntimeConfig[dcm_kp * 1.0缩放]）
函数形参：currentTimeUs，useAcc，gyroAverage
函数返回值：误差补偿Kp增益
函数描述：
  比例项：控制传感器的可信度
   Kp越大越相信加速度计；
   Kp越小越相信陀螺仪。
		当初始启动后解锁，增加Kp值，使其在解锁时加快初始收敛;
		解锁后，希望快速恢复收敛，以应对由于碰撞造成的姿态估计不正确
		未解锁状态[进行姿态复位]:
			- 等待一个低陀螺仪活动250毫秒的周期，以确保飞行器静止
			- 使用一个大的dcmKpGain值500ms，使姿态估计快速收敛
			- 重置增益到标准设置
**********************************************************************/
static float imuCalcKpGain(timeUs_t currentTimeUs, bool useAcc, float *gyroAverage)
{
	// 上一个解锁状态
    static bool lastArmState = false;
	// 陀螺静周期结束时间
    static timeUs_t gyroQuietPeriodTimeEnd = 0;
	// 姿态复位结束时间
    static timeUs_t attitudeResetTimeEnd = 0;
	// 姿态复位完成状态
    static bool attitudeResetCompleted = false;
	// 姿态复位激活状态
    bool attitudeResetActive = false;
	// 误差补偿Kp增益
    float ret;

	// --------------------------------------------------------------------------获取解锁状态
    const bool armState = ARMING_FLAG(ARMED);

	// --------------------------------------------------------------------------未解锁状态[进行姿态复位]
    if (!armState) {
		// ------------------------------如果之前解锁过，开始陀螺仪静周期
        if (lastArmState) {   
			// 更新陀螺静仪周期结束时间 = 当前时间节拍 + 250ms（姿态复位前解锁后的陀螺静周期）
            gyroQuietPeriodTimeEnd = currentTimeUs + ATTITUDE_RESET_QUIET_TIME;
			// 重置姿态复位结束时间
            attitudeResetTimeEnd = 0;
			// 姿态复位未完成
            attitudeResetCompleted = false;
        }

		// 姿态复位未结束 || 
		// 陀螺静仪周期未结束(陀螺仪活动超过阈值，重新启动静周期) || 
		// 姿态复位完成(有后续陀螺仪活动，重新启动重置周期,解决飞机坠毁后恢复飞行的情况)
        if ((attitudeResetTimeEnd > 0) || (gyroQuietPeriodTimeEnd > 0) || attitudeResetCompleted) {
			// 判断陀螺仪是否活动 - 超过阈值则重新启动陀螺仪静周期
            if ((fabsf(gyroAverage[X]) > ATTITUDE_RESET_GYRO_LIMIT)
                || (fabsf(gyroAverage[Y]) > ATTITUDE_RESET_GYRO_LIMIT)
                || (fabsf(gyroAverage[Z]) > ATTITUDE_RESET_GYRO_LIMIT)
                || (!useAcc)) {
                // 更新陀螺静周期结束时间 = 当前时间节拍 + 250ms（姿态复位前解锁后的陀螺静周期）
                gyroQuietPeriodTimeEnd = currentTimeUs + ATTITUDE_RESET_QUIET_TIME;
				// 重置姿态复位结束时间
                attitudeResetTimeEnd = 0;
            }
        }
		// 判断姿态复位是否结束
        if (attitudeResetTimeEnd > 0) {     
			// 判断姿态复位是否结束
            if (currentTimeUs >= attitudeResetTimeEnd) {
				// ------姿态复位结束
				// 重置陀螺静周期结束时间
                gyroQuietPeriodTimeEnd = 0;
				// 重置姿态复位结束时间
                attitudeResetTimeEnd = 0;
				// 姿态复位完成
                attitudeResetCompleted = true;
            } else {
				// ------姿态复位未结束 - 姿态复位激活
                attitudeResetActive = true;
            }
        } else if ((gyroQuietPeriodTimeEnd > 0) && (currentTimeUs >= gyroQuietPeriodTimeEnd)) {
            // 陀螺仪静周期结束 - 开始高增益周期，快速收敛
            attitudeResetTimeEnd = currentTimeUs + ATTITUDE_RESET_ACTIVE_TIME;
            gyroQuietPeriodTimeEnd = 0;
        }
    }
	// 更新解锁状态
    lastArmState = armState;

	// 判断姿态复位是否激活 - 在姿态复位期保持较高Kp值，反之则使用标准Kp值
    if (attitudeResetActive) {
		// 姿态重置时使用的误差补偿Kp值 - 25.0
		ret = ATTITUDE_RESET_KP_GAIN;
    } else {
		// 使用标准Kp值 - 0.25
		ret = imuRuntimeConfig.dcm_kp;
		if (!armState) {
		  // 未解锁时增加kP，使其在解锁时收敛得更快 - 2.5
		  ret = ret * 10.0f; 
		}
    }
    return ret;
}

/**********************************************************************
函数名称：imuMahonyAHRSupdate
函数功能：IMU AHRS更新
函数形参：时间常数，陀螺仪x，陀螺仪y，陀螺仪z
		  是否使用加速度计，加速度x，加速度y，加速度z
		  是否使用磁力计
		  是否使用使用GPS纠正姿态值偏航，GPS纠正姿态偏航值，比例增益
函数返回值：None 
函数描述：
	运用Mahony互补滤波算法[通过加速度计与磁力计对陀螺仪数据进行误差补偿]：
	  [角速度积分 ->(存在误差累积) 造成积分偏移(低频干扰)]
		在实际应用中，角速度测量通常不为理想状态，进而求解不出准确四元数，
	  [加速度正交 ->(存在机身振动) 造成振动误差(高频干扰)]
	  	通过加速度计补偿俯仰横滚。
	  	通过磁力计补偿偏航。
**********************************************************************/
static void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
                                bool useAcc, float ax, float ay, float az,
                                bool useMag,
                                bool useCOG, float courseOverGround, const float dcmKpGain)
{
	// 积分误差项由Ki缩放
    static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    

    // --------------------------------------------------------------------------1.计算旋转速率(rad/s)
    const float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));

	// --------------------------------------------------------------------------2.纠正姿态值偏航
    // ---------------------------------------------------------------(1)使用GPS纠正姿态值偏航
    float ex = 0, ey = 0, ez = 0;
    if (useCOG) {
		// 限制GPS纠正姿态值偏航
        while (courseOverGround >  M_PIf) {
            courseOverGround -= (2.0f * M_PIf);
        }
        while (courseOverGround < -M_PIf) {
            courseOverGround += (2.0f * M_PIf);
        }

        const float ez_ef = (- sin_approx(courseOverGround) * rMat[0][0] - cos_approx(courseOverGround) * rMat[1][0]);
        ex = rMat[2][0] * ez_ef;
        ey = rMat[2][1] * ez_ef;
        ez = rMat[2][2] * ez_ef;
    }

	// ---------------------------------------------------------------(2)使用磁力计纠正姿态值偏航
	//	- 磁力计数据归一化得到mx,my,mz，将数据转换到机体系得到hx，hy，hz
	//		 求hx和hy的平方和的平方根，得到bx，0，hz，忽略z轴的数据
	//		[bx，0，hz]和[hx，hy，hz]做叉乘得到误差即[0,0,-(hy * bx)]转化到机体系记得到误差
#ifdef USE_MAG
    // 获取磁力计数据
    float mx = mag.magADC[X];
    float my = mag.magADC[Y];
    float mz = mag.magADC[Z];
	// 磁力计测量的向量（机体坐标系）
    float recipMagNorm = sq(mx) + sq(my) + sq(mz);
    if (useMag && recipMagNorm > 0.01f) {
        // 磁力计归一化（把磁力计的三维向量转换为单位向量，因为是单位矢量到参考性的投影，只是按照比例缩小，并没有改变方向）
        recipMagNorm = invSqrt(recipMagNorm);
        mx *= recipMagNorm;
        my *= recipMagNorm;
        mz *= recipMagNorm;
		// 对于磁力计的校正，假设磁场垂直于重力(忽略z分量)
		// 这样磁场只会影响航向，不会影响滚转角/俯仰角
		// (hx;衔接;0) - 测量的mag场向量(假设z分量为零)
		// (bx;0;0) - 指向正北的参考mag场矢量(假设z分量为零)
        const float hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
        const float hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
        const float bx = sqrtf(hx * hx + hy * hy);
        // 磁力计误差是估计磁北和测量磁北的叉积
        const float ez_ef = -(hy * bx);
        // 旋转mag误差矢量和累积
        ex += rMat[2][0] * ez_ef;
        ey += rMat[2][1] * ez_ef;
        ez += rMat[2][2] * ez_ef;
    }
#else
    UNUSED(useMag);
#endif

    // --------------------------------------------------------------------------3.利用加速度计补偿角速度误差[Pitch，Roll]
    // ---------------------------------------------------------------(1)计算加速度误差
    // 加速度计测量的重力向量（机体坐标系）
    float recipAccNorm = sq(ax) + sq(ay) + sq(az);
    if (useAcc && recipAccNorm > 0.01f) {
        // 加速度归一化（把加速度计的三维向量转换为单位向量，因为是单位矢量到参考性的投影，只是按照比例缩小，并没有改变方向）
        recipAccNorm = invSqrt(recipAccNorm);
        ax *= recipAccNorm;
        ay *= recipAccNorm;
        az *= recipAccNorm;
        // 向量叉乘得出的值（向量间的误差，可以用向量叉积来表示）
        ex += (ay * rMat[2][2] - az * rMat[2][1]);
        ey += (az * rMat[2][0] - ax * rMat[2][2]);
        ez += (ax * rMat[2][1] - ay * rMat[2][0]);
    }

	// ---------------------------------------------------------------(2)互补滤波，修正角速度积分漂移[低频干扰]
    // ------------------------------------------------如果启用，计算并应用积分反馈
    if (imuRuntimeConfig.dcm_ki > 0.0f) {
        // 使用积分反馈 
        if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
			// 如果旋转速率超过一定的限度，停止积分 - 对于积分常数做了防止饱和处理，只有在未饱和的时候才加入积分误差处理
            const float dcmKiGain = imuRuntimeConfig.dcm_ki;
			// 积分误差乘以Ki
            integralFBx += dcmKiGain * ex * dt;    
            integralFBy += dcmKiGain * ey * dt;
            integralFBz += dcmKiGain * ez * dt;
        }
    } else {
    	// 不使用积分反馈
        integralFBx = 0.0f;   					   
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }
    // ------------------------------------------------应用比例和积分反馈 - 角速度融合加速度积分补偿值
    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;

    // --------------------------------------------------------------------------4.一阶龙格库塔法求解微分方程得出四元数进而反解出欧拉角
    /*
		|q0|		|0 		-w(bx)  -w(by)	-w(bz)||q0|
		|q1| = 0.5 	|w(bx)	0		w(bz)	-w(by)||q1|
		|q2|		|w(by)	-w(bz)	0		w(bx) ||q2|
		|q3|		|w(bz)	w(by)	-w(bx)	0	  ||q3|
    */
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
	
    quaternion buffer;
    buffer.w = q.w;
    buffer.x = q.x;
    buffer.y = q.y;
    buffer.z = q.z;

    q.w += (-buffer.x * gx - buffer.y * gy - buffer.z * gz);
    q.x += (+buffer.w * gx + buffer.y * gz - buffer.z * gy);
    q.y += (+buffer.w * gy - buffer.x * gz + buffer.z * gx);
    q.z += (+buffer.w * gz + buffer.x * gy - buffer.y * gx);

    // --------------------------------------------------------------------------6.四元数归一化
    float recipNorm = invSqrt(sq(q.w) + sq(q.x) + sq(q.y) + sq(q.z));
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;

	// --------------------------------------------------------------------------7.计算旋转矩阵
    imuComputeRotationMatrix();

	// 姿态计算完成
    attitudeIsEstablished = true;
}

/**********************************************************************
函数名称：imuUpdateEulerAngles
函数功能：IMU更新欧拉角
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
STATIC_UNIT_TESTED void imuUpdateEulerAngles(void)
{
    quaternionProducts buffer;
	// 四舍五入
	attitude.values.roll = lrintf(atan2_approx(rMat[2][1], rMat[2][2]) * (1800.0f / M_PIf));
	attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(-rMat[2][0])) * (1800.0f / M_PIf));
	attitude.values.yaw = lrintf((-atan2_approx(rMat[1][0], rMat[0][0]) * (1800.0f / M_PIf)));
    if (attitude.values.yaw < 0) {
        attitude.values.yaw += 3600;
    }
}

/**********************************************************************
函数名称：imuCalculateEstimatedAttitude
函数功能：IMU姿态计算 - 求解欧拉角
函数形参：当前时间节拍
函数返回值：None 
函数描述：None 
**********************************************************************/
static void imuCalculateEstimatedAttitude(timeUs_t currentTimeUs)
{
	// 变量定义和求时间间隔
    static timeUs_t previousIMUUpdateTime;
	// 是否使用加速度计
    bool useAcc = false;
	// 是否使用磁力计纠正姿态偏航
    bool useMag = false;
	// 是否通过imumahonyar修正偏航
    bool useCOG = false; 		
	// 当useCOG为true时使用,存储弧度
    float courseOverGround = 0; 		

	// ---------------------------------------------------------1.计算时差 [姿态解算的计算周期]
    const timeDelta_t deltaT = currentTimeUs - previousIMUUpdateTime;
    previousIMUUpdateTime = currentTimeUs;

	// ---------------------------------------------------------2.获取纠正姿态偏航方式
	// -----------------------------(1)判断磁力计是否可用[useMag] - 与GPS救援模式有关
#ifdef USE_MAG
    if (sensors(SENSOR_MAG) && compassIsHealthy()
#ifdef USE_GPS_RESCUE
        && !gpsRescueDisableMag()
#endif
        ) {
        useMag = true;
    }
#endif
	// -----------------------------(2)如果未使用磁力计，使用GPS来纠正姿态偏航[useCOG]
#if defined(USE_GPS)
    if (!useMag && sensors(SENSOR_GPS) && STATE(GPS_FIX) && gpsSol.numSat >= 5 && gpsSol.groundSpeed >= GPS_COG_MIN_GROUNDSPEED) {
        // 使用GPS纠正姿态值偏航 - 分度转弧度
        courseOverGround = DECIDEGREES_TO_RADIANS(gpsSol.groundCourse);
        useCOG = true;
		// 初始化GPS航向，理想情况下，这种情况在每次飞行中可能会发生不止一次，但现在，shouldinitializegpheading()只返回true一次
        if (useCOG && shouldInitializeGPSHeading()) {
			// IMU从RPY计算四元数，重置引用并重新初始化四元数
            imuComputeQuaternionFromRPY(&qP, attitude.values.roll, attitude.values.pitch, gpsSol.groundCourse);
			// 第一次重新初始化时不要使用COG
            useCOG = false; 
        }
    }
#endif

	// ---------------------------------------------------------3.获取陀螺仪数据累积平均值[角速度]
    float gyroAverage[XYZ_AXIS_COUNT];
    gyroGetAccumulationAverage(gyroAverage);

	// ---------------------------------------------------------4.判断加速度计是否可用[useAcc]
    if (accGetAccumulationAverage(accAverage)) {
        useAcc = imuIsAccelerometerHealthy(accAverage);
    }

	// ---------------------------------------------------------5.更新AHRS
    imuMahonyAHRSupdate(deltaT * 1e-6f,
                        DEGREES_TO_RADIANS(gyroAverage[X]), DEGREES_TO_RADIANS(gyroAverage[Y]), DEGREES_TO_RADIANS(gyroAverage[Z]),
                        useAcc, accAverage[X], accAverage[Y], accAverage[Z],
                        useMag,
                        useCOG, courseOverGround,  imuCalcKpGain(currentTimeUs, useAcc, gyroAverage));
	// ---------------------------------------------------------6.更新欧拉角
    imuUpdateEulerAngles();
}

/**********************************************************************
函数名称：imuUpdateAttitude
函数功能：IMU更新姿态
函数形参：currentTimeUs
函数返回值：None 
函数描述：
	由调度器以任务形式调用 - 100Hz。
**********************************************************************/
void imuUpdateAttitude(timeUs_t currentTimeUs)
{
	// 判断加速度计是否开启
	// acc.isAccelUpdatedAtLeastOnce在任务taskUpdateAccelerometer里，通过调用accUpdate将acc.isAccelUpdatedAtLeastOnce = true;
    if (sensors(SENSOR_ACC) && acc.isAccelUpdatedAtLeastOnce) {
		// -----------------------------------------------------求解欧拉角
        imuCalculateEstimatedAttitude(currentTimeUs);
    } else {
        acc.accADC[X] = 0;
        acc.accADC[Y] = 0;
        acc.accADC[Z] = 0;
    }
}
#endif // USE_ACC

