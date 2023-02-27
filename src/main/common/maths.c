/*********************************************************************************
 提供数学计算相关API。
*********************************************************************************/
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"

#include "axis.h"
#include "maths.h"

// ---------------------------------------------------------相关宏定义
#if defined(FAST_MATH) || defined(VERY_FAST_MATH)
#if defined(VERY_FAST_MATH)
// sin_approx maximum absolute error = 2.305023e-06
// cos_approx maximum absolute error = 2.857298e-06
#define sinPolyCoef3 -1.666568107e-1f
#define sinPolyCoef5  8.312366210e-3f
#define sinPolyCoef7 -1.849218155e-4f
#define sinPolyCoef9  0
#else
// Double: -1.666665709650470145824129400050267289858e-1
#define sinPolyCoef3 -1.666665710e-1f             
// Double:  8.333017291562218127986291618761571373087e-3
#define sinPolyCoef5  8.333017292e-3f           
// Double: -1.980661520135080504411629636078917643846e-4
#define sinPolyCoef7 -1.980661520e-4f        
// Double:  2.600054767890361277123254766503271638682e-6
#define sinPolyCoef9  2.600054768e-6f             
#endif

//-------------------------------------------------------------------------------------三角函数近似相关API

/**********************************************************************
函数名称：sin_approx
函数功能：sin近似
函数形参：x
函数返回值：x的sin近似值
函数描述：None 
**********************************************************************/
float sin_approx(float x)
{
    int32_t xint = x;
	// 在错误输入时停止(5 * 360°)
    if (xint < -32 || xint > 32) return 0.0f;        
	// 总是将输入角度转换为 -PI..PI
    while (x >  M_PIf) x -= (2.0f * M_PIf);                                    
    while (x < -M_PIf) x += (2.0f * M_PIf);
	// 只是选择 -90..+90 Degree
    if (x >  (0.5f * M_PIf)) x =  (0.5f * M_PIf) - (x - (0.5f * M_PIf));       
    else if (x < -(0.5f * M_PIf)) x = -(0.5f * M_PIf) - ((0.5f * M_PIf) + x);
    float x2 = x * x;
    return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

/**********************************************************************
函数名称：cos_approx
函数功能：cos近似
函数形参：x
函数返回值：x的cos近似值
函数描述：None 
**********************************************************************/
float cos_approx(float x)
{
    return sin_approx(x + (0.5f * M_PIf));
}

/**********************************************************************
函数名称：atan2_approx
函数功能：atan2近似
函数形参：x
函数返回值：res
函数描述：
	Crashpilot1000的初始实现 (https://github.com/Crashpilot1000/HarakiriWebstore1/blob/396715f73c6fcf859e0db0f34e12fe44bace6483/src/mw.c#L1292)
	多项式系数，Andor (http://www.dsprelated.com/showthread/comp.dsp/21872-1.php) optimized by Ledvinap to save one multiplication
	最大绝对误差0,000027度
	atan2近似最大绝对值 error = 7.152557e-07 rads (4.098114e-05 degree)
**********************************************************************/
float atan2_approx(float y, float x)
{
    #define atanPolyCoef1  3.14551665884836e-07f
    #define atanPolyCoef2  0.99997356613987f
    #define atanPolyCoef3  0.14744007058297684f
    #define atanPolyCoef4  0.3099814292351353f
    #define atanPolyCoef5  0.05030176425872175f
    #define atanPolyCoef6  0.1471039133652469f
    #define atanPolyCoef7  0.6444640676891548f
	
    float res, absX, absY;
    absX = fabsf(x);
    absY = fabsf(y);
    res  = MAX(absX, absY);
    if (res) res = MIN(absX, absY) / res;
    else res = 0.0f;
    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
    if (absY > absX) res = (M_PIf / 2.0f) - res;
    if (x < 0) res = M_PIf - res;
    if (y < 0) res = -res;
    return res;
}

/**********************************************************************
函数名称：acos_approx
函数功能：acos近似
函数形参：x
函数返回值：result
函数描述：
	http://http.developer.nvidia.com/Cg/acos.html
	数学函数手册
	M. Abramowitz and I.A. Stegun, Ed.
	acos近似最大绝对值 error = 6.760856e-05 rads (3.873685e-03 degree)
**********************************************************************/
float acos_approx(float x)
{
    float xa = fabsf(x);
    float result = sqrtf(1.0f - xa) * (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));
    if (x < 0.0f)
        return M_PIf - result;
    else
        return result;
}
#endif

//-------------------------------------------------------------------------------------方差计算相关API

/**********************************************************************
函数名称：devClear
函数功能：方差计算样本数量清除
函数形参：dev
函数返回值：None 
函数描述：None 
**********************************************************************/
void devClear(stdev_t *dev)
{
	// 清除样本数量
    dev->m_n = 0;
}

/**********************************************************************
函数名称：devPush
函数功能：方差计算过程
函数形参：设备，x
函数返回值：None 
函数描述：
	MS均方差 - 方差是各数据偏离平均值差值的平方和的平均数。
**********************************************************************/
void devPush(stdev_t *dev, float x)
{
	// 样本数量
    dev->m_n++;    
    if (dev->m_n == 1) {
		// 初始化
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    } else {
		// 1.计算各数据偏离平均值差值
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
		// 2.计算平方和
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
		// 将本次数据记录供下一次使用
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

/**********************************************************************
函数名称：devVariance
函数功能：求方差
函数形参：dev
函数返回值：result
函数描述：
	方差描述随机变量对于数学期望的偏离程度。
**********************************************************************/
float devVariance(stdev_t *dev)
{
	// 3.求平均值
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

/**********************************************************************
函数名称：devStandardDeviation
函数功能：求标准差
函数形参：dev
函数返回值：result
函数描述：
	方差与要处理数据的量纲是不一致。
**********************************************************************/
float devStandardDeviation(stdev_t *dev)
{
	// 标准差是方差的平方根
    return sqrtf(devVariance(dev));
}

//-------------------------------------------------------------------------------------死区应用相关API

/**********************************************************************
函数名称：applyDeadband
函数功能：应用死区
函数形参：value,deadband
函数返回值：result
函数描述：None 
**********************************************************************/
int32_t applyDeadband(const int32_t value, const int32_t deadband)
{
	// ABS:绝对值
    if (ABS(value) < deadband) {    
        return 0;
    }
    return value >= 0 ? value - deadband : value + deadband;
}

/**********************************************************************
函数名称：fapplyDeadband
函数功能：应用死区（浮点）
函数形参：value,deadband
函数返回值：result
函数描述：None 
**********************************************************************/
float fapplyDeadband(const float value, const float deadband)
{
	// fabsf：返回浮点的绝对值
    if (fabsf(value) < deadband) {  
        return 0;
    }
    return value >= 0 ? value - deadband : value + deadband;
}

//-------------------------------------------------------------------------------------缩放范围相关API

/**********************************************************************
函数名称：scaleRange
函数功能：缩放范围
函数形参：x,源起点，源终点，目标起点，目标终点
函数返回值：result
函数描述：
	例： x = 150，源范围100 - 200，目标缩放范围1000 - 2000
		 a = 1000 * 50 = 50000
		 b = 100
		 result = 50000 / 100 + 1000 = 1500
**********************************************************************/
int scaleRange(int x, int srcFrom, int srcTo, int destFrom, int destTo) {
	// 目标范围 * x范围
    long int a = ((long int) destTo - (long int) destFrom) * ((long int) x - (long int) srcFrom);
	// 源范围
    long int b = (long int) srcTo - (long int) srcFrom;
    return (a / b) + destFrom;
}

/**********************************************************************
函数名称：scaleRange
函数功能：缩放范围（浮点）
函数形参：x,srcFrom，srcTo，destFrom，destTo
函数返回值：result
函数描述：None 
**********************************************************************/
float scaleRangef(float x, float srcFrom, float srcTo, float destFrom, float destTo) {
    float a = (destTo - destFrom) * (x - srcFrom);
    float b = srcTo - srcFrom;
    return (a / b) + destFrom;
}

//-------------------------------------------------------------------------------------角度转弧度相关API

/**********************************************************************
函数名称：degreesToRadians
函数功能：角度转弧度
函数形参：degrees
函数返回值：result
函数描述：
	1度 = π/180弧度。
**********************************************************************/
float degreesToRadians(int16_t degrees)
{
    return degrees * RAD;
}

