#pragma once

#include <stdint.h>

// ---------------------------------------------------------平方
#ifndef sq
#define sq(x) ((x)*(x))
#endif
// ---------------------------------------------------------立方
#define power3(x) ((x)*(x)*(x))

// ---------------------------------------------------------sin/cos快速逼近
#define FAST_MATH             // 9阶近似
#define VERY_FAST_MATH        // 7阶近似

// ---------------------------------------------------------圆周率
#define M_PIf       3.14159265358979323846f

// ---------------------------------------------------------1度 = π/180弧度
#define RAD    (M_PIf / 180.0f)
// ---------------------------------------------------------度转分度
#define DEGREES_TO_DECIDEGREES(angle) ((angle) * 10)
// ---------------------------------------------------------分度转度
#define DECIDEGREES_TO_DEGREES(angle) ((angle) / 10)
// ---------------------------------------------------------度转弧度 - 1度=0.017453293弧度
#define DEGREES_TO_RADIANS(angle)     ((angle) * 0.0174532925f)
// ---------------------------------------------------------分度转弧度
#define DECIDEGREES_TO_RADIANS(angle) ((angle) / 10.0f * 0.0174532925f)
// ---------------------------------------------------------CM/S转换
#define CM_S_TO_KM_H(centimetersPerSecond) ((centimetersPerSecond) * 36 / 1000)
#define CM_S_TO_MPH(centimetersPerSecond)  ((centimetersPerSecond) * 10000 / 5080 / 88)

// ---------------------------------------------------------返回两者小值
#define MIN(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a < _b ? _a : _b; })
// ---------------------------------------------------------返回两者大值
#define MAX(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a > _b ? _a : _b; })
// ---------------------------------------------------------求绝对值
#define ABS(x)   \
  __extension__ ({ __typeof__ (x) _x = (x); \
  _x > 0 ? _x : -_x; })

// ---------------------------------------------------------频率转周期[秒]
#define HZ_TO_INTERVAL(x)    (1.0f / (x))

/* -------------------------方差计算结构体------------------------ */	
typedef struct stdev_s
{
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;

/* ------------------------t_fp向量类型结构体--------------------- */	
typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

/* -------------------------t_fp向量共用体------------------------ */	
typedef union u_fp_vector {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

/* -----------------------浮点欧拉角类型结构体-------------------- */	
// 注意，可以是度或弧度中的任意一个
typedef struct fp_angles {
    float roll;
    float pitch;
    float yaw;
} fp_angles_def;

/* ------------------------浮点欧拉角共用体----------------------- */	
typedef union {
    float raw[3];
    fp_angles_def angles;
} fp_angles_t;

/* -------------------------旋转矩阵结构体------------------------ */	
typedef struct fp_rotationMatrix_s {
    float m[3][3];             
} fp_rotationMatrix_t;

// -----------------------------------------------------------------------内联函数
// 在函数声明或定义中，函数返回类型前加上关键字inline。
// 可以解决一些频繁调用的函数大量消耗栈空间（栈内存）的问题。
// 1.一般函数的代码段只有一份，放在内存中的某个位置上，当程序调用它时，指令就跳转过来；
// 当下一次程序调用它时指令又跳转过来。
// 2.而内联函数是程序中调用几次内联函数，内联函数的代码就会复制几份放在对应的位置上。
// 没有指令跳转，指令按顺序执行。
// 3.内联函数一般在头文件中定义，而一般函数在头文件中声明，源文件中定义。
// 注意：内联函数一般只会用在函数内容非常简单的时候，内联函数的代码会在任何调用它的地方展开，
//       如果函数太复杂，代码膨胀带来的恶果很可能会大于效率的提高带来的益处。

/**********************************************************************
函数名称：constrain
函数功能：约束限制（整形）
函数形参：amt，low，high
函数返回值：None 
函数描述：None 
**********************************************************************/
static inline int constrain(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

/**********************************************************************
函数名称：constrain
函数功能：约束限制（浮点型）
函数形参：amt，low，high
函数返回值：None 
函数描述：None 
**********************************************************************/
static inline float constrainf(float amt, float low, float high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

int32_t applyDeadband(int32_t value, int32_t deadband);
float fapplyDeadband(float value, float deadband);
void devClear(stdev_t *dev);
void devPush(stdev_t *dev, float x);
float devVariance(stdev_t *dev);
float devStandardDeviation(stdev_t *dev);
float degreesToRadians(int16_t degrees);
int scaleRange(int x, int srcFrom, int srcTo, int destFrom, int destTo);
float scaleRangef(float x, float srcFrom, float srcTo, float destFrom, float destTo);
#if defined(FAST_MATH) || defined(VERY_FAST_MATH)
float sin_approx(float x);
float cos_approx(float x);
float atan2_approx(float y, float x);
float acos_approx(float x);
#define tan_approx(x)       (sin_approx(x) / cos_approx(x))
float exp_approx(float val);
float log_approx(float val);
float pow_approx(float a, float b);
#else
#define sin_approx(x)   	sinf(x)
#define cos_approx(x)   	cosf(x)
#define atan2_approx(y,x)   atan2f(y,x)
#define acos_approx(x)      acosf(x)
#define tan_approx(x)       tanf(x)
#define exp_approx(x)       expf(x)
#define log_approx(x)       logf(x)
#define pow_approx(a, b)    powf(b, a)
#endif

