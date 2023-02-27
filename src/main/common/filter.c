/*********************************************************************************
 提供滤波相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

// ---------------------------------------------------------圆周率
#define M_PI_FLOAT  3.14159265358979323846f
// ---------------------------------------------------------二阶巴特沃斯品质因数
#define BIQUAD_Q 1.0f / sqrtf(2.0f)            

//-------------------------------------------------------------------------------------空滤波器API

/**********************************************************************
函数名称：nullFilterApply
函数功能：空滤波器 - 无滤波
函数形参：filter，input
函数返回值：None 
函数描述：None 
**********************************************************************/
float nullFilterApply(filter_t *filter, float input)
{
    UNUSED(filter);
    return input;
}

//-------------------------------------------------------------------------------------一阶巴特沃斯低通滤波器API

/**********************************************************************
函数名称：pt1FilterGain
函数功能：计算一阶巴特沃斯滤波增益
函数形参：截止频率，dT
函数返回值：None 
函数描述：
	参考模电RC滤波计算公式来计算截止频率：f = 1/2πRC
**********************************************************************/
float pt1FilterGain(float f_cut, float dT)
{
    float RC = 1 / ( 2 * M_PI_FLOAT * f_cut);
    return dT / (RC + dT);
}

/**********************************************************************
函数名称：pt1FilterInit
函数功能：PT1滤波初始化
函数形参：filter，k
函数返回值：None 
函数描述：
	重置滤波输出和滤波增益。
**********************************************************************/
void pt1FilterInit(pt1Filter_t *filter, float k)
{
    filter->state = 0.0f;
    filter->k = k;
}

/**********************************************************************
函数名称：pt1FilterUpdateCutoff
函数功能：PT1滤波更新截止频率 - 动态低通滤波使用
函数形参：filter，滤波增益
函数返回值：None 
函数描述：
	通过pt1FilterGain函数设置截止频率求出滤波增益。
**********************************************************************/
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k)
{
    filter->k = k;
}

/**********************************************************************
函数名称：pt1FilterUpdateCutoff
函数功能：PT1滤波器应用
函数形参：filter，输入
函数返回值：滤波输出
函数描述：
	低通滤波器 - 当噪声频率超过截止频率时被压制，而不是直接截断。
**********************************************************************/
float pt1FilterApply(pt1Filter_t *filter, float input)
{
	// 输出 =   上一个数据 + 增益 *（前后两次数据差）
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}

//-------------------------------------------------------------------------------------二阶巴特沃斯低通滤波器API

/**********************************************************************
函数名称：biquadFilterInit
函数功能：二阶巴特沃斯低通滤波器初始化
函数形参：filter，截止频率，刷新率
函数返回值：None 
函数描述：
	二阶IIR滤波器。
**********************************************************************/
void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate)
{
    // w0 = 2π*f0/Fs
    const float omega = 2.0f * M_PI_FLOAT * filterFreq * refreshRate * 0.000001f;
	// sin(w0)
    const float sn = sin_approx(omega);
	// cos(w0)
    const float cs = cos_approx(omega);
	// α = sin(w0) / 2Q
    const float alpha = sn / (2.0f * BIQUAD_Q);

	// 复位系数
    float b0 = 0, b1 = 0, b2 = 0, a0 = 0, a1 = 0, a2 = 0;
	// 计算低通滤波器系数
    b0 = (1 - cs) * 0.5f;
    b1 = 1 - cs;
    b2 = (1 - cs) * 0.5f;
    a0 = 1 + alpha;
    a1 = -2 * cs;
    a2 = 1 - alpha;

    // 预计算系数
    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;

    // 初始化样本为0
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}

/**********************************************************************
函数名称：biquadFilterApply
函数功能：二阶巴特沃斯低通滤波
函数形参：filter，输入
函数返回值：滤波输出
函数描述：None 
**********************************************************************/
float biquadFilterApply(biquadFilter_t *filter, float input)
{
	// 计算结果
    const float result = filter->b0 * input + filter->x1;
	// 计算x1、x2样本
    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;
    return result;
}

//-------------------------------------------------------------------------------------滞后平均移动滤波器API

/**********************************************************************
函数名称：laggedMovingAverageInit
函数功能：滞后平均移动滤波器初始化
函数形参：滤波结构体，窗口大小[过滤窗口]，缓存
函数返回值：None 
函数描述：
	复位滤波器相关参数。
**********************************************************************/
void laggedMovingAverageInit(laggedMovingAverage_t *filter, uint16_t windowSize, float *buf)
{
    filter->movingWindowIndex = 0;
    filter->windowSize = windowSize;
    filter->buf = buf;
    filter->movingSum = 0;
    memset(filter->buf, 0, windowSize * sizeof(float));
    filter->primed = false;
}

/**********************************************************************
函数名称：laggedMovingAverageUpdate
函数功能：滞后平均移动滤波更新
函数形参：filter结构体，输入
函数返回值：filter->movingSum       / denom
函数描述：
	移动平均的本质是一种低通滤波，目的是过滤掉时间序列中的高频扰动，保留有用的低频趋势。
	以滞后性的代价换来了平滑性，移动平均必须在平滑性和滞后性之间取舍。
	
	用于对前馈设置点增量进行滤波。
**********************************************************************/
float laggedMovingAverageUpdate(laggedMovingAverage_t *filter, float input)
{
	// 去除上次输入
    filter->movingSum -= filter->buf[filter->movingWindowIndex];
	// 保存当前输入
    filter->buf[filter->movingWindowIndex] = input;		
	// 累加当前输入
    filter->movingSum += input;
	// 判断是否达到窗口大小
    if (++filter->movingWindowIndex == filter->windowSize) {
		// 复位窗口索引
        filter->movingWindowIndex = 0;
		// 就绪
        filter->primed = true;
    }
	// 计算分母项 - 就绪：windowSize，未就绪：movingWindowIndex
    const uint16_t denom = filter->primed ? filter->windowSize : filter->movingWindowIndex;
	// 计算平均值
    return filter->movingSum  / denom;
}

//-------------------------------------------------------------------------------------中值滤波器API

/**********************************************************************
函数名称：quickMedianFilter3
函数功能：快速中值滤波实现[窗口为3] - 非线性平滑技术
函数形参：数据
函数返回值：中位值
函数描述：
	用于气压计数据滤波。
**********************************************************************/
// 交换 ab -> ba
#define QMF_SWAP(a,b)   { int32_t temp=(a);(a)=(b);(b)=temp; }
// 排序
#define QMF_SORT(a,b)   { if ((a)>(b)) QMF_SWAP((a),(b)); }
// 将v复制到p，数量为n
#define QMF_COPY(p,v,n) { int32_t i; for (i=0; i<n; i++) p[i]=v[i]; }
int32_t quickMedianFilter3(int32_t * v)
{
	// 窗口为3
    int32_t p[3];
	// 将v复制到p
    QMF_COPY(p, v, 3);
	// 排序
    QMF_SORT(p[0], p[1]); QMF_SORT(p[1], p[2]); QMF_SORT(p[0], p[1]) ;
	// 返回中位值 - 0 1 2，返回1
    return p[1];
}

