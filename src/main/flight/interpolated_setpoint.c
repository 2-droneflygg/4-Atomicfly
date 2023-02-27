/**********************************************************************
FeedForward： - （动态调节P）
	1.前馈（FF）是增加操纵杆响应能力的驱动因素，是一个基于摇杆输入的额外提升参数。
	2.前馈（FF）与杆运动的瞬时导数或“变化率”成正比，摇杆移动得越快，得到的前馈就越多（动态摇杆响应）。
	3.帮助P值来推动飞机更快地响应摇杆的移动，与P不同，无论添加多少FF，前馈都不会引起振荡。
	4.使用前馈，可以获得更好的摇杆响应性，而不会将P推得太高而引起震荡。
	5.减少了输入和响应之间的延迟时间，更少的延迟意味着更少的错误，更少的I结束/超调。
	6.更高的增益值可以使飞机感觉更贴手，太高的值会导致过冲、电机发热量增加和电机饱和（电机无法跟上期望的速率），
	  较低或零值可使飞机手感更为平滑。
	7.FeedForward过多会导致：
		1.翻转开始时会出现过冲，尤其是当摇杆达到100％行程时;
		2.对RC步的过分响应;
		3.飞行员在寒冷或紧张时会发抖，电机走线出现尖峰，并带有较大的RC阶跃，短暂晃动;
		4.陀螺仪超越设定点;

引入了几种ff_2.0改进这些局限性的新颖技术：
ff_boost ：
	可显著减少快速设定值更改的延迟。
	大多数电机需要时间来加速/减速。在行动的一开始，就需要比在中期更加用力地去推动。
	FF和P在移动开始时均缓慢上升，这是因为最初手指移动缓慢，并且大多数在其Rates设置中使用Expo。
	因此，直到现在，除非真正使用了很高的FF增益（250+），否则电机并不需要立即去推动。
	但是，由于FF具有如此大的增益，因此很难控制超调，特别是由于在杆行程结束时应用了SuperRate，当需要四轴驱动开始减慢速度而不至于超调时，确实会导致FF推动右移。
	'ff_boost'是与摇杆加速度成比例的PID参数。从技术上讲，它是设定值的二阶导数。
	作为二阶导数，ff_boost在摇杆开始移动的那一刻就真正达到了峰值。
	然后，当摇杆处于中间行程时，它减小到零，因为在恒定的摇杆速度下，有很多FF，但是没有加速度，ff_boost最终随着操纵杆的速度变慢而变为负值。
	完全可以满足我们克服电机延迟所需的工作-提前推动，缓和中间移动并最终主动降低电机速度。
	使用适量的FF和ff_boost，四轴响应可以几乎完全无滞后地跟踪输入，而不会产生过冲。
	通常set ff_boost = 15，对于大多数四轴，默认值大约是正确的。
	较大的值可用于具有慢速线轴的电机。
	增强的强度直接与PID中设置的FF数量有关。将ff_boost视为“因数”，可调节FF应用于电机的时序。更大的提升使FF更早地投入使用。
	理想情况下，当boost / FF分量正确时，P应该要做的工作很少。
	ff_boost可以在没有其他任何ff功能处于活动状态的情况下工作。

ff_interpolate_sp ：
	从每个新的RC“步”计算FF的“数字”方式。提供了更清晰的前馈跟踪，并且延迟更少。
	set ff_interpolate_sp = ON分析每个新的传入RC数据包，并将计算出的设定值变化转换为FF的立即递增。
	每次递增都保持恒定，直到下一个RC数据步到达为止。
	丢弃的RC数据包通常会导致FF突然降为零。 set ff_interpolate_sp = AVERAGE旨在帮助解决此特定问题。

ff_max_rate_limit ：
	减弱/防止FF在翻转开始时引起的过冲。
	当执行快速翻转或滚动时，摇杆通常会在达到行程的物理极限时突然停止。
	通过将“ Expo”和“ SuperRate”应用于速率，设定值的增加率在摇杆停止之前最大，在这个时间点上，四轴本身旋转得非常快，并具有大量的旋转动量。
	Expo \ SuperRate效果也意味着FF也绝对庞大。所有这些，都在摇杆突然停止之前。
	即使有很多D，也大幅度过冲是不可避免的。
	ff_max_rate_limit可预测性地确定摇杆可能会达到其极限的情况，并在预测到这种情况时将FF减少。
	它具有“前瞻性”，可以在适当的时候先发制人地减少FF，通常完全消除了否则会发生的过冲。
	一个主要的好处是，显著减少了对置电动机旋转的需求。翻转变得比以前更干净，更准确。
	默认值100很好，并且当陀螺仪值达到设定值时，尝试达到FF输出0。
	较低的值会导致前馈影响以这些值的百分比逐渐减小为0（例如，在持续翻转时为50，一旦陀螺仪达到设定值的一半，前馈项将逐渐减小为零）。
	较高的值允许前馈继续影响PID控制器。
	如果过冲，尝试ff_max_rate_limit = 95。如果过冲控制得太好，尝试105至110。调整范围非常狭窄。

ff_interpolate_sp = 平均 ：
	由于为每个新的RC步计算了FF，因此当新RC步没有按预期到达时，就会遇到问题。
	通常，这是由“丢包”引起的，。当发生这种情况时，FF通常会突然降至零；当下一个有效数据值进入时，将发生双倍高度提升。FF跟踪中的这些零/双高对非常混乱。
	在AVERAGED模式下，插值算法将每两个连续的前馈值取平均值。
	如果突然下降到零而随后又有较大的上升，则将其更改为较小的下降，中间值和跳跃。总体而言，步距更小。
	使用会有一个缺点AVERAGED。如果进行快速输入，然后突然将操纵杆完全静止不动，则ff_spread将FF保持为设定时间的前一个高值，而不是立即降为零。
	在扩展值设置的持续时间内，这可能会导致过冲。当摇杆达到其物理极限时，最容易在翻转开始时看到它，但是在其他时间也可能发生。
	AVERAGED 对于具有大量RC平滑功能的远航/拍摄四轴最有用。
	对于竞速和普通用户而言，RC链路的质量主要取决于天线的质量以及计划飞行的距离。天线良好的近距离竞速，可以在不求平均的情况下发挥最佳性能。
**********************************************************************/
#include <math.h>
#include "platform.h"

#ifdef USE_INTERPOLATED_SP
#include "common/maths.h"
#include "fc/rc.h"
#include "flight/interpolated_setpoint.h"


static float setpointDeltaImpl[XYZ_AXIS_COUNT];				// 设置点增量接口
static float setpointDelta[XYZ_AXIS_COUNT];					// 设置点增量
static uint8_t holdCount[XYZ_AXIS_COUNT];					// 摇杆保持类型

static float prevSetpointSpeed[XYZ_AXIS_COUNT];             // 上一个设置点速度
static float prevAcceleration[XYZ_AXIS_COUNT];				// 上一个设置点加速度
static float prevRawSetpoint[XYZ_AXIS_COUNT];				// 上一个原始设置点（期望速率值）
static float prevDeltaImpl[XYZ_AXIS_COUNT];					// 上一个增量接口
static bool bigStep[XYZ_AXIS_COUNT];						// 大步距状态缓存
static uint8_t averagingCount;								// 平均计数

static float ffMaxRateLimit[XYZ_AXIS_COUNT];  				// 前馈最大速率限制
static float ffMaxRate[XYZ_AXIS_COUNT];						// 前馈最大速率

/* --------------------------滞后移动平均线组合结构体-------------------------- */	
typedef struct laggedMovingAverageCombined_s {
     laggedMovingAverage_t filter;
     float buf[4];
} laggedMovingAverageCombined_t;
// 滞后移动平均线组合结构体数组 - 3轴
laggedMovingAverageCombined_t  setpointDeltaAvg[XYZ_AXIS_COUNT];

/**********************************************************************
函数名称：interpolatedSpInit
函数功能：插值设置点初始化
函数形参：pidProfile
函数返回值：None  
函数描述：None 
**********************************************************************/
void interpolatedSpInit(const pidProfile_t *pidProfile) 
{
	// 获取前馈的最大速率缩放
    const float ffMaxRateScale = pidProfile->ff_max_rate_limit * 0.01f;
	// 滞后平均移动滤波器窗口大小
    averagingCount = pidProfile->ff_interpolate_sp;
	// 遍历三轴
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
		// 获取前馈最大速率 - 应用曲线（范围[-1.0,1.0]）得出的速率值
        ffMaxRate[i] = applyCurve(i, 1.0f);
		// 前馈最大速率限制 = 前馈最大速率 * 前馈最大速率缩放
        ffMaxRateLimit[i] = ffMaxRate[i] * ffMaxRateScale;
		// 滞后平均移动滤波器初始化
        laggedMovingAverageInit(&setpointDeltaAvg[i].filter, averagingCount, (float *)&setpointDeltaAvg[i].buf[0]);
    }
}

/**********************************************************************
函数名称：interpolatedSpApply
函数功能：插值设置点应用 
函数形参：axis，newRcFrame，前馈插值类型
函数返回值：设置点增量
函数描述：
	计算pidSetpoint增量。
**********************************************************************/
float interpolatedSpApply(int axis, bool newRcFrame, ffInterpolationType_t type) 
{
	// 判断RX是否有新数据	
    if (newRcFrame) {
		// 获取原始设置点（期望速率值）
        float rawSetpoint = getRawSetpoint(axis);
		// 获取RX数据获取间隔 = 当前Rx刷新率 * 1*10的-6次方（0.001 - 0.03s）
        const float rxInterval = getCurrentRxRefreshRate() * 1e-6f;
		// 获取RX数据获取频率（HZ）
        const float rxRate = 1.0f / rxInterval;
		// 获取设置点速度 = （当前原始期望速率值 - 上一个期望速率值）* RX数据频率 
        float setpointSpeed = (rawSetpoint - prevRawSetpoint[axis]) * rxRate;
		// 设置点加速度 = 当前设置点速度 - 上一个设置点速度
        float setpointAcceleration = setpointSpeed - prevSetpointSpeed[axis];
		// 设置点速度修正
        float setpointSpeedModified = setpointSpeed;
		// 设置点加速度修正
        float setpointAccelerationModified = setpointAcceleration;

        // ------------------------------------------1.检测摇杆是否有大步距（大动作）
        // 判断当前轴加速度是否为大步距 - 当前设置点加速度 > 3.0 * 上一个设置点加速度
        if (fabsf(setpointAcceleration) > 3.0f * fabsf(prevAcceleration[axis])) {
            bigStep[axis] = true;
        } else {
            bigStep[axis] = false;
        }
		
		// ------------------------------------------2.相同数据包（摇杆位置未移动） && 摇杆位置未即将达到极限
        if (setpointSpeed == 0 && fabsf(rawSetpoint) < 0.98f * ffMaxRate[axis]) {
			// 未处理过摇杆保持类型
            if (holdCount[axis] == 0) {
                if (bigStep[axis]) {
                    // 类型1 - 检测到大步距 - 之前的数据包有移动，在加速度变化较大的地方向前插值（上一个值）
                    setpointSpeedModified = prevSetpointSpeed[axis];
                    setpointAccelerationModified = prevAcceleration[axis];
                    holdCount[axis] = 1;
                } else {
                    // 类型2 - 小步距，不需要插值
                    setpointSpeedModified = 0.0f;
                    setpointSpeed = setpointSpeed / 2.0f;
                    holdCount[axis] = 2;
                }
            } else {
           		// 类型3 - 先前不变数据包之后的不变数据包 - 不需要插值
                holdCount[axis] = 3;
            }
        }
		// ------------------------------------------3.不同数据包（摇杆位置移动）  && 摇杆位置即将达到极限
		else {
            if (holdCount[axis] != 0) {
                // ---------------------------以不同的方式处理每种摇杆保持类型
                if (holdCount[axis] == 1) {      
					// 插补 - 下一个数据包的原始设定点速度应该是它的两倍
                    setpointSpeedModified = setpointSpeed / 2.0f;
                    setpointSpeed = setpointSpeedModified;
                    setpointAccelerationModified = (prevAcceleration[axis] + setpointAcceleration) / 2.0f;
                } else if (holdCount[axis] == 2) {
                    // 小变化，不需要插值
                } else if (holdCount[axis] == 3) {
					// 持续平稳期
                    if (averagingCount > 1) {
                        setpointAccelerationModified /= averagingCount;
                    }
                }
				// 重新初始化摇杆保持类型
                holdCount[axis] = 0;
            }
        }

		// ------------------------------------------4.摇杆在返回中心时平滑类型抑制前馈抖动
		// 仅当ff_average值大于或等于3时 - 用于平稳拍摄类型
        if (averagingCount > 2) {
            const float rawSetpointCentred = fabsf(rawSetpoint) / averagingCount;
            if (rawSetpointCentred < 1.0f) {
                setpointSpeedModified *= rawSetpointCentred;
                setpointAccelerationModified *= rawSetpointCentred;
                holdCount[axis] = 4;
            }
        }

		// ------------------------------------------设置点增量接口		
        setpointDeltaImpl[axis] = setpointSpeedModified * pidGetDT();
		// ------------------------------------------更新上一个设置点加速度
        prevAcceleration[axis] = setpointAcceleration;

		// ------------------------------------------更新设置点加速度
        setpointAcceleration *= pidGetDT();
		// ------------------------------------------
        setpointAccelerationModified *= pidGetDT();

		// ------------------------------------------获取前馈助推因素
        const float ffBoostFactor = pidGetFfBoostFactor();
        float clip = 1.0f;
        float boostAmount = 0.0f;
        if (ffBoostFactor != 0.0f) {
	        // 计算剪切系数以减少大峰值时的提升
            if (pidGetSpikeLimitInverse()) {
                clip = 1 / (1 + (setpointAcceleration * setpointAcceleration * pidGetSpikeLimitInverse()));
                clip *= clip;
            }
            // 不要从最大偏转开始往里夹第一步
            if (fabsf(prevRawSetpoint[axis]) > 0.95f * ffMaxRate[axis] && fabsf(setpointSpeed) > 3.0f * fabsf(prevSetpointSpeed[axis])) {
                clip = 1.0f;
            }
            // 计算升力并在最大偏转时防止回弹
            if (fabsf(rawSetpoint) < 0.95f * ffMaxRate[axis] || fabsf(setpointSpeed) > 3.0f * fabsf(prevSetpointSpeed[axis])) {
                boostAmount = ffBoostFactor * setpointAccelerationModified;
            }
        }

		// ------------------------------------------更新上一个设置点加速度
        prevSetpointSpeed[axis] = setpointSpeed;
		// ------------------------------------------更新上一个原始设置点
        prevRawSetpoint[axis] = rawSetpoint;
		// ------------------------------------------更新设置点增量接口
        setpointDeltaImpl[axis] += boostAmount * clip;

        // ------------------------------------------获取前馈平滑因素
        const float ffSmoothFactor = pidGetFfSmoothFactor();
		// ------------------------------------------更新设置点增量接口
        setpointDeltaImpl[axis] = prevDeltaImpl[axis] + ffSmoothFactor * (setpointDeltaImpl[axis] - prevDeltaImpl[axis]);
		// ------------------------------------------更新上一个增量接口
        prevDeltaImpl[axis] = setpointDeltaImpl[axis];

		// 判断前馈插值类型 - 设置设置点增量
        if (type == FF_INTERPOLATE_ON) {
			// 非滤波
            setpointDelta[axis] = setpointDeltaImpl[axis];
        } else {
        	// 对设置点增量进行滤波
            setpointDelta[axis] = laggedMovingAverageUpdate(&setpointDeltaAvg[axis].filter, setpointDeltaImpl[axis]);
        }
    }
	
    return setpointDelta[axis];
}

/**********************************************************************
函数名称：applyFfLimit
函数功能：应用前馈限制
函数形参：axis，value，Kp，currentPidSetpoint
函数返回值：value
函数描述：None 
**********************************************************************/
float applyFfLimit(int axis, float value, float Kp, float currentPidSetpoint) {
    if (fabsf(currentPidSetpoint) <= ffMaxRateLimit[axis]) {
        value = constrainf(value, (-ffMaxRateLimit[axis] - currentPidSetpoint) * Kp, (ffMaxRateLimit[axis] - currentPidSetpoint) * Kp);
    } else {
        value = 0;
    }
    return value;
}

/**********************************************************************
函数名称：shouldApplyFfLimits
函数功能：获取是否要应用前馈限制
函数形参：axis 
函数返回值：状态
函数描述：None 
**********************************************************************/
bool shouldApplyFfLimits(int axis)
{
    return ffMaxRateLimit[axis] != 0.0f && axis < FD_YAW;
}
#endif

