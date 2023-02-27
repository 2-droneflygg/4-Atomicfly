/*********************************************************************************
 提供RC处理相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/interpolated_setpoint.h"
#include "flight/gps_rescue.h"
#include "flight/pid.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "rc.h"

/* ---------------------------RPYT标志位枚举--------------------------- */
enum
{
    ROLL_FLAG = 1 << ROLL,
    PITCH_FLAG = 1 << PITCH,
    YAW_FLAG = 1 << YAW,
    THROTTLE_FLAG = 1 << THROTTLE,
};

// ---------------------------------------------------------速率函数指针类型
typedef float(applyRatesFn)(const int axis, float rcCommandf, const float rcCommandfAbs);

// ---------------------------------------------------------RC插值
#ifdef USE_INTERPOLATED_SP
// 原始设置点 - rc平滑应用设置在度/秒之前
static float rawSetpoint[XYZ_AXIS_COUNT];
// 原始偏差 - RC平滑前的杆偏差[-1.0,1.0]
static float rawDeflection[XYZ_AXIS_COUNT];
// 旧RC命令
static float oldRcCommand[XYZ_AXIS_COUNT];
#endif

// ---------------------------------------------------------是否有RX新数据
static bool isRxDataNew = false;

// ---------------------------------------------------------RC帧数
static FAST_RAM_ZERO_INIT uint32_t rcFrameNumber;

// ---------------------------------------------------------插值通道
FAST_RAM_ZERO_INIT uint8_t interpolationChannels;

// ---------------------------------------------------------应用速率访问
static applyRatesFn *applyRates;

// ---------------------------------------------------------设置点速率，RC偏差，RC绝对偏差
static float setpointRate[3], rcDeflection[3], rcDeflectionAbs[3];

// ---------------------------------------------------------当前RX更新速率
static uint16_t currentRxRefreshRate;

// ---------------------------------------------------------RC命令分频
static float rcCommandDivider = 500.0f;

// ---------------------------------------------------------RC偏航命令分频
static float rcCommandYawDivider = 500.0f;

// ---------------------------------------------------------油门PID衰减
static float throttlePIDAttenuation;

// ---------------------------------------------------------反转电机
static bool reverseMotors = false;

//-------------------------------------------------------------------------------------Rx刷新率相关API

/**********************************************************************
函数名称：updateRcRefreshRate
函数功能：更新Rc刷新率
函数形参：currentTimeUs
函数返回值：None
函数描述：None
**********************************************************************/
void updateRcRefreshRate(timeUs_t currentTimeUs)
{
    static timeUs_t lastRxTimeUs;

    timeDelta_t frameAgeUs;
    // 获取RX帧时差
    timeDelta_t refreshRateUs = rxGetFrameDelta(&frameAgeUs);
    if (!refreshRateUs || cmpTimeUs(currentTimeUs, lastRxTimeUs) <= frameAgeUs)
    {
        // 如果协议没有提供，在这里计算时差
        refreshRateUs = cmpTimeUs(currentTimeUs, lastRxTimeUs);
    }
    // 更新本次RX时间作为上次RX时间为下次使用
    lastRxTimeUs = currentTimeUs;
    // 约束限制 - RX任务运行频率为30Hz，所以限制1000-30000us(1ms - 30ms，0.001 - 0.03s)
    currentRxRefreshRate = constrain(refreshRateUs, 1000, 30000);
}

/**********************************************************************
函数名称：getCurrentRxRefreshRate
函数功能：获取当前Rx刷新率
函数形参：None
函数返回值：currentRxRefreshRate
函数描述：None
**********************************************************************/
uint16_t getCurrentRxRefreshRate(void)
{
    return currentRxRefreshRate;
}

//-------------------------------------------------------------------------------------RC曲线相关API

/**********************************************************************
函数名称：applyDflightRates
函数功能：应用Dflight速率（Rate曲线）
函数形参：轴，RC命令（float）[-1,1]，RC命令（绝对值）[0,1]
函数返回值：angleRate
函数描述：None
**********************************************************************/
// 设置点速率限制
#define SETPOINT_RATE_LIMIT 1998
// RC速率步距
#define RC_RATE_INCREMENTAL 14.54f
// 验证速率是否超范围
STATIC_ASSERT(CONTROL_RATE_CONFIG_RATE_LIMIT_MAX <= SETPOINT_RATE_LIMIT, CONTROL_RATE_CONFIG_RATE_LIMIT_MAX_too_large);
float applyDflightRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    // 1.应用固定曲线值EXPO[0,1] - 改变曲线的曲率
    if (currentControlRateProfile->rcExpo[axis])
    {
        // 缩放为[0,1]
        const float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
        rcCommandf = rcCommandf * power3(rcCommandfAbs) * expof + rcCommandf * (1 - expof);
    }
    // 2.应用线性角速度RC_Rate(0,2.55] - 速率的放大倍数（线性）
    float rcRate = currentControlRateProfile->rcRates[axis] / 100.0f;
    if (rcRate > 2.0f)
    {
        rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
    }
    float angleRate = 200.0f * rcRate * rcCommandf;
    // 3.应用曲线角速度Rate[0,1] - 既改变曲率由改变速率
    if (currentControlRateProfile->rates[axis])
    {
        const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (currentControlRateProfile->rates[axis] / 100.0f)), 0.01f, 1.00f));
        angleRate *= rcSuperfactor;
    }
    return angleRate;
}

/**********************************************************************
函数名称：applyCurve
函数功能：应用曲线
函数形参：axis，偏差
函数返回值：applyRates
函数描述：
    RPY应用曲线。
**********************************************************************/
float applyCurve(int axis, float deflection)
{
    return applyRates(axis, deflection, fabsf(deflection));
}

/**********************************************************************
函数名称：rcLookupThrottle
函数功能：查找RC油门
函数形参：tmp
函数返回值：lookupThrottleRC
函数描述：
    油门应用EXPO曲线。
**********************************************************************/
#define THROTTLE_LOOKUP_LENGTH 12
// lookup table for expo & mid THROTTLE
static int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];
static int16_t rcLookupThrottle(int32_t tmp)
{
    const int32_t tmp2 = tmp / 100;
    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
    return lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;
}

//-------------------------------------------------------------------------------------RC插值相关API

/**********************************************************************
函数名称：processRcInterpolation
函数功能：Rc插值进程
函数形参：None
函数返回值：更新的通道
函数描述：None
**********************************************************************/
static uint8_t processRcInterpolation(void)
{
    static FAST_RAM_ZERO_INIT float rcCommandInterp[4];
    static FAST_RAM_ZERO_INIT float rcStepSize[4];
    static FAST_RAM_ZERO_INIT int16_t rcInterpolationStepCount;

    uint16_t rxRefreshRate;
    uint8_t updatedChannel = 0;

    // ---------------------------------------------------------开启RC平滑
    if (rxConfig()->rcInterpolation)
    {
        // -------------------------------1.根据滤波类型获取RX刷新率
        // RC_SMOOTHING_AUTO使平滑滤波器动态适应表观传入RC步进速率
        // 手动设置可以微调到任何所需的平滑度，但会增加输入延迟
        switch (rxConfig()->rcInterpolation)
        {
        case RC_SMOOTHING_AUTO:
            // RX帧时差[1000,30000]us - 增加轻微的开销以防止斜升
            rxRefreshRate = currentRxRefreshRate + 1000;
            break;
        case RC_SMOOTHING_MANUAL:
            rxRefreshRate = 1000 * rxConfig()->rcInterpolationInterval;
            break;
        case RC_SMOOTHING_OFF:
        case RC_SMOOTHING_DEFAULT:
        default:
            rxRefreshRate = rxGetRefreshRate();
        }

        // -------------------------------2.根据RX新数据以及RX帧时差计算RC插值大小
        if (isRxDataNew && rxRefreshRate > 0)
        {
            // 获取RC插值步数 - RX帧时差 / PID循环时间
            rcInterpolationStepCount = rxRefreshRate / targetPidLooptime;
            // 遍历RPYT通道
            for (int channel = 0; channel < PRIMARY_CHANNEL_COUNT; channel++)
            {
                // 判断需要插值的通道 - 计算步的大小
                if ((1 << channel) & interpolationChannels)
                {
                    rcStepSize[channel] = (rcCommand[channel] - rcCommandInterp[channel]) / (float)rcInterpolationStepCount;
                }
            }
        }
        else
        {
            rcInterpolationStepCount--;
        }

        // -------------------------------3.将rcStepSize插入rcCommand
        if (rcInterpolationStepCount > 0)
        {
            for (updatedChannel = 0; updatedChannel < PRIMARY_CHANNEL_COUNT; updatedChannel++)
            {
                // 判断需要插值的通道 - 递增
                if ((1 << updatedChannel) & interpolationChannels)
                {
                    rcCommandInterp[updatedChannel] += rcStepSize[updatedChannel];
                    rcCommand[updatedChannel] = rcCommandInterp[updatedChannel];
                }
            }
        }
    }
    // ---------------------------------------------------------未开启RC平滑
    else
    {
        rcInterpolationStepCount = 0;
    }

    return updatedChannel;
}
//-------------------------------------------------------------------------------------油门变化相关API

/**********************************************************************
函数名称：checkForThrottleErrorResetState
函数功能：检查油门错误复位状态
函数形参：RX刷新率
函数返回值：None
函数描述：
    当检测到油门快速变化时反重力功能将增大I项。
**********************************************************************/
#define THROTTLE_BUFFER_MAX 20
#define THROTTLE_DELTA_MS 100
static void checkForThrottleErrorResetState(uint16_t rxRefreshRate)
{
    static int index;
    static int16_t rcCommandThrottlePrevious[THROTTLE_BUFFER_MAX];

    // 获取RX刷新率（ms）
    const int rxRefreshRateMs = rxRefreshRate / 1000;
    // 获取最大索引
    const int indexMax = constrain(THROTTLE_DELTA_MS / rxRefreshRateMs, 1, THROTTLE_BUFFER_MAX);
    // 获取油门速率阈值 - I项
    const int16_t throttleVelocityThreshold = currentPidProfile->itermThrottleThreshold;

    // 记录油门命令
    rcCommandThrottlePrevious[index++] = rcCommand[THROTTLE];
    if (index >= indexMax)
    {
        index = 0;
    }

    // 计算油门命令变化速度
    const int16_t rcCommandSpeed = rcCommand[THROTTLE] - rcCommandThrottlePrevious[index];

    // 判断是否激活反重力
    if (currentPidProfile->antiGravityMode == ANTI_GRAVITY_STEP)
    {
        // 判断是否超过油门速率阈值
        if (ABS(rcCommandSpeed) > throttleVelocityThreshold)
        {
            // 设置油门 PID - I项
            pidSetItermAccelerator(CONVERT_PARAMETER_TO_FLOAT(currentPidProfile->itermAcceleratorGain));
        }
        // 未达到反重力开启需求 - I项反重力增益为0
        else
        {
            // 设置油门 PID - I项
            pidSetItermAccelerator(0.0f);
        }
    }
}

//-------------------------------------------------------------------------------------计算设定点速率相关API

/**********************************************************************
函数名称：calculateSetpointRate
函数功能：计算设定点速率（期望速率值）
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
static void calculateSetpointRate(int axis)
{
    float angleRate;

#ifdef USE_GPS_RESCUE
    // ----------------------------------------计算GPS救援模式偏航速率
    if ((axis == FD_YAW) && FLIGHT_MODE(GPS_RESCUE_MODE))
    {
        // 获取GPS救援模式偏航速率
        angleRate = gpsRescueGetYawRate();
        // 将摇杆输入设为中心点，以避免摇杆偏转(如加速度限制)
        rcDeflection[axis] = 0;
        rcDeflectionAbs[axis] = 0;
    }
    // ----------------------------------------其他模式
    else
#endif
    {
        // 缩放rcCommandf到范围[-1.0,1.0]
        float rcCommandf;
        if (axis == FD_YAW)
        {
            rcCommandf = rcCommand[axis] / rcCommandYawDivider;
        }
        else
        {
            rcCommandf = rcCommand[axis] / rcCommandDivider;
        }
        // 更新RC偏差 - 外环角度控制[-1.0,1.0]
        rcDeflection[axis] = rcCommandf;
        // RC命令绝对值
        const float rcCommandfAbs = fabsf(rcCommandf);
        // 更新RC偏差绝对值
        rcDeflectionAbs[axis] = rcCommandfAbs;

        // 应用RC速率
        angleRate = applyRates(axis, rcCommandf, rcCommandfAbs);
    }

    // ----------------------------------------设定点速率 - 速率期望值 （配置文件(deg/sec)rate限制 - 1998）
    setpointRate[axis] = constrainf(angleRate, -1.0f * currentControlRateProfile->rate_limit[axis], 1.0f * currentControlRateProfile->rate_limit[axis]);
}

//-------------------------------------------------------------------------------------RC命令相关API

/**********************************************************************
函数名称：updateRcCommands
函数功能：RC命令更新进程
函数形参：None
函数返回值：None
函数描述：
    由taskUpdateRxMain（更新RC数据任务函数）调用。

    rcData：     [1000,2000]
    rcCommand：T[1000,2000]
             RPY[-500,500]
**********************************************************************/
void updateRcCommands(void)
{
    // ----------------------------------------有RX新数据
    isRxDataNew = true;
    // ----------------------------------------判断油门是否达到TPA起始值 - 仅俯仰和横滚动态PID调节，取决于油门值
    int32_t prop;
    // ----------------------------------------1.油门TPA功能
    if (rcData[THROTTLE] < currentControlRateProfile->tpa_breakpoint)
    {
        prop = 100;
        // 油门PID衰减
        throttlePIDAttenuation = 1.0f;
    }
    else
    {
        if (rcData[THROTTLE] < 2000)
        {
            // 100 - dynThrPID * (THROTTLE - tpa_breakpoint) / (2000 - tpa_breakpoint)
            prop = 100 - (uint16_t)currentControlRateProfile->dynThrPID * (rcData[THROTTLE] - currentControlRateProfile->tpa_breakpoint) / (2000 - currentControlRateProfile->tpa_breakpoint);
        }
        else
        {
            prop = 100 - currentControlRateProfile->dynThrPID;
        }
        // 油门PID衰减 - 转换为百分比
        throttlePIDAttenuation = prop / 100.0f;
    }
    // ----------------------------------------2.获取RPY命令 - 转换为[-500,500]
    for (int axis = 0; axis < 3; axis++)
    {
        int32_t tmp = MIN(ABS(rcData[axis] - rxConfig()->midrc), 500);
        // ------------------------------(1)获取RC命令
        if (axis == ROLL || axis == PITCH)
        {
            // -----------------横滚俯仰
            // 应用俯仰横滚死区
            if (tmp > rcControlsConfig()->deadband)
            {
                tmp -= rcControlsConfig()->deadband;
            }
            else
            {
                tmp = 0;
            }
            rcCommand[axis] = tmp;
        }
        else
        {
            // -----------------偏航
            // 应用偏航死区
            if (tmp > rcControlsConfig()->yaw_deadband)
            {
                tmp -= rcControlsConfig()->yaw_deadband;
            }
            else
            {
                tmp = 0;
            }
            // 判断偏航是否反转
            rcCommand[axis] = tmp * -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
        }
        // ------------------------------(2)转换负方向
        if (rcData[axis] < rxConfig()->midrc)
        {
            rcCommand[axis] = -rcCommand[axis];
        }
    }
    // ----------------------------------------3.约束油门[1000,2000]
    int32_t tmp;
    tmp = constrain(rcData[THROTTLE], rxConfig()->mincheck, PWM_RANGE_MAX);
    // （1500-1050）* 1000 /（2000-1050）
    tmp = (uint32_t)(tmp - rxConfig()->mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - rxConfig()->mincheck);

    // ----------------------------------------4.低电压截止
    if (getLowVoltageCutoff()->enabled)
    {
        tmp = tmp * getLowVoltageCutoff()->percentage / 100;
    }
    // ----------------------------------------5.获取油门命令
    rcCommand[THROTTLE] = rcLookupThrottle(tmp);
}

/**********************************************************************
函数名称：processRcCommand
函数功能：RC命令处理进程
函数形参：None
函数返回值：None
函数描述：
    由subTaskRcCommand（RC命令子任务）调用 -> taskMainPidLoop。
**********************************************************************/
void processRcCommand(void)
{
    // ----------------------------------------记录插值更新的通道变量
    uint8_t updatedChannel;
    // ----------------------------------------如果RX有新数据则进行RC帧累加
    if (isRxDataNew)
    {
        rcFrameNumber++;
    }
    // ----------------------------------------有RC新数据 && 反重力开启 - 当检测到油门快速变化时反重力功能将增大I项
    if (isRxDataNew && pidAntiGravityEnabled())
    {
        // 检查油门错误复位状态
        checkForThrottleErrorResetState(currentRxRefreshRate);
    }

    // ----------------------------------------插值定位点 - 从每个新的RC计算FF,提供了更清晰的前馈跟踪，并且延迟更少
    //				分析每个新的传入RC数据包，并将计算出的设定值变化转换为FF的立即递增
    //			    每次递增都保持恒定，直到下一个RC数据到达为止
#ifdef USE_INTERPOLATED_SP
    if (isRxDataNew)
    {
        // 遍历RPY - 原始rcCommand [-500,500]
        for (int i = FD_ROLL; i <= FD_YAW; i++)
        {
            // 更新旧RC命令为当前新命令 - 为下一次做准备
            oldRcCommand[i] = rcCommand[i];

            // 转换为速率[-1,1]
            float rcCommandf;
            if (i == FD_YAW)
            {
                rcCommandf = rcCommand[i] / rcCommandYawDivider;
            }
            else
            {
                rcCommandf = rcCommand[i] / rcCommandDivider;
            }
            // 取绝对值（float类型） - [0,1]
            const float rcCommandfAbs = fabsf(rcCommandf);
            // 原始设置点 - 应用速率(期望速率值)
            rawSetpoint[i] = applyRates(i, rcCommandf, rcCommandfAbs);
            // 原始偏差[-1,1]
            rawDeflection[i] = rcCommandf;
        }
    }
#endif
    // ----------------------------------------RC插值进程 - RC系统的运行速率没有飞控PID运行速率那么快
    //              这意味着RC信号进入PID环路时会存在信号真空期
    //   			开启RC插值会在无RC信号的真空期内对其进行插值处理
    // 				这会使P和D表现得更加顺滑
    switch (rxConfig()->rc_smoothing_type)
    {
    case RC_SMOOTHING_TYPE_INTERPOLATION:
    default:
        // 插值更新的通道
        updatedChannel = processRcInterpolation();
        break;
    }
    // ----------------------------------------计算设置速率 (期望速率值)
    if (isRxDataNew || updatedChannel)
    {
        // 油门通道不需要计算速率
        const uint8_t maxUpdatedAxis = isRxDataNew ? FD_YAW : MIN(updatedChannel, FD_YAW);
        for (int axis = FD_ROLL; axis <= maxUpdatedAxis; axis++)
        {
            // 计算设置速率 (期望速率值)
            calculateSetpointRate(axis);
        }
    }
    // ----------------------------------------RC新数据处理完毕 - 复位为下一次做准备
    isRxDataNew = false;
}

//-------------------------------------------------------------------------------------RC进程初始化相关API

/**********************************************************************
函数名称：initRcProcessing
函数功能：RC进程初始化
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void initRcProcessing(void)
{
    rcCommandDivider = 500.0f - rcControlsConfig()->deadband;
    rcCommandYawDivider = 500.0f - rcControlsConfig()->yaw_deadband;
    // ----------------------------------------查找油门
    for (int i = 0; i < THROTTLE_LOOKUP_LENGTH; i++)
    {
        const int16_t tmp = 10 * i - currentControlRateProfile->thrMid8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - currentControlRateProfile->thrMid8;
        if (tmp < 0)
            y = currentControlRateProfile->thrMid8;
        lookupThrottleRC[i] = 10 * currentControlRateProfile->thrMid8 + tmp * (100 - currentControlRateProfile->thrExpo8 + (int32_t)currentControlRateProfile->thrExpo8 * (tmp * tmp) / (y * y)) / 10;
        // [MINTHROTTLE;MAXTHROTTLE]
        lookupThrottleRC[i] = PWM_RANGE_MIN + (PWM_RANGE_MAX - PWM_RANGE_MIN) * lookupThrottleRC[i] / 1000;
    }
    // ----------------------------------------选择RC速率类型
    switch (currentControlRateProfile->rates_type)
    {
    case RATES_TYPE_DFLIGHT:
    default:
        applyRates = applyDflightRates;
        break;
    }
    // ----------------------------------------选择插值类型
    interpolationChannels = 0;
    switch (rxConfig()->rcInterpolationChannels)
    {
    case INTERPOLATION_CHANNELS_RPYT:
        interpolationChannels |= THROTTLE_FLAG;

        FALLTHROUGH;
    case INTERPOLATION_CHANNELS_RPY:
        interpolationChannels |= YAW_FLAG;

        FALLTHROUGH;
    case INTERPOLATION_CHANNELS_RP:
        interpolationChannels |= ROLL_FLAG | PITCH_FLAG;
        break;
    case INTERPOLATION_CHANNELS_RPT:
        interpolationChannels |= ROLL_FLAG | PITCH_FLAG;

        FALLTHROUGH;
    case INTERPOLATION_CHANNELS_T:
        interpolationChannels |= THROTTLE_FLAG;

        break;
    }
    // ----------------------------------------反自旋
#ifdef USE_YAW_SPIN_RECOVERY
    const int maxYawRate = (int)applyRates(FD_YAW, 1.0f, 1.0f);
    initYawSpinRecovery(maxYawRate);
#endif
}

//-------------------------------------------------------------------------------------数据访问相关接口

/**********************************************************************
函数名称：rcSmoothingIsEnabled
函数功能：获取RC平滑滤波是否开启
函数形参：None
函数返回值：是否开启
函数描述：None
**********************************************************************/
bool rcSmoothingIsEnabled(void)
{
    return !(rxConfig()->rcInterpolation == RC_SMOOTHING_OFF);
}

/**********************************************************************
函数名称：getRcFrameNumber
函数功能：获取RC帧数
函数形参：None
函数返回值：rcFrameNumber
函数描述：None
**********************************************************************/
uint32_t getRcFrameNumber()
{
    return rcFrameNumber;
}

/**********************************************************************
函数名称：getSetpointRate
函数功能：获取设置点速率
函数形参：axis
函数返回值：setpointRate
函数描述：None
**********************************************************************/
float getSetpointRate(int axis)
{
    return setpointRate[axis];
}

/**********************************************************************
函数名称：getRcDeflection
函数功能：获取RC偏差
函数形参：axis
函数返回值：rcDeflection
函数描述：
    外环角度控制[-1.0,1.0]。
**********************************************************************/
float getRcDeflection(int axis)
{
    return rcDeflection[axis];
}

/**********************************************************************
函数名称：getRcDeflectionAbs
函数功能：获取RC偏差abs
函数形参：axis
函数返回值：rcDeflectionAbs
函数描述：
    外环角度控制[0,1.0]。
**********************************************************************/
float getRcDeflectionAbs(int axis)
{
    return rcDeflectionAbs[axis];
}

#ifdef USE_INTERPOLATED_SP
/**********************************************************************
函数名称：getThrottlePIDAttenuation
函数功能：获取原始设置点
函数形参：None
函数返回值：rawSetpoint
函数描述：None
**********************************************************************/
float getRawSetpoint(int axis)
{
    return rawSetpoint[axis];
}
#endif

/**********************************************************************
函数名称：getThrottlePIDAttenuation
函数功能：获取油门PID衰减
函数形参：None
函数返回值：throttlePIDAttenuation
函数描述：None
**********************************************************************/
float getThrottlePIDAttenuation(void)
{
    return throttlePIDAttenuation;
}
