/**********************************************************************
PID控制器:
	负责对摇杆输入做出反应，并通过使用陀螺仪/加速计(取决于飞行模式)保持飞行器在空中的稳定.
	“PID”是一组整定参数，控制PID控制器的操作.
	PID控制器的目标是使飞行器在所有三个轴上的旋转速率达到摇杆控制的速率.
	目标旋转速率和陀螺仪测量到的实际旋转速率之间的误差，控制器会将这个误差降到零.
	
 ## PID调校:
	P项：比例运算器，仅和误差幅值有关.
		 控制用于使飞行器朝向目标角度或旋转速率的校正强度.
		 如果P值太低，飞行器就很难控制，因为它的反应速度不够快，无法保持自身的稳定.
		 如果它被设置得太高，飞行器会在不断地超过目标产生震荡.
	I项：积分运算器，和误差累积时间有关.
	     修正了小的、长期的错误.
		 如果设置得太低，飞行器的姿态就会慢慢漂移.
		 如果它设置的太高，飞行器会振荡(但振荡速度比P设置的太高要慢).
	D项：微分运算器，和误差的变化频率有关.
	     试图通过监控错误的变化率来增加系统的稳定性.
		 如果误差迅速收敛到零，D项会导致修正的强度后退，以避免超出目标.
		 
 ## TPA和TPA起始点:（模式有PD和D）
 	*TPA*表示油门PD衰减，减少其PD增益时，油门应用超过TPA阈值/断点，以消除快速振荡.
	TPA应用与全油门PD值降低相关，用来达到全油门应用PD值的阻尼.
	
	TPA = %的阻尼
	tpa_breakpoint = 油门曲线中开始应用TPA的点

	例如:TPA（dynThrPID） = 50和tpa_breakpoint = 1500(假设油门范围1000 - 2000)
	*在油门通道上的1500点，PD将开始被阻尼.
	*在3/4油门(1750)，PD降低了大约25%（此时TPA因素为0.75（75%））.
	*在满油门(2000)的全部阻尼设置在TPA应用(本例中减少50%).
	*当施加更多的油门时，TPA会导致旋转速率的增加。
	*由于PD和速率的耦合，当应用更多的油门时，可以获得更快的翻转和横滚，当使用TPA时，不会影响转速.

 ### 反重力：
 	当检测到油门快速变化时反重力功能将增大 I 项.
 	较高的增益值将在快速抖动油门时，提供更好的稳定性和更强的姿态锁定能力（抬头现象）.
**********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config_reset.h"

#include "drivers/dshot_command.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/interpolated_setpoint.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "pid.h"

/* ------------------------------PID名称------------------------- */	
const char pidNames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "LEVEL;"
    "MAG;";

// ------------------------------------------------------------------------------------------------------PID
FAST_RAM_ZERO_INIT uint32_t targetPidLooptime;				            	 // 目标PID循环时间
FAST_RAM_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];	  				 // PID数据结构体变量 
static FAST_RAM_ZERO_INIT float dT;							  			 	 // 时间常数
static FAST_RAM_ZERO_INIT float pidFrequency;				  			 	 // PID频率
#define PID_PROCESS_DENOM_DEFAULT       1									 // PID进程默认分母项
static FAST_RAM_ZERO_INIT pidCoefficient_t pidCoefficient[XYZ_AXIS_COUNT];   // PID系数
static FAST_RAM_ZERO_INIT float maxVelocity[XYZ_AXIS_COUNT];				 // 最大速率
static FAST_RAM_ZERO_INIT float previousPidSetpoint[XYZ_AXIS_COUNT];     	 // 前一个PID设定点

// ------------------------------------------------------------------------------------------------------前馈
#ifdef USE_INTERPOLATED_SP
static FAST_RAM_ZERO_INIT ffInterpolationType_t ffFromInterpolatedSetpoint;
#endif
static FAST_RAM_ZERO_INIT float ffBoostFactor;								 // 前馈增压因素
static FAST_RAM_ZERO_INIT float ffSmoothFactor;							     // 前馈平滑因素
static FAST_RAM_ZERO_INIT float ffSpikeLimitInverse;						 // 前馈逆峰值限制
static FAST_RAM_ZERO_INIT float feedForwardTransition;						 // 前馈转换

// ------------------------------------------------------------------------------------------------------I项
#if defined(USE_ITERM_RELAX)
static FAST_RAM_ZERO_INIT pt1Filter_t windupLpf[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT uint8_t itermRelax;								 // I项释放应用轴系
static FAST_RAM_ZERO_INIT uint8_t itermRelaxType;							 // I项释放类型
static uint8_t itermRelaxCutoff;											 // I项释放截止频率
#endif
static FAST_RAM_ZERO_INIT float itermLimit;									 // I项限制
static FAST_RAM_ZERO_INIT float itermWindupPointInv;
static FAST_RAM_ZERO_INIT float itermAccelerator;						 	 // 油门（通过检测油门变化率判断反重力是否开启） I项
static FAST_RAM_ZERO_INIT bool zeroThrottleItermReset;						 // 零油门I项复位

static FAST_RAM_ZERO_INIT uint8_t antiGravityMode;			  			 	 // 反重力模式
static FAST_RAM_ZERO_INIT float antiGravityThrottleHpf; 					 // 反重力油门频率
static FAST_RAM_ZERO_INIT uint16_t itermAcceleratorGain;					 // I项加速度
static FAST_RAM_ZERO_INIT float antiGravityOsdCutoff;                        // 反重力OSD截止频率
static FAST_RAM_ZERO_INIT bool antiGravityEnabled;			 			 	 // 反重力使能状态
#define ANTI_GRAVITY_THROTTLE_FILTER_CUTOFF 15      		 			 	 // 反重力油门滤波截止频率
static FAST_RAM_ZERO_INIT pt1Filter_t antiGravityThrottleLpf;                // 反重力油门低通滤波

// ------------------------------------------------------------------------------------------------------D项
#if defined(USE_D_MIN)
#define D_MIN_GAIN_FACTOR 0.00005f
#define D_MIN_SETPOINT_GAIN_FACTOR 0.00005f
#define D_MIN_RANGE_HZ 80    								 			 	 // Biquad低通输入截止到D峰值附近的propwash频率
#define D_MIN_LOWPASS_HZ 10  					 			 			 	 // PT1低通截止，平滑升压效果
static FAST_RAM_ZERO_INIT biquadFilter_t dMinRange[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT pt1Filter_t dMinLowpass[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float dMinPercent[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float dMinGyroGain;
static FAST_RAM_ZERO_INIT float dMinSetpointGain;
#endif

static FAST_RAM_ZERO_INIT filterApplyFnPtr dtermLowpassApplyFn;			 	 // D项低通滤波函数指针
static FAST_RAM_ZERO_INIT dtermLowpass_t dtermLowpass[XYZ_AXIS_COUNT];	 	 // D项低通滤波器
static FAST_RAM_ZERO_INIT filterApplyFnPtr dtermLowpass2ApplyFn;		 	 // D项低通滤波2函数指针
static FAST_RAM_ZERO_INIT dtermLowpass_t dtermLowpass2[XYZ_AXIS_COUNT];      // D项低通滤波器2

#ifdef USE_DYN_LPF
static FAST_RAM uint8_t dynLpfFilter = DYN_LPF_NONE;						 // 动态低通滤波类型
static FAST_RAM_ZERO_INIT uint16_t dynLpfMin;								 // 最小值
static FAST_RAM_ZERO_INIT uint16_t dynLpfMax;								 // 最大值
static FAST_RAM_ZERO_INIT uint8_t dynLpfCurveExpo;							 // 固定曲线值
#endif

// ------------------------------------------------------------------------------------------------------油门
#if defined(USE_THROTTLE_BOOST)
FAST_RAM_ZERO_INIT float throttleBoost;										 // 油门增压
pt1Filter_t throttleLpf;												     // 油门低通滤波器	 
#endif

#ifdef USE_THRUST_LINEARIZATION
FAST_RAM_ZERO_INIT float thrustLinearization;
FAST_RAM_ZERO_INIT float thrustLinearizationReciprocal;
FAST_RAM_ZERO_INIT float thrustLinearizationB;
#endif

// ------------------------------------------------------------------------------------------------------自稳增益
static FAST_RAM_ZERO_INIT float levelGain;

// ------------------------------------------------------------------------------------------------------崩溃恢复
static FAST_RAM_ZERO_INIT bool inCrashRecoveryMode = false;	  			 	 // 是否在崩溃恢复模式
static FAST_RAM_ZERO_INIT timeDelta_t crashTimeLimitUs;
static FAST_RAM_ZERO_INIT timeDelta_t crashTimeDelayUs;
static FAST_RAM_ZERO_INIT int32_t crashRecoveryAngleDeciDegrees;
static FAST_RAM_ZERO_INIT float crashRecoveryRate;
static FAST_RAM_ZERO_INIT float crashDtermThreshold;
static FAST_RAM_ZERO_INIT float crashGyroThreshold;
static FAST_RAM_ZERO_INIT float crashSetpointThreshold;
static FAST_RAM_ZERO_INIT float crashLimitYaw;
#define CRASH_RECOVERY_DETECTION_DELAY_US 1000000   		 			 	 // 在进入自稳模式后，在崩溃恢复检测之前有1秒的延迟


PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 2);
#ifdef USE_RUNAWAY_TAKEOFF
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT,							 // PID进程默认分母项
    .runaway_takeoff_prevention = true,
    .runaway_takeoff_deactivate_throttle = 20,  			 			 	 // 油门等级%需要累积钝化时间
    .runaway_takeoff_deactivate_delay = 500     			 			 	 // 在成功起飞时，取消激活前的累计时间(毫秒)
);
#endif
PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 15);
void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
		// ------------------------------------------------------------------------PID参数
        .pid = {
            [PID_ROLL] =  { 42, 85, 35, 60 },
            [PID_PITCH] = { 46, 90, 38, 65 },
            [PID_YAW] =   { 45, 90, 0, 60 },
            [PID_LEVEL] = { 50, 50, 75, 0 },
            [PID_MAG] =   { 40, 0, 0, 0 },
        },

		// ------------------------------------------------------------------------前馈 - 动态调节P
        .ff_interpolate_sp = FF_INTERPOLATE_AVG2,
        .ff_spike_limit = 60,
        .ff_max_rate_limit = 100,
        .ff_smooth_factor = 37,
        .feedForwardTransition = 0,
        .ff_boost = 15,

		// ------------------------------------------------------------------------I项释放 &&    反重力    - 动态调节I
		.iterm_relax = ITERM_RELAX_RP,
        .iterm_relax_type = ITERM_RELAX_SETPOINT,
        .iterm_relax_cutoff = ITERM_RELAX_CUTOFF_DEFAULT,
		.itermWindupPointPercent = 100,
		.itermThrottleThreshold = 250,
		.itermAcceleratorGain = 3500,
		.itermLimit = 400,
		.antiGravityMode = ANTI_GRAVITY_SMOOTH,

		// ------------------------------------------------------------------------D_MIN - 动态调节D
        .d_min = { 22, 24, 0 },      				// roll, pitch, yaw
        .d_min_gain = 37,
        .d_min_advance = 20,

		// ------------------------------------------------------------------------D项低通滤波器 
		.dterm_filter_type = FILTER_PT1,			// D Term 低通滤波器1动态滤波器类型
        .dterm_lowpass_hz = 150,    				// D Term 低通滤波器1截止频率 [Hz]  
        .dterm_filter2_type = FILTER_PT1,			// D Term 低通滤波器2截止频率 [Hz]
        .dterm_lowpass2_hz = 113,   				// D Term 低通滤波器2截止频率 [Hz]
        .dyn_lpf_dterm_min_hz = 53,					// D Term 低通滤波器1动态最低截止频率 [Hz]
        .dyn_lpf_dterm_max_hz = 128,				// D Term 低通滤波器1动态最高截止频率 [Hz]
        .dyn_lpf_curve_expo = 5,

		// ------------------------------------------------------------------------pidSum
        .pidSumLimit = PIDSUM_LIMIT,
        .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
        .thrustLinearization = 0,
        .motor_output_limit = 100,

		// ------------------------------------------------------------------------油门
        .throttle_boost = 5,
        .throttle_boost_cutoff = 15,

		// ------------------------------------------------------------------------自稳模式
        .levelAngleLimit = 55,

		// ------------------------------------------------------------------------加速度限制
        .yawRateAccelLimit = 0,
        .rateAccelLimit = 0,

		// ------------------------------------------------------------------------崩溃恢复
        .crash_dthreshold = 50,     				// degrees/second/second
        .crash_gthreshold = 400,    				// degrees/second
        .crash_setpoint_threshold = 350, 			// degrees/second
        .crash_time = 500,          				// ms
        .crash_delay = 0,           				// ms
        .crash_recovery_angle = 10, 				// degrees
        .crash_recovery_rate = 100, 				// degrees/second
		.crash_limit_yaw = 200,
        .crash_recovery = PID_CRASH_RECOVERY_OFF,   // off by default
        
		// ------------------------------------------------------------------------PID配置文件
        .auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY,
        .profileName = { 0 },
    );
#ifndef USE_D_MIN
    pidProfile->pid[PID_ROLL].D = 22;
    pidProfile->pid[PID_PITCH].D = 24;
#endif
}
void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

/**********************************************************************
函数名称：pidCopyProfile
函数功能：PID复制配置文件
函数形参：dstPidProfileIndex，srcPidProfileIndex
函数返回值：None 
函数描述：None 
**********************************************************************/
void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT
        && dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}

//-------------------------------------------------------------------------------------PID初始化部分API

/**********************************************************************
函数名称：pidSetTargetLooptime
函数功能：设置PID循环时间
函数形参：pidLooptime
函数返回值：None 
函数描述：None 
**********************************************************************/
static void pidSetTargetLooptime(uint32_t pidLooptime)
{
	// 跟随陀螺仪采样速率 - 125us
    targetPidLooptime = pidLooptime;
	// 时间常数dT = 125（us） / 1000000 = 0.000125（s）
    dT = targetPidLooptime * 1e-6f;  
	// PID频率 = 1 / 0.000125 = 8,000（8K）
    pidFrequency = 1.0f / dT;
#ifdef USE_DSHOT
    dshotSetPidLoopTime(targetPidLooptime);
#endif
}

/**********************************************************************
函数名称：pidInitConfig
函数功能：PID配置初始化
函数形参：pidProfile
函数返回值：None 
函数描述：None 
**********************************************************************/
void pidInitConfig(const pidProfile_t *pidProfile)
{
    if (pidProfile->feedForwardTransition == 0) {
        feedForwardTransition = 0;
    } else {
        feedForwardTransition = 100.0f / pidProfile->feedForwardTransition;
    }
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pidCoefficient[axis].Kp = PTERM_SCALE * pidProfile->pid[axis].P;
        pidCoefficient[axis].Ki = ITERM_SCALE * pidProfile->pid[axis].I;
        pidCoefficient[axis].Kd = DTERM_SCALE * pidProfile->pid[axis].D;
        pidCoefficient[axis].Kf = FEEDFORWARD_SCALE * (pidProfile->pid[axis].F / 100.0f);
    }
    {
        pidCoefficient[FD_YAW].Ki *= 2.5f;
    }

    levelGain = pidProfile->pid[PID_LEVEL].P / 10.0f;
    maxVelocity[FD_ROLL] = maxVelocity[FD_PITCH] = pidProfile->rateAccelLimit * 100 * dT;
    maxVelocity[FD_YAW] = pidProfile->yawRateAccelLimit * 100 * dT;
    itermWindupPointInv = 1.0f;
    if (pidProfile->itermWindupPointPercent < 100) {
        const float itermWindupPoint = pidProfile->itermWindupPointPercent / 100.0f;
        itermWindupPointInv = 1.0f / (1.0f - itermWindupPoint);
    }
    itermAcceleratorGain = pidProfile->itermAcceleratorGain;
    crashTimeLimitUs = pidProfile->crash_time * 1000;
    crashTimeDelayUs = pidProfile->crash_delay * 1000;
    crashRecoveryAngleDeciDegrees = pidProfile->crash_recovery_angle * 10;
    crashRecoveryRate = pidProfile->crash_recovery_rate;
    crashGyroThreshold = pidProfile->crash_gthreshold;
    crashDtermThreshold = pidProfile->crash_dthreshold;
    crashSetpointThreshold = pidProfile->crash_setpoint_threshold;
    crashLimitYaw = pidProfile->crash_limit_yaw;
    itermLimit = pidProfile->itermLimit;
#if defined(USE_THROTTLE_BOOST)
    throttleBoost = pidProfile->throttle_boost * 0.1f;
#endif
    antiGravityMode = pidProfile->antiGravityMode;

	// 计算将触发OSD显示的反重力值
	// 对于经典AG来说，要么是1.0用于关闭，> 1.0用于打开
	// 对于新的AG，它是一个连续的浮动值，所以想要触发OSD
	// 当超过其可能范围的25%时显示。这提供了一个有用的提示的AG激活，而不过度显示。
    antiGravityOsdCutoff = 0.0f;
    if (antiGravityMode == ANTI_GRAVITY_SMOOTH) {
        antiGravityOsdCutoff += ((itermAcceleratorGain - 1000) / 1000.0f) * 0.25f;
    }

#if defined(USE_ITERM_RELAX)
    itermRelax = pidProfile->iterm_relax;
    itermRelaxType = pidProfile->iterm_relax_type;
    itermRelaxCutoff = pidProfile->iterm_relax_cutoff;
#endif

#ifdef USE_DYN_LPF
    if (pidProfile->dyn_lpf_dterm_min_hz > 0) {
        switch (pidProfile->dterm_filter_type) {
        case FILTER_PT1:
            dynLpfFilter = DYN_LPF_PT1;
            break;
        default:
            dynLpfFilter = DYN_LPF_NONE;
            break;
        }
    } else {
        dynLpfFilter = DYN_LPF_NONE;
    }
    dynLpfMin = pidProfile->dyn_lpf_dterm_min_hz;
    dynLpfMax = pidProfile->dyn_lpf_dterm_max_hz;
    dynLpfCurveExpo = pidProfile->dyn_lpf_curve_expo;
#endif

#ifdef USE_THRUST_LINEARIZATION
    thrustLinearization = pidProfile->thrustLinearization / 100.0f;
    if (thrustLinearization != 0.0f) {
        thrustLinearizationReciprocal = 1.0f / thrustLinearization;
        thrustLinearizationB = (1.0f - thrustLinearization) / (2.0f * thrustLinearization);
    }
#endif
#if defined(USE_D_MIN)
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        const uint8_t dMin = pidProfile->d_min[axis];
        if ((dMin > 0) && (dMin < pidProfile->pid[axis].D)) {
            dMinPercent[axis] = dMin / (float)(pidProfile->pid[axis].D);
        } else {
            dMinPercent[axis] = 0;
        }
    }
    dMinGyroGain = pidProfile->d_min_gain * D_MIN_GAIN_FACTOR / D_MIN_LOWPASS_HZ;
    dMinSetpointGain = pidProfile->d_min_gain * D_MIN_SETPOINT_GAIN_FACTOR * pidProfile->d_min_advance * pidFrequency / (100 * D_MIN_LOWPASS_HZ);
    // 低通与增益成反比，因为较强的低通会降低峰值效应
#endif
#ifdef USE_INTERPOLATED_SP
    ffFromInterpolatedSetpoint = pidProfile->ff_interpolate_sp;
    ffSmoothFactor = 1.0f - ((float)pidProfile->ff_smooth_factor) / 100.0f;
    interpolatedSpInit(pidProfile);
#endif
}

/**********************************************************************
函数名称：pidInit
函数功能：PID初始化
函数形参：pidProfile
函数返回值：None 
函数描述：None 
**********************************************************************/
void pidInit(const pidProfile_t *pidProfile)
{
    pidSetTargetLooptime(gyro.targetLooptime);   // 初始化PID循环时间
    pidInitFilters(pidProfile);					 // PID滤波初始化						
    pidInitConfig(pidProfile);					 // PID配置初始化
}

/**********************************************************************
函数名称：pidInitFilters
函数功能：PID滤波初始化
函数形参：pidProfile
函数返回值：None 
函数描述：None 
**********************************************************************/
void pidInitFilters(const pidProfile_t *pidProfile)
{
	// 确保偏航轴为2
    STATIC_ASSERT(FD_YAW == 2, FD_YAW_incorrect); 			

	// ---------------------------------------------如果没有设置looptime，则所有的滤波器为null
    if (targetPidLooptime == 0) {
        dtermLowpassApplyFn = nullFilterApply;
        return;
    }

	// ---------------------------------------------PID奈奎斯特频率
	// 奈奎斯特频率（Nyquist frequency）是离散信号系统采样频率的一半。
	// 采样定理指出，只要离散系统的奈奎斯特频率高于被采样信号的最高频率或带宽，
	// 就可以真实的还原被测信号，反之，会因为频谱混叠而不能真实还原被测信号。
	// 8000 / 2 = 4000
    const uint32_t pidFrequencyNyquist = pidFrequency / 2; 

    // ---------------------------------------------Dterm低通滤波器1
    uint16_t dterm_lowpass_hz = pidProfile->dterm_lowpass_hz;

	// 如果开启动态低通滤波，那么静态低通滤波器1将变为动态低通滤波器1
#ifdef USE_DYN_LPF
    if (pidProfile->dyn_lpf_dterm_min_hz) {
        dterm_lowpass_hz = pidProfile->dyn_lpf_dterm_min_hz;
    }
#endif

    if (dterm_lowpass_hz > 0 && dterm_lowpass_hz < pidFrequencyNyquist) {
		// 判断滤波器类型
        switch (pidProfile->dterm_filter_type) {
	        case FILTER_PT1:
				// 应用PT1滤波
	            dtermLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
	            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
					// PT1滤波初始化
	                pt1FilterInit(&dtermLowpass[axis].pt1Filter, pt1FilterGain(dterm_lowpass_hz, dT));
	            }
	            break;
	        default:
	            dtermLowpassApplyFn = nullFilterApply;
	            break;
        }
    } else {
        dtermLowpassApplyFn = nullFilterApply;
    }

    // ---------------------------------------------Dterm低通滤波器2
    if (pidProfile->dterm_lowpass2_hz == 0 || pidProfile->dterm_lowpass2_hz > pidFrequencyNyquist) {
    	dtermLowpass2ApplyFn = nullFilterApply;
    } else {
        switch (pidProfile->dterm_filter2_type) {
	        case FILTER_PT1:
				// 应用PT1滤波
	            dtermLowpass2ApplyFn = (filterApplyFnPtr)pt1FilterApply;
	            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
					// PT1滤波初始化
	                pt1FilterInit(&dtermLowpass2[axis].pt1Filter, pt1FilterGain(pidProfile->dterm_lowpass2_hz, dT));
	            }
	            break;
	        default:
	            dtermLowpass2ApplyFn = nullFilterApply;
	            break;
        }
    }

	// ---------------------------------------------油门增压低通滤波
#if defined(USE_THROTTLE_BOOST)
	// PT1滤波初始化
    pt1FilterInit(&throttleLpf, pt1FilterGain(pidProfile->throttle_boost_cutoff, dT));
#endif

	// ---------------------------------------------I值释放
#if defined(USE_ITERM_RELAX)
    if (itermRelax) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
			// PT1滤波初始化
            pt1FilterInit(&windupLpf[i], pt1FilterGain(itermRelaxCutoff, dT));
        }
    }
#endif

	// ---------------------------------------------DMIN低通滤波
#if defined(USE_D_MIN)
	// 初始化所有轴的滤波器，即d_min[axis]值为0
	// 否则，如果pidProfile->d_min_xxx参数被添加，在飞行中调整和过渡从0到>0的功能不能工作，因为滤波器没有初始化
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
		// biquad低通滤波初始化
        biquadFilterInit(&dMinRange[axis], D_MIN_RANGE_HZ, targetPidLooptime);
		// PT1低通滤波初始化
        pt1FilterInit(&dMinLowpass[axis], pt1FilterGain(D_MIN_LOWPASS_HZ, dT));
     }
#endif

	// ---------------------------------------------反重力
    pt1FilterInit(&antiGravityThrottleLpf, pt1FilterGain(ANTI_GRAVITY_THROTTLE_FILTER_CUTOFF, dT));
    ffBoostFactor = (float)pidProfile->ff_boost / 10.0f;
    ffSpikeLimitInverse = pidProfile->ff_spike_limit ? 1.0f / ((float)pidProfile->ff_spike_limit / 10.0f) : 0.0f;
}

/**********************************************************************
函数名称：pidGetDT
函数功能：获取PID时间常数dT
函数形参：None
函数返回值：dT
函数描述：None
**********************************************************************/
float pidGetDT()
{
    return dT;
}

//-------------------------------------------------------------------------------------前馈部分API

/**********************************************************************
函数名称：pidGetSpikeLimitInverse
函数功能：获取逆峰值限制
函数形参：None 
函数返回值：ffSpikeLimitInverse
函数描述：None 
**********************************************************************/
float pidGetSpikeLimitInverse()
{
    return ffSpikeLimitInverse;
}

/**********************************************************************
函数名称：pidGetFfBoostFactor
函数功能：PID获取前馈助推因素
函数形参：None 
函数返回值：ffBoostFactor
函数描述：None 
**********************************************************************/
float pidGetFfBoostFactor()
{
    return ffBoostFactor;
}

/**********************************************************************
函数名称：pidGetFfSmoothFactor
函数功能：PID获取前馈平滑因素
函数形参：None 
函数返回值：ffSmoothFactor
函数描述：None 
**********************************************************************/
float pidGetFfSmoothFactor()
{
    return ffSmoothFactor;
}

//-------------------------------------------------------------------------------------I项部分API

/**********************************************************************
函数名称：pidResetIterm
函数功能：PID复位I项
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void pidResetIterm(void)
{	
	// 遍历三个轴RPY - I 复位
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].I = 0.0f;
    }
}

/**********************************************************************
函数名称：pidSetItermReset
函数功能：设置 PID I项复位状态
函数形参：enabled
函数返回值：None
函数描述：None
**********************************************************************/
void pidSetItermReset(bool enabled)
{
    zeroThrottleItermReset = enabled;
}

/**********************************************************************
函数名称：pidSetItermAccelerator
函数功能：设置PID        - I项加速度
函数形参：newItermAccelerator
函数返回值：None 
函数描述：None 
**********************************************************************/
void pidSetItermAccelerator(float newItermAccelerator)
{
    itermAccelerator = newItermAccelerator;
}

/**********************************************************************
函数名称：pidSetAntiGravityState
函数功能：PID设置反重力状态 - 是否激活反重力
函数形参：newState
函数返回值：None
函数描述：None
**********************************************************************/
void pidSetAntiGravityState(bool newState)
{
	// 判断反重力是否激活
    if (newState != antiGravityEnabled) {
        itermAccelerator = 0.0f;
    }
    antiGravityEnabled = newState;
}

/**********************************************************************
函数名称：pidAntiGravityEnabled
函数功能：获取PID反重力使能状态
函数形参：None
函数返回值：状态
函数描述：None
**********************************************************************/
bool pidAntiGravityEnabled(void)
{
    return antiGravityEnabled;
} 

/**********************************************************************
函数名称：pidOsdAntiGravityActive
函数功能：OSD获取反重力激活
函数形参：None 
函数返回值：激活状态 
函数描述：None 
**********************************************************************/
bool pidOsdAntiGravityActive(void)
{
    return (itermAccelerator > antiGravityOsdCutoff);
}

/**********************************************************************
函数名称：pidUpdateAntiGravityThrottleFilter
函数功能：PID更新反重力滤波油门
函数形参：油门
函数返回值：None 
函数描述：
	当检测到油门快速变化时反重力功能将增大 I 项。
	由混控器调用。
**********************************************************************/
void pidUpdateAntiGravityThrottleFilter(float throttle)
{
	// 反重力类型为平滑 - 使之不会猛增I项
    if (antiGravityMode == ANTI_GRAVITY_SMOOTH) {
		// 当前油门 - 滤波后的油门
        antiGravityThrottleHpf = throttle - pt1FilterApply(&antiGravityThrottleLpf, throttle);
    }
}

/**********************************************************************
函数名称：applyItermRelax
函数功能：应用Iterm释放
函数形参：axis，iterm，gyroRate，itermErrorRate，currentPidSetpoint
函数返回值：None 
函数描述：
	在进行大动作快速机动时抑制I值的积累。
	由PID控制器调用。
**********************************************************************/
#if defined(USE_ITERM_RELAX)
STATIC_UNIT_TESTED void applyItermRelax(const int axis, const float iterm,
    const float gyroRate, float *itermErrorRate, float *currentPidSetpoint)
{
    const float setpointLpf = pt1FilterApply(&windupLpf[axis], *currentPidSetpoint);
    const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);

    if (itermRelax) {
        if (axis < FD_YAW || itermRelax == ITERM_RELAX_RPY || itermRelax == ITERM_RELAX_RPY_INC) {
            const float itermRelaxFactor = MAX(0, 1 - setpointHpf / ITERM_RELAX_SETPOINT_THRESHOLD);
            const bool isDecreasingI =
                ((iterm > 0) && (*itermErrorRate < 0)) || ((iterm < 0) && (*itermErrorRate > 0));
            if ((itermRelax >= ITERM_RELAX_RP_INC) && isDecreasingI) {
                // 什么也不做，使用预先计算的itermErrorRate
            } else if (itermRelaxType == ITERM_RELAX_SETPOINT) {
                *itermErrorRate *= itermRelaxFactor;
            } else if (itermRelaxType == ITERM_RELAX_GYRO) {
                *itermErrorRate = fapplyDeadband(setpointLpf - gyroRate, setpointHpf);
            } else {
                *itermErrorRate = 0.0f;
            }
        }
    }
}
#endif

//-------------------------------------------------------------------------------------D项部分API

/**********************************************************************
函数名称：dynDtermLpfCutoffFreq
函数功能：计算D项动态低通滤波截止频率
函数形参：油门，最低截止频率，最高截止频率，固定曲线值
函数返回值：(dynLpfMax - dynLpfMin) * curve + dynLpfMin
函数描述：None
**********************************************************************/
float dynDtermLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo) {
    const float expof = expo / 10.0f;
    static float curve;
	// 曲线
    curve = throttle * (1 - throttle) * expof + throttle;
    return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
}

/**********************************************************************
函数名称：dynLpfDTermUpdate
函数功能：D项动态低通滤波更新
函数形参：throttle
函数返回值：None
函数描述：动态求截止频率。
**********************************************************************/
#ifdef USE_DYN_LPF
void dynLpfDTermUpdate(float throttle)
{
    static unsigned int cutoffFreq;
    if (dynLpfFilter != DYN_LPF_NONE) {
		// 根据曲线值求解截止频率
        if (dynLpfCurveExpo > 0) {
            cutoffFreq = dynDtermLpfCutoffFreq(throttle, dynLpfMin, dynLpfMax, dynLpfCurveExpo);
        } else {
            cutoffFreq = fmax(dynThrottle(throttle) * dynLpfMax, dynLpfMin);
        }

        if (dynLpfFilter == DYN_LPF_PT1) {
			// 遍历三轴
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
				// 根据截止频率计算滤波增益
                pt1FilterUpdateCutoff(&dtermLowpass[axis].pt1Filter, pt1FilterGain(cutoffFreq, dT));
            }
        }
    }
}
#endif

//-------------------------------------------------------------------------------------油门部分API

#ifdef USE_THRUST_LINEARIZATION
/**********************************************************************
函数名称：pidCompensateThrustLinearization
函数功能：油门线性化补偿
函数形参：throttle
函数返回值：throttle
函数描述：None 
**********************************************************************/
float pidCompensateThrustLinearization(float throttle)
{
    if (thrustLinearization != 0.0f) {
        throttle = throttle * (throttle * thrustLinearization + 1.0f - thrustLinearization);
    }
    return throttle;
}

/**********************************************************************
函数名称：pidApplyThrustLinearization
函数功能：应用油门线性化
函数形参：motorOutput
函数返回值：motorOutput
函数描述：None 
**********************************************************************/
float pidApplyThrustLinearization(float motorOutput)
{
    if (thrustLinearization != 0.0f) {
        if (motorOutput > 0.0f) {
            motorOutput = sqrtf(motorOutput * thrustLinearizationReciprocal +
                                thrustLinearizationB * thrustLinearizationB) - thrustLinearizationB;
        }
    }
    return motorOutput;
}
#endif

//-------------------------------------------------------------------------------------水平模式部分API

#if defined(USE_ACC)
/**********************************************************************
函数名称：pidLevel
函数功能：水平模式PID计算
函数形参：轴，PID配置文件，零偏，当前PID设置点
函数返回值：当前PID设置点
函数描述：
	1.外环角度环仅有比例[Kp]控制。
	2.角速度和角度的关系：
	 （1）角速度 = 角度/时间;
	 （2）角速度单位是(rad/s);
	 （3）角速度就是单位时间转过的弧度。
**********************************************************************/
float pidLevel(int axis, const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim, float currentPidSetpoint) 
{
	// 1.计算自稳模式期望角度，并将角度限制在最大倾角，pidProfile->levelAngleLimit = 55，rc偏差（外环角度控制[-1.0,1.0]）
    float angle = pidProfile->levelAngleLimit * getRcDeflection(axis);
	// 计算救援模式期望角度 - AI_PITCH,AI_ROLL
#ifdef USE_GPS_RESCUE
	// 角度单位：百分之度
    angle += gpsRescueAngle[axis] / 100; 		
#endif
	// 进行角度限制
    angle = constrainf(angle, -pidProfile->levelAngleLimit, pidProfile->levelAngleLimit);

	// 2.计算角度误差 - 期望角度 - ((原始测量角度值 - 零偏)/10)
    const float errorAngle = angle - ((attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f);

	// 3.自稳模式 || GPS救援模式 - 控制基于角度
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(GPS_RESCUE_MODE)) {
		// 计算当前PID设置点[期望角速度](外环角度只有P控制) - levelGain = PID_LEVEL_Kp/10
        currentPidSetpoint = errorAngle * levelGain;        
    } 
    return currentPidSetpoint;
}

//-------------------------------------------------------------------------------------崩溃恢复部分API

/**********************************************************************
函数名称：handleCrashRecovery
函数功能：崩溃（坠毁）恢复处理
函数形参：崩溃恢复模式，欧拉角，轴系，当前时间节拍，陀螺仪角速度，当前PID设置点，偏差
函数返回值：None 
函数描述：None 
**********************************************************************/
// 崩溃恢复检测时间
static timeUs_t crashDetectedAtUs;
static void handleCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const rollAndPitchTrims_t *angleTrim,
    const int axis, const timeUs_t currentTimeUs, const float gyroRate, float *currentPidSetpoint, float *errorRate)
{
    if (inCrashRecoveryMode && cmpTimeUs(currentTimeUs, crashDetectedAtUs) > crashTimeDelayUs) {
        if (crash_recovery == PID_CRASH_RECOVERY_BEEP) {
            BEEP_ON;
        }
        if (axis == FD_YAW) {
			// 偏航轴    - 约束限制
            *errorRate = constrainf(*errorRate, -crashLimitYaw, crashLimitYaw);
        } else {
            // 在横摇和俯仰轴上，计算当前的设定值和错误率，以使飞机从坠毁中恢复
            if (sensors(SENSOR_ACC)) {
                // 角度偏差
                const float errorAngle =  -(attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
                *currentPidSetpoint = errorAngle * levelGain;
                *errorRate = *currentPidSetpoint - gyroRate;
            }
        }
		// 重置I项，因为在崩溃之前累积的错误现在是无意义的
		// 在崩溃恢复期间可能是极端的，特别是在偏航轴
        pidData[axis].I = 0.0f;
        if (cmpTimeUs(currentTimeUs, crashDetectedAtUs) > crashTimeLimitUs
            || (getMotorMixRange() < 1.0f
                   && fabsf(gyro.gyroADCf[FD_ROLL]) < crashRecoveryRate
                   && fabsf(gyro.gyroADCf[FD_PITCH]) < crashRecoveryRate
                   && fabsf(gyro.gyroADCf[FD_YAW]) < crashRecoveryRate)) {
            if (sensors(SENSOR_ACC)) {
                // 检查水平
                if (ABS(attitude.raw[FD_ROLL] - angleTrim->raw[FD_ROLL]) < crashRecoveryAngleDeciDegrees
                   && ABS(attitude.raw[FD_PITCH] - angleTrim->raw[FD_PITCH]) < crashRecoveryAngleDeciDegrees) {
                    inCrashRecoveryMode = false;
                    BEEP_OFF;
                }
            } else {
                inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        }
    }
}

/**********************************************************************
函数名称：detectAndSetCrashRecovery
函数功能：检测和设置崩溃恢复
函数形参：crash_recovery，angleTrim，axis，currentTimeUs，delta，errorRate
函数返回值：None 
函数描述：None 
**********************************************************************/
static void detectAndSetCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const int axis,
    const timeUs_t currentTimeUs, const float delta, const float errorRate)
{
	// 如果崩溃恢复和加速计启用，检查崩溃
    if (crash_recovery || FLIGHT_MODE(GPS_RESCUE_MODE)) {
		// 解锁
        if (ARMING_FLAG(ARMED)) {
            if (getMotorMixRange() >= 1.0f && !inCrashRecoveryMode
                && fabsf(delta) > crashDtermThreshold
                && fabsf(errorRate) > crashGyroThreshold
                && fabsf(getSetpointRate(axis)) < crashSetpointThreshold) {
                // 判断崩溃恢复类型
                if (crash_recovery == PID_CRASH_RECOVERY_DISARM) {
					// 设置解锁禁用
                    setArmingDisabled(ARMING_DISABLED_CRASH_DETECTED);
					// 禁止解锁
                    disarm(DISARM_REASON_CRASH_PROTECTION);
                } else {
					// 使能崩溃恢复模式
                    inCrashRecoveryMode = true;
					// 更新崩溃检测时间
                    crashDetectedAtUs = currentTimeUs;
                }
            }
            if (inCrashRecoveryMode && cmpTimeUs(currentTimeUs, crashDetectedAtUs) < crashTimeDelayUs && (fabsf(errorRate) < crashGyroThreshold
                || fabsf(getSetpointRate(axis)) > crashSetpointThreshold)) {
                inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        } else if (inCrashRecoveryMode) {
            inCrashRecoveryMode = false;
            BEEP_OFF;
        }
    }
}
#endif // USE_ACC

/**********************************************************************
函数名称：crashRecoveryModeActive
函数功能：获取崩溃恢复模式激活状态
函数形参：None
函数返回值：inCrashRecoveryMode
函数描述：None
**********************************************************************/
bool crashRecoveryModeActive(void)
{
    return inCrashRecoveryMode;
}

//-------------------------------------------------------------------------------------PID设置点速率限制部分API

/**********************************************************************
函数名称：accelerationLimit
函数功能：Setpoint速率限制
函数形参：axis，currentPidSetpoint
函数返回值：currentPidSetpoint
函数描述：None 
**********************************************************************/
static float accelerationLimit(int axis, float currentPidSetpoint)
{
    static float previousSetpoint[XYZ_AXIS_COUNT];
    const float currentVelocity = currentPidSetpoint - previousSetpoint[axis];

    if (fabsf(currentVelocity) > maxVelocity[axis]) {
        currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + maxVelocity[axis] : previousSetpoint[axis] - maxVelocity[axis];
    }

    previousSetpoint[axis] = currentPidSetpoint;
    return currentPidSetpoint;
}

//-------------------------------------------------------------------------------------PID控制器

/**********************************************************************
函数名称：pidController
函数功能：PID控制器
函数形参：pidProfile,currentTimeUs
函数返回值：None
函数描述：
	基于2自由度参考设计(matlab)
**********************************************************************/
void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
	// -------------------------------上一个D项陀螺仪速率
    static float previousGyroRateDterm[XYZ_AXIS_COUNT];		
#ifdef USE_INTERPOLATED_SP
	// -------------------------------上一帧
    static FAST_RAM_ZERO_INIT uint32_t lastFrameNumber;		
#endif
#if defined(USE_ACC)
	// -------------------------------自稳模式起始时间
    static timeUs_t levelModeStartTimeUs = 0;               
    // -------------------------------GPS救援模式前一个状态
    static bool gpsRescuePreviousState = false;				
#endif
	// -------------------------------TPA因素 - 获取油门PID衰减
    const float tpaFactor = getThrottlePIDAttenuation();	
	// -------------------------------获取ROLL和PITCH角度零偏
#if defined(USE_ACC)
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
#else
    UNUSED(pidProfile);
    UNUSED(currentTimeUs);
#endif
	// -------------------------------TPA_kp因素 - 应用类型
#ifdef USE_TPA_MODE
    const float tpaFactorKp = (currentControlRateProfile->tpaMode == TPA_MODE_PD) ? tpaFactor : 1.0f;
#else
    const float tpaFactorKp = tpaFactor;
#endif
	// -------------------------------获取反自旋激活状态
#ifdef USE_YAW_SPIN_RECOVERY
    const bool yawSpinActive = gyroYawSpinDetected();		
#endif

	// -------------------------------自稳模式和GPS救援模式相关
#if defined(USE_ACC)
	// 获取GPS救援模式激活状态
    const bool gpsRescueIsActive = FLIGHT_MODE(GPS_RESCUE_MODE);
    levelMode_e levelMode;
	// 飞行模式为自稳模式 ||          GPS救援模式
    if (FLIGHT_MODE(ANGLE_MODE) || gpsRescueIsActive) {
        levelMode = LEVEL_MODE_RP;
    } else {			
        levelMode = LEVEL_MODE_OFF;
    }

	// -------------------------------跟踪何时进入自稳模式，以便可以在崩溃恢复激活之前增加一个保护时间
    if (levelMode) {
		// 当GPS救援被激活时重置起始时间
        if ((levelModeStartTimeUs == 0) || (gpsRescueIsActive && !gpsRescuePreviousState)) {
            levelModeStartTimeUs = currentTimeUs;
        }
    } else {
        levelModeStartTimeUs = 0;
    }
	// 更新GPS救援状态下一次使用
    gpsRescuePreviousState = gpsRescueIsActive;
#endif

    // -------------------------------反重力 - 计算基于反重力的I项增益
    if ((antiGravityMode == ANTI_GRAVITY_SMOOTH) && antiGravityEnabled) {
		// 通过检测油门变化率判断反重力是否开启I项
        itermAccelerator = fabsf(antiGravityThrottleHpf) * 0.01f * (itermAcceleratorGain - 1000);
    }
	// 计算增益
    float agGain = dT * itermAccelerator * AG_KI;
    // 当超过终点时，逐步缩小整合
    float dynCi = dT;
	// 约束itermWindupPointInv设置
    if (itermWindupPointInv > 1.0f) {
        dynCi *= constrainf((1.0f - getMotorMixRange()) * itermWindupPointInv, 0.0f, 1.0f);
    }

    // -------------------------------预先计算D项陀螺仪数据 - D项低通滤波
    float gyroRateDterm[XYZ_AXIS_COUNT];
	// 遍历三个轴RPY
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        gyroRateDterm[axis] = gyro.gyroADCf[axis];
        gyroRateDterm[axis] = dtermLowpassApplyFn((filter_t *) &dtermLowpass[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = dtermLowpass2ApplyFn((filter_t *) &dtermLowpass2[axis], gyroRateDterm[axis]);
    }

	// -------------------------------判断是否有新的RC帧数据
#ifdef USE_INTERPOLATED_SP
    bool newRcFrame = false;
    if (lastFrameNumber != getRcFrameNumber()) {
        lastFrameNumber = getRcFrameNumber();
        newRcFrame = true;
    }
#endif

    /* --------------------------------PID控制器--------------------------------*/
	// 遍历三个轴RPY
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
		// --------------------------------------------------------------------------1.获取当前设置点速率 - RC输入期望值（期望速率值）
        float currentPidSetpoint = getSetpointRate(axis);
        if (maxVelocity[axis]) {
			// 进行加速度限制
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
        }

		// --------------------------------------------------------------------------2.自稳模式串级计算
#if defined(USE_ACC)
        switch (levelMode) {
	        case LEVEL_MODE_OFF:
	            break;
	        case LEVEL_MODE_R:
				// 基于陀螺仪的俯仰控制 - 使用角速度
	            if (axis == FD_PITCH) {                     
	                break;
	            }
	            FALLTHROUGH;
	        case LEVEL_MODE_RP:
				// 基于陀螺仪的偏航控制 - 使用角速度
	            if (axis == FD_YAW) {						
	                break;
	            }
				// 当前设置点速率 = ROLL/PITCH 外环角度偏差[Kp比例控制]       ->内环期望角速度
            	currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
        }
#endif
		// 如果反自旋被激活 - 偏航设置点速率将被重置
#ifdef USE_YAW_SPIN_RECOVERY
        if ((axis == FD_YAW) && yawSpinActive) {
            currentPidSetpoint = 0.0f;
        }
#endif 

        // --------------------------------------------------------------------------3.计算偏差 - 内环角速度偏差
        // 获取当前陀螺仪角速度 - deg/s
        const float gyroRate = gyro.gyroADCf[axis];    
		// 偏差 = 期望值 - 测量值
        float errorRate = currentPidSetpoint - gyroRate; 
#if defined(USE_ACC)
		// 崩溃恢复处理
        handleCrashRecovery(pidProfile->crash_recovery, angleTrim, axis, currentTimeUs, gyroRate, &currentPidSetpoint, &errorRate);
#endif

        const float previousIterm = pidData[axis].I;
        float itermErrorRate = errorRate;

		// I项释放 - 未在崩溃恢复模式
#if defined(USE_ITERM_RELAX)
        if (!inCrashRecoveryMode) {
			// 应用I项释放
            applyItermRelax(axis, previousIterm, gyroRate, &itermErrorRate, &currentPidSetpoint);
            errorRate = currentPidSetpoint - gyroRate;
        }
#endif

        // --------------------------------------------------------------------------4.低阶PID控制
		// 带导数项可选滤波的2自由度PID控制器
		// b = 1，只有c(前馈权重)可以调整(数量测量或误差的导数)

        // ----------------------------------------------------------------计算P项
        // 俯仰横滚 - P = kp * 偏差 * TPA因素（衰减）
        pidData[axis].P = pidCoefficient[axis].Kp * errorRate * tpaFactorKp;

        // ----------------------------------------------------------------计算I项
        float Ki;
        float axisDynCi;
        {
            Ki = pidCoefficient[axis].Ki;
			// 只对偏航施加保护
            axisDynCi = (axis == FD_YAW) ? dynCi : dT; 	
        }
        pidData[axis].I = constrainf(previousIterm + (Ki * axisDynCi + agGain) * itermErrorRate, -itermLimit, itermLimit);

        // ----------------------------------------------------------------计算pidSetpoint误差
        float pidSetpointDelta = 0;
#ifdef USE_INTERPOLATED_SP
		// 判断是否开启前馈插值(插值类型不为FF_INTERPOLATE_OFF)
        if (ffFromInterpolatedSetpoint) {
			// 设置点增量 = 插值设置点应用
            pidSetpointDelta = interpolatedSpApply(axis, newRcFrame, ffFromInterpolatedSetpoint);
        } else {
         	// 设置点增量 = 当前PID设置点 - 上一个PID设置点
            pidSetpointDelta = currentPidSetpoint - previousPidSetpoint[axis];
        }
#endif
		// 更新当前PID设置点为下一次使用
        previousPidSetpoint[axis] = currentPidSetpoint;

        // ----------------------------------------------------------------计算D项
        // 判断是否使用D项
        if ((pidCoefficient[axis].Kd > 0)) {
			// 速率变化除以dT得到微分(即dr/dT) - dT是固定的，从目标PID回路时间计算
			// 为了避免DTerm的峰值动态发生
            const float delta = - (gyroRateDterm[axis] - previousGyroRateDterm[axis]) * pidFrequency;

#if defined(USE_ACC)
            if (cmpTimeUs(currentTimeUs, levelModeStartTimeUs) > CRASH_RECOVERY_DETECTION_DELAY_US) {
				// 检测和设置崩溃恢复处理
                detectAndSetCrashRecovery(pidProfile->crash_recovery, axis, currentTimeUs, delta, errorRate);
            }
#endif

            float dMinFactor = 1.0f;
#if defined(USE_D_MIN)
            if (dMinPercent[axis] > 0) {
                float dMinGyroFactor = biquadFilterApply(&dMinRange[axis], delta);
				// 1.DMIN陀螺仪（测量值）因素 = |DMIN陀螺仪因素| * DMIN陀螺仪增益
                dMinGyroFactor = fabsf(dMinGyroFactor) * dMinGyroGain;
				// 2.DMIN设置点（摇杆误差）因素 = |PID设置点增量| * DMIN设置点增益
                const float dMinSetpointFactor = (fabsf(pidSetpointDelta)) * dMinSetpointGain;
                dMinFactor = MAX(dMinGyroFactor, dMinSetpointFactor);
                dMinFactor = dMinPercent[axis] + (1.0f - dMinPercent[axis]) * dMinFactor;
                dMinFactor = pt1FilterApply(&dMinLowpass[axis], dMinFactor);
                dMinFactor = MIN(dMinFactor, 1.0f);
            }
#endif
            pidData[axis].D = pidCoefficient[axis].Kd * delta * tpaFactor * dMinFactor;
        }else {
            pidData[axis].D = 0;
        }
        previousGyroRateDterm[axis] = gyroRateDterm[axis];

		// ----------------------------------------------------------------计算前馈
        // 获取前馈增益 - 前馈仅在手动模式下有效
        const float feedforwardGain = (flightModeFlags) ? 0.0f : pidCoefficient[axis].Kf;
        if (feedforwardGain > 0) {
			// 如果feedForwardTransition == 0，则没有转换
            float transition = feedForwardTransition > 0 ? MIN(1.f, getRcDeflectionAbs(axis) * feedForwardTransition) : 1;
            float feedForward = feedforwardGain * transition * pidSetpointDelta * pidFrequency;
#ifdef USE_INTERPOLATED_SP
			// 计算F项 - 判断是否应用前馈限制
            pidData[axis].F = shouldApplyFfLimits(axis) ? applyFfLimit(axis, feedForward, pidCoefficient[axis].Kp, currentPidSetpoint) : feedForward;
#endif
        } else {
            pidData[axis].F = 0;
        }

		// ----------------------------------------------------------------判断反自旋模式是否被激活
#ifdef USE_YAW_SPIN_RECOVERY
        if (yawSpinActive) {
			// 反自旋禁用I
            pidData[axis].I = 0;  		
            if (axis <= FD_PITCH)  {
                // 俯仰和横滚的PID为零，偏航P以纠正自旋
                pidData[axis].P = 0;
                pidData[axis].D = 0;
                pidData[axis].F = 0;
            }
        }
#endif

        // ----------------------------------------------------------------计算PID SUM(P+I+D+F)
        const float pidSum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;
        {
            pidData[axis].Sum = pidSum;
        }
    }
	
	/* -------------------------------PID控制复位-------------------------------*/
	// 零油门I项复位 - 消除I项累积
	if (zeroThrottleItermReset) {
        pidResetIterm();
    }
}

