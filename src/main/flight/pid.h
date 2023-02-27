#pragma once

#include <stdbool.h>
#include "common/time.h"
#include "common/filter.h"
#include "common/axis.h"
#include "pg/pg.h"

#define MAX_PID_PROCESS_DENOM       16			// PID进程最大分母项
#define PID_MIXER_SCALING           1000.0f		// PID混控比例
#define PIDSUM_LIMIT                500			// PID和限制
#define PIDSUM_LIMIT_YAW            400			// PID和偏航限制
#define PIDSUM_LIMIT_MIN            100			// PID和最小限制
#define PIDSUM_LIMIT_MAX            1000		// PID和最大限制

// PID控制器的缩放因子
#define PTERM_SCALE 0.032029f
#define ITERM_SCALE 0.244381f
#define DTERM_SCALE 0.000529f

// 用常数标度因子代替Kd分量的前馈计算，这个值与之前的Kd默认值26 (26 * DTERM_SCALE)相同
#define FEEDFORWARD_SCALE 0.013754f

// Full iterm 抑制 in setpoint mode at 高通滤波 setpoint rate > 40deg/sec
#define ITERM_RELAX_SETPOINT_THRESHOLD 40.0f
#define ITERM_RELAX_CUTOFF_DEFAULT 10

// 反重力I常数
#define AG_KI 21.586988f;

// I项加速增益
#define ITERM_ACCELERATOR_GAIN_OFF 1000
#define ITERM_ACCELERATOR_GAIN_MAX 30000

/* --------------------------PID索引枚举-------------------------- */	
typedef enum {
    PID_ROLL,										// 横滚
    PID_PITCH,										// 俯仰
    PID_YAW,										// 偏航
    PID_LEVEL,										// 自稳
    PID_MAG,										// 锁头
    PID_ITEM_COUNT
} pidIndex_e;

/* -------------------------PID轴系数据结构体--------------------- */	
typedef struct pidAxisData_s {
    float P;
    float I;
    float D;
    float F;
    float Sum;
} pidAxisData_t;

/* ----------------------------PID结构体-------------------------- */	
typedef struct pidf_s {
    uint8_t P;
    uint8_t I;
    uint8_t D;
    uint16_t F;
} pidf_t;

/* ---------------------------PID系数结构体---------------------- */	
typedef struct pidCoefficient_s {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
} pidCoefficient_t;

/* ---------------------------iterm释放枚举----------------------- */	
typedef enum {
    ITERM_RELAX_OFF,
    ITERM_RELAX_RP,
    ITERM_RELAX_RPY,
    ITERM_RELAX_RP_INC,
    ITERM_RELAX_RPY_INC,
    ITERM_RELAX_COUNT,
} itermRelax_e;

/* -------------------------iterm释放类型枚举--------------------- */	
typedef enum {
    ITERM_RELAX_GYRO,
    ITERM_RELAX_SETPOINT,
    ITERM_RELAX_TYPE_COUNT,
} itermRelaxType_e;

/* --------------------------反重力模式枚举----------------------- */	
typedef enum {
    ANTI_GRAVITY_SMOOTH,
    ANTI_GRAVITY_STEP
} antiGravityMode_e;

/* ------------------------D项低通滤波共用体------------------- */	
typedef union dtermLowpass_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
} dtermLowpass_t;

/* -------------------------PID崩溃恢复枚举----------------------- */	
typedef enum {
    PID_CRASH_RECOVERY_OFF = 0,
    PID_CRASH_RECOVERY_ON,
    PID_CRASH_RECOVERY_BEEP,
    PID_CRASH_RECOVERY_DISARM,
} pidCrashRecovery_e;

/* ---------------------------水平模式枚举----------------------- */	
typedef enum {
    LEVEL_MODE_OFF = 0,
    LEVEL_MODE_R,
    LEVEL_MODE_RP,
} levelMode_e;

/* -------------------------PID配置文件结构体--------------------- */	
#define MAX_PROFILE_NAME_LENGTH 8u 					// 配置文件名称最大长度
typedef struct pidProfile_s {
	// ------------------------------------------------------------------------PID参数
    pidf_t  pid[PID_ITEM_COUNT];					// PID参数

	// ------------------------------------------------------------------------前馈 - 动态调节P
    uint8_t ff_interpolate_sp;              	 	// 前馈插值定位点
    uint8_t ff_max_rate_limit;              	 	// 前最大速率限制
    uint8_t ff_spike_limit;                 	 	// 前馈摇杆预测推断限制
    uint8_t ff_smooth_factor;               	 	// 前馈插值平滑因素
    uint8_t feedForwardTransition;          		// 前馈权值转换
    uint8_t ff_boost;                       		// 高通滤波后的前馈添加到前馈的数量，100表示100%添加

	// ------------------------------------------------------------------------I项释放 &&    反重力    - 动态调节I
    uint8_t iterm_relax_type;               		// I项释放类型
    uint8_t iterm_relax_cutoff;             		// I项释放截止频率
    uint8_t iterm_relax;                    		// I项释放应用轴系
    uint8_t itermWindupPointPercent;        		// I项起始阈值，电机饱和的百分比
    uint16_t itermThrottleThreshold;        		// I项油门阈值
    uint16_t itermAcceleratorGain;          		// 当itermThrottlethreshold被命中时，Iterm加速器增益
    uint16_t itermLimit;							// I项限制 
    uint8_t  antiGravityMode;               		// 反重力模式类型 - 当检测到油门快速变化时反重力功能将增大I项

	// ------------------------------------------------------------------------D_MIN - 动态调节D
    uint8_t d_min[XYZ_AXIS_COUNT];          		// D_MIN参数
    uint8_t d_min_gain;                     		// D_MIN增益
    uint8_t d_min_advance;                  		// D_MIN超前

	// ------------------------------------------------------------------------D项低通滤波器 
    uint8_t dterm_filter_type;              		// D项低通滤波器1类型
    uint16_t dterm_lowpass_hz;              		// D项低通滤波1截止频率
    uint8_t dterm_filter2_type;             		// D项低通滤波器2类型
    uint16_t dterm_lowpass2_hz;             		// D项低通滤波2截止频率
    uint16_t dyn_lpf_dterm_min_hz;          		// D项动态低通滤波器1最小频率               
    uint16_t dyn_lpf_dterm_max_hz;          		// D项动态低通滤波器1最大频率  
    uint8_t dyn_lpf_curve_expo;             	 	// 设置动态D项低通滤波器的曲线

	// ------------------------------------------------------------------------pidSum
    uint16_t pidSumLimit;                           // PID和限制
    uint16_t pidSumLimitYaw;                        // 偏航PID和限制
    uint8_t thrustLinearization;            		// pid线性化的补偿因子
    uint8_t motor_output_limit;             		// 电机输出上限(百分比)

	// ------------------------------------------------------------------------油门
    uint8_t throttle_boost;                 		// 油门增压
    uint8_t throttle_boost_cutoff;          		// 油门增压截止

	// ------------------------------------------------------------------------自稳模式
    uint8_t levelAngleLimit;                		// 自稳模式角度限制

	// ------------------------------------------------------------------------加速度限制
    uint16_t yawRateAccelLimit;             		// 偏航加速度限制 - deg/sec/ms
    uint16_t rateAccelLimit;                		// 速率加速度限制 - 横摇/俯仰 - 度/秒/毫秒

	// ------------------------------------------------------------------------崩溃恢复
    uint16_t crash_dthreshold;              		// D项崩溃阈值
    uint16_t crash_gthreshold;              		// 陀螺仪崩溃阈值
    uint16_t crash_setpoint_threshold;      		// 设定点崩溃阈值
    uint16_t crash_time;                    		// 崩溃时间 - ms
    uint16_t crash_delay;                   		// 崩溃延时 - ms
    uint8_t crash_recovery_angle;           		// 崩溃恢复角度 - degrees
    uint8_t crash_recovery_rate;            		// 崩溃恢复速率 - degree/second
    uint16_t crash_limit_yaw;               		// 限制偏航崩溃错误率，撞机不会造成巨大的油门增加
    uint8_t crash_recovery;                			// 在崩溃恢复模式时发出哔哔声

	// ------------------------------------------------------------------------PID配置文件
    int8_t auto_profile_cell_count;         		// 如果使用自动PID配置文件切换，则将使用此配置文件的单元数
    char profileName[MAX_PROFILE_NAME_LENGTH + 1];  // 概要文件的描述性名称
} pidProfile_t;
// 声明PID配置文件结构体
PG_DECLARE_ARRAY(pidProfile_t, PID_PROFILE_COUNT, pidProfiles);

/* ---------------------------PID配置结构体----------------------- */	
typedef struct pidConfig_s {
    uint8_t pid_process_denom;              	    // PID控制器进程分母项
    uint8_t runaway_takeoff_prevention;          	// off, on -使能pidsum失控起飞逻辑
    uint16_t runaway_takeoff_deactivate_delay;   	// 未激活(成功飞行)前“飞行中”条件的ms延迟
    uint8_t runaway_takeoff_deactivate_throttle; 	// 在未激活阶段所需的最小油门百分比
} pidConfig_t;
// 声明PID配置结构体
PG_DECLARE(pidConfig_t, pidConfig);

extern const char pidNames[];
extern pidAxisData_t pidData[3];
extern uint32_t targetPidLooptime;
extern float throttleBoost;
extern pt1Filter_t throttleLpf;
union rollAndPitchTrims_u;
void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs);
void pidResetIterm(void);
void pidSetItermAccelerator(float newItermAccelerator);
void pidInitFilters(const pidProfile_t *pidProfile);
void pidInitConfig(const pidProfile_t *pidProfile);
void pidInit(const pidProfile_t *pidProfile);
void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex);
bool crashRecoveryModeActive(void);
void pidUpdateAntiGravityThrottleFilter(float throttle);
bool pidOsdAntiGravityActive(void);
void pidSetAntiGravityState(bool newState);
bool pidAntiGravityEnabled(void);
#ifdef USE_THRUST_LINEARIZATION
float pidApplyThrustLinearization(float motorValue);
float pidCompensateThrustLinearization(float throttle);
#endif
void dynLpfDTermUpdate(float throttle);
void pidSetItermReset(bool enabled);
float pidGetDT();
float pidGetFfBoostFactor();
float pidGetFfSmoothFactor();
float pidGetSpikeLimitInverse();
float dynDtermLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo);

