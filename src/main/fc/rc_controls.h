#pragma once

#include <stdbool.h>

#include "common/filter.h"
#include "pg/pg.h"

// 摇杆方向定义
#define ROL_LO (1 << (2 * ROLL))
#define ROL_CE (3 << (2 * ROLL))
#define ROL_HI (2 << (2 * ROLL))
#define PIT_LO (1 << (2 * PITCH))
#define PIT_CE (3 << (2 * PITCH))
#define PIT_HI (2 << (2 * PITCH))
#define YAW_LO (1 << (2 * YAW))
#define YAW_CE (3 << (2 * YAW))
#define YAW_HI (2 << (2 * YAW))
#define THR_LO (1 << (2 * THROTTLE))
#define THR_CE (3 << (2 * THROTTLE))
#define THR_HI (2 << (2 * THROTTLE))

// RC固定曲线值最大值
#define CONTROL_RATE_CONFIG_RC_EXPO_MAX          100
// RC速率最大值
#define CONTROL_RATE_CONFIG_RC_RATES_MAX         255
// RC速率限制
#define CONTROL_RATE_CONFIG_RATE_LIMIT_MIN	     200
#define CONTROL_RATE_CONFIG_RATE_LIMIT_MAX	     1998
// (Super)比率被限制在[0,100]范围扩大了比赛飞行费率，所以高于100的值不会有区别。
#define CONTROL_RATE_CONFIG_RATE_MAX             255
// TPA最大值
#define CONTROL_RATE_CONFIG_TPA_MAX              100

/* ------------------------------通道标签枚举------------------------------ */	
typedef enum rc_alias {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8,
    AUX9,
    AUX10,
    AUX11,
    AUX12
} rc_alias_e;
// 通道数定义
#define PRIMARY_CHANNEL_COUNT (THROTTLE + 1)

/* ------------------------------油门状态枚举------------------------------ */	
typedef enum {
    THROTTLE_LOW = 0,											 // 低位
    THROTTLE_HIGH												 // 高位
} throttleStatus_e;
// 空中模式死区
#define AIRMODEDEADBAND 12

/* ----------------------------rollPitch状态枚举--------------------------- */	
typedef enum {
    NOT_CENTERED = 0,
    CENTERED													 // 居中
} rollPitchStatus_e;

/* -------------------------------RC滤波枚举------------------------------- */	
typedef enum {
    RC_SMOOTHING_OFF = 0,										 // 关闭
    RC_SMOOTHING_DEFAULT,										 // 默认
    RC_SMOOTHING_AUTO,											 // 自动
    RC_SMOOTHING_MANUAL											 // 手动
} rcSmoothing_t;
	
/* -----------------------------RC平滑类型枚举----------------------------- */	
typedef enum {
    RC_SMOOTHING_TYPE_INTERPOLATION,							 // 插值
} rcSmoothingType_e;

/* ---------------------------RC平滑输入滤波器枚举-------------------------- */	
typedef enum {
    RC_SMOOTHING_INPUT_PT1,
    RC_SMOOTHING_INPUT_BIQUAD
} rcSmoothingInputFilter_e;

/* ---------------------------RC平滑导数滤波器枚举-------------------------- */	
typedef enum {
    RC_SMOOTHING_DERIVATIVE_OFF,
    RC_SMOOTHING_DERIVATIVE_PT1,
    RC_SMOOTHING_DERIVATIVE_BIQUAD,
    RC_SMOOTHING_DERIVATIVE_AUTO,
} rcSmoothingDerivativeFilter_e;

/* -----------------------------RC平滑滤波器训练结构体---------------------------- */	
typedef struct rcSmoothingFilterTraining_s {
    float sum;
    int count;
    uint16_t min;
    uint16_t max;
} rcSmoothingFilterTraining_t;

/* ----------------------------RC平滑滤波器类型枚举共用体-------------------------- */	
typedef union rcSmoothingFilterTypes_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
} rcSmoothingFilterTypes_t;

/* --------------------------------RC平滑滤波器结构体------------------------------ */	
typedef struct rcSmoothingFilter_s {
    bool filterInitialized;									     // 滤波器初始化状态
    rcSmoothingFilterTypes_t filter[4];                          // RC平滑滤波器类型
    rcSmoothingInputFilter_e inputFilterType;					 // RC输入滤波器
    uint8_t inputCutoffSetting;									 // 输入截至设置
    uint16_t inputCutoffFrequency;								 // 输入截至频率
    rcSmoothingDerivativeFilter_e derivativeFilterTypeSetting;   // 滤波器类型设置
    rcSmoothingDerivativeFilter_e derivativeFilterType;          // 滤波器类型
    uint8_t derivativeCutoffSetting;							 // 导数截止设置
    uint16_t derivativeCutoffFrequency;							 // 导数截止频率
    int averageFrameTimeUs;										 // 帧平均时间
    rcSmoothingFilterTraining_t training;						 // RC平滑滤波器训练
    uint8_t autoSmoothnessFactor;                                // 自动平滑因素
} rcSmoothingFilter_t;	

/* ----------------------------------RC控制配置结构体------------------------------- */	
typedef struct rcControlsConfig_s {
    uint8_t deadband;                       					 // 俯仰and横滚死区
    uint8_t yaw_deadband;                   					 // 偏航死区
    uint8_t alt_hold_deadband;              					 // 定义油门杆保持高度时的中立区，默认设置为+/-40
    uint8_t alt_hold_fast_change;           				     // 当禁用时，当油门超出alt_hold_deadband定义的死区时关闭阀值;当启用时，高度变化缓慢成比例的移动
    bool yaw_control_reversed;              					 // 偏航反转
} rcControlsConfig_t;
// 声明RC控制配置结构体
PG_DECLARE(rcControlsConfig_t, rcControlsConfig);

/* -----------------------------------解锁配置结构体-------------------------------- */
typedef struct armingConfig_s {
    uint8_t gyro_cal_on_first_arm;          					 // 允许解锁/锁定在油门下+左右滚动
    uint8_t auto_disarm_delay;              					 // 自动解锁延时
} armingConfig_t;
// 声明解锁配置结构体
PG_DECLARE(armingConfig_t, armingConfig);

extern float rcCommand[4];
bool areUsingSticksToArm(void);
throttleStatus_e calculateThrottleStatus(void);
void processRcStickPositions();
void rcControlsInit(void);

