#pragma once

#include "pg/pg.h"

#define FAILSAFE_POWER_ON_DELAY_US (1000 * 1000 * 5)
#define MILLIS_PER_TENTH_SECOND      100
#define MILLIS_PER_SECOND           1000
#define PERIOD_OF_1_SECONDS            1 * MILLIS_PER_SECOND
#define PERIOD_OF_3_SECONDS            3 * MILLIS_PER_SECOND
#define PERIOD_OF_30_SECONDS          30 * MILLIS_PER_SECOND
#define PERIOD_RXDATA_FAILURE        200    		// millis
#define PERIOD_RXDATA_RECOVERY       200    		// millis

/* --------------------------失控保护配置结构体-------------------------- */	
typedef struct failsafeConfig_s {
    uint16_t failsafe_throttle;             		// 用于着陆的油门水平-指定值在1000..2000(略低于悬停的pwm脉冲宽度)。中心油门= 1500。
    uint16_t failsafe_throttle_low_delay;   		// 时间油门必须低于“min_check”，以“只是解除”，而不是“完全故障安全程序”。
    uint8_t failsafe_delay;                 		// 信号丢失后故障安全激活的保护时间。1步= 0.1秒- 1秒示例(10)
    uint8_t failsafe_off_delay;             		// 马达停止前的着陆时间为0.1秒。1步= 0.1秒- 20秒示例(200)
    uint8_t failsafe_switch_mode;           		// 故障安全开关动作为0:阶段1(与rc链路丢失相同)，1:立即解除武器，2:阶段2
    uint8_t failsafe_procedure;             		// 选中的全故障保护程序为0:自动着陆，1:删除
    uint16_t failsafe_recovery_delay;       		// 从故障安全程序中恢复所需的有效rx数据时间(0.1秒)(外加200毫秒)
    uint8_t failsafe_stick_threshold;       		// 退出GPS救援程序时，操纵杆偏转百分比
} failsafeConfig_t;
// 声明失控保护配置结构体
PG_DECLARE(failsafeConfig_t, failsafeConfig);

/* ---------------------------失控保护阶段枚举--------------------------- */	
// 
typedef enum {
    FAILSAFE_IDLE = 0,								// 空闲
    FAILSAFE_RX_LOSS_DETECTED,						// RX丢失检测
    FAILSAFE_LANDING,								// 着陆中
    FAILSAFE_LANDED,								// 安全着陆
    FAILSAFE_RX_LOSS_MONITORING,					// RX丢失监测
    FAILSAFE_RX_LOSS_RECOVERED,						// RX丢失恢复
    FAILSAFE_GPS_RESCUE								// GPS救援
} failsafePhase_e;

/* ----------------------------RX链接状态枚举---------------------------- */	
typedef enum {
    FAILSAFE_RXLINK_DOWN = 0,						// RX链接掉线
    FAILSAFE_RXLINK_UP								// RX链接在线
} failsafeRxLinkState_e;

/* ---------------------------失控保护程序枚举--------------------------- */	
typedef enum {
    FAILSAFE_PROCEDURE_AUTO_LANDING = 0,			// 自动着陆
    FAILSAFE_PROCEDURE_DROP_IT,						// 坠落
#ifdef USE_GPS_RESCUE
    FAILSAFE_PROCEDURE_GPS_RESCUE,					// GPS救援
#endif
    FAILSAFE_PROCEDURE_COUNT   						// 必须为最后一个
} failsafeProcedure_e;

extern const char * const failsafeProcedureNames[FAILSAFE_PROCEDURE_COUNT];

/* -------------------------失控保护开关模式枚举------------------------- */	
typedef enum {
    FAILSAFE_SWITCH_MODE_STAGE1 = 0,				// 阶段1
    FAILSAFE_SWITCH_MODE_KILL,						// 终止
    FAILSAFE_SWITCH_MODE_STAGE2						// 阶段2
} failsafeSwitchMode_e;

/* --------------------------失控保护状态结构体-------------------------- */	
typedef struct failsafeState_s {
    int16_t events;
    bool monitoring;
    bool active;
    uint32_t rxDataFailurePeriod;
    uint32_t rxDataRecoveryPeriod;
    uint32_t validRxDataReceivedAt;
    uint32_t validRxDataFailedAt;
    uint32_t throttleLowPeriod;             		// 在此期间油门杆必须低于“min_check”
    uint32_t landingShouldBeFinishedAt;
    uint32_t receivingRxDataPeriod;         		// 有效rxData所需的时间段
    uint32_t receivingRxDataPeriodPreset;   		// 预置有效rxData所需的时间
    failsafePhase_e phase;
    failsafeRxLinkState_e rxLinkState;
} failsafeState_t;

void failsafeInit(void);
void failsafeReset(void);
void failsafeStartMonitoring(void);
void failsafeUpdateState(void);
 bool failsafeIsMonitoring(void);
bool failsafeIsActive(void);
bool failsafeIsReceivingRxData(void);
 void failsafeOnValidDataReceived(void);
void failsafeOnValidDataFailed(void);

