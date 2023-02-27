#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "pg/pg.h"

#define MAX_NAME_LENGTH 16u

/* ---------------------------配置状态枚举----------------------------- */
typedef enum {
    CONFIGURATION_STATE_DEFAULTS_BARE = 0,
    CONFIGURATION_STATE_DEFAULTS_CUSTOM,
    CONFIGURATION_STATE_CONFIGURED,
} configurationState_e;

/* ------------------------调度程序优化速率枚举------------------------ */
typedef enum {
    SCHEDULER_OPTIMIZE_RATE_OFF = 0,
    SCHEDULER_OPTIMIZE_RATE_ON,
    SCHEDULER_OPTIMIZE_RATE_AUTO,
} schedulerOptimizeRate_e;
	
/* --------------------------系统配置结构体---------------------------- */	
typedef struct systemConfig_s {
    uint8_t pidProfileIndex;        	// PID配置文件索引
    uint8_t activeRateProfile;			// 速率配置文件
    uint8_t cpu_overclock;			    // MCU超频
    uint8_t powerOnArmingGraceTime; 	// 解锁宽限时间(seconds)
    char boardIdentifier[sizeof(TARGET_BOARD_IDENTIFIER) + 1]; // 目标飞控板标识
    uint8_t hseMhz;                 	// 外部晶振频率
    uint8_t configurationState;     	// 配置状态
    uint8_t schedulerOptimizeRate;		// 调度程序优化速率
} systemConfig_t;
// 声明系统配置结构体
PG_DECLARE(systemConfig_t, systemConfig);

struct pidProfile_s;
extern struct pidProfile_s *currentPidProfile;
void initEEPROM(void);
bool resetEEPROM(bool useCustomDefaults);
bool readEEPROM(void);
void writeEEPROM(void);
void writeUnmodifiedConfigToEEPROM(void);
void ensureEEPROMStructureIsValid(void);
void saveConfigAndNotify(void);
void validateAndFixGyroConfig(void);
uint8_t getCurrentPidProfileIndex(void);
void changePidProfile(uint8_t pidProfileIndex);
void changePidProfileFromCellCount(uint8_t cellCount);
void resetPidProfile(struct pidProfile_s *profile);
uint8_t getCurrentControlRateProfileIndex(void);
void changeControlRateProfile(uint8_t profileIndex);
void resetConfig(void);
void setRebootRequired(void);
bool getRebootRequired(void);

