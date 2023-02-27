#pragma once

#include <stdbool.h>

#include "pg/pg.h"

#define BOXID_NONE 255

// ---------------------------------------------------------相关宏定义
// 最大模式激活条件数量
#define MAX_MODE_ACTIVATION_CONDITION_COUNT 20	 
// 通道范围最小值
#define CHANNEL_RANGE_MIN 900					 
// 通道范围最大值
#define CHANNEL_RANGE_MAX 2100					 	
// 模式步转通道值
#define MODE_STEP_TO_CHANNEL_VALUE(step) (CHANNEL_RANGE_MIN + 25 * step)

/* ---------------------------BOX_ID枚举--------------------------- */	
typedef enum {
    // 解锁标志位
    BOXARM = 0,									 	// 解锁
    // 飞行模式-当添加新的飞行模式时，src/main/fc/rc_mode.c中的“modeActivationConditions”的参数组版本必须增加，以确保RC模式配置被重置
    BOXANGLE = 1,									// 自稳模式
    BOXFAILSAFE = 2,							 	// 失控保护模式
    BOXGPSRESCUE = 3,							 	// GPS救援模式
    BOXID_FLIGHTMODE_LAST = BOXGPSRESCUE,
    BOXBEEPERON = 4,						     	// 蜂鸣器
    BOXOSD = 5,								 		// OSD显示控制
    BOXAIRMODE = 6,							 		// 空中模式
    BOXFLIPOVERAFTERCRASH = 7,					 	// 反乌龟模式
    BOXBEEPGPSCOUNT = 8,						 	// 蜂鸣器鸣叫GPS数量
    CHECKBOX_ITEM_COUNT
} boxId_e;
/* ---------------------------BOX位掩码---------------------------- */	
// 使用uint32_t数组成员来表示boxId掩码
typedef struct boxBitmask_s { uint32_t bits[(CHECKBOX_ITEM_COUNT + 31) / 32]; } boxBitmask_t;

/* -------------------------通道范围结构体------------------------- */	
// 通道值范围：900 ~ 2100
// 900到2100之间有48步，步骤间隔 25
// 0对应900
// 48对应2100
typedef struct channelRange_s {
    uint8_t startStep;							 	// 起始步
    uint8_t endStep;							 	// 结束步
} channelRange_t;
// 检测范围是否可用
#define IS_RANGE_USABLE(range) ((range)->startStep < (range)->endStep)  

/* -----------------------模式激活条件结构体------------------------*/
typedef struct modeActivationCondition_s {
    boxId_e modeId;             				 	// 模式ID
    uint8_t auxChannelIndex;    				 	// 通道索引
    channelRange_t range;						 	// 范围
} modeActivationCondition_t;
// 通道范围设置结构体数组
PG_DECLARE_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions);

/* -----------------------模式激活文件结构体------------------------*/
typedef struct modeActivationProfile_s {
    modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];
} modeActivationProfile_t;

bool IS_RC_MODE_ACTIVE(boxId_e boxId);
void rcModeUpdate(boxBitmask_t *newState);
bool airmodeIsEnabled(void);
bool isRangeActive(uint8_t auxChannelIndex, const channelRange_t *range);
void updateActivatedModes(void);
bool isModeActivationConditionPresent(boxId_e modeId);
void removeModeActivationCondition(boxId_e modeId);
bool isModeActivationConditionConfigured(const modeActivationCondition_t *mac, const modeActivationCondition_t *emptyMac);
void analyzeModeActivationConditions(void);

