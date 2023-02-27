/********************************************************************************************************
RC模式：
	通过使用通道范围配置飞行模式。
	使用通道范围来定义遥控器上的开关分配相应的模式。
	当接收机给出的通道值在最小/最大范围内将激活该模式。
********************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "rc_modes.h"

#include "common/bitarray.h"
#include "common/maths.h"
#include "drivers/time.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "config/config.h"
#include "fc/rc_controls.h"

#include "rx/rx.h"

// ---------------------------------------------------------STICKY_MODE_BOOT_DELAY_US
#define STICKY_MODE_BOOT_DELAY_US  5e6
// ---------------------------------------------------------RC模式激活掩码
boxBitmask_t rcModeActivationMask; 			
static boxBitmask_t stickyModesEverDisabled;	
// ---------------------------------------------------------AIRMODE使能状态
static bool airmodeEnabled;						
// ---------------------------------------------------------模式激活相关
static int activeMacCount = 0;					
static uint8_t activeMacArray[MAX_MODE_ACTIVATION_CONDITION_COUNT];

// 重置PG注册数组
PG_REGISTER_ARRAY_WITH_RESET_FN(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions, PG_MODE_ACTIVATION_PROFILE, 2);
// 默认通道范围设置 - 范围计算：900+25*Step
void pgResetFn_modeActivationConditions(modeActivationCondition_t     * mac)
{	
	// ARM - 解锁通道
	mac[0].modeId = BOXARM;
	mac[0].auxChannelIndex = 0;
	mac[0].range.startStep = 32;  // 1700
	mac[0].range.endStep = 48;    // 2100

	// ANGLE - 自稳模式
	mac[1].modeId = BOXANGLE;
	mac[1].auxChannelIndex = 1;
	mac[1].range.startStep = 16;  // 1300
	mac[1].range.endStep = 48;    // 2100

	// GPS RESCUE - GPS救援模式
	mac[6].modeId = BOXGPSRESCUE;
	mac[6].auxChannelIndex = 3;
	mac[6].range.startStep = 32;  // 1700
	mac[6].range.endStep = 48;    // 2100

	// BEEPE - 蜂鸣器
	mac[8].modeId = BOXBEEPERON;
	mac[8].auxChannelIndex = 3;
	mac[8].range.startStep = 0;   // 900
	mac[8].range.endStep = 16;    // 1300

	// OSD - OSD显示控制
	mac[9].modeId = BOXOSD;
	mac[9].auxChannelIndex = 4;
	mac[9].range.startStep = 34;  // 1750
	mac[9].range.endStep = 48;    // 2100

	// AIR_MODE - 空中模式
	mac[12].modeId = BOXAIRMODE;
	mac[12].auxChannelIndex = 2;
	mac[12].range.startStep = 0;   // 900
	mac[12].range.endStep = 16;    // 1300

	// FLIP_OVER_AFTER_CRASH - 反乌龟模式
	mac[14].modeId = BOXFLIPOVERAFTERCRASH;
	mac[14].auxChannelIndex = 4;
	mac[14].range.startStep = 0;   // 900
	mac[14].range.endStep = 16;    // 1300

	// BOXBEEPGPSCOUNT - 蜂鸣器鸣叫GPS数量
	mac[16].modeId = BOXBEEPGPSCOUNT;
	mac[16].auxChannelIndex = 5;
	mac[16].range.startStep = 34; // 1750
	mac[16].range.endStep = 48;   // 2100
}

/**********************************************************************
函数名称：IS_RC_MODE_ACTIVE
函数功能：获取RC模式是否激活
函数形参：boxId
函数返回值：dispatchEnabled
函数描述：None
**********************************************************************/
bool IS_RC_MODE_ACTIVE(boxId_e boxId)
{
	// 获取boxId在掩码中的数组位
    return bitArrayGet(&rcModeActivationMask, boxId);
}

/**********************************************************************
函数名称：rcModeUpdate
函数功能：RC模式更新
函数形参：newState
函数返回值：None
函数描述：None
**********************************************************************/
void rcModeUpdate(boxBitmask_t *newState)
{
    rcModeActivationMask = *newState;
}

/**********************************************************************
函数名称：airmodeIsEnabled
函数功能：获取空中模式使能状态
函数形参：None
函数返回值：空中模式使能状态
函数描述：None
**********************************************************************/
bool airmodeIsEnabled(void) {
    return airmodeEnabled;
}

/**********************************************************************
函数名称：isRangeActive
函数功能：获取范围是否激活
函数形参：通道索引，范围
函数返回值：范围激活状态
函数描述：None
**********************************************************************/
bool isRangeActive(uint8_t auxChannelIndex, const channelRange_t *range) {
	// 检测范围是否可用
    if (!IS_RANGE_USABLE(range)) {
        return false;
    }
	// 约束范围
    const uint16_t channelValue = constrain(rcData[auxChannelIndex + NON_AUX_CHANNEL_COUNT], CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX - 1);
    return (channelValue >= 900 + (range->startStep * 25) &&
            channelValue < 900 + (range->endStep * 25));
}

/**********************************************************************
函数名称：updateMasksForMac
函数功能：更新Mac掩码
函数形参：mac，andMask，newMask，bActive
函数返回值：范围激活状态
函数描述：
 * 以下是每次MAC更新时可能出现的逻辑状态:
 *      AND     NEW
 *      ---     ---
 *       F       F      - 没有先前的和macs评估，没有先前的活动或macs
 *       F       T      - 至少有1个以前的活动或mac(***这个状态是锁存的真***)
 *       T       F      - 所有以前的和macs活动，没有以前的活动或macs
 *       T       T      - 至少有1个以前未激活的和mac，没有以前激活的或mac
**********************************************************************/
void updateMasksForMac(const modeActivationCondition_t *mac, boxBitmask_t *andMask, boxBitmask_t *newMask, bool bActive)
{
    if (bitArrayGet(andMask, mac->modeId) || !bitArrayGet(newMask, mac->modeId)) {
        if (bActive) {
            bitArrayClr(andMask, mac->modeId);
            bitArraySet(newMask, mac->modeId);
        }
    }
}

/**********************************************************************
函数名称：updateMasksForStickyModes
函数功能：更新摇杆模式掩码
函数形参：mac，andMask，newMask
函数返回值：None
函数描述：None
**********************************************************************/
void updateMasksForStickyModes(const modeActivationCondition_t *mac, boxBitmask_t *andMask, boxBitmask_t *newMask)
{
	// 该模式ID已激活
    if (IS_RC_MODE_ACTIVE(mac->modeId)) {
		// 清除模式位
        bitArrayClr(andMask, mac->modeId);
		// 设置模式位
        bitArraySet(newMask, mac->modeId);
    } 
	// 该模式未激活
	else {
        bool bActive = isRangeActive(mac->auxChannelIndex, &mac->range);

        if (bitArrayGet(&stickyModesEverDisabled, mac->modeId)) {
            updateMasksForMac(mac, andMask, newMask, bActive);
        } else {
            if (micros() >= STICKY_MODE_BOOT_DELAY_US && !bActive) {
                bitArraySet(&stickyModesEverDisabled, mac->modeId);
            }
        }
    }
}

/**********************************************************************
函数名称：updateActivatedModes
函数功能：更新激活模式
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void updateActivatedModes(void)
{
    boxBitmask_t newMask, andMask, stickyModes;
    memset(&andMask, 0, sizeof(andMask));
    memset(&newMask, 0, sizeof(newMask));
    memset(&stickyModes, 0, sizeof(stickyModes));

    // 遍历所有激活Mac - 确定哪些条件设置/清除模式
    for (int i = 0; i < activeMacCount; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(activeMacArray[i]);
		// 获取模式位
        if (bitArrayGet(&stickyModes, mac->modeId)) {
			// 更新摇杆模式掩码
            updateMasksForStickyModes(mac, &andMask, &newMask);
        } else if (mac->modeId < CHECKBOX_ITEM_COUNT) {
        	// 获取范围是否激活
            bool bActive = isRangeActive(mac->auxChannelIndex, &mac->range);
			// 更新Mac掩码
            updateMasksForMac(mac, &andMask, &newMask, bActive);
        }
    }
	// 异或数组位 - 同出0异出1
    bitArrayXor(&newMask, sizeof(newMask), &newMask, &andMask);
	// RC模式更新
    rcModeUpdate(&newMask);
	// 更新AIRMODE状态
    airmodeEnabled = featureIsEnabled(FEATURE_AIRMODE) || IS_RC_MODE_ACTIVE(BOXAIRMODE);
}

/**********************************************************************
函数名称：isModeActivationConditionPresent
函数功能：模式激活条件存在状态
函数形参：modeId
函数返回值：状态
函数描述：None
**********************************************************************/
bool isModeActivationConditionPresent(boxId_e modeId)
{
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);

        if (mac->modeId == modeId && IS_RANGE_USABLE(&mac->range)) {
            return true;
        }
    }
    return false;
}


/**********************************************************************
函数名称：removeModeActivationCondition
函数功能：移除模式激活条件
函数形参：modeId
函数返回值：None
函数描述：None
**********************************************************************/
void removeModeActivationCondition(const boxId_e modeId)
{
    unsigned offset = 0;
    for (unsigned i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        modeActivationCondition_t *mac = modeActivationConditionsMutable(i);

        if (mac->modeId == modeId && !offset) {
            offset++;
        }

        if (offset) {
            while (i + offset < MAX_MODE_ACTIVATION_CONDITION_COUNT && modeActivationConditions(i + offset)->modeId == modeId) {
                offset++;
            }

            if (i + offset < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
                memcpy(mac, modeActivationConditions(i + offset), sizeof(modeActivationCondition_t));
            } else {
                memset(mac, 0, sizeof(modeActivationCondition_t));
            }
        }
    }
}

/**********************************************************************
函数名称：isModeActivationConditionConfigured
函数功能：获取模式激活条件配置状态
函数形参：mac，emptyMac
函数返回值：状态
函数描述：None
**********************************************************************/
bool isModeActivationConditionConfigured(const modeActivationCondition_t *mac, const modeActivationCondition_t *emptyMac)
{
    if (memcmp(mac, emptyMac, sizeof(*emptyMac))) {
        return true;
    } else {
        return false;
    }
}

/**********************************************************************
函数名称：analyzeModeActivationConditions
函数功能：分析模式激活条件
函数形参：None
函数返回值：None
函数描述：
	建立modeActivationConditions索引的列表
	然后可以使用这个函数，通过只计算已使用的条件来加速处理。
**********************************************************************/
void analyzeModeActivationConditions(void)
{
    modeActivationCondition_t emptyMac;
    memset(&emptyMac, 0, sizeof(emptyMac));
	// 激活模式数量	
    activeMacCount = 0;
	// 遍历所有可激活的模式
    for (uint8_t i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);
		if (isModeActivationConditionConfigured(mac, &emptyMac)) {
            activeMacArray[activeMacCount++] = i;
        }
    }
}

