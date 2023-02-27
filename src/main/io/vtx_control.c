#include <stdbool.h>
#include <stdint.h>
#include <drivers/vtx_table.h>

#include "platform.h"

#if defined(USE_VTX_CONTROL) && defined(USE_VTX_COMMON)
#include "common/maths.h"

#include "config/config_eeprom.h"

#include "drivers/light_led.h"
#include "drivers/time.h"
#include "drivers/vtx_common.h"

#include "config/config.h"
#include "fc/runtime_config.h"

#include "io/vtx.h"
#include "io/vtx_control.h"

#include "osd/osd.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"


PG_REGISTER_WITH_RESET_TEMPLATE(vtxConfig_t, vtxConfig, PG_VTX_CONFIG, 1);
PG_RESET_TEMPLATE(vtxConfig_t, vtxConfig,
    .halfDuplex = true
);

static uint8_t locked = 0;

/**********************************************************************
函数名称：vtxUpdateBandAndChannel
函数功能：vtx更新频组和通道
函数形参：bandStep，channelStep
函数返回值：None
函数描述：None
**********************************************************************/
static void vtxUpdateBandAndChannel(uint8_t bandStep, uint8_t channelStep)
{
    if (ARMING_FLAG(ARMED)) {
        locked = 1;
    }

    if (!locked && vtxCommonDevice()) {
        vtxSettingsConfigMutable()->band += bandStep;
        vtxSettingsConfigMutable()->channel += channelStep;
    }
}

/**********************************************************************
函数名称：vtxIncrementBand
函数功能：vtx增加频组
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void vtxIncrementBand(void)
{
    vtxUpdateBandAndChannel(+1, 0);
}

/**********************************************************************
函数名称：vtxDecrementBand
函数功能：vtx减少频组
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void vtxDecrementBand(void)
{
    vtxUpdateBandAndChannel(-1, 0);
}

/**********************************************************************
函数名称：vtxIncrementChannel
函数功能：vtx增加通道
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void vtxIncrementChannel(void)
{
    vtxUpdateBandAndChannel(0, +1);
}

/**********************************************************************
函数名称：vtxDecrementChannel
函数功能：vtx减少通道
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void vtxDecrementChannel(void)
{
    vtxUpdateBandAndChannel(0, -1);
}

/**********************************************************************
函数名称：vtxUpdateActivatedChannel
函数功能：vtx更新激活通道
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void vtxUpdateActivatedChannel(void)
{
	// 如果解锁则禁用通道设置
    if (ARMING_FLAG(ARMED)) {
        locked = 1;
    }
	// 获取图传设备
    if (vtxCommonDevice()) {
        static uint8_t lastIndex = -1;
	
        for (uint8_t index = 0; index < MAX_CHANNEL_ACTIVATION_CONDITION_COUNT; index++) {
            const vtxChannelActivationCondition_t *vtxChannelActivationCondition = &vtxConfig()->vtxChannelActivationConditions[index];

            if (isRangeActive(vtxChannelActivationCondition->auxChannelIndex, &vtxChannelActivationCondition->range)
                && index != lastIndex) {
                lastIndex = index;
				// 只有在未解锁时才可以使用
                if (!locked) {
                    if (vtxChannelActivationCondition->band > 0) {
                        vtxSettingsConfigMutable()->band = vtxChannelActivationCondition->band;
                    }
                    if (vtxChannelActivationCondition->channel > 0) {
                        vtxSettingsConfigMutable()->channel = vtxChannelActivationCondition->channel;
                    }
                }

                if (vtxChannelActivationCondition->power > 0) {
                    vtxSettingsConfigMutable()->power = vtxChannelActivationCondition->power;
                }
                break;
            }
        }
    }
}

/**********************************************************************
函数名称：vtxCycleBandOrChannel
函数功能：vtx循环频组或通道
函数形参：bandStep，channelStep
函数返回值：None
函数描述：None
**********************************************************************/
void vtxCycleBandOrChannel(const uint8_t bandStep, const uint8_t channelStep)
{
    const vtxDevice_t *vtxDevice = vtxCommonDevice();
    if (vtxDevice) {
        uint8_t band = 0, channel = 0;

        const bool haveAllNeededInfo = vtxCommonGetBandAndChannel(vtxDevice, &band, &channel);
        if (!haveAllNeededInfo) {
            return;
        }

        int newChannel = channel + channelStep;
        if (newChannel > vtxTableChannelCount) {
            newChannel = 1;
        } else if (newChannel < 1) {
            newChannel = vtxTableChannelCount;
        }

        int newBand = band + bandStep;
        if (newBand > vtxTableBandCount) {
            newBand = 1;
        } else if (newBand < 1) {
            newBand = vtxTableBandCount;
        }

        vtxSettingsConfigMutable()->band = newBand;
        vtxSettingsConfigMutable()->channel = newChannel;
    }
}

/**********************************************************************
函数名称：vtxCyclePower
函数功能：vtx循环功率
函数形参：powerStep
函数返回值：None
函数描述：None
**********************************************************************/
void vtxCyclePower(const uint8_t powerStep)
{
    const vtxDevice_t *vtxDevice = vtxCommonDevice();
    if (vtxDevice) {
        uint8_t power = 0;
        const bool haveAllNeededInfo = vtxCommonGetPowerIndex(vtxDevice, &power);
        if (!haveAllNeededInfo) {
            return;
        }

        int newPower = power + powerStep;
        if (newPower >= vtxTablePowerLevels) {
            newPower = 1;
        } else if (newPower < 0) {
            newPower = vtxTablePowerLevels;
        }

        vtxSettingsConfigMutable()->power = newPower;
    }
}
#endif

