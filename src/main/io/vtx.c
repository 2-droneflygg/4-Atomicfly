#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_VTX_COMMON)
#include "common/maths.h"
#include "common/time.h"

#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"

#include "config/config.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"

#include "io/vtx_control.h"
#include "io/serial.h"
#include "io/vtx_smartaudio.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "vtx.h"


PG_REGISTER_WITH_RESET_FN(vtxSettingsConfig_t, vtxSettingsConfig, PG_VTX_SETTINGS_CONFIG, 0);
void pgResetFn_vtxSettingsConfig(vtxSettingsConfig_t *vtxSettingsConfig)
{
    vtxSettingsConfig->band = VTX_TABLE_DEFAULT_BAND;
    vtxSettingsConfig->channel = VTX_TABLE_DEFAULT_CHANNEL;
    vtxSettingsConfig->power = VTX_TABLE_DEFAULT_POWER;
	vtxSettingsConfig->freq = VTX_TABLE_DEFAULT_FREQ;
    vtxSettingsConfig->pitModeFreq = VTX_TABLE_DEFAULT_PITMODE_FREQ;
    vtxSettingsConfig->lowPowerDisarm = VTX_LOW_POWER_DISARM_OFF;
}

/**********************************************************************
函数名称：vtxInit
函数功能：vtx初始化
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void vtxInit(void)
{
    bool settingsUpdated = false;                // 环境设置标志位

    vtxDevice_t *vtxDevice = vtxCommonDevice();  // 获取VTX设备

    if (!vtxDevice) {
		// 如果一个设备都没有注册，没有任何表可以参考
		// 在这种情况下，不要操作设置，只需return
        return;
    }

    // 当“频组/通道”指定时，“参数组”中的同步频率
    const uint16_t freq = vtxCommonLookupFrequency(vtxDevice, vtxSettingsConfig()->band, vtxSettingsConfig()->channel);
    if (vtxSettingsConfig()->band && freq != vtxSettingsConfig()->freq) {
        vtxSettingsConfigMutable()->freq = freq;
        settingsUpdated = true;
    }

    if (settingsUpdated) {
		// 保存配置和通知
        saveConfigAndNotify();
    }
}

/**********************************************************************
函数名称：vtxGetSettings
函数功能：获取vtx设置
函数形参：None
函数返回值：settings
函数描述：None
**********************************************************************/
STATIC_UNIT_TESTED vtxSettingsConfig_t vtxGetSettings(void)
{
	// 获取设置
    vtxSettingsConfig_t settings = {
        .band = vtxSettingsConfig()->band,
        .channel = vtxSettingsConfig()->channel,
        .power = vtxSettingsConfig()->power,
        .freq = vtxSettingsConfig()->freq,
        .pitModeFreq = vtxSettingsConfig()->pitModeFreq,
        .lowPowerDisarm = vtxSettingsConfig()->lowPowerDisarm,
    };
		
    if (!ARMING_FLAG(ARMED) && !failsafeIsActive() &&
        (settings.lowPowerDisarm == VTX_LOW_POWER_DISARM_ALWAYS ||
        (settings.lowPowerDisarm == VTX_LOW_POWER_DISARM_UNTIL_FIRST_ARM && !ARMING_FLAG(WAS_EVER_ARMED)))) {
        settings.power = VTX_TABLE_DEFAULT_POWER;
    }
    return settings;
}

/**********************************************************************
函数名称：vtxProcessBandAndChannel
函数功能：vtx频组通道进程
函数形参：vtxDevice
函数返回值：状态
函数描述：None
**********************************************************************/
static bool vtxProcessBandAndChannel(vtxDevice_t *vtxDevice)
{
    if (!ARMING_FLAG(ARMED)) {
        uint8_t vtxBand;
        uint8_t vtxChan;
        if (vtxCommonGetBandAndChannel(vtxDevice, &vtxBand, &vtxChan)) {
            const vtxSettingsConfig_t settings = vtxGetSettings();
            if (vtxBand != settings.band || vtxChan != settings.channel) {
				// vtxCommon设置频组和通道
                vtxCommonSetBandAndChannel(vtxDevice, settings.band, settings.channel);
                return true;
            }
        }
    }
    return false;
}

/**********************************************************************
函数名称：vtxProcessPower
函数功能：vtx功率进程
函数形参：vtxDevice
函数返回值：状态
函数描述：None
**********************************************************************/
static bool vtxProcessPower(vtxDevice_t *vtxDevice)
{
    uint8_t vtxPower;
    if (vtxCommonGetPowerIndex(vtxDevice, &vtxPower)) {
        const vtxSettingsConfig_t settings = vtxGetSettings();
        if (vtxPower != settings.power) {
            vtxCommonSetPowerByIndex(vtxDevice, settings.power);
            return true;
        }
    }
    return false;
}

/**********************************************************************
函数名称：vtxProcessPitMode
函数功能：vtx PIT模式进程
函数形参：vtxDevice
函数返回值：状态
函数描述：None
**********************************************************************/
static bool vtxProcessPitMode(vtxDevice_t *vtxDevice)
{
    static bool prevPmSwitchState = false;

    unsigned vtxStatus;
    if (!ARMING_FLAG(ARMED) && vtxCommonGetStatus(vtxDevice, &vtxStatus)) {
        bool currPmSwitchState = false;

        if (currPmSwitchState != prevPmSwitchState) {
            prevPmSwitchState = currPmSwitchState;

            if (currPmSwitchState) {
                if (!(vtxStatus & VTX_STATUS_PIT_MODE)) {
                    vtxCommonSetPitMode(vtxDevice, true);
                    return true;
                }
            } else {
                if (vtxStatus & VTX_STATUS_PIT_MODE) {
                    vtxCommonSetPitMode(vtxDevice, false);
                    return true;
                }
            }
        }
    }

    return false;
}

/**********************************************************************
函数名称：vtxProcessStateUpdate
函数功能：vtx进程状态更新
函数形参：vtxDevice
函数返回值：状态
函数描述：None
**********************************************************************/
static bool vtxProcessStateUpdate(vtxDevice_t *vtxDevice)
{
    const vtxSettingsConfig_t vtxSettingsState = vtxGetSettings();
    vtxSettingsConfig_t vtxState = vtxSettingsState;

    if (vtxSettingsState.band) {
        vtxCommonGetBandAndChannel(vtxDevice, &vtxState.band, &vtxState.channel);
    }

    vtxCommonGetPowerIndex(vtxDevice, &vtxState.power);

    return (bool)memcmp(&vtxSettingsState, &vtxState, sizeof(vtxSettingsConfig_t));
}

/**********************************************************************
函数名称：vtxUpdate
函数功能：vtx更新
函数形参：currentTimeUs
函数返回值：None
函数描述：
	由调度器以任务形式调用.
**********************************************************************/
void vtxUpdate(timeUs_t currentTimeUs)
{
    static uint8_t currentSchedule = 0;
	// 获取图传设备
    vtxDevice_t *vtxDevice = vtxCommonDevice();  
    if (vtxDevice) {
        const uint8_t startingSchedule = currentSchedule;	// 起始调度
        bool vtxUpdatePending = false; 						// 更新挂起
	
		// 功能调度 - !startingSchedule
        do {
            switch (currentSchedule) {
                case VTX_PARAM_POWER:			// -----------------------------功率
                    vtxUpdatePending = vtxProcessPower(vtxDevice);
                    break;
                case VTX_PARAM_BANDCHAN:		// -----------------------------频点
                    if (vtxGetSettings().band) {
                        vtxUpdatePending = vtxProcessBandAndChannel(vtxDevice);
                    }
                    break;
                case VTX_PARAM_PITMODE:			// -----------------------------PIT模式
                    vtxUpdatePending = vtxProcessPitMode(vtxDevice);
                    break;
                case VTX_PARAM_CONFIRM:			// -----------------------------SET
                    vtxUpdatePending = vtxProcessStateUpdate(vtxDevice);
                    break;
                default:
                    break;
            }
            currentSchedule = (currentSchedule + 1) % VTX_PARAM_COUNT;
        } while (!vtxUpdatePending && currentSchedule != startingSchedule);

		// vtx公共进程 - 未解锁 || !startingSchedule
        if (!ARMING_FLAG(ARMED) || vtxUpdatePending) {
            vtxCommonProcess(vtxDevice, currentTimeUs);
        }
    }
}

#endif

