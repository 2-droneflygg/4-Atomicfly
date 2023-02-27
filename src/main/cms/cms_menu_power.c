#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS
#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_power.h"

#include "config/feature.h"

#include "sensors/battery.h"
#include "sensors/current.h"
#include "sensors/voltage.h"

#include "config/config.h"

voltageMeterSource_e batteryConfig_voltageMeterSource;
currentMeterSource_e batteryConfig_currentMeterSource;

uint16_t batteryConfig_vbatmaxcellvoltage;

uint8_t voltageSensorADCConfig_vbatscale;

int16_t currentSensorADCConfig_scale;
int16_t currentSensorADCConfig_offset;

uint16_t cms_vbatwarningcellvoltage;

/**********************************************************************
函数名称：cmsx_Power_onEnter
函数功能：进入Power菜单
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_Power_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    batteryConfig_voltageMeterSource = batteryConfig()->voltageMeterSource;
    batteryConfig_currentMeterSource = batteryConfig()->currentMeterSource;

    batteryConfig_vbatmaxcellvoltage = batteryConfig()->vbatmaxcellvoltage;

    voltageSensorADCConfig_vbatscale = voltageSensorADCConfig(0)->vbatscale;

    currentSensorADCConfig_scale = currentSensorADCConfig()->scale;
    currentSensorADCConfig_offset = currentSensorADCConfig()->offset;

	cms_vbatwarningcellvoltage = batteryConfig()->vbatwarningcellvoltage;
    return NULL;
}

/**********************************************************************
函数名称：cmsx_Power_onExit
函数功能：退出Power菜单
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_Power_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    batteryConfigMutable()->voltageMeterSource = batteryConfig_voltageMeterSource;
    batteryConfigMutable()->currentMeterSource = batteryConfig_currentMeterSource;

    batteryConfigMutable()->vbatmaxcellvoltage = batteryConfig_vbatmaxcellvoltage;

    voltageSensorADCConfigMutable(0)->vbatscale = voltageSensorADCConfig_vbatscale;

    currentSensorADCConfigMutable()->scale = currentSensorADCConfig_scale;
    currentSensorADCConfigMutable()->offset = currentSensorADCConfig_offset;

	batteryConfigMutable()->vbatwarningcellvoltage = cms_vbatwarningcellvoltage;
    return NULL;
}

// 电源菜单管理内容
static const OSD_Entry cmsx_menuPowerEntries[] =
{
    { "-- POWER --", OME_Label, NULL, NULL, 0},

    { "V METER", OME_TAB, NULL, &(OSD_TAB_t){ &batteryConfig_voltageMeterSource, VOLTAGE_METER_COUNT - 1, voltageMeterSourceNames }, REBOOT_REQUIRED },
    { "I METER", OME_TAB, NULL, &(OSD_TAB_t){ &batteryConfig_currentMeterSource, CURRENT_METER_COUNT - 1, currentMeterSourceNames }, REBOOT_REQUIRED },

    { "VBAT CLMAX", OME_UINT16, NULL, &(OSD_UINT16_t) { &batteryConfig_vbatmaxcellvoltage, VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX, 1 }, 0 },

    { "VBAT SCALE", OME_UINT8, NULL, &(OSD_UINT8_t){ &voltageSensorADCConfig_vbatscale, VBAT_SCALE_MIN, VBAT_SCALE_MAX, 1 }, 0 },

    { "IBAT SCALE", OME_INT16, NULL, &(OSD_INT16_t){ &currentSensorADCConfig_scale, -16000, 16000, 5 }, 0 },
    { "IBAT OFFSET", OME_INT16, NULL, &(OSD_INT16_t){ &currentSensorADCConfig_offset, -32000, 32000, 5 }, 0 },

    { "VBAT WARNING", OME_UINT16, NULL, &(OSD_UINT16_t) { &cms_vbatwarningcellvoltage, 350, 370, 1 }, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

// 电源菜单配置
CMS_Menu cmsx_menuPower = {
    .onEnter = cmsx_Power_onEnter,
    .onExit = cmsx_Power_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuPowerEntries
};

#endif

