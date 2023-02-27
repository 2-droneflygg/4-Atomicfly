#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS
#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/time.h"

#include "fc/rc_controls.h"

#include "flight/mixer.h"

#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/battery.h"

#include "cms_menu_misc.h"

/**********************************************************************
函数名称：cmsx_menuRcOnEnter
函数功能：进入RC菜单
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_menuRcOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    inhibitSaveMenu();

    return NULL;
}

/**********************************************************************
函数名称：cmsx_menuRcConfirmBack
函数功能：确认退出RC菜单
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_menuRcConfirmBack(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);

    if (self && self->type == OME_Back) {
        return NULL;
    } else {
        return MENU_CHAIN_BACK;
    }
}

// RC菜单管理内容
static const OSD_Entry cmsx_menuRcEntries[] =
{
    { "-- RC PREV --", OME_Label, NULL, NULL, 0},

    { "ROLL",  OME_INT16, NULL, &(OSD_INT16_t){ &rcData[ROLL],     1, 2500, 0 }, DYNAMIC },
    { "PITCH", OME_INT16, NULL, &(OSD_INT16_t){ &rcData[PITCH],    1, 2500, 0 }, DYNAMIC },
    { "THR",   OME_INT16, NULL, &(OSD_INT16_t){ &rcData[THROTTLE], 1, 2500, 0 }, DYNAMIC },
    { "YAW",   OME_INT16, NULL, &(OSD_INT16_t){ &rcData[YAW],      1, 2500, 0 }, DYNAMIC },

    { "AUX1",  OME_INT16, NULL, &(OSD_INT16_t){ &rcData[AUX1],     1, 2500, 0 }, DYNAMIC },
    { "AUX2",  OME_INT16, NULL, &(OSD_INT16_t){ &rcData[AUX2],     1, 2500, 0 }, DYNAMIC },
    { "AUX3",  OME_INT16, NULL, &(OSD_INT16_t){ &rcData[AUX3],     1, 2500, 0 }, DYNAMIC },
    { "AUX4",  OME_INT16, NULL, &(OSD_INT16_t){ &rcData[AUX4],     1, 2500, 0 }, DYNAMIC },

    { "BACK",  OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

// RC菜单配置
CMS_Menu cmsx_menuRcPreview = {
    .onEnter = cmsx_menuRcOnEnter,
    .onExit = cmsx_menuRcConfirmBack,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuRcEntries
};

static uint16_t motorConfig_minthrottle;
static uint8_t motorConfig_digitalIdleOffsetValue;
static uint8_t rxConfig_fpvCamAngleDegrees;

/**********************************************************************
函数名称：cmsx_menuMiscOnEnter
函数功能：进入MISC菜单
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_menuMiscOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);
    motorConfig_minthrottle = motorConfig()->minthrottle;
    motorConfig_digitalIdleOffsetValue = motorConfig()->digitalIdleOffsetValue / 10;
    return NULL;
}

/**********************************************************************
函数名称：cmsx_menuMiscOnExit
函数功能：退出MISC菜单
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_menuMiscOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);
    motorConfigMutable()->minthrottle = motorConfig_minthrottle;
    motorConfigMutable()->digitalIdleOffsetValue = 10 * motorConfig_digitalIdleOffsetValue;
    return NULL;
}

// MISC菜单管理内容
static const OSD_Entry menuMiscEntries[]=
{
    { "-- MISC --", OME_Label, NULL, NULL, 0 },
    { "MIN THR",       OME_UINT16,  NULL,          &(OSD_UINT16_t){ &motorConfig_minthrottle,            1000, 2000, 1 }, REBOOT_REQUIRED },
    { "DIGITAL IDLE",  OME_UINT8,   NULL,          &(OSD_UINT8_t) { &motorConfig_digitalIdleOffsetValue,    0,  200, 1 }, REBOOT_REQUIRED },
    { "RC PREV",       OME_Submenu, cmsMenuChange, &cmsx_menuRcPreview, 0},
    { "BACK", OME_Back, NULL, NULL, 0},
    { NULL, OME_END, NULL, NULL, 0}
};

// MISC菜单配置
CMS_Menu cmsx_menuMisc = {
    .onEnter = cmsx_menuMiscOnEnter,
    .onExit = cmsx_menuMiscOnExit,
    .onDisplayUpdate = NULL,
    .entries = menuMiscEntries
};

#endif // CMS

