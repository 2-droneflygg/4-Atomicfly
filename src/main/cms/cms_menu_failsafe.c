#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS_FAILSAFE_MENU
#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_failsafe.h"

#ifdef USE_CMS_GPS_RESCUE_MENU
#include "cms/cms_menu_gps_rescue.h"
#endif

#include "config/feature.h"

#include "config/config.h"

#include "flight/failsafe.h"

#include "rx/rx.h"


uint8_t failsafeConfig_failsafe_procedure;
uint8_t failsafeConfig_failsafe_delay;
uint8_t failsafeConfig_failsafe_off_delay;
uint16_t failsafeConfig_failsafe_throttle;

/**********************************************************************
函数名称：cmsx_Failsafe_onEnter
函数功能：进入Failsafe菜单
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_Failsafe_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);
    failsafeConfig_failsafe_procedure = failsafeConfig()->failsafe_procedure;
    failsafeConfig_failsafe_delay = failsafeConfig()->failsafe_delay;
    failsafeConfig_failsafe_off_delay = failsafeConfig()->failsafe_off_delay;
    failsafeConfig_failsafe_throttle = failsafeConfig()->failsafe_throttle;
    return NULL;
}

/**********************************************************************
函数名称：cmsx_Failsafe_onExit
函数功能：退出Failsafe菜单
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_Failsafe_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);
    failsafeConfigMutable()->failsafe_procedure = failsafeConfig_failsafe_procedure;
    failsafeConfigMutable()->failsafe_delay = failsafeConfig_failsafe_delay;
    failsafeConfigMutable()->failsafe_off_delay = failsafeConfig_failsafe_off_delay;
    failsafeConfigMutable()->failsafe_throttle = failsafeConfig_failsafe_throttle;
    return NULL;
}

// 失控保护菜单管理内容
static const OSD_Entry cmsx_menuFailsafeEntries[] =
{
    { "-- FAILSAFE --", OME_Label, NULL, NULL, 0},
    { "PROCEDURE",        OME_TAB,    NULL, &(OSD_TAB_t)    { &failsafeConfig_failsafe_procedure, FAILSAFE_PROCEDURE_COUNT - 1, failsafeProcedureNames }, REBOOT_REQUIRED },
    { "GUARD TIME",       OME_FLOAT,  NULL, &(OSD_FLOAT_t)  { &failsafeConfig_failsafe_delay, 0, 200, 1, 100 }, REBOOT_REQUIRED },
    { "STAGE 2 DELAY",    OME_FLOAT,  NULL, &(OSD_FLOAT_t)  { &failsafeConfig_failsafe_off_delay, 0, 200, 1, 100 }, REBOOT_REQUIRED },
    { "STAGE 2 THROTTLE", OME_UINT16, NULL, &(OSD_UINT16_t) { &failsafeConfig_failsafe_throttle, PWM_PULSE_MIN, PWM_PULSE_MAX, 1 }, REBOOT_REQUIRED },
#ifdef USE_CMS_GPS_RESCUE_MENU
    { "GPS RESCUE",       OME_Submenu, cmsMenuChange, &cmsx_menuGpsRescue, 0},
#endif
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

// 失控保护菜单管理配置
CMS_Menu cmsx_menuFailsafe = {
    .onEnter = cmsx_Failsafe_onEnter,
    .onExit = cmsx_Failsafe_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuFailsafeEntries
};
#endif

