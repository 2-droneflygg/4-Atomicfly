// 主菜单结构和支持功能

#include <stdbool.h>
#include <stdio.h>

#include "platform.h"

#ifdef USE_CMS
#include "cms/cms.h"
#include "cms/cms_types.h"
#include "scheduler/scheduler.h"

// Sub menus
#include "cms/cms_menu_imu.h"
#include "cms/cms_menu_failsafe.h"
#include "cms/cms_menu_firmware.h"
#include "cms/cms_menu_misc.h"
#include "cms/cms_menu_osd.h"
#include "cms/cms_menu_power.h"
#include "cms/cms_menu_saveexit.h"

// VTX supplied menus
#include "cms/cms_menu_vtx_common.h"

#include "common/printf.h"

#include "config/config.h"

#include "fc/core.h"
#include "fc/runtime_config.h"

#include "sensors/acceleration.h"

#include "cms_menu_main.h"

#define CALIBRATION_STATUS_MAX_LENGTH 9

#define CALIBRATION_STATUS_REQUIRED "REQUIRED"
#define CALIBRATION_STATUS_ACTIVE   "  ACTIVE"
#define CALIBRATION_STATUS_COMPLETE "COMPLETE"

#if defined(USE_ACC)
static char accCalibrationStatus[CALIBRATION_STATUS_MAX_LENGTH];
#endif

// Features菜单管理内容
static const OSD_Entry menuFeaturesEntries[] =
{
    {"--- FEATURES ---", OME_Label, NULL, NULL, 0},
#if defined(USE_VTX_CONTROL)
#if defined(USE_VTX_SMARTAUDIO) || defined(USE_VTX_TRAMP)
    {"VTX", OME_Funcall, cmsSelectVtx, NULL, 0},
#endif
#endif // VTX_CONTROL
    {"POWER", OME_Submenu, cmsMenuChange, &cmsx_menuPower, 0},
#ifdef USE_CMS_FAILSAFE_MENU
    {"FAILSAFE", OME_Submenu, cmsMenuChange, &cmsx_menuFailsafe, 0},
#endif
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

// Features菜单配置
static CMS_Menu cmsx_menuFeatures = {
    .onEnter = NULL,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = menuFeaturesEntries,
};

/**********************************************************************
函数名称：cmsx_SaveExitMenu
函数功能：保存退出菜单
函数形参：pDisplay，ptr
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_SaveExitMenu(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(ptr);

    cmsMenuChange(pDisplay, getSaveExitMenu());

    return NULL;
}


#define SETUP_POPUP_MAX_ENTRIES 1   // 随着新条目的添加而增加

// 动态菜单管理内容
static OSD_Entry setupPopupMenuEntries[SETUP_POPUP_MAX_ENTRIES + 3];

/**********************************************************************
函数名称：setupPopupMenuBuild
函数功能：动态菜单构建（加速度计未进行校准）
函数形参：None
函数返回值：NULL
函数描述：None 
**********************************************************************/
static bool setupPopupMenuBuild(void)
{
    uint8_t menuIndex = 0;
    updateArmingStatus();

    cmsAddMenuEntry(&setupPopupMenuEntries[menuIndex], "-- SETUP MENU --", OME_Label, NULL, NULL, 0);

    // Add menu entries for uncompleted setup tasks
#if defined(USE_ACC)
    if (sensors(SENSOR_ACC) && (getArmingDisableFlags() & ARMING_DISABLED_ACC_CALIBRATION)) {
        cmsAddMenuEntry(&setupPopupMenuEntries[++menuIndex], "CALIBRATE ACC", OME_Funcall, cmsCalibrateAccMenu, accCalibrationStatus, DYNAMIC);
    }
#endif

    cmsAddMenuEntry(&setupPopupMenuEntries[++menuIndex], "EXIT", OME_Back, NULL, NULL, DYNAMIC);
    cmsAddMenuEntry(&setupPopupMenuEntries[++menuIndex], "NULL", OME_END, NULL, NULL, 0);

    return (menuIndex > 2);  // return true if any setup items were added
}

/**********************************************************************
函数名称：setupPopupMenuBuild
函数功能：校准加速度计弹出菜单显示更新
函数形参：pDisp，selected
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *setupPopupMenuOnDisplayUpdate(displayPort_t *pDisp, const OSD_Entry *selected)
{
    UNUSED(pDisp);
    UNUSED(selected);

#if defined(USE_ACC)
    // 更新ACC校准状态信息
    tfp_sprintf(accCalibrationStatus, accIsCalibrationComplete() ? accHasBeenCalibrated() ? CALIBRATION_STATUS_COMPLETE : CALIBRATION_STATUS_REQUIRED : CALIBRATION_STATUS_ACTIVE);
#endif

    return NULL;
}

// 动态菜单配置
CMS_Menu cmsx_menuSetupPopup = {
    .onEnter = NULL,
    .onExit = NULL,
    .onDisplayUpdate = setupPopupMenuOnDisplayUpdate,
    .entries = setupPopupMenuEntries,
};

/**********************************************************************
函数名称：mainMenuOnEnter
函数功能：进入MAIN菜单
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *mainMenuOnEnter(displayPort_t *pDisp)
{
    if (setupPopupMenuBuild()) {
        // 如果发现设置问题，那么切换到动态构建菜单
        cmsMenuChange(pDisp, &cmsx_menuSetupPopup);
    }
    return NULL;
}

/**********************************************************************
函数名称：cmsx_SystemLoadUpdate
函数功能：更新循环时间 && 系统负载 - 仅在主菜单界面显示
函数形参：pDisp，selected
函数返回值：NULL
函数描述：None 
**********************************************************************/
static char SystemLoadStatus[10] = {0};
static char CycleTimeStatus[10] = {0};
static const void *cmsx_SystemLoadUpdate(displayPort_t *pDisp, const OSD_Entry *selected)
{
    UNUSED(pDisp);
    UNUSED(selected);
	// 获取循环时间 - PID循环
	uint16_t cycle = getTaskDeltaTimeUs(TASK_PID);
	sprintf(CycleTimeStatus,"%d",cycle);
	// 获取系统负载百分比
	uint16_t load = getAverageSystemLoadPercent();
	sprintf(SystemLoadStatus,"%d",load);
    return NULL;
}

// 主菜单管理内容
static const OSD_Entry menuMainEntries[] =
{
    {"-- MAIN --",  OME_Label,    NULL, NULL, 0},
    {"CYCLETIME",   OME_String,   NULL, CycleTimeStatus, DYNAMIC},
    {"SYSTEMLOAD",  OME_String,   NULL, SystemLoadStatus, DYNAMIC},
    {"PROFILE",     OME_Submenu,  cmsMenuChange, &cmsx_menuImu, 0},
    {"FEATURES",    OME_Submenu,  cmsMenuChange, &cmsx_menuFeatures, 0},
#ifdef USE_OSD
    {"OSD",         OME_Submenu,  cmsMenuChange, &cmsx_menuOsd, 0},
#endif
    {"FC&FIRMWARE", OME_Submenu,  cmsMenuChange, &cmsx_menuFirmware, 0},
    {"MISC",        OME_Submenu,  cmsMenuChange, &cmsx_menuMisc, 0},
    {"SAVE/EXIT",   OME_Funcall,  cmsx_SaveExitMenu, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0},
};

// 主菜单配置
CMS_Menu cmsx_menuMain = {
    .onEnter = mainMenuOnEnter,
    .onExit = NULL,
    .onDisplayUpdate = cmsx_SystemLoadUpdate,
    .entries = menuMainEntries,
};
#endif

