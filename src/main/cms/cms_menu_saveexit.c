#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS
#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_saveexit.h"

#include "config/feature.h"

#include "config/config.h"

// 保存退出菜单管理内容
static const OSD_Entry cmsx_menuSaveExitEntries[] =
{
    { "-- SAVE/EXIT --", OME_Label, NULL, NULL, 0},
    { "EXIT",            OME_OSD_Exit, cmsMenuExit,   (void *)CMS_EXIT, 0},
    { "SAVE&EXIT",       OME_OSD_Exit, cmsMenuExit,   (void *)CMS_POPUP_SAVE, 0},
    { "SAVE&REBOOT",     OME_OSD_Exit, cmsMenuExit,   (void *)CMS_POPUP_SAVEREBOOT, 0},
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

// 保存退出菜单配置
static CMS_Menu cmsx_menuSaveExit = {
    .onEnter = NULL,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuSaveExitEntries
};

// 保存退出-重启菜单内容管理
static const OSD_Entry cmsx_menuSaveExitRebootEntries[] =
{
    { "-- SAVE/EXIT (REBOOT REQD)", OME_Label, NULL, NULL, 0},
    { "EXIT&REBOOT", OME_OSD_Exit, cmsMenuExit,   (void *)CMS_POPUP_EXITREBOOT, 0},
    { "SAVE&REBOOT", OME_OSD_Exit, cmsMenuExit,   (void *)CMS_POPUP_SAVEREBOOT, 0},
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

// 保存退出-重启菜单配置
static CMS_Menu cmsx_menuSaveExitReboot = {
    .onEnter = NULL,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuSaveExitRebootEntries
};

/**********************************************************************
函数名称：getSaveExitMenu
函数功能：获取保存退出菜单
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
CMS_Menu *getSaveExitMenu(void)
{
   if (getRebootRequired()) {
        return &cmsx_menuSaveExitReboot;
    } else {
        return &cmsx_menuSaveExit;
    }
}
#endif

