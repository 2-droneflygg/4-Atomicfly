#pragma once

#include "drivers/display.h"

#include "common/time.h"

#include "cms/cms_types.h"

// 开机界面OSD帮助字符：进入OSD菜单摇杆值：油门中+偏航左+俯仰上 
#define CMS_STARTUP_HELP_TEXT1 	   "MENU:THR MID"
#define CMS_STARTUP_HELP_TEXT2     "+ YAW LEFT"
#define CMS_STARTUP_HELP_TEXT3     "+ PITCH UP"

// cms菜单退出特殊的指针值
#define CMS_EXIT             (0)
#define CMS_EXIT_SAVE        (1)
#define CMS_EXIT_SAVEREBOOT  (2)
#define CMS_POPUP_SAVE       (3)
#define CMS_POPUP_SAVEREBOOT (4)
#define CMS_POPUP_EXITREBOOT (5)

/* ---------------------------CMS按键枚举------------------------- */	
typedef enum {
    CMS_KEY_NONE,				// 空
    CMS_KEY_UP,					// 上
    CMS_KEY_DOWN,				// 下
    CMS_KEY_LEFT,				// 左
    CMS_KEY_RIGHT,				// 右
    CMS_KEY_ESC,				// 退出
    CMS_KEY_MENU,				// 菜单
    CMS_KEY_SAVEMENU,			// 保存
} cms_key_e;

extern bool cmsInMenu;
extern displayPort_t *pCurrentDisplay;
bool cmsDisplayPortRegister(displayPort_t *pDisplay);
void cmsInit(void);
void cmsHandler(timeUs_t currentTimeUs);
bool cmsDisplayPortSelect(displayPort_t *instance);
void cmsMenuOpen(void);
const void *cmsMenuChange(displayPort_t *pPort, const void *ptr);
const void *cmsMenuExit(displayPort_t *pPort, const void *ptr);
void inhibitSaveMenu(void);
void cmsAddMenuEntry(OSD_Entry *menuEntry, char *text, OSD_MenuElement type, CMSEntryFuncPtr func, void *data, uint8_t flags);

