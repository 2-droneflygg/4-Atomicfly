/*********************************************************************************
 提供一系列OSD菜单操作API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS
#include "build/build_config.h"
#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_menu_main.h"
#include "cms/cms_menu_saveexit.h"
#include "cms/cms_types.h"

#include "common/maths.h"
#include "common/typeconversion.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/motor.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/rc_modes.h"

#include "flight/mixer.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"
#include "osd/osd.h"

#include "rx/rx.h"

/* ------------------------CMS上下文结构体---------------------- */	
// 通俗的理解，上下文（context），也就是执行任务所需要的相关信息
// 这个任务可以是一段代码，一个线程，一个进程，一个函数
// 当这个“任务”，相关信息需要保存下来，就可以使用Context来记录了
typedef struct cmsCtx_s {
    const CMS_Menu *menu;         						// 此上下文的菜单
    uint8_t page;                 						// 菜单中的页面（翻页操作）
    int8_t cursorRow;             						// 光标行
} cmsCtx_t;

// ---------------------------------------------------------菜单栈
static uint8_t menuStackIdx = 0;						// 菜单栈索引
#define CMS_MENU_STACK_LIMIT 10							// CMS菜单堆栈限制 - 最大支持多少级子菜单
static cmsCtx_t menuStack[CMS_MENU_STACK_LIMIT];		// 菜单栈 - 暂存菜单页（如退出当前菜单页使用）
static cmsCtx_t currentCtx;								// 当前菜单上下文

// ---------------------------------------------------------菜单页面
static int8_t pageCount;          						// 当前菜单中的页数
static const OSD_Entry *pageTop;  						// 当前页面的第一个条目
static uint8_t pageMaxRow;        						// 当前页的最大行数

// ---------------------------------------------------------菜单设备[显示端口]
#ifndef CMS_MAX_DEVICE
#define CMS_MAX_DEVICE    4								// 显示最大显示设备数
#endif
displayPort_t *pCurrentDisplay;							// 当前显示端口
static displayPort_t *cmsDisplayPorts[CMS_MAX_DEVICE];  // CMS显示端口信息
static unsigned cmsDeviceCount;							// CMS设备数量
static int cmsCurrentDevice = -1;						// CMS当前设备

// ---------------------------------------------------------菜单光标
#ifdef USE_OSD
static unsigned int osdProfileCursor = 1;				// OSD配置文件光标
#endif

// ---------------------------------------------------------菜单行列数
// MAX7456（PAL）
// 30行 x 16行
// MAX7456 (NTSC)
// 30行 x 13行
#define CMS_MAX_ROWS     16								// 定义最大行数
#define NORMAL_SCREEN_MIN_COLS 18       				// 定义最小列数
static uint8_t leftMenuColumn;							// 菜单列最左
static uint8_t rightMenuColumn;							// 菜单列最右
static uint8_t linesPerMenuItem;						// 每个菜单项行数
static uint8_t maxMenuItems;							// 最大菜单项
static bool    smallScreen;								// 小屏幕

// ---------------------------------------------------------OSD元素编辑状态
static bool osdElementEditing = false;  				// OSD元素是否正在编辑

// --------------------------------------------------------菜单保存
static bool saveMenuInhibited = false;  				// 禁止保存菜单状态

// ---------------------------------------------------------OSD菜单进入状态
bool cmsInMenu = false;									// CMS是否在菜单界面

// ---------------------------------------------------------菜单链回退
int menuChainBack;

// ---------------------------------------------------------判断摇杆操作宏
#define IS_HI(X)  (rcData[X] > 1750)
#define IS_LO(X)  (rcData[X] < 1250)
#define IS_MID(X) (rcData[X] > 1250 && rcData[X] < 1750)

// ---------------------------------------------------------按键延时消抖时间 - msec
#define BUTTON_TIME   250 						
#define BUTTON_PAUSE  500 		

// ---------------------------------------------------------菜单轮询时间
#define CMS_POLL_INTERVAL_US   100000   				// 轮询间隔动态值(微秒)


//-------------------------------------------------------------------------------------菜单显示端口相关API

/**********************************************************************
函数名称：cmsDisplayPortRegister
函数功能：CMS显示端口注册
函数形参：pDisplay
函数返回值：状态 
函数描述：
	注册到CMS显示端口信息。
**********************************************************************/
bool cmsDisplayPortRegister(displayPort_t *pDisplay)
{
	// 判断是否超过最大设备数
    if (cmsDeviceCount >= CMS_MAX_DEVICE) {
        return false;
    }
	// 注册到CMS显示端口信息
    cmsDisplayPorts[cmsDeviceCount++] = pDisplay;
    return true;
}

/**********************************************************************
函数名称：cmsDisplayPortSelectCurrent
函数功能：CMS选择当前显示端口
函数形参：None 
函数返回值：cmsDisplayPorts
函数描述：None 
**********************************************************************/
static displayPort_t *cmsDisplayPortSelectCurrent(void)
{
	// 判断有无菜单显示设备
    if (cmsDeviceCount == 0) {
        return NULL;
    }
	// 设备索引最小为 0
    if (cmsCurrentDevice < 0) {
        cmsCurrentDevice = 0;
    }
	// 返回当前显示端口
    return cmsDisplayPorts[cmsCurrentDevice];
}

/**********************************************************************
函数名称：cmsDisplayPortSelectNext
函数功能：CMS选择下一个显示端口
函数形参：None 
函数返回值：cmsDisplayPorts
函数描述：None 
**********************************************************************/
static displayPort_t *cmsDisplayPortSelectNext(void)
{
	// 判断有无菜单显示设备
    if (cmsDeviceCount == 0) {
        return NULL;
    }
	// 递增
    cmsCurrentDevice = (cmsCurrentDevice + 1) % cmsDeviceCount; // -1 Okay
	// 返回下一个显示端口信息
    return cmsDisplayPorts[cmsCurrentDevice];
}

//-------------------------------------------------------------------------------------菜单行数相关API

/**********************************************************************
函数名称：cmsUpdateMaxRow
函数功能：更新CMS菜单最大行数
函数形参：instance
函数返回值：None 
函数描述：None 
**********************************************************************/
static void cmsUpdateMaxRow(displayPort_t *instance)
{
    UNUSED(instance);
    pageMaxRow = 0;
	// 轮询菜单项条目
    for (const OSD_Entry *ptr = pageTop; ptr->type != OME_END; ptr++) {
        pageMaxRow++;
    }
	// 最大菜单项限制
    if (pageMaxRow > maxMenuItems) {
        pageMaxRow = maxMenuItems;
    }
	// 最大行限制
    if (pageMaxRow > CMS_MAX_ROWS) {
        pageMaxRow = CMS_MAX_ROWS;
    }
    pageMaxRow--;
}

//-------------------------------------------------------------------------------------菜单光标相关API

/**********************************************************************
函数名称：cmsCursorAbsolute
函数功能：CMS绝对光标
函数形参：instance
函数返回值：绝对光标
函数描述：None 
**********************************************************************/
static uint8_t cmsCursorAbsolute(displayPort_t *instance)
{
    UNUSED(instance);
    return currentCtx.cursorRow + currentCtx.page * maxMenuItems;
}

//-------------------------------------------------------------------------------------菜单页操作相关API

/**********************************************************************
函数名称：cmsPageSelect
函数功能：CMS选择页
函数形参：instance，newpage
函数返回值：None 
函数描述：None 
**********************************************************************/
// 运行时入口标志
uint8_t runtimeEntryFlags[CMS_MAX_ROWS] = { 0 };		
static void cmsPageSelect(displayPort_t *instance, int8_t newpage)
{
    currentCtx.page = (newpage + pageCount) % pageCount;
    pageTop = &currentCtx.menu->entries[currentCtx.page * maxMenuItems];
	// 更新CMS菜单最大行数
    cmsUpdateMaxRow(instance);

    const OSD_Entry *p;
    int i;
    for (p = pageTop, i = 0; (p <= pageTop + pageMaxRow); p++, i++) {
        runtimeEntryFlags[i] = p->flags;
    }
	// 清屏
    displayClearScreen(instance);
}

/**********************************************************************
函数名称：cmsPageNext
函数功能：CMS下一页
函数形参：instance
函数返回值：None 
函数描述：None 
**********************************************************************/
static void cmsPageNext(displayPort_t *instance)
{
    cmsPageSelect(instance, currentCtx.page + 1);
}

/**********************************************************************
函数名称：cmsPagePrev
函数功能：CMS上一页
函数形参：instance
函数返回值：None 
函数描述：None 
**********************************************************************/
static void cmsPagePrev(displayPort_t *instance)
{
    cmsPageSelect(instance, currentCtx.page - 1);
}

/**********************************************************************
函数名称：cmsMenuCountPage
函数功能：cms菜单页面计数
函数形参：pDisplay
函数返回值：None 
函数描述：None 
**********************************************************************/
static void cmsMenuCountPage(displayPort_t *pDisplay)
{
    UNUSED(pDisplay);
    const OSD_Entry *p;
    for (p = currentCtx.menu->entries; p->type != OME_END; p++);
    	pageCount = (p - currentCtx.menu->entries - 1) / maxMenuItems + 1;
}

//-------------------------------------------------------------------------------------菜单格式化相关API

/**********************************************************************
函数名称：cmsFormatFloat
函数功能：CMS格式化浮点
函数形参：value，floatString
函数返回值：None 
函数描述：None 
**********************************************************************/
static void cmsFormatFloat(int32_t value, char *floatString)
{
    uint8_t k;
    // np. 3450
    itoa(100000 + value, floatString, 10); // 从整数值的abs创建字符串
    // 103450
    floatString[0] = floatString[1];
    floatString[1] = floatString[2];
    floatString[2] = '.';
    // 03.450
    // usuwam koncowe zera i kropke
    // 保持小数点第1位
    for (k = 5; k > 3; k--) {
        if (floatString[k] == '0' || floatString[k] == '.') {
            floatString[k] = 0;
        } else {
            break;
        }
    }
    // oraz zero wiodonce
    if (floatString[0] == '0') {
        floatString[0] = ' ';
    }
}

//-------------------------------------------------------------------------------------菜单绘制相关API

/**********************************************************************
函数名称：cmsPadLeftToSize
函数功能：缓冲区向左填充空字符，即向右对齐
函数形参：buf，size
函数返回值：None 
函数描述：None 
**********************************************************************/
static void cmsPadLeftToSize(char *buf, int size)
{
    int i,j;
	// 计算缓存大小
    int len = strlen(buf);
	// 数据移动
    for (i = size - 1, j = size - len; i - j >= 0; i--) {
    	buf[i] = buf[i - j];
    }
	// 填充空字符
    for ( ; i >= 0 ; i--) {
    	buf[i] = ' ';
    }
	// 填充字符串终止
    buf[size] = 0;
}

/**********************************************************************
函数名称：cmsPadToSize
函数功能：cms字符填充大小
函数形参：buf，size
函数返回值：None 
函数描述：None 
**********************************************************************/
static void cmsPadToSize(char *buf, int size)
{
    // 确保字符串终止
    buf[size] = 0x00,
    cmsPadLeftToSize(buf, size);
}

/**********************************************************************
函数名称：cmsDrawMenuItemValue
函数功能：绘制菜单项的值
函数形参：显示端口，缓冲区，行，最大大小
函数返回值：0
函数描述：None 
**********************************************************************/
static int cmsDrawMenuItemValue(displayPort_t *pDisplay, char *buff, uint8_t row, uint8_t maxSize)
{
    int colpos;
    int cnt;
	// 填充空字符对齐
    cmsPadToSize(buff, maxSize);
	// 计算列
    colpos = rightMenuColumn - maxSize;
    cnt = displayWrite(pDisplay, colpos, row, DISPLAYPORT_ATTR_NONE, buff);
    return cnt;
}

/**********************************************************************
函数名称：cmsDrawMenuEntry
函数功能：cms绘制菜单项
函数形参：pDisplay，p，row，selectedRow，flags
函数返回值：None 
函数描述：None 
**********************************************************************/
static int cmsDrawMenuEntry(displayPort_t *pDisplay, const OSD_Entry *p, uint8_t row, bool selectedRow, uint8_t *flags)
{
    #define CMS_DRAW_BUFFER_LEN 12
    #define CMS_NUM_FIELD_LEN 5
    #define CMS_CURSOR_BLINK_DELAY_MS 500
	// 为空结束符留出空间
    char buff[CMS_DRAW_BUFFER_LEN +1];
    int cnt = 0;
#ifndef USE_OSD
    UNUSED(selectedRow);
#endif
    if (smallScreen) {
        row++;
    }
    switch (p->type) {
	    case OME_String:
	        if (IS_PRINTVALUE(*flags) && p->data) {
	            strncpy(buff, p->data, CMS_DRAW_BUFFER_LEN);
	            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_DRAW_BUFFER_LEN);
	            CLR_PRINTVALUE(*flags);
	        }
	        break;
	    case OME_Submenu:
	    case OME_Funcall:
	        if (IS_PRINTVALUE(*flags)) {
	            buff[0]= 0x0;

	            if (p->type == OME_Submenu && p->func && *flags & OPTSTRING) {
	                // 特殊情况下的子菜单项可选的值显示
	                const char *str = p->func(pDisplay, p->data);
	                strncpy(buff, str, CMS_DRAW_BUFFER_LEN);
	            } else if (p->type == OME_Funcall && p->data) {
	                strncpy(buff, p->data, CMS_DRAW_BUFFER_LEN);
	            }
	            strncat(buff, ">", CMS_DRAW_BUFFER_LEN);

	            row = smallScreen ? row - 1 : row;
	            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, strlen(buff));
	            CLR_PRINTVALUE(*flags);
	        }
	        break;
	    case OME_Bool:
	        if (IS_PRINTVALUE(*flags) && p->data) {
	            if (*((uint8_t *)(p->data))) {
	              strcpy(buff, "YES");
	            } else {
	              strcpy(buff, "NO ");
	            }

	            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, 3);
	            CLR_PRINTVALUE(*flags);
	        }
	        break;
	    case OME_TAB:
	        if (IS_PRINTVALUE(*flags)) {
	            OSD_TAB_t *ptr = p->data;
	            char * str = (char *)ptr->names[*ptr->val];
	            strncpy(buff, str, CMS_DRAW_BUFFER_LEN);
	            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_DRAW_BUFFER_LEN);
	            CLR_PRINTVALUE(*flags);
	        }
	        break;
#ifdef USE_OSD
	    case OME_VISIBLE:
	        if (IS_PRINTVALUE(*flags) && p->data) {
	            uint16_t *val = (uint16_t *)p->data;
	            bool cursorBlink = millis() % (2 * CMS_CURSOR_BLINK_DELAY_MS) < CMS_CURSOR_BLINK_DELAY_MS;
	            for (unsigned x = 1; x < OSD_PROFILE_COUNT + 1; x++) {
	                if (VISIBLE_IN_OSD_PROFILE(*val, x)) {
	                    if (osdElementEditing && cursorBlink && selectedRow && (x == osdProfileCursor)) {
	                        strcpy(buff + x - 1, " ");
	                    } else {
	                        strcpy(buff + x - 1, "X");
	                    }
	                } else {
	                    if (osdElementEditing && cursorBlink && selectedRow && (x == osdProfileCursor)) {
	                        strcpy(buff + x - 1, " ");
	                    } else {
	                        strcpy(buff + x - 1, "-");
	                    }
	                }
	            }
	            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, 3);
	            CLR_PRINTVALUE(*flags);
	        }
	        break;
#endif
	    case OME_UINT8:
	        if (IS_PRINTVALUE(*flags) && p->data) {
	            OSD_UINT8_t *ptr = p->data;
	            itoa(*ptr->val, buff, 10);
	            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_NUM_FIELD_LEN);
	            CLR_PRINTVALUE(*flags);
	        }
	        break;
	    case OME_INT8:
	        if (IS_PRINTVALUE(*flags) && p->data) {
	            OSD_INT8_t *ptr = p->data;
	            itoa(*ptr->val, buff, 10);
	            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_NUM_FIELD_LEN);
	            CLR_PRINTVALUE(*flags);
	        }
	        break;
	    case OME_UINT16:
	        if (IS_PRINTVALUE(*flags) && p->data) {
	            OSD_UINT16_t *ptr = p->data;
	            itoa(*ptr->val, buff, 10);
	            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_NUM_FIELD_LEN);
	            CLR_PRINTVALUE(*flags);
	        }
	        break;
	    case OME_INT16:
	        if (IS_PRINTVALUE(*flags) && p->data) {
	            OSD_INT16_t *ptr = p->data;
	            itoa(*ptr->val, buff, 10);
	            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_NUM_FIELD_LEN);
	            CLR_PRINTVALUE(*flags);
	        }
	        break;
	    case OME_FLOAT:
	        if (IS_PRINTVALUE(*flags) && p->data) {
	            OSD_FLOAT_t *ptr = p->data;
	            cmsFormatFloat(*ptr->val * ptr->multipler, buff);
	            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_NUM_FIELD_LEN);
	            CLR_PRINTVALUE(*flags);
	        }
	        break;
	    case OME_Label:
	        if (IS_PRINTVALUE(*flags) && p->data) {
	            // 带有可选字符串的标签，紧跟着文本
	            cnt = displayWrite(pDisplay, leftMenuColumn + 1 + (uint8_t)strlen(p->text), row, DISPLAYPORT_ATTR_NONE, p->data);
	            CLR_PRINTVALUE(*flags);
	        }
	        break;
	    case OME_OSD_Exit:
	    case OME_END:
	    case OME_Back:
	        break;
	    default:
	        break;
    }

    return cnt;
}

/**********************************************************************
函数名称：cmsMenuCountPage
函数功能：cms菜单回退
函数形参：pDisplay
函数返回值：None 
函数描述：None 
**********************************************************************/
STATIC_UNIT_TESTED const void *cmsMenuBack(displayPort_t *pDisplay)
{
    // 让onExit函数决定是否允许退出
    if (currentCtx.menu->onExit) {
        const void *result = currentCtx.menu->onExit(pDisplay, pageTop + currentCtx.cursorRow);
        if (result == MENU_CHAIN_BACK) {
            return result;
        }
    }
    saveMenuInhibited = false;
    if (!menuStackIdx) {
        return NULL;
    }
    currentCtx = menuStack[--menuStackIdx];
    cmsMenuCountPage(pDisplay);
    cmsPageSelect(pDisplay, currentCtx.page);
    return NULL;
}

/**********************************************************************
函数名称：rowIsSkippable
函数功能：跳过只读条目
函数形参：row
函数返回值：状态
函数描述：
	跳过标签、字符串和动态只读条目。
**********************************************************************/
static bool rowIsSkippable(const OSD_Entry *row)
{
    if (row->type == OME_Label) {
        return true;
    }
    if (row->type == OME_String) {
        return true;
    }
    if ((row->type == OME_UINT16 || row->type == OME_INT16) && row->flags == DYNAMIC) {
        return true;
    }
    return false;
}

/**********************************************************************
函数名称：cmsDrawMenu
函数功能：cms绘制菜单
函数形参：显示设备信息，当前时间节拍
函数返回值：None 
函数描述：None 
**********************************************************************/
static void cmsDrawMenu(displayPort_t *pDisplay, uint32_t currentTimeUs)
{
	// 检查合法性
    if (!pageTop || !cmsInMenu) {
        return;
    }
    uint8_t i;
    const OSD_Entry *p;
	// 计算首行
    uint8_t top = smallScreen ? 1 : (pDisplay->rows - pageMaxRow)/2;

    // 轮询绘制标志 - 用于动态更新菜单条目
    bool drawPolled = false;
	// 上一个轮询时间 - 用于动态更新菜单条目
    static uint32_t lastPolledUs = 0;

	// 到了动态条目更新绘制的时间
    if (currentTimeUs > lastPolledUs + CMS_POLL_INTERVAL_US) {
        drawPolled = true;
        lastPolledUs = currentTimeUs;
    }

	// UINT32_MAX
    uint32_t room = displayTxBytesFree(pDisplay);

	// 清除
    if (pDisplay->cleared) {
		// 遍历所有行
        for (p = pageTop, i= 0; (p <= pageTop + pageMaxRow); p++, i++) {
            SET_PRINTLABEL(runtimeEntryFlags[i]);
            SET_PRINTVALUE(runtimeEntryFlags[i]);
        }
        pDisplay->cleared = false;
    } 
	// 更新动态绘制条目
	else if (drawPolled) {
        for (p = pageTop, i = 0; (p <= pageTop + pageMaxRow); p++, i++) {
			// 判断是否需要动态更新
            if (IS_DYNAMIC(p))
				// 设置值需要重新绘制
                SET_PRINTVALUE(runtimeEntryFlags[i]);
        }
    }

    // 光标操作 - 跳过标签、字符串和动态只读条目
    while (rowIsSkippable(pageTop + currentCtx.cursorRow)) { 		
        currentCtx.cursorRow++;
    }

	// 清除旧光标位置
    if (pDisplay->cursorRow >= 0 && currentCtx.cursorRow != pDisplay->cursorRow) {
        room -= displayWrite(pDisplay, leftMenuColumn, top + pDisplay->cursorRow * linesPerMenuItem, DISPLAYPORT_ATTR_NONE, " ");
    }
    if (room < 30) {
        return;
    }

	// 绘制新光标位置
    if (pDisplay->cursorRow != currentCtx.cursorRow) {
        room -= displayWrite(pDisplay, leftMenuColumn, top + currentCtx.cursorRow * linesPerMenuItem, DISPLAYPORT_ATTR_NONE, ">");
        pDisplay->cursorRow = currentCtx.cursorRow;
    }
    if (room < 30) {
        return;
    }

	// 调用当前菜单状态更新函数
    if (currentCtx.menu->onDisplayUpdate) {
        const void *result = currentCtx.menu->onDisplayUpdate(pDisplay, pageTop + currentCtx.cursorRow);
        if (result == MENU_CHAIN_BACK) {
            cmsMenuBack(pDisplay);
            return;
        }
    }

    // 打印文本标签
    for (i = 0, p = pageTop; (p <= pageTop + pageMaxRow); i++, p++) {
        if (IS_PRINTLABEL(runtimeEntryFlags[i])) {
            uint8_t coloff = leftMenuColumn;
            coloff += (p->type == OME_Label) ? 0 : 1;
            room -= displayWrite(pDisplay, coloff, top + i * linesPerMenuItem, DISPLAYPORT_ATTR_NONE, p->text);
            CLR_PRINTLABEL(runtimeEntryFlags[i]);
            if (room < 30) {
                return;
            }
        }
		// XXX在列表中后面位置的查询值可能不是
		// 如果列表中间没有足够的空间，则打印XXX
        if (IS_PRINTVALUE(runtimeEntryFlags[i])) {
            bool selectedRow = i == currentCtx.cursorRow;
			// 绘制菜单项
            room -= cmsDrawMenuEntry(pDisplay, p, top + i * linesPerMenuItem, selectedRow, &runtimeEntryFlags[i]);
            if (room < 30) {
                return;
            }
        }
    }
}

/**********************************************************************
函数名称：cmsMenuChange
函数功能：cms菜单改变
函数形参：pDisplay，ptr
函数返回值：None 
函数描述：None 
**********************************************************************/
const void *cmsMenuChange(displayPort_t *pDisplay, const void *ptr)
{
	// 获取当前菜单指针
    const CMS_Menu *pMenu = (const CMS_Menu *)ptr;
	// 判断菜单是否为空
    if (!pMenu) {
        return NULL;
    }

	// ---------------------------------------------------更换菜单
    if (pMenu != currentCtx.menu) {
		// 禁止保存菜单
        saveMenuInhibited = false;
        if (currentCtx.menu) {
			// 如果打开初始的顶级菜单，那么currentCtx.menu将为空，无事可做
			// 否则，在移动到所选菜单之前，将当前菜单堆叠
            if (menuStackIdx >= CMS_MENU_STACK_LIMIT - 1) {
                // 已达到菜单堆栈限制-防止阵列溢出
                return NULL;
            }
			// 保存当前菜单上下文
            menuStack[menuStackIdx++] = currentCtx;
        }
		// 更新当前菜单
        currentCtx.menu = pMenu;
        currentCtx.cursorRow = 0;
		// 判断菜单条目是否为空
        if (pMenu->onEnter) {
            const void *result = pMenu->onEnter(pDisplay);
            if (result == MENU_CHAIN_BACK) {
                return cmsMenuBack(pDisplay);
            }
        }
		// cms菜单页面计数
        cmsMenuCountPage(pDisplay);
		// CMS选择页
        cmsPageSelect(pDisplay, 0);
    } 
	// ---------------------------------------------------对当前菜单进行翻页
	else {
		// (pMenu == curretMenu)情况发生时，重新打开显示循环
		// currentCtx cursorRow被保存为绝对;将其转换回page + relative
        int8_t cursorAbs = currentCtx.cursorRow;
        currentCtx.cursorRow = cursorAbs % maxMenuItems;
		// cms菜单页面计数
        cmsMenuCountPage(pDisplay);
		// CMS选择页
        cmsPageSelect(pDisplay, cursorAbs / maxMenuItems);
    }
    return NULL;
}

/**********************************************************************
函数名称：cmsMenuOpen
函数功能：cms菜单打开
函数形参：None
函数返回值：None 
函数描述：None 
**********************************************************************/
void cmsMenuOpen(void)
{
    const CMS_Menu *startMenu;
	// --------------------------------------还未进入cms菜单状态 - 进入cms菜单
    if (!cmsInMenu) {
        // CMS选择当前显示端口
        pCurrentDisplay = cmsDisplayPortSelectCurrent();
		// 判断是否有显示端口 - 无显示端口直接返回
        if (!pCurrentDisplay) {
            return;
        }
		// CMS在菜单界面
        cmsInMenu = true;
		// 初始化当前上下文
        currentCtx = (cmsCtx_t){ NULL, 0, 0 };
		// 获取起始菜单 - 主菜单（menuMain）
        startMenu = &cmsx_menuMain;
		// 初始化菜单栈索引
        menuStackIdx = 0;
		// 设置解锁禁用 - CMS菜单界面不允许解锁
        setArmingDisabled(ARMING_DISABLED_CMS_MENU);
		// 显示层选择 - 前景层
        displayLayerSelect(pCurrentDisplay, DISPLAYPORT_LAYER_FOREGROUND); 
    } 
	// --------------------------------------已进入cms菜单状态 - 判断是否更改显示端口
	else {
        // 选择下一个显示端口
        displayPort_t *pNextDisplay = cmsDisplayPortSelectNext();
		// 配置起始菜单
        startMenu = currentCtx.menu;
		// 显示端口被更改
        if (pNextDisplay != pCurrentDisplay) {
            // 将cursorRow转换为绝对值
            currentCtx.cursorRow = cmsCursorAbsolute(pCurrentDisplay);
			// 释放当前显示端口
            displayRelease(pCurrentDisplay);
			// 更新当前显示端口
            pCurrentDisplay = pNextDisplay;
        } 
		// 显示端口未更改且已经进入CMS菜单 - 不需要再做任何事情直接返回
		else {
            return;
        }
    }
	// --------------------------------------抓取当前显示端口供CMS菜单使用并清除屏幕
    displayGrab(pCurrentDisplay); 
	// 判断屏幕大小并进行初始化配置
    if ( pCurrentDisplay->cols < NORMAL_SCREEN_MIN_COLS) {
	      smallScreen       = true;
	      linesPerMenuItem  = 2;                // 预留光标移动位置
	      leftMenuColumn    = 0;
	      rightMenuColumn   = pCurrentDisplay->cols;
	      maxMenuItems      = (pCurrentDisplay->rows) / linesPerMenuItem;
    } else {
	      smallScreen       = false;
	      linesPerMenuItem  = 1;
	      leftMenuColumn    = 2;				// 预留光标移动位置
	      rightMenuColumn   = pCurrentDisplay->cols - 2;
	      maxMenuItems      = pCurrentDisplay->rows - 2;
    }
	// --------------------------------------判断是否使用满屏幕
    if (pCurrentDisplay->useFullscreen) {
    	  leftMenuColumn = 0;
    	  rightMenuColumn   = pCurrentDisplay->cols;
    	  maxMenuItems      = pCurrentDisplay->rows;
    }
	// --------------------------------------cms菜单改变
    cmsMenuChange(pCurrentDisplay, startMenu);
}

/**********************************************************************
函数名称：cmsTraverseGlobalExit
函数功能：cms遍历全局退出
函数形参：pMenu
函数返回值：None 
函数描述：None 
**********************************************************************/
static void cmsTraverseGlobalExit(const CMS_Menu *pMenu)
{
    for (const OSD_Entry *p = pMenu->entries; p->type != OME_END ; p++) {
		// 一直调用到没有子菜单
        if (p->type == OME_Submenu) {
			// 递归调用
            cmsTraverseGlobalExit(p->data);
        }
    }
}

/**********************************************************************
函数名称：cmsMenuExit
函数功能：cms菜单退出
函数形参：pDisplay，ptr
函数返回值：None 
函数描述：None 
**********************************************************************/
const void *cmsMenuExit(displayPort_t *pDisplay, const void *ptr)
{
    int exitType = (int)ptr;
	// ------------------------判断退出类型
    switch (exitType) {
	    case CMS_EXIT_SAVE:
	    case CMS_EXIT_SAVEREBOOT:
	    case CMS_POPUP_SAVE:
	    case CMS_POPUP_SAVEREBOOT:
			// 遍历全局退出  		- 一直调用到没有子菜单
	        cmsTraverseGlobalExit(&cmsx_menuMain);
			// 判断当前上下文是否有退出函数 - 强制退出
	        if (currentCtx.menu->onExit) {
	            currentCtx.menu->onExit(pDisplay, (OSD_Entry *)NULL); 
	        }
			// 判断退出类型
	        if ((exitType == CMS_POPUP_SAVE) || (exitType == CMS_POPUP_SAVEREBOOT)) {
	            // 遍历菜单堆栈并调用它们的onExit函数
	            for (int i = menuStackIdx - 1; i >= 0; i--) {
	                if (menuStack[i].menu->onExit) {
	                    menuStack[i].menu->onExit(pDisplay, (OSD_Entry *)NULL);
	                }
	            }
	        }
			// 保存配置和通知
	        saveConfigAndNotify();
	        break;
	    case CMS_EXIT:
	        break;
    }
	// ------------------------配置为不在CMS菜单
    cmsInMenu = false;
	// ------------------------显示释放
    displayRelease(pDisplay);
    currentCtx.menu = NULL;
	// ------------------------判断退出类型是否为REBOOT
    if ((exitType == CMS_EXIT_SAVEREBOOT) || (exitType == CMS_POPUP_SAVEREBOOT) || (exitType == CMS_POPUP_EXITREBOOT)) {
		// 清屏
		displayClearScreen(pDisplay);
		// 字符串提示
        displayWrite(pDisplay, 5, 3, DISPLAYPORT_ATTR_NONE, "REBOOTING...");
		// 显示重新同步
        displayResync(pDisplay); 
		// 停止电机
        stopMotors();
		// 关闭电机
        motorShutdown();
        delay(200);
		// 系统复位
        systemReset();
    }
	// ------------------------重置解锁失能
    unsetArmingDisabled(ARMING_DISABLED_CMS_MENU);
    return NULL;
}

//-------------------------------------------------------------------------------------菜单按键处理相关API

/**********************************************************************
函数名称：cmsHandleKey
函数功能：cms按键处理
函数形参：显示端口，key值
函数返回值：按键延时消抖时间
函数描述：None 
**********************************************************************/
STATIC_UNIT_TESTED uint16_t cmsHandleKey(displayPort_t *pDisplay, cms_key_e key)
{
    uint16_t res = BUTTON_TIME;
	// 记录菜单内容
    const OSD_Entry *p;

    if (!currentCtx.menu) {
        return res;
    }

	// ------------------------------------------菜单
    if (key == CMS_KEY_MENU) {
		// 打开菜单
        cmsMenuOpen();
        return BUTTON_PAUSE;
    }

	// ------------------------------------------退出
    if (key == CMS_KEY_ESC) {
        if (osdElementEditing) {
			// 失能OSD元素编辑状态
            osdElementEditing = false;
        } else {
        	// 菜单回退
            cmsMenuBack(pDisplay);
        }
        return BUTTON_PAUSE;
    }

	// ------------------------------------------保存
    if (key == CMS_KEY_SAVEMENU && !saveMenuInhibited) {
        osdElementEditing = false;
		// cms菜单改变 - 更换菜单 || 翻页
        cmsMenuChange(pDisplay, getSaveExitMenu());
        return BUTTON_PAUSE;
    }

	// ------------------------------------------下键
    if ((key == CMS_KEY_DOWN) && (!osdElementEditing)) {
		// 光标移动
        if (currentCtx.cursorRow < pageMaxRow) {
            currentCtx.cursorRow++;
        } else {
        	// 翻页
            cmsPageNext(pDisplay);
			// 在任何情况下都要去顶端
            currentCtx.cursorRow = 0;    
        }
    }

	// ------------------------------------------上键
    if ((key == CMS_KEY_UP) && (!osdElementEditing)) {
		// 光标移动
        currentCtx.cursorRow--;
        // 跳过非标题标签、字符串和动态只读条目
        while ((rowIsSkippable(pageTop + currentCtx.cursorRow)) && currentCtx.cursorRow > 0) {
            currentCtx.cursorRow--;
        }
		// 翻页
        if (currentCtx.cursorRow == -1 || (pageTop + currentCtx.cursorRow)->type == OME_Label) {
            // 转到上一页
            cmsPagePrev(pDisplay);
			// 光标变为当前页的最大行数
            currentCtx.cursorRow = pageMaxRow;
        }
    }

	// ------------------------------------------返回上下键延时消抖时间
    if ((key == CMS_KEY_DOWN || key == CMS_KEY_UP) && (!osdElementEditing)) {
        return res;
    }

	// ------------------------------------------记录当前菜单内容
    p = pageTop + currentCtx.cursorRow;

	// ------------------------------------------判断当前条目类型
    switch (p->type) {
        case OME_Submenu:		// ---------子菜单  
        	// 判断是否进入子菜单
            if (key == CMS_KEY_RIGHT) {
                cmsMenuChange(pDisplay, p->data);
                res = BUTTON_PAUSE;
            }
            break;

        case OME_Funcall:;		// ---------功能调用
            const void *retval;
			// 调用功能函数
            if (p->func && key == CMS_KEY_RIGHT) {
                retval = p->func(pDisplay, p->data);
				// 判断是否退出
                if (retval == MENU_CHAIN_BACK) {
                    cmsMenuBack(pDisplay);
                }
                res = BUTTON_PAUSE;
            }
            break;

        case OME_OSD_Exit:		// ---------退出
            if (p->func && key == CMS_KEY_RIGHT) {
                p->func(pDisplay, p->data);
                res = BUTTON_PAUSE;
            }
            break;

        case OME_Back:			// ---------回退
            cmsMenuBack(pDisplay);
            res = BUTTON_PAUSE;
            osdElementEditing = false;
            break;

        case OME_Bool:			// ---------Bool
            if (p->data) {
                uint8_t *val = p->data;
                const uint8_t previousValue = *val;
                *val = (key == CMS_KEY_RIGHT) ? 1 : 0;
                SET_PRINTVALUE(runtimeEntryFlags[currentCtx.cursorRow]);
                if ((p->flags & REBOOT_REQUIRED) && (*val != previousValue)) {
                    setRebootRequired();
                }
                if (p->func) {
                    p->func(pDisplay, p->data);
                }
            }
            break;

#ifdef USE_OSD
        case OME_VISIBLE:		// ---------VISIBLE 
            if (p->data) {
                uint16_t *val = (uint16_t *)p->data;
                const uint16_t previousValue = *val;
				// 未在编辑中，右按键进入编辑
                if ((key == CMS_KEY_RIGHT) && (!osdElementEditing)) {
                    osdElementEditing = true;
                    osdProfileCursor = 1;
                } 
				// 该条目正在编辑中
				else if (osdElementEditing) {
					// 上键
                    if (key == CMS_KEY_UP) {
                        *val |= OSD_PROFILE_FLAG(osdProfileCursor);
                    }
					// 下键
                    if (key == CMS_KEY_DOWN) {
                        *val &= ~OSD_PROFILE_FLAG(osdProfileCursor);
                    }
                }
                SET_PRINTVALUE(runtimeEntryFlags[currentCtx.cursorRow]);
                if ((p->flags & REBOOT_REQUIRED) && (*val != previousValue)) {
                    setRebootRequired();
                }
            }
            break;
#endif

        case OME_UINT8:			// ---------UINT8 
        case OME_FLOAT:			// ---------FLOAT
            if (p->data) {
                OSD_UINT8_t *ptr = p->data;
                const uint16_t previousValue = *ptr->val;
                if (key == CMS_KEY_RIGHT) {
                    if (*ptr->val < ptr->max) {
                        *ptr->val += ptr->step;
                    }
                } else {
                    if (*ptr->val > ptr->min) {
                        *ptr->val -= ptr->step;
                    }
                }
                SET_PRINTVALUE(runtimeEntryFlags[currentCtx.cursorRow]);
                if ((p->flags & REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                    setRebootRequired();
                }
                if (p->func) {
                    p->func(pDisplay, p);
                }
            }
            break;

        case OME_TAB:			// ---------TAB  
            if (p->type == OME_TAB) {
                OSD_TAB_t *ptr = p->data;
                const uint8_t previousValue = *ptr->val;

                if (key == CMS_KEY_RIGHT) {
                    if (*ptr->val < ptr->max) {
                        *ptr->val += 1;
                    }
                } else {
                    if (*ptr->val > 0) {
                        *ptr->val -= 1;
                    }
                }
                if (p->func) {
                    p->func(pDisplay, p->data);
                }
                SET_PRINTVALUE(runtimeEntryFlags[currentCtx.cursorRow]);
                if ((p->flags & REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                    setRebootRequired();
                }
            }
            break;

        case OME_INT8:			// ---------INT8  
            if (p->data) {
                OSD_INT8_t *ptr = p->data;
                const int8_t previousValue = *ptr->val;
                if (key == CMS_KEY_RIGHT) {
                    if (*ptr->val < ptr->max) {
                        *ptr->val += ptr->step;
                    }
                } else {
                    if (*ptr->val > ptr->min) {
                        *ptr->val -= ptr->step;
                    }
                }
                SET_PRINTVALUE(runtimeEntryFlags[currentCtx.cursorRow]);
                if ((p->flags & REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                    setRebootRequired();
                }
                if (p->func) {
                    p->func(pDisplay, p);
                }
            }
            break;

        case OME_UINT16:		// ---------UINT16
            if (p->data) {
                OSD_UINT16_t *ptr = p->data;
                const uint16_t previousValue = *ptr->val;
                if (key == CMS_KEY_RIGHT) {
                    if (*ptr->val < ptr->max) {
                        *ptr->val += ptr->step;
                    }
                } else {
                    if (*ptr->val > ptr->min) {
                        *ptr->val -= ptr->step;
                    }
                }
                SET_PRINTVALUE(runtimeEntryFlags[currentCtx.cursorRow]);
                if ((p->flags & REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                    setRebootRequired();
                }
                if (p->func) {
                    p->func(pDisplay, p);
                }
            }
            break;

        case OME_INT16:			// ---------INT16 
            if (p->data) {
                OSD_INT16_t *ptr = p->data;
                const int16_t previousValue = *ptr->val;
                if (key == CMS_KEY_RIGHT) {
                    if (*ptr->val < ptr->max) {
                        *ptr->val += ptr->step;
                    }
                } else {
                    if (*ptr->val > ptr->min) {
                        *ptr->val -= ptr->step;
                    }
                }
                SET_PRINTVALUE(runtimeEntryFlags[currentCtx.cursorRow]);
                if ((p->flags & REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                    setRebootRequired();
                }
                if (p->func) {
                    p->func(pDisplay, p);
                }
            }
            break;

        case OME_String:		// ---------字符串
            break;

        case OME_Label:			// ---------标签
        case OME_END:			// ---------结束
            break;
    }

	// ------------------------------------------返回按键延时消抖时间
    return res;
}

/**********************************************************************
函数名称：cmsHandleKeyWithRepeat
函数功能：cms按键重复处理
函数形参：pDisplay，key，repeatCount
函数返回值：ret
函数描述：None 
**********************************************************************/
uint16_t cmsHandleKeyWithRepeat(displayPort_t *pDisplay, cms_key_e key, int repeatCount)
{
    uint16_t ret = 0;
	// 重复按键处理 - repeatCount次
    for (int i = 0 ; i < repeatCount ; i++) {
        ret = cmsHandleKey(pDisplay, key);
    }
    return ret;
}

//-------------------------------------------------------------------------------------菜单调度相关API

/**********************************************************************
函数名称：cmsUpdate
函数功能：cms更新
函数形参：currentTimeUs
函数返回值：None 
函数描述：None 
**********************************************************************/
static void cmsUpdate(uint32_t currentTimeUs)
{
    static int16_t rcDelayMs = BUTTON_TIME;				 // RC延时消抖 - 250ms
    static int holdCount = 1;			 			     // 保持计数
    static int repeatCount = 1;			 				 // 需要按键重复处理的数量 
    static int repeatBase = 0;							 // 重复基础
    static uint32_t lastCalledMs = 0;    			     // 上一次调用时间
    const uint32_t currentTimeMs = currentTimeUs / 1000; // 获取当前时间节拍 - ms

	// ---------------------------------------------------------------------------------------未进入CMS菜单
    if (!cmsInMenu) {
        // 检测菜单调用 - 油门：中 + 偏航：左 + 俯仰：上 ，并且未解锁 
        if (IS_MID(THROTTLE) && IS_LO(YAW) && IS_HI(PITCH) && !ARMING_FLAG(ARMED)) {
			// 打开CMS菜单
            cmsMenuOpen();
			// RC延时消抖 - 500ms
            rcDelayMs = BUTTON_PAUSE;   
        }
    }
	// ---------------------------------------------------------------------------------------已进入CMS菜单
	else {
		// -------------------------------------获取RC按键操作
        cms_key_e key = CMS_KEY_NONE;
        if (IS_MID(THROTTLE) && IS_LO(YAW) && IS_HI(PITCH) && !ARMING_FLAG(ARMED)) {
            key = CMS_KEY_MENU;
        } else if (IS_HI(PITCH)) {		// 上
            key = CMS_KEY_UP;
        } else if (IS_LO(PITCH)) {		// 下
            key = CMS_KEY_DOWN;
        } else if (IS_LO(ROLL))  {		// 左
            key = CMS_KEY_LEFT;
        } else if (IS_HI(ROLL))  {		// 右
            key = CMS_KEY_RIGHT;
        } else if (IS_LO(YAW))   {		// 退出
            key = CMS_KEY_ESC;
        } else if (IS_HI(YAW))   {		// 保存
            key = CMS_KEY_SAVEMENU;
        }

        if (key == CMS_KEY_NONE) {
            // 无按键按下 - 复位重复按键处理
            holdCount = 1;
            repeatCount = 1;
            repeatBase = 0;
        } else {
            // 按键正在被按下 - 保持计数
            ++holdCount;
        }
		
		// -------------------------------------RC延时消抖和按键重复处理 - 一直到rcDelayMs <= 0处理按键操作
        if (rcDelayMs > 0) {
			// rcDelayMs递减 = RC延时时间 - 时差
            rcDelayMs -= (currentTimeMs - lastCalledMs);
        } 
		// 检测是否有按键按下
		else if (key) {
        	// cms按键重复处理 - repeatCount为重复处理key操作的次数，同时更新延时消抖时间
            rcDelayMs = cmsHandleKeyWithRepeat(pCurrentDisplay, key, repeatCount);
			// （左键 || 右键） &&       按键重复计数20次以上 - 用于需要左右快速调整数值
            if (((key == CMS_KEY_LEFT) || (key == CMS_KEY_RIGHT)) && (holdCount > 20)) {
                // 重复处理操作则减少RC延时消抖时间
                rcDelayMs /= (holdCount - 20);
                // 判断是否达到最小延时限制 - 50ms
                if (rcDelayMs <= 50) {
                    // 开始多次调用处理
                    if (repeatBase == 0) {
                        repeatBase = holdCount;
                    }
					// 计算重复处理次数
                    repeatCount = repeatCount + (holdCount - repeatBase) / 5;
					// 限制最大重复处理次数 - 5次
                    if (repeatCount > 5) {
                        repeatCount = 5;
                    }
                }
            }
        }

		// 绘制CMS菜单
        cmsDrawMenu(pCurrentDisplay, currentTimeUs);
    }
	// 更新调用时间
    lastCalledMs = millis();
}

/**********************************************************************
函数名称：cmsHandler
函数功能：cms更新处理
函数形参：当前时间节拍
函数返回值：None 
函数描述：	
	由调度器以任务形式调用。
**********************************************************************/
void cmsHandler(timeUs_t currentTimeUs)
{
	// 判断是否有CMS设备
    if (cmsDeviceCount > 0) {
		// CMS菜单更新
        cmsUpdate(currentTimeUs);
    }
}

//-------------------------------------------------------------------------------------菜单保存相关API

/**********************************************************************
函数名称：inhibitSaveMenu
函数功能：禁止保存菜单
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void inhibitSaveMenu(void)
{
	// 使能禁止保存菜单
    saveMenuInhibited = true;
}

//-------------------------------------------------------------------------------------菜单添加内容相关API

/**********************************************************************
函数名称：cmsAddMenuEntry
函数功能：添加菜单条目
函数形参：menuEntry，text，type，func，data，flags
函数返回值：None 
函数描述：None 
**********************************************************************/
void cmsAddMenuEntry(OSD_Entry *menuEntry, char *text, OSD_MenuElement type, CMSEntryFuncPtr func, void *data, uint8_t flags)
{
    menuEntry->text = text;
    menuEntry->type = type;
    menuEntry->func = func;
    menuEntry->data = data;
    menuEntry->flags = flags;
}

//-------------------------------------------------------------------------------------菜单初始化相关API

/**********************************************************************
函数名称：cmsInit
函数功能：cms初始化
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void cmsInit(void)
{
	// 当前设备数量
    cmsDeviceCount = 0;
	// 当前设备
    cmsCurrentDevice = -1;
}
#endif // CMS

