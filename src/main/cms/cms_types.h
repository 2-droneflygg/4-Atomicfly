/*********************************************************************************
 提供一系列OSD菜单类型配置。
*********************************************************************************/
#pragma once

// ---------------------------------------------------------标志位
#define PRINT_VALUE     0x01    									// 值已经改变，需要重新绘制
#define PRINT_LABEL     0x02    									// 文字标签需要重新绘制
#define DYNAMIC         0x04    									// 值应该动态更新
#define OPTSTRING       0x08    									// (临时)标志OME_Submenu，指示应该调用func来获取一个要显示的字符串。
#define REBOOT_REQUIRED 0x10    									// 如果修改了该值，需要重启

// ---------------------------------------------------------值
#define IS_PRINTVALUE(x) ((x) & PRINT_VALUE)			            // 获取值是否需要重新绘制
#define SET_PRINTVALUE(x) do { (x) |= PRINT_VALUE; } while (0)		// 设置值需要重新绘制
#define CLR_PRINTVALUE(x) do { (x) &= ~PRINT_VALUE; } while (0)		// 设置值不需要重新绘制

// ---------------------------------------------------------文字标签
#define IS_PRINTLABEL(x) ((x) & PRINT_LABEL)						// 获取文字标签是否需要绘制
#define SET_PRINTLABEL(x) do { (x) |= PRINT_LABEL; } while (0)		// 设置文字标签需要重新绘制
#define CLR_PRINTLABEL(x) do { (x) &= ~PRINT_LABEL; } while (0)		// 设置文字标签不需要重新绘制

// ---------------------------------------------------------动态绘制
#define IS_DYNAMIC(p) ((p)->flags & DYNAMIC)						// 获取值是否需要动态绘制

// ---------------------------------------------------------CMSMenuFuncPtr函数链接的特殊返回值
#define MENU_CHAIN_BACK  (&menuChainBack)    						// 导致自动cmsMenuBack

/* ---------------------------元素类型枚举-------------------------- */	
typedef enum {
    OME_Label,			 								// 标签					
    OME_Back,			 								// 回退
    OME_OSD_Exit,		 								// 退出
    OME_Submenu,		 								// 子菜单
    OME_Funcall,		 								// 功能调用
    OME_Bool,
    OME_INT8,
    OME_UINT8,
    OME_UINT16,
    OME_INT16,
    OME_String,			 								// 字符串
    OME_FLOAT,  		 								// 浮点数：只有255的值，不能是2.55或25.5，仅为PID的
#ifdef USE_OSD
    OME_VISIBLE,
#endif
    OME_TAB,
    OME_END,
    OME_MAX = OME_END
} OSD_MenuElement;

/* -------------------------OSD菜单内容结构体----------------------- */	
typedef const void *(*CMSEntryFuncPtr)(displayPort_t *displayPort, const void *ptr);
typedef struct {
    const char * text;									// 文本显示	
    OSD_MenuElement type;								// 元素类型
    CMSEntryFuncPtr func;								// 功能回调函数
    void *data;											// 数据
    uint8_t flags;										// 标志
} __attribute__((packed)) OSD_Entry;

/* ---------------------------OSD菜单结构体------------------------- */	
typedef const void *(*CMSMenuFuncPtr)(displayPort_t *pDisp);
typedef const void *(*CMSMenuOnExitPtr)(displayPort_t *pDisp, const OSD_Entry *self);
typedef const void *(*CMSMenuOnDisplayUpdatePtr)(displayPort_t *pDisp, const OSD_Entry *selected);
typedef struct {
    const CMSMenuFuncPtr onEnter;					    // 进入函数
    const CMSMenuOnExitPtr onExit;						// 退出函数
    const CMSMenuOnDisplayUpdatePtr onDisplayUpdate;	// 状态更新函数
    const OSD_Entry *entries;							// 显示条目
} CMS_Menu;

/* --------------------------OSD UINT8结构体------------------------ */	
typedef struct {
    uint8_t *val;
    uint8_t min;
    uint8_t max;
    uint8_t step;
} OSD_UINT8_t;

/* ---------------------------OSD INT8结构体------------------------ */	
typedef struct {
    int8_t *val;
    int8_t min;
    int8_t max;
    int8_t step;
} OSD_INT8_t;

/* --------------------------OSD INT16结构体------------------------ */	
typedef struct {
    int16_t *val;
    int16_t min;
    int16_t max;
    int16_t step;
} OSD_INT16_t;

/* --------------------------OSD UINT16结构体----------------------- */	
typedef struct {
    uint16_t *val;
    uint16_t min;
    uint16_t max;
    uint16_t step;
} OSD_UINT16_t;

/* --------------------------OSD FLOAT结构体------------------------ */	
typedef struct {
    uint8_t *val;
    uint8_t min;
    uint8_t max;
    uint8_t step;
    uint16_t multipler;
} OSD_FLOAT_t;

/* ---------------------------OSD TAB结构体------------------------- */	
typedef struct {
    uint8_t *val;
    uint8_t max;
    const char * const *names;
} OSD_TAB_t;

/* --------------------------OSD String结构体----------------------- */	
typedef struct {
    char *val;
} OSD_String_t;

extern int menuChainBack;

