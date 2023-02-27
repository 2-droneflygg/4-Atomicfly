/*********************************************************************************
 外部中断/事件控制器 (EXTI)：
	 1.外部中断/事件控制器包含多达 23 个用于产生事件/中断请求的边沿检测器;
	 2.每根输入线都可单独进行配置;
	 3.选择类型（中断或事件）和相应的触发事件（上升沿触发、下降沿触发或边沿触发）;
	 4.每根输入线还可单独屏蔽;
	 5.挂起寄存器用于保持中断请求的状态线。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_EXTI
#include "drivers/nvic.h"
#include "io_impl.h"
#include "drivers/exti.h"

/* ----------------------------EXTI通道中断回调函数描述--------------------------- */	
typedef struct {
    extiCallbackRec_t* handler;
} extiChannelRec_t;
// 16个通道外部中断回调函数记录
extiChannelRec_t extiChannelRecs[16];    

// ---------------------------------------------------------EXTI分组  
//                                      0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
static const uint8_t extiGroups[16] = { 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6 };

// ---------------------------------------------------------中断源分组
#define EXTI_IRQ_GROUPS 7

// ---------------------------------------------------------EXTI分组优先级
static uint8_t extiGroupPriority[EXTI_IRQ_GROUPS];

// ---------------------------------------------------------EXTI分组中断线
static const uint8_t extiGroupIRQn[EXTI_IRQ_GROUPS] = {
    EXTI0_IRQn,
    EXTI1_IRQn,
    EXTI2_IRQn,
    EXTI3_IRQn,
    EXTI4_IRQn,
    EXTI9_5_IRQn,
    EXTI15_10_IRQn
};
	
// ---------------------------------------------------------触发方式查找表
static uint32_t triggerLookupTable[] = {
    [EXTI_TRIGGER_RISING] = EXTI_Trigger_Rising,         // 上升沿触发
    [EXTI_TRIGGER_FALLING] = EXTI_Trigger_Falling,		 // 下降沿触发
    [EXTI_TRIGGER_BOTH] = EXTI_Trigger_Rising_Falling    // 边沿触发
};
	
// ---------------------------------------------------------中断屏蔽寄存器 - 屏蔽来自 x 线的中断请求
#define EXTI_REG_IMR (EXTI->IMR)   

// ---------------------------------------------------------挂起寄存器 - 当在外部中断线上发生了选择的边沿事件，该位被置“1”，在此位中写入“1”可以清除它
#define EXTI_REG_PR  (EXTI->PR)

/**********************************************************************
函数名称：EXTIInit
函数功能：EXTI初始化
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void EXTIInit(void)
{
    // 启用SYSCFG时钟，否则EXTI中断服务函数不会被调用 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	// 16个通道中断服务回调函数初始化为NULL
    memset(extiChannelRecs, 0, sizeof(extiChannelRecs));
	// EXTI分组优先级都初始化为0xff
    memset(extiGroupPriority, 0xff, sizeof(extiGroupPriority));
}

/**********************************************************************
函数名称：EXTIHandlerInit
函数功能：注册EXTI中断服务函数 - 注册中断服务函数要先于EXTI配置
函数形参：EXTI中断服务函数回调描述，EXTI线中断服务函数
函数返回值：None
函数描述：
	注册回调中断服务函数。
**********************************************************************/
void EXTIHandlerInit(extiCallbackRec_t *self, extiHandlerCallback *fn)
{
    self->fn = fn;
}

/**********************************************************************
函数名称：EXTIConfig
函数功能：EXTI配置
函数形参：IO，中断服务回调函数，优先级，IO配置，触发方式
函数返回值：None
函数描述：None
**********************************************************************/
void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, ioConfig_t config, extiTrigger_t trigger)
{
	// 获取IO索引
    int chIdx = IO_GPIOPinIdx(io);   
    if (chIdx < 0) {
        return;
    }

	// 获取EXTI分组
    int group = extiGroups[chIdx];

	// 注册中断服务回调函数
    extiChannelRec_t *rec = &extiChannelRecs[chIdx];
    rec->handler = cb;
	
	// IO初始化
    IOConfigGPIO(io, config);

	// 选择作为EXTI线使用的GPIO管脚
    SYSCFG_EXTILineConfig(IO_EXTI_PortSourceGPIO(io), IO_EXTI_PinSource(io));

	// 获取EXTI线
    uint32_t extiLine = IO_EXTI_Line(io);

	// 清除EXTI线挂起位
    EXTI_ClearITPendingBit(extiLine);

	// 初始化EXTI外部中断/事件控制器
    EXTI_InitTypeDef EXTIInit;
    EXTIInit.EXTI_Line = extiLine;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = triggerLookupTable[trigger];
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

	// 初始化NVIC中断控制器
    if (extiGroupPriority[group] > irqPriority) {
        extiGroupPriority[group] = irqPriority;
		// 配置NVIC
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = extiGroupIRQn[group];
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		// NVIC初始化
        NVIC_Init(&NVIC_InitStructure);
    }
}

/**********************************************************************
函数名称：EXTIEnable
函数功能：使能IO的外部中断
函数形参：io，enable
函数返回值：None
函数描述：None
**********************************************************************/
void EXTIEnable(IO_t io, bool enable)
{
    uint32_t extiLine = IO_EXTI_Line(io);
    if (!extiLine) {
        return;
    }
    if (enable) {
        EXTI_REG_IMR |= extiLine;
    } else {
        EXTI_REG_IMR &= ~extiLine;
    }
}

/**********************************************************************
函数名称：EXTI_IRQHandler
函数功能：EXTI中断服务函数
函数形参：io，enable
函数返回值：None
函数描述：None
**********************************************************************/
#define EXTI_EVENT_MASK 0xFFFF    // 外部中断事件掩码
void EXTI_IRQHandler(void)
{
	// 获取外部中断激活状态
    uint32_t exti_active = (EXTI_REG_IMR & EXTI_REG_PR) & EXTI_EVENT_MASK;
    while (exti_active) {
        unsigned idx = 31 - __builtin_clz(exti_active);
        uint32_t mask = 1 << idx;
		// 中断服务回调函数
        extiChannelRecs[idx].handler->fn(extiChannelRecs[idx].handler);
        EXTI_REG_PR = mask;      // 清除挂起的掩码(写入1)
        exti_active &= ~mask;    // 更新激活状态
    }
}

// EXTI中断服务函数命名宏
#define _EXTI_IRQ_HANDLER(name)                 \
    void name(void) {                           \
        EXTI_IRQHandler();                      \
    }                                           \
    struct dummy                                \
    /**/

// EXTI中断服务函数 - 调用EXTI_IRQHandler
_EXTI_IRQ_HANDLER(EXTI0_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI1_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI2_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI3_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI4_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI9_5_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI15_10_IRQHandler);
#endif

