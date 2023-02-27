/*********************************************************************************
 1.STM32定时器一共有3类：
 	高级控制定时器（ TIM1 和 TIM8）：只提供最基本的定时/计数功能以及DAC的触发功能。
 	通用定时器（TIM2 到 TIM5）（TIM9 到 TIM14）：包括了基本定时器的功能，
 						同时还具有输入捕获、输出比较、PWM功能（PWM输入以及PWM输出）。
 	基本定时器（TIM6 和 TIM7）：包括了通用定时器的功能，
 						同时还具有PWM互补输出、死区功能以及刹车功能。
 2.发生如下事件时生成中断/DMA请求：
 	更新：计数器上溢/下溢、计数器初始化（通过软件或内部/外部触发）;
 	触发事件（计数器启动、停止、初始化或通过内部/外部触发计数）;
 	输入捕获;
 	输出比较。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_TIMER
#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/nvic.h"

#include "drivers/io.h"
#include "rcc.h"
#include "drivers/system.h"

#include "timer.h"

// ---------------------------------------------------------定时器标签宏
#define TIM_N(n) (1 << (n))

// ---------------------------------------------------------返回定时器在定时器表中的索引，最低定时器的索引为0
#define TIMER_INDEX(i)  BITCOUNT((TIM_N(i) - 1) & USED_TIMERS)

// ---------------------------------------------------------使用的定时器数量 
// TIM1 2 channels
// TIM2 4 channels
// TIM3 4 channels
// TIM4 4 channels
#define USED_TIMER_COUNT  BITCOUNT(USED_TIMERS)

// ---------------------------------------------------------每个定时器通道数
// TIM_Channel_1..4
#define CC_CHANNELS_PER_TIMER 4             		   

/* ------------------------------定时器配置结构体------------------------------ */	
typedef struct timerConfig_s {
    timerCCHandlerRec_t *edgeCallback[CC_CHANNELS_PER_TIMER];	   // 边沿回调函数（多个通道）
    timerOvrHandlerRec_t *overflowCallback[CC_CHANNELS_PER_TIMER]; // 溢出回调函数（多个通道）
    timerOvrHandlerRec_t *overflowCallbackActive; 				   // 激活溢出回调
    uint32_t forcedOverflowTimerValue;							   // 强制溢出定时器值
} timerConfig_t;
timerConfig_t timerConfig[USED_TIMER_COUNT]; 

/* -----------------------------定时器通道信息结构体--------------------------- */	
typedef struct {
    channelType_t type;											   // 通道类型
} timerChannelInfo_t;
timerChannelInfo_t timerChannelInfo[TIMER_CHANNEL_COUNT];

/* -------------------------------定时器信息结构体----------------------------- */	
typedef struct {
    uint8_t priority;											   // 优先级				
} timerInfo_t;
timerInfo_t timerInfo[USED_TIMER_COUNT];

// ---------------------------------------------------------预配置使用的定时器（TIMx）
TIM_TypeDef * const usedTimers[USED_TIMER_COUNT] = {
#define _DEF(i) TIM##i
#if USED_TIMERS & TIM_N(1)
    _DEF(1),
#endif
#if USED_TIMERS & TIM_N(2)
    _DEF(2),
#endif
#if USED_TIMERS & TIM_N(3)
    _DEF(3),
#endif
#if USED_TIMERS & TIM_N(4)
    _DEF(4),
#endif
#if USED_TIMERS & TIM_N(5)
    _DEF(5),
#endif
#if USED_TIMERS & TIM_N(6)
    _DEF(6),
#endif
#if USED_TIMERS & TIM_N(7)
    _DEF(7),
#endif
#if USED_TIMERS & TIM_N(8)
    _DEF(8),
#endif
#if USED_TIMERS & TIM_N(9)
    _DEF(9),
#endif
#if USED_TIMERS & TIM_N(10)
    _DEF(10),
#endif
#if USED_TIMERS & TIM_N(11)
    _DEF(11),
#endif
#if USED_TIMERS & TIM_N(12)
    _DEF(12),
#endif
#if USED_TIMERS & TIM_N(13)
    _DEF(13),
#endif
#if USED_TIMERS & TIM_N(14)
    _DEF(14),
#endif
#if USED_TIMERS & TIM_N(15)
    _DEF(15),
#endif
#if USED_TIMERS & TIM_N(16)
    _DEF(16),
#endif
#if USED_TIMERS & TIM_N(17)
    _DEF(17),
#endif
#undef _DEF
};

// ---------------------------------------------------------将计时器索引映射到计时器编号(usedTimers数组的直接副本)
const int8_t timerNumbers[USED_TIMER_COUNT] = {
#define _DEF(i) i
#if USED_TIMERS & TIM_N(1)
    _DEF(1),
#endif
#if USED_TIMERS & TIM_N(2)
    _DEF(2),
#endif
#if USED_TIMERS & TIM_N(3)
    _DEF(3),
#endif
#if USED_TIMERS & TIM_N(4)
    _DEF(4),
#endif
#if USED_TIMERS & TIM_N(5)
    _DEF(5),
#endif
#if USED_TIMERS & TIM_N(6)
    _DEF(6),
#endif
#if USED_TIMERS & TIM_N(7)
    _DEF(7),
#endif
#if USED_TIMERS & TIM_N(8)
    _DEF(8),
#endif
#if USED_TIMERS & TIM_N(9)
    _DEF(9),
#endif
#if USED_TIMERS & TIM_N(10)
    _DEF(10),
#endif
#if USED_TIMERS & TIM_N(11)
    _DEF(11),
#endif
#if USED_TIMERS & TIM_N(12)
    _DEF(12),
#endif
#if USED_TIMERS & TIM_N(13)
    _DEF(13),
#endif
#if USED_TIMERS & TIM_N(14)
    _DEF(14),
#endif
#if USED_TIMERS & TIM_N(15)
    _DEF(15),
#endif
#if USED_TIMERS & TIM_N(16)
    _DEF(16),
#endif
#if USED_TIMERS & TIM_N(17)
    _DEF(17),
#endif
#undef _DEF
};

//-------------------------------------------------------------------------------------获取定时器配置相关API

/**********************************************************************
函数名称：lookupTimerIndex
函数功能：获取定时器索引
函数形参：TIMx
函数返回值：定时器索引
函数描述：None
**********************************************************************/
static uint8_t lookupTimerIndex(const TIM_TypeDef *tim)
{
#define _CASE_SHF 10           // 可以安全地将定时器地址移到右边（如果某些计时器重叠，gcc将输出错误）
#define _CASE_(tim, index) case ((unsigned)tim >> _CASE_SHF): return index; break
#define _CASE(i) _CASE_(TIM##i##_BASE, TIMER_INDEX(i))

// 让gcc来做这些工作，switch应该是相当优化的
    switch ((unsigned)tim >> _CASE_SHF) {
#if USED_TIMERS & TIM_N(1)
        _CASE(1);
#endif
#if USED_TIMERS & TIM_N(2)
        _CASE(2);
#endif
#if USED_TIMERS & TIM_N(3)
        _CASE(3);
#endif
#if USED_TIMERS & TIM_N(4)
        _CASE(4);
#endif
#if USED_TIMERS & TIM_N(5)
        _CASE(5);
#endif
#if USED_TIMERS & TIM_N(6)
        _CASE(6);
#endif
#if USED_TIMERS & TIM_N(7)
        _CASE(7);
#endif
#if USED_TIMERS & TIM_N(8)
        _CASE(8);
#endif
#if USED_TIMERS & TIM_N(9)
        _CASE(9);
#endif
#if USED_TIMERS & TIM_N(10)
        _CASE(10);
#endif
#if USED_TIMERS & TIM_N(11)
        _CASE(11);
#endif
#if USED_TIMERS & TIM_N(12)
        _CASE(12);
#endif
#if USED_TIMERS & TIM_N(13)
        _CASE(13);
#endif
#if USED_TIMERS & TIM_N(14)
        _CASE(14);
#endif
#if USED_TIMERS & TIM_N(15)
        _CASE(15);
#endif
#if USED_TIMERS & TIM_N(16)
        _CASE(16);
#endif
#if USED_TIMERS & TIM_N(17)
        _CASE(17);
#endif
    default:  return ~1;  // 确保最终索引在范围内
    }
#undef _CASE
#undef _CASE_
}

/**********************************************************************
函数名称：timerGetNumberByIndex
函数功能：通过标签获取定时器编号
函数形参：index
函数返回值：timerNumbers[index]
函数描述：None
**********************************************************************/
int8_t timerGetNumberByIndex(uint8_t index)
{
    if (index < USED_TIMER_COUNT) {
        return timerNumbers[index];
    } else {
        return 0;
    }
}

/**********************************************************************
函数名称：timerGetTIMNumber
函数功能：获取定时器编号
函数形参：tim
函数返回值：timerNumbers[index]
函数描述：None
**********************************************************************/
int8_t timerGetTIMNumber(const TIM_TypeDef *tim)
{
    const uint8_t index = lookupTimerIndex(tim);
    return timerGetNumberByIndex(index);
}

/**********************************************************************
函数名称：timerRCC
函数功能：获取定时器时钟
函数形参：tim
函数返回值：定时器时钟
函数描述：None
**********************************************************************/
rccPeriphTag_t timerRCC(TIM_TypeDef *tim)
{
    for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (timerDefinitions[i].TIMx == tim) {
            return timerDefinitions[i].rcc;
        }
    }
    return 0;
}

/**********************************************************************
函数名称：lookupChannelIndex
函数功能：获取通道索引
函数形参：channel
函数返回值：通道索引
函数描述：None
**********************************************************************/
static inline uint8_t lookupChannelIndex(const uint16_t channel)
{
    return channel >> 2;
}

/**********************************************************************
函数名称：timerLookupChannelIndex
函数功能：获取定时器通道索引
函数形参：channel
函数返回值：通道索引
函数描述：None
**********************************************************************/
uint8_t timerLookupChannelIndex(const uint16_t channel)
{
    return lookupChannelIndex(channel);
}

/**********************************************************************
函数名称：timerChCCR
函数功能：获取定时器通道地址（外设地址）
函数形参：定时器硬件信息
函数返回值：ccr
函数描述：None
**********************************************************************/
volatile timCCR_t* timerChCCR(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CCR1 + timHw->channel);
}

/**********************************************************************
函数名称：timerCCR
函数功能：获取定时器通道地址（外设地址）
函数形参：TIMx,通道
函数返回值：None
函数描述：None
**********************************************************************/
volatile timCCR_t* timerCCR(TIM_TypeDef *tim, uint8_t channel)
{
    return (volatile timCCR_t*)((volatile char*)&tim->CCR1 + channel);
}

/**********************************************************************
函数名称：timerInputIrq
函数功能：获取定时器输入捕获中断源
函数形参：tim
函数返回值：输入捕获中断源
函数描述：None
**********************************************************************/
uint8_t timerInputIrq(TIM_TypeDef *tim)
{
    for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (timerDefinitions[i].TIMx == tim) {
            return timerDefinitions[i].inputIrq;
        }
    }
    return 0;
}

/**********************************************************************
函数名称：timerDmaSource
函数功能：获取定时器通道DMA源
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
uint16_t timerDmaSource(uint8_t channel)
{
    switch (channel) {
    case TIM_Channel_1:
        return TIM_DMA_CC1;
    case TIM_Channel_2:
        return TIM_DMA_CC2;
    case TIM_Channel_3:
        return TIM_DMA_CC3;
    case TIM_Channel_4:
        return TIM_DMA_CC4;
    }
    return 0;
}

/**********************************************************************
函数名称：getFilter
函数功能：计算输入滤波器常数
函数形参：ticks
函数返回值：0x0f
函数描述：
	应该把DTS设置为更高的值，以允许合理的输入滤波，
	注意：[0]预分频器确实使用DTS进行采样-序列不再是单调的
**********************************************************************/
static unsigned getFilter(unsigned ticks)
{
    static const unsigned ftab[16] = {
        1*1,                 	// fDTS !
        1*2, 1*4, 1*8,       	// fCK_INT
        2*6, 2*8,            	// fDTS/2
        4*6, 4*8,
        8*6, 8*8,
        16*5, 16*6, 16*8,
        32*5, 32*6, 32*8
    };
    for (unsigned i = 1; i < ARRAYLEN(ftab); i++)
        if (ftab[i] > ticks)
            return i - 1;
    return 0x0f;
}

//-------------------------------------------------------------------------------------定时器配置相关API

/**********************************************************************
函数名称：timerNVICConfigure
函数功能：定时器NVIC中断配置
函数形参：中断源
函数返回值：None
函数描述：None
**********************************************************************/
void timerNVICConfigure(uint8_t irq)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**********************************************************************
函数名称：configTimeBase
函数功能：配置定时器时基单元
函数形参：TIMx，周期值，hz
函数返回值：None
函数描述：None
**********************************************************************/
void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint32_t hz)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	// 用其默认值填充每个TIM_TimeBaseInitStruct成员
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);			   			
    TIM_TimeBaseStructure.TIM_Period = (period - 1) & 0xFFFF;  			// 自动重装载周期值
    TIM_TimeBaseStructure.TIM_Prescaler = (timerClock(tim) / hz) - 1;	// 预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;						// 时钟分割
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;			// 向上计数
    // 初始化定时器时基单元
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);						
}	

/**********************************************************************
函数名称：timerConfigure
函数功能：定时器配置
函数形参：定时器硬件配置信息，周期，频率
函数返回值：None
函数描述：None
**********************************************************************/
void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint32_t hz)
{
	// 配置定时器时基单元
    configTimeBase(timerHardwarePtr->tim, period, hz);
	// 使能定时器
    TIM_Cmd(timerHardwarePtr->tim, ENABLE);

	// 获取定时器输入捕获中断源
    uint8_t irq = timerInputIrq(timerHardwarePtr->tim);
	// 配置定时器NVIC中断
    timerNVICConfigure(irq);
    // 对需要它的定时器启用第二个IRQ
    switch (irq) {
	    case TIM1_CC_IRQn:	// TIM1
	        timerNVICConfigure(TIM1_UP_TIM10_IRQn);
	        break;
	    case TIM8_CC_IRQn:	// TIM8
	        timerNVICConfigure(TIM8_UP_TIM13_IRQn);
	        break;
    }
}

/**********************************************************************
函数名称：timerChConfigIC
函数功能：配置定时器通道输入捕获
函数形参：定时器硬件信息，polarityRising，inputFilterTicks
函数返回值：None
函数描述：None
**********************************************************************/
void timerChConfigIC(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;
	
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = timHw->channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarityRising ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = getFilter(inputFilterTicks);

    TIM_ICInit(timHw->tim, &TIM_ICInitStructure);
}

/**********************************************************************
函数名称：timerOCPreloadConfig
函数功能：定时器通道配置
函数形参：tim,channel,preload
函数返回值：None
函数描述：None
**********************************************************************/
void timerOCPreloadConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t preload)
{
    switch (channel) {
    case TIM_Channel_1:
        TIM_OC1PreloadConfig(tim, preload);   // 通道1
        break;
    case TIM_Channel_2:
        TIM_OC2PreloadConfig(tim, preload);	  // 通道2
        break;
    case TIM_Channel_3:
        TIM_OC3PreloadConfig(tim, preload);   // 通道3
        break;
    case TIM_Channel_4:
        TIM_OC4PreloadConfig(tim, preload);   // 通道4
        break;
    }
}

/**********************************************************************
函数名称：timerOCInit
函数功能：定时器通道初始化
函数形参：TIMx,通道,timerOCInit
函数返回值：None
函数描述：None
**********************************************************************/
void timerOCInit(TIM_TypeDef *tim, uint8_t channel, TIM_OCInitTypeDef *init)
{
    switch (channel) {
    case TIM_Channel_1:
        TIM_OC1Init(tim, init);   			  // 通道1
        break;
    case TIM_Channel_2:
        TIM_OC2Init(tim, init);   			  // 通道2
        break;
    case TIM_Channel_3:
        TIM_OC3Init(tim, init);   			  // 通道3
        break;
    case TIM_Channel_4:
        TIM_OC4Init(tim, init);   			  // 通道4
        break;
    }
}

/**********************************************************************
函数名称：timerInit
函数功能：定时器初始化
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void timerInit(void)
{
    memset(timerConfig, 0, sizeof (timerConfig));

    // 使能定时器外设时钟
    for (unsigned i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        RCC_ClockCmd(timerRCC(TIMER_HARDWARE[i].tim), ENABLE);
    }
    // 初始化定时器通道类型
    for (unsigned i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        timerChannelInfo[i].type = TYPE_FREE;
    }
	// 初始化定时器优先级
    for (unsigned i = 0; i < USED_TIMER_COUNT; i++) {
        timerInfo[i].priority = ~0;
    }
}

//-------------------------------------------------------------------------------------定时器中断相关API

/**********************************************************************
函数名称：timerChCCHandlerInit
函数功能：定时器通道中断服务回调函数初始化
函数形参：self，fn
函数返回值：None
函数描述：None
**********************************************************************/
void timerChCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn)
{
    self->fn = fn;
}

/**********************************************************************
函数名称：timerChOvrHandlerInit
函数功能：定时器溢出中断服务回调函数初始化
函数形参：self，fn
函数返回值：None
函数描述：None
**********************************************************************/
void timerChOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn)
{
    self->fn = fn;
    self->next = NULL;
}

/**********************************************************************
函数名称：timerChConfig_UpdateOverflow
函数功能：更新溢出回调列表
函数形参：cfg，tim
函数返回值：None
函数描述：
	一些同步机制是必要的，以避免干扰其他通道(BASEPRI现在使用)。
**********************************************************************/
static void timerChConfig_UpdateOverflow(timerConfig_t *cfg, TIM_TypeDef *tim) {
    timerOvrHandlerRec_t **chain = &cfg->overflowCallbackActive;
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        for (int i = 0; i < CC_CHANNELS_PER_TIMER; i++)
            if (cfg->overflowCallback[i]) {
                *chain = cfg->overflowCallback[i];
                chain = &cfg->overflowCallback[i]->next;
            }
			
        *chain = NULL;
    }
    // 使能或失能溢出中断
    TIM_ITConfig(tim, TIM_IT_Update, cfg->overflowCallbackActive ? ENABLE : DISABLE);
}

/**********************************************************************
函数名称：timerChConfigCallbacks
函数功能：设置通道的边沿和溢出中断回调,尽量避免溢出回调
函数形参：timHw，edgeCallback，overflowCallback
函数返回值：None
函数描述：None
**********************************************************************/
// 定义定时器中断源
#define TIM_IT_CCx(ch) (TIM_IT_CC1 << ((ch) / 4))
void timerChConfigCallbacks(const timerHardware_t *timHw, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
{
	// 获取定时器索引
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
	// 获取通道索引
    uint8_t channelIndex = lookupChannelIndex(timHw->channel);
	
	// 在更改回调为空之前失能通道中断
    if (edgeCallback == NULL)  
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel), DISABLE);
	
    // 设置回调信息
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallback;			// 通道回调			
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;	// 溢出回调
	
    // 重新使能通道中断
    if (edgeCallback)
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel), ENABLE);

	// 更新溢出回调列表
    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

/**********************************************************************
函数名称：timCCxHandler
函数功能：定时器中断处理
函数形参：TIMx，定时器配置信息
函数返回值：None
函数描述：None
**********************************************************************/
static void timCCxHandler(TIM_TypeDef *tim, timerConfig_t *timerConfig)
{
    uint16_t capture;
    unsigned tim_status;
	// DMA/中断使能寄存器 (TIMx_DIER)
    tim_status = tim->SR & tim->DIER;
	// 循环判断是否发生中断
    while (tim_status) {
		// 通过读取CCR来清除标记，确保正确调用handler
		// 当前的顺序是最高位优先，代码不应该依赖于特定的顺序(无论如何都会引入竞争条件)
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        tim->SR = mask;
        tim_status &= mask;
        switch (bit) {
            case __builtin_clz(TIM_IT_Update): {
                if (timerConfig->forcedOverflowTimerValue != 0) {
                    capture = timerConfig->forcedOverflowTimerValue - 1;
                    timerConfig->forcedOverflowTimerValue = 0;
                } else {
                    capture = tim->ARR;
                }
				// 获取定时器溢出回调是否激活
                timerOvrHandlerRec_t *cb = timerConfig->overflowCallbackActive;
                while (cb) {
					// 调用溢出中断回调函数
                    cb->fn(cb, capture);
					// NULL
                    cb = cb->next;
                }
                break;
            }
            case __builtin_clz(TIM_IT_CC1):
                timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim->CCR1);
                break;
            case __builtin_clz(TIM_IT_CC2):
                timerConfig->edgeCallback[1]->fn(timerConfig->edgeCallback[1], tim->CCR2);
                break;
            case __builtin_clz(TIM_IT_CC3):
                timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], tim->CCR3);
                break;
            case __builtin_clz(TIM_IT_CC4):
                timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], tim->CCR4);
                break;
        }
    }
}

// ---------------------------------------------------------定义中断服务函数宏
#define _TIM_IRQ_HANDLER(name, i)                                       \
    void name(void)                                                     \
    {                                                                   \
        timCCxHandler(TIM ## i, &timerConfig[TIMER_INDEX(i)]);          \
    } struct dummy
    
// 当两个定时器都需要检查状态位时，共享中断服务函数
#define _TIM_IRQ_HANDLER2(name, i, j)                                   \
    void name(void)                                                     \
    {                                                                   \
        timCCxHandler(TIM ## i, &timerConfig[TIMER_INDEX(i)]);          \
        timCCxHandler(TIM ## j, &timerConfig[TIMER_INDEX(j)]);          \
    } struct dummy


// ---------------------------------------------------------定义中断服务函数
#if USED_TIMERS & TIM_N(1)                                   // 高级定时器1和通用定时器10
_TIM_IRQ_HANDLER(TIM1_CC_IRQHandler, 1);
#if USED_TIMERS & TIM_N(10)
_TIM_IRQ_HANDLER2(TIM1_UP_TIM10_IRQHandler, 1, 10);   // 两个计时器都在使用
#else
_TIM_IRQ_HANDLER(TIM1_UP_TIM10_IRQHandler, 1);               // 不使用TIM10
#endif
#endif

#if USED_TIMERS & TIM_N(2)
_TIM_IRQ_HANDLER(TIM2_IRQHandler, 2);
#endif

#if USED_TIMERS & TIM_N(3)
_TIM_IRQ_HANDLER(TIM3_IRQHandler, 3);
#endif

#if USED_TIMERS & TIM_N(4)
_TIM_IRQ_HANDLER(TIM4_IRQHandler, 4);
#endif

#if USED_TIMERS & TIM_N(5)
_TIM_IRQ_HANDLER(TIM5_IRQHandler, 5);
#endif

#if USED_TIMERS & TIM_N(8)                                   // 高级定时器8和通用定时器13
_TIM_IRQ_HANDLER(TIM8_CC_IRQHandler, 8);
_TIM_IRQ_HANDLER(TIM8_UP_IRQHandler, 8);
#if USED_TIMERS & TIM_N(13)
_TIM_IRQ_HANDLER2(TIM8_UP_TIM13_IRQHandler, 8, 13);   // 两个计时器都在使用
#else
_TIM_IRQ_HANDLER(TIM8_UP_TIM13_IRQHandler, 8);                // 不使用TIM13
#endif
#endif

#if USED_TIMERS & TIM_N(9)
_TIM_IRQ_HANDLER(TIM1_BRK_TIM9_IRQHandler, 9);
#endif

#if USED_TIMERS & TIM_N(11)
_TIM_IRQ_HANDLER(TIM1_TRG_COM_TIM11_IRQHandler, 11);
#endif

#if USED_TIMERS & TIM_N(12)
_TIM_IRQ_HANDLER(TIM8_BRK_TIM12_IRQHandler, 12);
#endif

#if USED_TIMERS & TIM_N(14)
_TIM_IRQ_HANDLER(TIM8_TRG_COM_TIM14_IRQHandler, 14);
#endif

#if USED_TIMERS & TIM_N(15)
_TIM_IRQ_HANDLER(TIM1_BRK_TIM15_IRQHandler, 15);
#endif

#if USED_TIMERS & TIM_N(17)
_TIM_IRQ_HANDLER(TIM1_TRG_COM_TIM17_IRQHandler, 17);
#endif
#endif

