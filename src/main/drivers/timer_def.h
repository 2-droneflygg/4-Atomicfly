#pragma once

#include "platform.h"
#include "common/utils.h"

// ---------------------------------------------------------允许DMA相关成员的条件定义
#if defined(USE_TIMER_DMA)
# define DEF_TIM_DMA_COND(...)  __VA_ARGS__
#endif

// ---------------------------------------------------------获取定时器引脚标签宏
#if defined(USE_TIMER_MGMT)
#define TIMER_GET_IO_TAG(pin)    DEFIO_TAG_E(pin)
#endif

// ---------------------------------------------------------map至底道(由底道起的长条N);仅当通道N存在时有效
#define DEF_TIM_TCH2BTCH(timch)  CONCAT(B, timch)
#define BTCH_TIM1_CH1N  	BTCH_TIM1_CH1
#define BTCH_TIM1_CH2N  	BTCH_TIM1_CH2
#define BTCH_TIM1_CH3N  	BTCH_TIM1_CH3
#define BTCH_TIM8_CH1N  	BTCH_TIM8_CH1
#define BTCH_TIM8_CH2N  	BTCH_TIM8_CH2
#define BTCH_TIM8_CH3N  	BTCH_TIM8_CH3
#define BTCH_TIM20_CH1N 	BTCH_TIM20_CH1
#define BTCH_TIM20_CH2N 	BTCH_TIM20_CH2
#define BTCH_TIM20_CH3N 	BTCH_TIM20_CH3
#define BTCH_TIM13_CH1N 	BTCH_TIM13_CH1
#define BTCH_TIM14_CH1N 	BTCH_TIM14_CH1
#define BTCH_TIM15_CH1N 	BTCH_TIM15_CH1
#define BTCH_TIM16_CH1N 	BTCH_TIM16_CH1
#define BTCH_TIM17_CH1N 	BTCH_TIM17_CH1

// ---------------------------------------------------------通道表 - D(chan n, n型)
#define DEF_TIM_CH_GET(ch) CONCAT2(DEF_TIM_CH__, ch)
#define DEF_TIM_CH__CH_CH1  	D(1, 0)
#define DEF_TIM_CH__CH_CH2  	D(2, 0)
#define DEF_TIM_CH__CH_CH3  	D(3, 0)
#define DEF_TIM_CH__CH_CH4  	D(4, 0)
#define DEF_TIM_CH__CH_CH1N 	D(1, 1)
#define DEF_TIM_CH__CH_CH2N 	D(2, 1)
#define DEF_TIM_CH__CH_CH3N 	D(3, 1)

// ---------------------------------------------------------定时器表 - D(tim n)
#define DEF_TIM_TIM_GET(tim) CONCAT2(DEF_TIM_TIM__, tim)
#define DEF_TIM_TIM__TIM_TIM1   D(1)
#define DEF_TIM_TIM__TIM_TIM2   D(2)
#define DEF_TIM_TIM__TIM_TIM3   D(3)
#define DEF_TIM_TIM__TIM_TIM4   D(4)
#define DEF_TIM_TIM__TIM_TIM5   D(5)
#define DEF_TIM_TIM__TIM_TIM6   D(6)
#define DEF_TIM_TIM__TIM_TIM7   D(7)
#define DEF_TIM_TIM__TIM_TIM8   D(8)
#define DEF_TIM_TIM__TIM_TIM9   D(9)
#define DEF_TIM_TIM__TIM_TIM10  D(10)
#define DEF_TIM_TIM__TIM_TIM11  D(11)
#define DEF_TIM_TIM__TIM_TIM12  D(12)
#define DEF_TIM_TIM__TIM_TIM13  D(13)
#define DEF_TIM_TIM__TIM_TIM14  D(14)
#define DEF_TIM_TIM__TIM_TIM15  D(15)
#define DEF_TIM_TIM__TIM_TIM16  D(16)
#define DEF_TIM_TIM__TIM_TIM17  D(17)
#define DEF_TIM_TIM__TIM_TIM18  D(18)
#define DEF_TIM_TIM__TIM_TIM19  D(19)
#define DEF_TIM_TIM__TIM_TIM20  D(20)
#define DEF_TIM_TIM__TIM_TIM21  D(21)
#define DEF_TIM_TIM__TIM_TIM22  D(22)

// ---------------------------------------------------------从DMA表中获取记录
// TIMx通道y的DMA表项，有两个变量:
// #define DEF_TIM_DMA__BTCH_TIMx_CHy D(var0)，D(var1)
// D(…)中的参数是特定于目标的
// 没有DMA的通道的DMA表
// #define DEF_TIM_DMA__BTCH_TIMx_CHy无
// N通道首先转换为相应的基通道
// 创建访问器宏并从table中调用它
// DMA_VARIANT_MISSING用于满足变量参数(-Wpedantic)并获得更好的错误消息(未定义的符号而不是预处理器错误)
#define DEF_TIM_DMA_GET(variant, timch) PP_CALL(CONCAT(DEF_TIM_DMA_GET_VARIANT__, variant), CONCAT(DEF_TIM_DMA__, DEF_TIM_TCH2BTCH(timch)), DMA_VARIANT_MISSING, DMA_VARIANT_MISSING, ERROR)
#define DEF_TIM_DMA_GET_VARIANT__0(_0, ...) _0
#define DEF_TIM_DMA_GET_VARIANT__1(_0, _1, ...) _1
#define DEF_TIM_DMA_GET_VARIANT__2(_0, _1, _2, ...) _2
#define DEF_TIM_DMA_GET_VARIANT__3(_0, _1, _2, _3, ...) _3
#define DEF_TIM_DMA_GET_VARIANT__4(_0, _1, _2, _3, _4, ...) _4
#define DEF_TIM_DMA_GET_VARIANT__5(_0, _1, _2, _3, _4, _5, ...) _5
#define DEF_TIM_DMA_GET_VARIANT__6(_0, _1, _2, _3, _4, _5, _6, ...) _6
#define DEF_TIM_DMA_GET_VARIANT__7(_0, _1, _2, _3, _4, _5, _6, _7, ...) _7
#define DEF_TIM_DMA_GET_VARIANT__8(_0, _1, _2, _3, _4, _5, _6, _7, _8, ...) _8
#define DEF_TIM_DMA_GET_VARIANT__9(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, ...) _9
#define DEF_TIM_DMA_GET_VARIANT__10(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, ...) _10
#define DEF_TIM_DMA_GET_VARIANT__11(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, ...) _11
#define DEF_TIM_DMA_GET_VARIANT__12(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, ...) _12
#define DEF_TIM_DMA_GET_VARIANT__13(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, ...) _13
#define DEF_TIM_DMA_GET_VARIANT__14(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, ...) _14
#define DEF_TIM_DMA_GET_VARIANT__15(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, ...) _15

// ---------------------------------------------------------从AF表中获取记录
// D(…)中的参数是特定于目标的
#define DEF_TIM_AF_GET(timch, pin) CONCAT4(DEF_TIM_AF__, pin, __, timch)

// ---------------------------------------------------------定义输出类型(n通道)
#define DEF_TIM_OUTPUT(ch)         CONCAT(DEF_TIM_OUTPUT__, DEF_TIM_CH_GET(ch))
#define DEF_TIM_OUTPUT__D(chan_n, n_channel) PP_IIF(n_channel, TIMER_OUTPUT_N_CHANNEL, TIMER_OUTPUT_NONE)

// ---------------------------------------------------------定时器硬件配置宏 - 定时器，通道，引脚，标志，输出，DMA选项
#define DEF_TIM(tim, chan, pin, flags, out, dmaopt) {           \
    tim,                                                        \
    TIMER_GET_IO_TAG(pin),                                      \
    DEF_TIM_CHANNEL(CH_ ## chan),                               \
    flags,                                                      \
    (DEF_TIM_OUTPUT(CH_ ## chan) | out),                        \
    DEF_TIM_AF(TIM_ ## tim)                                     \
    DEF_TIM_DMA_COND(/* add comma */ ,                          \
        DEF_TIM_DMA_STREAM(dmaopt, TCH_## tim ## _ ## chan),    \
        DEF_TIM_DMA_CHANNEL(dmaopt, TCH_## tim ## _ ## chan)    \
    )                                                           \
    DEF_TIM_DMA_COND(/* add comma */ ,                          \
        DEF_TIM_DMA_STREAM(0, TCH_## tim ## _UP),               \
        DEF_TIM_DMA_CHANNEL(0, TCH_## tim ## _UP),              \
        DEF_TIM_DMA_HANDLER(0, TCH_## tim ## _UP)               \
    )                                                           \
}                                                               \
/**/

// ---------------------------------------------------------定时器通道定义宏
#define DEF_TIM_CHANNEL(ch)                   				CONCAT(DEF_TIM_CHANNEL__, DEF_TIM_CH_GET(ch))
#define DEF_TIM_CHANNEL__D(chan_n, n_channel) 				TIM_Channel_ ## chan_n

// ---------------------------------------------------------定时器复用宏
#define DEF_TIM_AF(tim)                       				CONCAT(DEF_TIM_AF__, DEF_TIM_TIM_GET(tim))
#define DEF_TIM_AF__D(tim_n)                  				GPIO_AF_TIM ## tim_n

// ---------------------------------------------------------定时器通道DMA宏
#define DEF_TIM_DMA_CHANNEL(variant, timch)  			    CONCAT(DEF_TIM_DMA_CHANNEL__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_CHANNEL__D(dma_n, stream_n, chan_n)     DMA_Channel_ ## chan_n
#define DEF_TIM_DMA_CHANNEL__NONE                           DMA_Channel_0

// ---------------------------------------------------------定时器DMA流宏
#define DEF_TIM_DMA_STREAM(variant, timch)   			    CONCAT(DEF_TIM_DMA_STREAM__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_STREAM__D(dma_n, stream_n, chan_n)  	(dmaResource_t *)DMA ## dma_n ## _Stream ## stream_n
#define DEF_TIM_DMA_STREAM__NONE                        	NULL

// ---------------------------------------------------------定时器DMA中断服务函数宏
#define DEF_TIM_DMA_HANDLER(variant, timch)   				CONCAT(DEF_TIM_DMA_HANDLER__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_HANDLER__D(dma_n, stream_n, chan_n) 	DMA ## dma_n ## _ST ## stream_n ## _HANDLER
#define DEF_TIM_DMA_HANDLER__NONE                       	0

// ---------------------------------------------------------F4定时器DMA流映射 - D(DMAx, Stream, Channel) 
#define DEF_TIM_DMA__BTCH_TIM1_CH1    D(2, 6, 0),D(2, 1, 6),D(2, 3, 6)
#define DEF_TIM_DMA__BTCH_TIM1_CH2    D(2, 6, 0),D(2, 2, 6)
#define DEF_TIM_DMA__BTCH_TIM1_CH3    D(2, 6, 0),D(2, 6, 6)
#define DEF_TIM_DMA__BTCH_TIM1_CH4    D(2, 4, 6)
#define DEF_TIM_DMA__BTCH_TIM2_CH1    D(1, 5, 3)
#define DEF_TIM_DMA__BTCH_TIM2_CH2    D(1, 6, 3)
#define DEF_TIM_DMA__BTCH_TIM2_CH3    D(1, 1, 3)
#define DEF_TIM_DMA__BTCH_TIM2_CH4    D(1, 7, 3),D(1, 6, 3)
#define DEF_TIM_DMA__BTCH_TIM3_CH1    D(1, 4, 5)
#define DEF_TIM_DMA__BTCH_TIM3_CH2    D(1, 5, 5)
#define DEF_TIM_DMA__BTCH_TIM3_CH3    D(1, 7, 5)
#define DEF_TIM_DMA__BTCH_TIM3_CH4    D(1, 2, 5)
#define DEF_TIM_DMA__BTCH_TIM4_CH1    D(1, 0, 2)
#define DEF_TIM_DMA__BTCH_TIM4_CH2    D(1, 3, 2)
#define DEF_TIM_DMA__BTCH_TIM4_CH3    D(1, 7, 2)
#define DEF_TIM_DMA__BTCH_TIM5_CH1    D(1, 2, 6)
#define DEF_TIM_DMA__BTCH_TIM5_CH2    D(1, 4, 6)
#define DEF_TIM_DMA__BTCH_TIM5_CH3    D(1, 0, 6)
#define DEF_TIM_DMA__BTCH_TIM5_CH4    D(1, 1, 6),D(1, 3, 6)
#define DEF_TIM_DMA__BTCH_TIM8_CH1    D(2, 2, 0),D(2, 2, 7)
#define DEF_TIM_DMA__BTCH_TIM8_CH2    D(2, 2, 0),D(2, 3, 7)
#define DEF_TIM_DMA__BTCH_TIM8_CH3    D(2, 2, 0),D(2, 4, 7)
#define DEF_TIM_DMA__BTCH_TIM8_CH4    D(2, 7, 7)
#define DEF_TIM_DMA__BTCH_TIM4_CH4    NONE
#define DEF_TIM_DMA__BTCH_TIM9_CH1    NONE
#define DEF_TIM_DMA__BTCH_TIM9_CH2    NONE
#define DEF_TIM_DMA__BTCH_TIM10_CH1   NONE
#define DEF_TIM_DMA__BTCH_TIM11_CH1   NONE
#define DEF_TIM_DMA__BTCH_TIM12_CH1   NONE
#define DEF_TIM_DMA__BTCH_TIM12_CH2   NONE
#define DEF_TIM_DMA__BTCH_TIM13_CH1   NONE
#define DEF_TIM_DMA__BTCH_TIM14_CH1   NONE

// ---------------------------------------------------------TIM_UP表
#define DEF_TIM_DMA__BTCH_TIM1_UP     D(2, 5, 6)
#define DEF_TIM_DMA__BTCH_TIM2_UP     D(1, 7, 3)
#define DEF_TIM_DMA__BTCH_TIM3_UP     D(1, 2, 5)
#define DEF_TIM_DMA__BTCH_TIM4_UP     D(1, 6, 2)
#define DEF_TIM_DMA__BTCH_TIM5_UP     D(1, 0, 6)
#define DEF_TIM_DMA__BTCH_TIM6_UP     D(1, 1, 7)
#define DEF_TIM_DMA__BTCH_TIM7_UP     D(1, 4, 1)
#define DEF_TIM_DMA__BTCH_TIM8_UP     D(2, 1, 7)
#define DEF_TIM_DMA__BTCH_TIM9_UP     NONE
#define DEF_TIM_DMA__BTCH_TIM10_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM11_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM12_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM13_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM14_UP    NONE

