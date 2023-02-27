/*********************************************************************************
 提供定时器硬件信息配置以及获取定时器时钟API。
*********************************************************************************/
#include "platform.h"

#ifdef USE_TIMER
#include "common/utils.h"

#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/timer_def.h"

#include "stm32f4xx.h"
#include "rcc.h"
#include "timer.h"


// ---------------------------------------------------------定时器定义 - 14个
const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = TIM1,  .rcc = RCC_APB2(TIM1),  .inputIrq = TIM1_CC_IRQn},
    { .TIMx = TIM2,  .rcc = RCC_APB1(TIM2),  .inputIrq = TIM2_IRQn},
    { .TIMx = TIM3,  .rcc = RCC_APB1(TIM3),  .inputIrq = TIM3_IRQn},
    { .TIMx = TIM4,  .rcc = RCC_APB1(TIM4),  .inputIrq = TIM4_IRQn},
    { .TIMx = TIM5,  .rcc = RCC_APB1(TIM5),  .inputIrq = TIM5_IRQn},
    { .TIMx = TIM6,  .rcc = RCC_APB1(TIM6),  .inputIrq = 0},
    { .TIMx = TIM7,  .rcc = RCC_APB1(TIM7),  .inputIrq = 0},
    { .TIMx = TIM8,  .rcc = RCC_APB2(TIM8),  .inputIrq = TIM8_CC_IRQn},
    { .TIMx = TIM9,  .rcc = RCC_APB2(TIM9),  .inputIrq = TIM1_BRK_TIM9_IRQn},
    { .TIMx = TIM10, .rcc = RCC_APB2(TIM10), .inputIrq = TIM1_UP_TIM10_IRQn},
    { .TIMx = TIM11, .rcc = RCC_APB2(TIM11), .inputIrq = TIM1_TRG_COM_TIM11_IRQn},
    { .TIMx = TIM12, .rcc = RCC_APB1(TIM12), .inputIrq = TIM8_BRK_TIM12_IRQn},
    { .TIMx = TIM13, .rcc = RCC_APB1(TIM13), .inputIrq = TIM8_UP_TIM13_IRQn},
    { .TIMx = TIM14, .rcc = RCC_APB1(TIM14), .inputIrq = TIM8_TRG_COM_TIM14_IRQn},
};

// ---------------------------------------------------------完整的定时器硬件信息配置
#if defined(USE_TIMER_MGMT)
const timerHardware_t fullTimerHardware[FULL_TIMER_CHANNEL_COUNT] = {
    // 从'timer def.h'自动生成
//PORTA
    DEF_TIM(TIM2, CH1, PA0, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM2, CH2, PA1, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM2, CH3, PA2, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM2, CH4, PA3, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM2, CH1, PA5, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH1N, PA7, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH1, PA8, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH2, PA9, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH3, PA10, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH1N, PA11, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM2, CH1, PA15, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM5, CH1, PA0, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM5, CH2, PA1, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM5, CH3, PA2, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM5, CH4, PA3, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM3, CH1, PA6, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM3, CH2, PA7, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM9, CH1, PA2, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM9, CH2, PA3, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH1N, PA5, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH1N, PA7, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM13, CH1, PA6, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM14, CH1, PA7, TIM_USE_ANY, 0, 0),

//PORTB
    DEF_TIM(TIM1, CH2N, PB0, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH3N, PB1, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM2, CH2, PB3, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM2, CH3, PB10, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM2, CH4, PB11, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH1N, PB13, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH2N, PB14, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH3N, PB15, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM3, CH3, PB0, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM3, CH4, PB1, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM3, CH1, PB4, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM3, CH2, PB5, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM4, CH1, PB6, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM4, CH2, PB7, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM4, CH3, PB8, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM4, CH4, PB9, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH2N, PB0, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH3N, PB1, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM10, CH1, PB8, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM11, CH1, PB9, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH2N, PB14, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH3N, PB15, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM12, CH1, PB14, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM12, CH2, PB15, TIM_USE_ANY, 0, 0),
    
//PORTC
    DEF_TIM(TIM3, CH1, PC6, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM3, CH2, PC7, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM3, CH3, PC8, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM3, CH4, PC9, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH1, PC6, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH2, PC7, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH3, PC8, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH4, PC9, TIM_USE_ANY, 0, 0),

//PORTD
    DEF_TIM(TIM4, CH1, PD12, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM4, CH2, PD13, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM4, CH3, PD14, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM4, CH4, PD15, TIM_USE_ANY, 0, 0),

//PORTE
    DEF_TIM(TIM1, CH1N, PE8, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH1, PE9, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH2N, PE10, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH2, PE11, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH3N, PE12, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH3, PE13, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH4, PE14, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM9, CH1, PE5, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM9, CH2, PE6, TIM_USE_ANY, 0, 0),

//PORTF
    DEF_TIM(TIM10, CH1, PF6, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM11, CH1, PF7, TIM_USE_ANY, 0, 0),
};
#endif

/**********************************************************************
函数名称：timerClock
函数功能：获取定时器时钟
函数形参：TIMx
函数返回值：TIM_Prescaler
函数描述：
	需要从dma和定时器到引脚的映射，这些值都应该在这里设置为dmaMotors数组。
	只有某些引脚有OC输出(已经用于正常的PWM等)，但是对于特定的定时器和通道，只有特定的DMA数据流流/通道可用。
    DMA1
    Channel Stream0     Stream1     Stream2     Stream3     Stream4     Stream5     Stream6     Stream7
    0
    1
    2       TIM4_CH1                            TIM4_CH2                                        TIM4_CH3
    3                   TIM2_CH3                                        TIM2_CH1    TIM2_CH1    TIM2_CH4
                                                                                    TIM2_CH4
    4
    5                               TIM3_CH4                TIM3_CH1    TIM3_CH2                TIM3_CH3
    6       TIM5_CH3    TIM5_CH4    TIM5_CH1    TIM5_CH4    TIM5_CH2
    7

    DMA2
    Channel Stream0     Stream1     Stream2     Stream3     Stream4     Stream5     Stream6     Stream7
    0                               TIM8_CH1                                        TIM1_CH1
                                    TIM8_CH2                                        TIM1_CH2
                                    TIM8_CH3                                        TIM1_CH3
    1
    2
    3
    4
    5
    6       TIM1_CH1    TIM1_CH2    TIM1_CH1                TIM1_CH4                TIM1_CH3
    7                   TIM8_CH1    TIM8_CH2    TIM8_CH3                                        TIM8_CH4
**********************************************************************/
uint32_t timerClock(TIM_TypeDef *tim)
{
    if (tim == TIM8 || tim == TIM1 || tim == TIM9 || tim == TIM10 || tim == TIM11) {
        return SystemCoreClock;
    } else {
        return SystemCoreClock / 2;
    }
}
#endif

