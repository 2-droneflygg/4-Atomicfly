/**********************************************************************
 定时器默认配置：
**********************************************************************/
#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

// ---------------------------------------------------------------------------------定时器硬件配置
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM10, CH1, PB8,  TIM_USE_NONE,       0, 0), // PPM
    DEF_TIM(TIM4,  CH4, PB9,  TIM_USE_NONE,       0, 0), // S2_IN

    DEF_TIM(TIM8,  CH1, PC6,  TIM_USE_NONE,       0, 0), // S3_IN, UART6_TX
    DEF_TIM(TIM8,  CH2, PC7,  TIM_USE_NONE,       0, 0), // S4_IN, UART6_RX
    DEF_TIM(TIM8,  CH3, PC8,  TIM_USE_NONE,       0, 0), // S5_IN
    DEF_TIM(TIM8,  CH4, PC9,  TIM_USE_NONE,       0, 0), // S6_IN

    DEF_TIM(TIM3,  CH1, PC6,  TIM_USE_MOTOR,      0, 0), // MOTOR 1
    DEF_TIM(TIM3,  CH2, PC7,  TIM_USE_MOTOR,      0, 0), // MOTOR 2
    DEF_TIM(TIM3,  CH3, PC8,  TIM_USE_MOTOR,      0, 0), // MOTOR 3
    DEF_TIM(TIM3,  CH4, PC9,  TIM_USE_MOTOR,      0, 0), // MOTOR 4

    DEF_TIM(TIM5,  CH2, PA1,  TIM_USE_NONE,       0, 0), // MOTOR 5
    DEF_TIM(TIM4,  CH1, PB6,  TIM_USE_NONE,       0, 0), // 空
 
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_NONE,       0, 0), // MOTOR 6
    
    DEF_TIM(TIM1,  CH2, PA9,  TIM_USE_NONE,       0, 0), // UART1_TX
    DEF_TIM(TIM1,  CH3, PA10, TIM_USE_NONE,       0, 0), // UART1_RX
};
	
