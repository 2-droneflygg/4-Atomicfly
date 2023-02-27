/*********************************************************************************
 提供一系列Dshot_PWM输出操作相关API：
 	加载数据包 -> DSHOT_DMA缓存（准备发送数据包[定时器PWM占空比]）。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_DSHOT
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "stm32f4xx.h"
#include "drivers/dshot.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "pwm_output_dshot_shared.h"

// ---------------------------------------------------------电机定时器DMA数量
FAST_RAM_ZERO_INIT uint8_t dmaMotorTimerCount = 0;
// ---------------------------------------------------------电机定时器DMA信息块
motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
// ---------------------------------------------------------电机DMA输出信息块
motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];		

/**********************************************************************
函数名称：getMotorDmaOutput
函数功能：获取电机DMA输出信息块
函数形参：电机索引
函数返回值：电机DMA输出结构体
函数描述：None
**********************************************************************/
motorDmaOutput_t *getMotorDmaOutput(uint8_t index)
{
    return &dmaMotors[index];
}

/**********************************************************************
函数名称：getTimerIndex
函数功能：获取定时器索引
函数形参：TIMx
函数返回值：dmaMotorTimerCount - 1
函数描述：None
**********************************************************************/
uint8_t getTimerIndex(TIM_TypeDef *timer)
{
	// 遍历所有电机定时器DMA信息
    for (int i = 0; i < dmaMotorTimerCount; i++) {
        if (dmaMotorTimers[i].timer == timer) {
            return i;
        }
    }
	// 信息块没有则追加注册
    dmaMotorTimers[dmaMotorTimerCount++].timer = timer;
    return dmaMotorTimerCount - 1;
}

/**********************************************************************
函数名称：pwmWriteDshotInt
函数功能：PWM写DSHOT命令（int）
函数形参：index，value
函数返回值：None
函数描述：None
**********************************************************************/
void pwmWriteDshotInt(uint8_t index, uint16_t value)
{
	// 获取电机DMA输出信息
    motorDmaOutput_t *const motor = &dmaMotors[index];
	// 检查合法性
    if (!motor->configured) {
        return;
    }
    // 获取Dshot命令是否正在处理
    if (dshotCommandIsProcessing()) {
		// 获取当前Dshot命令
        value = dshotCommandGetCurrent(index);
        if (value) {
			// --------使能请求遥测位
            motor->protocolControl.requestTelemetry = true;
        }
    }
	// --------获取油门数据
    motor->protocolControl.value = value;
	// 准备Dshot数据包
    uint16_t packet = prepareDshotPacket(&motor->protocolControl);

	// 加载数据包 -> DSHOT_DMA缓存（准备发送数据包）
    uint8_t bufferSize;
#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        bufferSize = loadDmaBuffer(&motor->timer->dmaBurstBuffer[timerLookupChannelIndex(motor->timerHardware->channel)], 4, packet);
        motor->timer->dmaBurstLength = bufferSize * 4;
    } else
#endif
    {
        bufferSize = loadDmaBuffer(motor->dmaBuffer, 1, packet);
        motor->timer->timerDmaSources |= motor->timerDmaSource;
        xDMA_SetCurrDataCounter(motor->dmaRef, bufferSize);
        xDMA_Cmd(motor->dmaRef, ENABLE);
    }
}
#endif // USE_DSHOT

