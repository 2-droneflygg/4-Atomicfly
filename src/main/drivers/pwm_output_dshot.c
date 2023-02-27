/*********************************************************************************
 提供一系列Dshot_PWM输出底层驱动配置相关API。
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
#include "drivers/system.h"
#include "stm32f4xx.h"
#include "drivers/dshot.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/dshot_command.h"
#include "pwm_output_dshot_shared.h"

/**********************************************************************
函数名称：pwmDshotSetDirectionOutput
函数功能：配置PWM_Dshot定向输出
函数形参：电机DMA输出信息，定时器通道，DMA
函数返回值：None
函数描述：None
**********************************************************************/
void pwmDshotSetDirectionOutput(motorDmaOutput_t * const motor, TIM_OCInitTypeDef *pOcInit, DMA_InitTypeDef* pDmaInit)
{
	// 获取定时器硬件信息
    const timerHardware_t * const timerHardware = motor->timerHardware;
    TIM_TypeDef *timer = timerHardware->tim;
	// DMA控制器_数据流
    dmaResource_t *dmaRef = motor->dmaRef;
	// 定时器上溢DMA配置（DMA控制器_数据流）
#if defined(USE_DSHOT_DMAR) 
    if (useBurstDshot) {
        dmaRef = timerHardware->dmaTimUPRef;
    }
#endif
	// DMA重置为默认配置
    xDMA_DeInit(dmaRef);
	// 定时器通道配置 - 失能
    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Disable);
	// 定时器通道初始化
    timerOCInit(timer, timerHardware->channel, pOcInit);
	// 定时器通道配置 - 使能
    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Enable);
	// DMA传输方向 - 内存到外设
#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        pDmaInit->DMA_DIR = DMA_DIR_MemoryToPeripheral;
    } else
#endif
    {
        pDmaInit->DMA_DIR = DMA_DIR_MemoryToPeripheral;
    }
	// DMA初始化
    xDMA_Init(dmaRef, pDmaInit);
	// 使能DMA传输完成中断
    xDMA_ITConfig(dmaRef, DMA_IT_TC, ENABLE);
}

/**********************************************************************
函数名称：pwmCompleteDshotMotorUpdate
函数功能：完成PWM_Dshot电机更新
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void pwmCompleteDshotMotorUpdate(void)
{
    // 获取Dshot命令队列是否为空
    if (!dshotCommandQueueEmpty()) {
		// 获取是否使能Dshot命令输出
        if (!dshotCommandOutputIsEnabled(dshotPwmDevice.count)) {
            return;
        }
    }
	// 遍历所有电机定时器DMA
    for (int i = 0; i < dmaMotorTimerCount; i++) {
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
			// 写入DMA数据流要传输的数据单元数
            xDMA_SetCurrDataCounter(dmaMotorTimers[i].dmaBurstRef, dmaMotorTimers[i].dmaBurstLength);
			// 使能DMA传输
            xDMA_Cmd(dmaMotorTimers[i].dmaBurstRef, ENABLE);
			// 配置定时器DMA - 4个字节
            TIM_DMAConfig(dmaMotorTimers[i].timer, TIM_DMABase_CCR1, TIM_DMABurstLength_4Transfers);
			// 使能定时器更新DMA
            TIM_DMACmd(dmaMotorTimers[i].timer, TIM_DMA_Update, ENABLE);
        } else
#endif
        {
        	// 禁用自动重载寄存器
            TIM_ARRPreloadConfig(dmaMotorTimers[i].timer, DISABLE);
			// 配置重装载值
            dmaMotorTimers[i].timer->ARR = dmaMotorTimers[i].outputPeriod;
			// 使能自动重载寄存器
            TIM_ARRPreloadConfig(dmaMotorTimers[i].timer, ENABLE);
			// 配置计数器值为0
            TIM_SetCounter(dmaMotorTimers[i].timer, 0);
			// 使能定时器DMA
            TIM_DMACmd(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources, ENABLE);
			// 定时器DMA源
            dmaMotorTimers[i].timerDmaSources = 0;
        }
    }
}

/**********************************************************************
函数名称：motor_DMA_IRQHandler
函数功能：电机DMA中断处理
函数形参：描述符
函数返回值：None
函数描述：None
**********************************************************************/
static void motor_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
	// 判断是否传输完成
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
		// 获取电机定时器DMA信息
        motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
#ifdef USE_DSHOT_DMAR
		// 传输完成则禁用DMA
        if (useBurstDshot) {
            xDMA_Cmd(motor->timerHardware->dmaTimUPRef, DISABLE);
            TIM_DMACmd(motor->timerHardware->tim, TIM_DMA_Update, DISABLE);
        } else
#endif
        {
            xDMA_Cmd(motor->dmaRef, DISABLE);
            TIM_DMACmd(motor->timerHardware->tim, motor->timerDmaSource, DISABLE);
        }
		// 清除传输完成标志位
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

/**********************************************************************
函数名称：pwmDshotMotorHardwareConfig
函数功能：PWM - Dshot硬件配置
函数形参：定时器硬件信息，电机索引，电机PWM协议，输出选项
函数返回值：None
函数描述：None
**********************************************************************/
bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, motorPwmProtocolTypes_e pwmProtocolType, uint8_t output)
{
    TIM_OCInitTypeDef ocInitStruct;
    DMA_InitTypeDef   dmaInitStruct;
	#define OCINIT ocInitStruct
	#define DMAINIT dmaInitStruct
    dmaResource_t *dmaRef = NULL;
    uint32_t dmaChannel = 0;

	// 获取DMA信息块
#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);
    if (dmaSpec != NULL) {
        dmaRef = dmaSpec->ref;
        dmaChannel = dmaSpec->channel;
    }
#endif
#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
		// 定时器上溢DMA配置（DMA控制器_数据流）
        dmaRef = timerHardware->dmaTimUPRef;
		// 定时器上溢DMA通道
        dmaChannel = timerHardware->dmaTimUPChannel;
    }
#endif
	// 检查DMA控制器_数据流合法性
    if (dmaRef == NULL) {
        return false;
    }

	// 获取电机DMA输出信息
    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];
	// TIMx
    TIM_TypeDef *timer = timerHardware->tim;

	// 当同一个定时器的不同通道被依次处理时，configureTimer始终为true，导致定时器和相关的DMA初始化不止一次
	// 为了解决这个问题，getTimerIndex必须被扩展，以返回一个新的计时器被请求
    const uint8_t timerIndex = getTimerIndex(timer);
    const bool configureTimer = (timerIndex == dmaMotorTimerCount-1);

	// 注册电机DMA输出相关信息
    motor->timer = &dmaMotorTimers[timerIndex];
    motor->index = motorIndex;
    motor->timerHardware = timerHardware;

	// 获取电机IO引脚
    const IO_t motorIO = IOGetByTag(timerHardware->tag);
	// 上下拉配置
    uint8_t pupMode = 0;
    pupMode = (output & TIMER_OUTPUT_INVERTED) ? GPIO_PuPd_DOWN : GPIO_PuPd_UP;
	// 配置IO
    motor->iocfg = IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, pupMode);
	// 复用IO
    IOConfigGPIOAF(motorIO, motor->iocfg, timerHardware->alternateFunction);

    if (configureTimer) {
        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
        TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		// 使能时钟
        RCC_ClockCmd(timerRCC(timer), ENABLE);
		// 失能定时器
        TIM_Cmd(timer, DISABLE);
		// 配置定时器时基
        TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(lrintf((float) timerClock(timer) / getDshotHz(pwmProtocolType) + 0.01f) - 1);
        TIM_TimeBaseStructure.TIM_Period = (MOTOR_BITLENGTH) - 1;
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		// 初始化定时器
        TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);
    }

	// 配置定时器通道
    TIM_OCStructInit(&OCINIT);
    OCINIT.TIM_OCMode = TIM_OCMode_PWM1;
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        OCINIT.TIM_OutputNState = TIM_OutputNState_Enable;
        OCINIT.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
        OCINIT.TIM_OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_Low : TIM_OCNPolarity_High;
    } else {
        OCINIT.TIM_OutputState = TIM_OutputState_Enable;
        OCINIT.TIM_OCIdleState = TIM_OCIdleState_Set;
        OCINIT.TIM_OCPolarity =  (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
    }
    OCINIT.TIM_Pulse = 0;

	// 注册电机DMA输出相关信息
#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->dmaBurstRef = dmaRef;
    } else
#endif
    {
        motor->timerDmaSource = timerDmaSource(timerHardware->channel);
        motor->timer->timerDmaSources &= ~motor->timerDmaSource;
    }

	// 失能DMA
    xDMA_Cmd(dmaRef, DISABLE);
    xDMA_DeInit(dmaRef);
    DMA_StructInit(&DMAINIT);
#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
		// DMA初始化
        dmaInit(timerHardware->dmaTimUPIrqHandler, OWNER_TIMUP, timerGetTIMNumber(timerHardware->tim));
        motor->timer->dmaBurstBuffer = &dshotBurstDmaBuffer[timerIndex][0];
        DMAINIT.DMA_Channel = timerHardware->dmaTimUPChannel;
        DMAINIT.DMA_Memory0BaseAddr = (uint32_t)motor->timer->dmaBurstBuffer;
        DMAINIT.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMAINIT.DMA_FIFOMode = DMA_FIFOMode_Enable;
        DMAINIT.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
        DMAINIT.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMAINIT.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMAINIT.DMA_PeripheralBaseAddr = (uint32_t)&timerHardware->tim->DMAR;
        DMAINIT.DMA_BufferSize = DSHOT_DMA_BUFFER_SIZE; 
        DMAINIT.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMAINIT.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMAINIT.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
        DMAINIT.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        DMAINIT.DMA_Mode = DMA_Mode_Normal;
        DMAINIT.DMA_Priority = DMA_Priority_High;
    } else
#endif
    {
        dmaInit(dmaGetIdentifier(dmaRef), OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
        motor->dmaBuffer = &dshotDmaBuffer[motorIndex][0];
        DMAINIT.DMA_Channel = dmaChannel;
        DMAINIT.DMA_Memory0BaseAddr = (uint32_t)motor->dmaBuffer;
        DMAINIT.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMAINIT.DMA_FIFOMode = DMA_FIFOMode_Enable;
        DMAINIT.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMAINIT.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMAINIT.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMAINIT.DMA_PeripheralBaseAddr = (uint32_t)timerChCCR(timerHardware);
        DMAINIT.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMAINIT.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMAINIT.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
        DMAINIT.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        DMAINIT.DMA_Mode = DMA_Mode_Normal;
        DMAINIT.DMA_Priority = DMA_Priority_High;
    }

    // 注册DMA控制器_数据流
    motor->dmaRef = dmaRef;

	// 配置PWM_Dshot定向输出
    pwmDshotSetDirectionOutput(motor, &OCINIT, &DMAINIT);
	
#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaSetHandler(timerHardware->dmaTimUPIrqHandler, motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, motor->index);
    } else
#endif
    {
        dmaSetHandler(dmaGetIdentifier(dmaRef), motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, motor->index);
    }

    TIM_Cmd(timer, ENABLE);
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_CCxNCmd(timer, timerHardware->channel, TIM_CCxN_Enable);
    } else {
        TIM_CCxCmd(timer, timerHardware->channel, TIM_CCx_Enable);
    }
    if (configureTimer) {
        TIM_ARRPreloadConfig(timer, ENABLE);
        TIM_CtrlPWMOutputs(timer, ENABLE);
        TIM_Cmd(timer, ENABLE);
    }
    motor->configured = true;

    return true;
}
#endif

