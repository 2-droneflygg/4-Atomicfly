/*********************************************************************************
 提供串口重新配置和尝试开始串口DMA发送API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART
#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"


/**********************************************************************
函数名称：uartReconfigure
函数功能：重新配置串口
函数形参：串口配置
函数返回值：None
函数描述：None
**********************************************************************/
void uartReconfigure(uartPort_t *uartPort)
{
    USART_InitTypeDef USART_InitStructure;
	// 失能串口
    USART_Cmd(uartPort->USARTx, DISABLE);
	// 配置波特率
    USART_InitStructure.USART_BaudRate = uartPort->port.baudRate;
	// wordlen必须为9表示奇偶位 - 对RX无关紧要，但会在TX上给出坏数据!
    if (uartPort->port.options & SERIAL_PARITY_EVEN) {
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    } else {
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    }
	// 配置停止位和校验位
    USART_InitStructure.USART_StopBits = (uartPort->port.options & SERIAL_STOPBITS_2) ? USART_StopBits_2 : USART_StopBits_1;
    USART_InitStructure.USART_Parity   = (uartPort->port.options & SERIAL_PARITY_EVEN) ? USART_Parity_Even : USART_Parity_No;
	// 配置串口模式
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = 0;
    if (uartPort->port.mode & MODE_RX)
        USART_InitStructure.USART_Mode |= USART_Mode_Rx;
    if (uartPort->port.mode & MODE_TX)
        USART_InitStructure.USART_Mode |= USART_Mode_Tx;
	// 串口初始化
    USART_Init(uartPort->USARTx, &USART_InitStructure);
	// 配置半双工通信
    if (uartPort->port.options & SERIAL_BIDIR)
        USART_HalfDuplexCmd(uartPort->USARTx, ENABLE);
    else
        USART_HalfDuplexCmd(uartPort->USARTx, DISABLE);
	// 使能串口
    USART_Cmd(uartPort->USARTx, ENABLE);
    // 初始化接收DMA或中断
    DMA_InitTypeDef DMA_InitStructure;
    if (uartPort->port.mode & MODE_RX) {
        if (uartPort->rxDMAResource) {
            DMA_StructInit(&DMA_InitStructure);
            DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
            DMA_InitStructure.DMA_PeripheralBaseAddr = uartPort->rxDMAPeripheralBaseAddr;
            DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
            DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
            DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
            DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
            DMA_InitStructure.DMA_BufferSize = uartPort->port.rxBufferSize;
#ifdef STM32F4
            DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
            DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
            DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
            DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
            DMA_InitStructure.DMA_Channel = uartPort->rxDMAChannel;
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
            DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uartPort->port.rxBuffer;
#endif
            xDMA_DeInit(uartPort->rxDMAResource);
            xDMA_Init(uartPort->rxDMAResource, &DMA_InitStructure);
            xDMA_Cmd(uartPort->rxDMAResource, ENABLE);
            USART_DMACmd(uartPort->USARTx, USART_DMAReq_Rx, ENABLE);
            uartPort->rxDMAPos = xDMA_GetCurrDataCounter(uartPort->rxDMAResource);
        } else {
            USART_ClearITPendingBit(uartPort->USARTx, USART_IT_RXNE);
            USART_ITConfig(uartPort->USARTx, USART_IT_RXNE, ENABLE);
            USART_ITConfig(uartPort->USARTx, USART_IT_IDLE, ENABLE);
        }
    }
    // 初始化发送DMA或中断
    if (uartPort->port.mode & MODE_TX) {
        if (uartPort->txDMAResource) {
            DMA_StructInit(&DMA_InitStructure);
            DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
            DMA_InitStructure.DMA_PeripheralBaseAddr = uartPort->txDMAPeripheralBaseAddr;
            DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
            DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
            DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
            DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
            DMA_InitStructure.DMA_BufferSize = uartPort->port.txBufferSize;
#ifdef STM32F4
            DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
            DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
            DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
            DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
            DMA_InitStructure.DMA_Channel = uartPort->txDMAChannel;
            DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
#endif
            xDMA_DeInit(uartPort->txDMAResource);
            xDMA_Init(uartPort->txDMAResource, &DMA_InitStructure);
#ifdef STM32F4
            xDMA_ITConfig(uartPort->txDMAResource, DMA_IT_TC | DMA_IT_FE | DMA_IT_TE | DMA_IT_DME, ENABLE);
#endif
            xDMA_SetCurrDataCounter(uartPort->txDMAResource, 0);
            USART_DMACmd(uartPort->USARTx, USART_DMAReq_Tx, ENABLE);
        } else {
            USART_ITConfig(uartPort->USARTx, USART_IT_TXE, ENABLE);
        }
    }
	// 使能串口
    USART_Cmd(uartPort->USARTx, ENABLE);
}

/**********************************************************************
函数名称：uartTryStartTxDMA
函数功能：尝试开始串口DMA发送
函数形参：串口配置信息
函数返回值：None
函数描述：None
**********************************************************************/
#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s)
{
	// uartTryStartTxDMA必须被临界保护，因为它被调用uartWrite和handleUsartTxDma (ISR)
    ATOMIC_BLOCK(NVIC_PRIO_SERIALUART_TXDMA) {
    	// 判断数据流是否正在传输中
        if (IS_DMA_ENABLED(s->txDMAResource)) {
            return;
        }
		// 获取DMA数据流传输中剩余的数据单元数
        if (xDMA_GetCurrDataCounter(s->txDMAResource)) {
            // 可能是过早的TC
            goto reenable;
        }
		// 判断数据是否发送完毕
        if (s->port.txBufferHead == s->port.txBufferTail) {
            // 没有数据需要DMA传输
            s->txDMAEmpty = true;
            return;
        }
        // 配置下一个缓冲区传输的内存地址
        xDMA_MemoryTargetConfig(s->txDMAResource, (uint32_t)&s->port.txBuffer[s->port.txBufferTail], DMA_Memory_0);
		// 配置数据尾位置和要传输的数据单元数
        if (s->port.txBufferHead > s->port.txBufferTail) {
			// 写入DMA数据流要传输的数据单元数
            xDMA_SetCurrDataCounter(s->txDMAResource, s->port.txBufferHead - s->port.txBufferTail);
            s->port.txBufferTail = s->port.txBufferHead;
        } else {
            xDMA_SetCurrDataCounter(s->txDMAResource, s->port.txBufferSize - s->port.txBufferTail);
            s->port.txBufferTail = 0;
        }
		// 有数据需要DMA传输
        s->txDMAEmpty = false;
    reenable:
		// 使能DMA
        xDMA_Cmd(s->txDMAResource, ENABLE);
    }
}
#endif
#endif // USE_UART

