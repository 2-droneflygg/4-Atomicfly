/*********************************************************************************
 提供串口硬件配置信息和串口初始化及相关中断处理API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART
#include "drivers/system.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

// ---------------------------------------------------------串口硬件配置信息
const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART1
    {
        .device = UARTDEV_1,
        .reg = USART1,
        .rxDMAChannel = DMA_Channel_4,
        .txDMAChannel = DMA_Channel_4,
        .rxPins = { { DEFIO_TAG_E(PA10) }, { DEFIO_TAG_E(PB7) },
            },
        .txPins = { { DEFIO_TAG_E(PA9) }, { DEFIO_TAG_E(PB6) },
            },
        .af = GPIO_AF_USART1,
        .rcc = RCC_APB2(USART1),
        .irqn = USART1_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART1,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
    },
#endif
#ifdef USE_UART3
    {
        .device = UARTDEV_3,
        .reg = USART3,
        .rxDMAChannel = DMA_Channel_4,
        .txDMAChannel = DMA_Channel_4,
        .rxPins = { { DEFIO_TAG_E(PB11) }, { DEFIO_TAG_E(PC11) }, { DEFIO_TAG_E(PD9) } },
        .txPins = { { DEFIO_TAG_E(PB10) }, { DEFIO_TAG_E(PC10) }, { DEFIO_TAG_E(PD8) } },
        .af = GPIO_AF_USART3,
        .rcc = RCC_APB1(USART3),
        .irqn = USART3_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART3_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART3,
        .txBuffer = uart3TxBuffer,
        .rxBuffer = uart3RxBuffer,
        .txBufferSize = sizeof(uart3TxBuffer),
        .rxBufferSize = sizeof(uart3RxBuffer),
    },
#endif
#ifdef USE_UART6
    {
        .device = UARTDEV_6,
        .reg = USART6,
        .rxDMAChannel = DMA_Channel_5,
        .txDMAChannel = DMA_Channel_5,


        .rxPins = { { DEFIO_TAG_E(PC7) },
            { DEFIO_TAG_E(PG9) },
            },
        .txPins = { { DEFIO_TAG_E(PC6) },
            { DEFIO_TAG_E(PG14) },
            },
        .af = GPIO_AF_USART6,
        .rcc = RCC_APB2(USART6),
        .irqn = USART6_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART6_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART6,
        .txBuffer = uart6TxBuffer,
        .rxBuffer = uart6RxBuffer,
        .txBufferSize = sizeof(uart6TxBuffer),
        .rxBufferSize = sizeof(uart6RxBuffer),
    },
#endif
};

/**********************************************************************
函数名称：serialUART
函数功能：串口初始化
函数形参：device，baudRate，mode，options
函数返回值：串行端口信息
函数描述：None
**********************************************************************/
uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
	// 获取串口设备配置
    uartDevice_t *uart = uartDevmap[device];    
    if (!uart) 	return NULL;
	// 获取串口硬件配置 	
    const uartHardware_t *hardware = uart->hardware;
    if (!hardware) return NULL; 
	// 注册串口信息
    uartPort_t *s = &(uart->port);
    s->port.vTable = uartVTable;
    s->port.baudRate = baudRate;
    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;
    s->USARTx = hardware->reg;
#ifdef USE_DMA
	// 配置串口DMA
    uartConfigureDma(uart);
#endif
	// 获取IO
    IO_t txIO = IOGetByTag(uart->tx.pin);
    IO_t rxIO = IOGetByTag(uart->rx.pin);
	// 使能串口时钟
    if (hardware->rcc) {
        RCC_ClockCmd(hardware->rcc, ENABLE);
    }
	// IO初始化
    if (options & SERIAL_BIDIR) {
        IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        IOConfigGPIOAF(txIO, (options & SERIAL_BIDIR_PP) ? IOCFG_AF_PP : IOCFG_AF_OD, hardware->af);
    } else {
        if ((mode & MODE_TX) && txIO) {
            IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(txIO, IOCFG_AF_PP_UP, hardware->af);
        }
        if ((mode & MODE_RX) && rxIO) {
            IOInit(rxIO, OWNER_SERIAL_RX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(rxIO, IOCFG_AF_PP_UP, hardware->af);
        }
    }
	// 初始化NVIC中断
#ifdef USE_DMA
    if (!(s->rxDMAResource)) {
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = hardware->irqn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(hardware->rxPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(hardware->rxPriority);
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }
#endif
    return s;
}

/**********************************************************************
函数名称：handleUsartTxDma
函数功能：串口发送DMA处理
函数形参：串口配置信息
函数返回值：None
函数描述：None
**********************************************************************/
static void handleUsartTxDma(uartPort_t *s)
{
    uartTryStartTxDMA(s);
}

/**********************************************************************
函数名称：uartIrqHandler
函数功能：串口中断处理
函数形参：串口配置信息
函数返回值：None
函数描述：None
**********************************************************************/
void uartIrqHandler(uartPort_t *s)
{	
	// 接收数据
    if (!s->rxDMAResource && (USART_GetITStatus(s->USARTx, USART_IT_RXNE) == SET)) {
		// 有回调函数 - 调用ISR回调函数立即处理数据
        if (s->port.rxCallback) {		
            s->port.rxCallback(s->USARTx->DR, s->port.rxCallbackData);
        } 
		// 无回调函数 - 使用接收缓冲区进行数据缓存
		else {						
            s->port.rxBuffer[s->port.rxBufferHead] = s->USARTx->DR;
			// 接收缓冲区头偏移
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
    }
	// 判断发送数据寄存器是否为空
    if (!s->txDMAResource && (USART_GetITStatus(s->USARTx, USART_IT_TXE) == SET)) {
		// 发送缓冲区未头尾相接
        if (s->port.txBufferTail != s->port.txBufferHead) {
			// 发送数据
            USART_SendData(s->USARTx, s->port.txBuffer[s->port.txBufferTail]);
			// 发送缓冲区头尾偏移
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } 
		else {
			// 禁用发送寄存器为空中断
            USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
        }
    }
	// 清除上溢错误标志位
    if (USART_GetITStatus(s->USARTx, USART_IT_ORE) == SET) {
        USART_ClearITPendingBit(s->USARTx, USART_IT_ORE);		
    }
	// 空闲中断
    if (USART_GetITStatus(s->USARTx, USART_IT_IDLE) == SET) {
		// 判断是否有空闲回调
        if (s->port.idleCallback) {
            s->port.idleCallback();
        }
        // clear
        (void) s->USARTx->SR;
        (void) s->USARTx->DR;
    }
}

/**********************************************************************
函数名称：uartDmaIrqHandler
函数功能：串口DMA中断处理
函数形参：DMA通道描述符
函数返回值：None
函数描述：None
**********************************************************************/
void uartDmaIrqHandler(dmaChannelDescriptor_t* descriptor)
{
	// 获取串口信息
    uartPort_t *s = &(((uartDevice_t*)(descriptor->userParam))->port);
	// DMA传输完成
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
    	// 清除传输完成标志位
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);		
		// 清除半传输标志位
        DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF);			
        if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_FEIF)) {
        	// 清除FIFO溢出标志位
            DMA_CLEAR_FLAG(descriptor, DMA_IT_FEIF);		
        }
		// 处理串口发送DMA
        handleUsartTxDma(s);
    }
	// DMA传输错误
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF)) {
    	// 清除传输错误标志位
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TEIF);			
    }
	// DMA直接模式错误
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_DMEIF)) {
		// 清除直接模式错误标志位
        DMA_CLEAR_FLAG(descriptor, DMA_IT_DMEIF);			
    }
}
#endif

