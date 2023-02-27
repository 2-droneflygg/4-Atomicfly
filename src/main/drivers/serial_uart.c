/*********************************************************************************
 提供一系列串口操作实现API（标准串行底层驱动程序API实现）。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/rcc.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#include "pg/serial_uart.h"

// ---------------------------------------------------------定义串口缓存区
#define UART_TX_BUFFER_ATTRIBUTE                    // NONE
#define UART_RX_BUFFER_ATTRIBUTE                    // NONE

#define UART_BUFFERS(n) \
    UART_BUFFER(UART_TX_BUFFER_ATTRIBUTE, n, T); \
    UART_BUFFER(UART_RX_BUFFER_ATTRIBUTE, n, R); struct dummy_s
    
#ifdef USE_UART1
UART_BUFFERS(1);
#endif
#ifdef USE_UART3
UART_BUFFERS(3);
#endif
#ifdef USE_UART6
UART_BUFFERS(6);
#endif
#undef UART_BUFFERS

/**********************************************************************
函数名称：uartOpen
函数功能：打开串口并进行初始化
函数形参：device，rxCallback，rxCallbackData，baudRate，mode，options
函数返回值：串口配置
函数描述：None
**********************************************************************/
serialPort_t *uartOpen(UARTDevice_e device, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
	// 串口初始化并获取串口信息
    uartPort_t *s = serialUART(device, baudRate, mode, options);
	// 判断合法性
    if (!s)
        return (serialPort_t *)s;
#ifdef USE_DMA
    s->txDMAEmpty = true;
#endif
    // 初始化缓冲区
    s->port.rxBufferHead = s->port.rxBufferTail = 0;
    s->port.txBufferHead = s->port.txBufferTail = 0;
    // 回调只适用于基于irq的RX
    s->port.rxCallback = rxCallback;
    s->port.rxCallbackData = rxCallbackData;
    s->port.mode = mode;
    s->port.baudRate = baudRate;
    s->port.options = options;
	// 重新配置串口
    uartReconfigure(s);
    return (serialPort_t *)s;
}

/**********************************************************************
函数名称：uartSetBaudRate
函数功能：设置串口波特率
函数形参：串口设备信息，波特率
函数返回值：None
函数描述：None
**********************************************************************/
static void uartSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.baudRate = baudRate;
    uartReconfigure(uartPort);
}

/**********************************************************************
函数名称：uartSetMode
函数功能：设置串口模式
函数形参：串口设备信息，模式
函数返回值：None
函数描述：None
**********************************************************************/
static void uartSetMode(serialPort_t *instance, portMode_e mode)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.mode = mode;
    uartReconfigure(uartPort);
}

/**********************************************************************
函数名称：uartTotalRxBytesWaiting
函数功能：等待的Rx字节总数
函数形参：串口设备信息
函数返回值：Rx字节总数
函数描述：None
**********************************************************************/
static uint32_t uartTotalRxBytesWaiting(const serialPort_t *instance)
{
    const uartPort_t *s = (const uartPort_t*)instance;

#ifdef USE_DMA
    if (s->rxDMAResource) {
        uint32_t rxDMAHead = xDMA_GetCurrDataCounter(s->rxDMAResource);

		// s->rxDMAPos和rxDMAHead表示到终端的距离缓冲区，他们一边前进一边倒数
        if (s->rxDMAPos >= rxDMAHead) {
            return s->rxDMAPos - rxDMAHead;
        } else {
            return s->port.rxBufferSize + s->rxDMAPos - rxDMAHead;
        }
    }
#endif

    if (s->port.rxBufferHead >= s->port.rxBufferTail) {
        return s->port.rxBufferHead - s->port.rxBufferTail;
    } else {
        return s->port.rxBufferSize + s->port.rxBufferHead - s->port.rxBufferTail;
    }
}

/**********************************************************************
函数名称：uartTotalTxBytesFree
函数功能：TX字节释放总数
函数形参：串口设备信息
函数返回值：TX字节释放总数
函数描述：None
**********************************************************************/
static uint32_t uartTotalTxBytesFree(const serialPort_t *instance)
{
    const uartPort_t *s = (const uartPort_t*)instance;

    uint32_t bytesUsed;

    if (s->port.txBufferHead >= s->port.txBufferTail) {
        bytesUsed = s->port.txBufferHead - s->port.txBufferTail;
    } else {
        bytesUsed = s->port.txBufferSize + s->port.txBufferHead - s->port.txBufferTail;
    }

#ifdef USE_DMA
    if (s->txDMAResource) {
		/*
		* 当我们对DMA请求排队时，我们在传输完成之前推进Tx缓冲区尾部，所以我们必须添加
		* 正在进行的转移的剩余大小改为:
		*/
        bytesUsed += xDMA_GetCurrDataCounter(s->txDMAResource);

		/*
		* 如果写入Tx缓冲区非常快，我们可能已经将头部推进到缓冲区
		* 当前DMA传输所占用的空间。在这种情况下，“bytesUsed”的总数实际上最终会更大
		* 大于总Tx缓冲区大小，因为我们将最终传输相同的缓冲区两次。(我们会
		* 传输旧字节和新字节的垃圾混合)。
		* 友善地对待来电者，假装我们的缓冲区永远只能100%满。
		*/
        if (bytesUsed >= s->port.txBufferSize - 1) {
            return 0;
        }
    }
#endif
    return (s->port.txBufferSize - 1) - bytesUsed;
}

/**********************************************************************
函数名称：isUartTransmitBufferEmpty
函数功能：获取串口传输缓冲区是否为空
函数形参：串口设备信息
函数返回值：result
函数描述：None
**********************************************************************/
static bool isUartTransmitBufferEmpty(const serialPort_t *instance)
{
    const uartPort_t *s = (const uartPort_t *)instance;
#ifdef USE_DMA
    if (s->txDMAResource) {
        return s->txDMAEmpty;
    } else
#endif
    {
        return s->port.txBufferTail == s->port.txBufferHead;
    }
}

/**********************************************************************
函数名称：uartRead
函数功能：uart读
函数形参：串口设备信息
函数返回值：数据
函数描述：None
**********************************************************************/
static uint8_t uartRead(serialPort_t *instance)
{
    uint8_t ch;
    uartPort_t *s = (uartPort_t *)instance;

#ifdef USE_DMA
    if (s->rxDMAResource) {
        ch = s->port.rxBuffer[s->port.rxBufferSize - s->rxDMAPos];
        if (--s->rxDMAPos == 0)
            s->rxDMAPos = s->port.rxBufferSize;
    } else
#endif
    {
        ch = s->port.rxBuffer[s->port.rxBufferTail];
        if (s->port.rxBufferTail + 1 >= s->port.rxBufferSize) {
            s->port.rxBufferTail = 0;
        } else {
            s->port.rxBufferTail++;
        }
    }

    return ch;
}

/**********************************************************************
函数名称：uartWrite
函数功能：uart写
函数形参：串口设备信息，ch
函数返回值：None
函数描述：None
**********************************************************************/
static void uartWrite(serialPort_t *instance, uint8_t ch)
{
    uartPort_t *s = (uartPort_t *)instance;

    s->port.txBuffer[s->port.txBufferHead] = ch;

    if (s->port.txBufferHead + 1 >= s->port.txBufferSize) {
        s->port.txBufferHead = 0;
    } else {
        s->port.txBufferHead++;
    }

#ifdef USE_DMA
    if (s->txDMAResource) {
        uartTryStartTxDMA(s);
    } else
#endif
    {
        USART_ITConfig(s->USARTx, USART_IT_TXE, ENABLE);
    }
}

// 串口虚函数表
const struct serialPortVTable uartVTable[] = {
    {
        .serialWrite = uartWrite,
        .serialTotalRxWaiting = uartTotalRxBytesWaiting,
        .serialTotalTxFree = uartTotalTxBytesFree,
        .serialRead = uartRead,
        .serialSetBaudRate = uartSetBaudRate,
        .isSerialTransmitBufferEmpty = isUartTransmitBufferEmpty,
        .setMode = uartSetMode,
    }
};

/**********************************************************************
函数名称：uartConfigureDma
函数功能：串口DMA配置
函数形参：uartdev
函数返回值：None
函数描述：None
**********************************************************************/
#ifdef USE_DMA
void uartConfigureDma(uartDevice_t *uartdev)
{
    uartPort_t *s = &(uartdev->port);
    const uartHardware_t *hardware = uartdev->hardware;

#ifdef USE_DMA_SPEC
    UARTDevice_e device = hardware->device;
    const dmaChannelSpec_t *dmaChannelSpec;

    if (serialUartConfig(device)->txDmaopt != DMA_OPT_UNUSED) {
        dmaChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_TX, device, serialUartConfig(device)->txDmaopt);
        if (dmaChannelSpec) {
            s->txDMAResource = dmaChannelSpec->ref;
            s->txDMAChannel = dmaChannelSpec->channel;
        }
    }

    if (serialUartConfig(device)->rxDmaopt != DMA_OPT_UNUSED) {
        dmaChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, device, serialUartConfig(device)->txDmaopt);
        if (dmaChannelSpec) {
            s->rxDMAResource = dmaChannelSpec->ref;
            s->rxDMAChannel = dmaChannelSpec->channel;
        }
    }
#else
    // 非USE_DMA_SPEC不支持可配置的UART DMA开/关
    if (hardware->rxDMAResource) {
        s->rxDMAResource = hardware->rxDMAResource;
        s->rxDMAChannel = hardware->rxDMAChannel;
    }

    if (hardware->txDMAResource) {
        s->txDMAResource = hardware->txDMAResource;
        s->txDMAChannel = hardware->txDMAChannel;
    }
#endif

    if (s->txDMAResource) {
        dmaIdentifier_e identifier = dmaGetIdentifier(s->txDMAResource);
		// DMA初始化
        dmaInit(identifier, OWNER_SERIAL_TX, RESOURCE_INDEX(hardware->device));
		// 设置DMA中断服务函数
        dmaSetHandler(identifier, uartDmaIrqHandler, hardware->txPriority, (uint32_t)uartdev);
        s->txDMAPeripheralBaseAddr = (uint32_t)&UART_REG_TXD(hardware->reg);
    }

    if (s->rxDMAResource) {
        dmaIdentifier_e identifier = dmaGetIdentifier(s->rxDMAResource);
        dmaInit(identifier, OWNER_SERIAL_RX, RESOURCE_INDEX(hardware->device));
        s->rxDMAPeripheralBaseAddr = (uint32_t)&UART_REG_RXD(hardware->reg);
    }
}
#endif

// ---------------------------------------------------------串口中断服务函数定义
#define UART_IRQHandler(type, number, dev)                    \
    void type ## number ## _IRQHandler(void)                  \
    {                                                         \
        uartPort_t *s = &(uartDevmap[UARTDEV_ ## dev]->port); \
        uartIrqHandler(s);                                    \
    }

#ifdef USE_UART1
UART_IRQHandler(USART, 1, 1) 	// USART1 Rx/Tx IRQ Handler
#endif
#ifdef USE_UART3
UART_IRQHandler(USART, 3, 3) 	// USART3 Rx/Tx IRQ Handler
#endif
#ifdef USE_UART6
UART_IRQHandler(USART, 6, 6) 	// USART6 Rx/Tx IRQ Handler
#endif
#endif // USE_UART

