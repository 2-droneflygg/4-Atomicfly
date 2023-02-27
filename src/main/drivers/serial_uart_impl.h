/*********************************************************************************
 提供串口相关配置定义。
*********************************************************************************/
#pragma once

// ---------------------------------------------------------串口设备最大数量
#define UARTDEV_COUNT_MAX 6

// ---------------------------------------------------------串口硬件最大引脚数
#define UARTHARDWARE_MAX_PINS 4

// ---------------------------------------------------------串口缓冲区大小
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     128
#endif
#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE     256
#endif

// ---------------------------------------------------------串口缓冲区定义宏
#define UART_BUFFER(type, n, rxtx) type volatile uint8_t uart ## n ## rxtx ## xBuffer[UART_ ## rxtx ## X_BUFFER_SIZE]
#define UART_BUFFERS_EXTERN(n) \
    UART_BUFFER(extern, n, R); \
    UART_BUFFER(extern, n, T); struct dummy_s

#ifdef USE_UART1
UART_BUFFERS_EXTERN(1);
#endif
#ifdef USE_UART3
UART_BUFFERS_EXTERN(3);
#endif
#ifdef USE_UART6
UART_BUFFERS_EXTERN(6);
#endif
#undef UART_BUFFERS_EXTERN

// ---------------------------------------------------------配置的UART个数
#ifdef USE_UART1
#define UARTDEV_COUNT_1 1
#else
#define UARTDEV_COUNT_1 0
#endif
#ifdef USE_UART3
#define UARTDEV_COUNT_3 1
#else
#define UARTDEV_COUNT_3 0
#endif
#ifdef USE_UART6
#define UARTDEV_COUNT_6 1
#else
#define UARTDEV_COUNT_6 0
#endif
// 获取串口设备数量
#define UARTDEV_COUNT (UARTDEV_COUNT_1 + UARTDEV_COUNT_3 + UARTDEV_COUNT_6)

// ---------------------------------------------------------串口数据寄存器
#define UART_REG_RXD(base) ((base)->DR)
#define UART_REG_TXD(base) ((base)->DR)

/* -----------------------------串口引脚定义结构体----------------------------- */	
typedef struct uartPinDef_s {
    ioTag_t pin;
} uartPinDef_t;

/* -----------------------------串口硬件信息结构体----------------------------- */	
typedef struct uartHardware_s {
    UARTDevice_e device;    						// 串口设备
    USART_TypeDef* reg;								// USARTx
#ifdef USE_DMA
    dmaResource_t *txDMAResource;					
    dmaResource_t *rxDMAResource;
    uint32_t txDMAChannel;
    uint32_t rxDMAChannel;
#endif // USE_DMA
    uartPinDef_t rxPins[UARTHARDWARE_MAX_PINS];
    uartPinDef_t txPins[UARTHARDWARE_MAX_PINS];
    rccPeriphTag_t rcc;
    uint8_t af;
    uint8_t irqn;
    uint8_t txPriority;
    uint8_t rxPriority;
    volatile uint8_t *txBuffer;
    volatile uint8_t *rxBuffer;
    uint16_t txBufferSize;
    uint16_t rxBufferSize;
} uartHardware_t;

/* -----------------------------串口设备信息结构体----------------------------- */	
// uartDevice_t是一个实际的设备实例
typedef struct uartDevice_s {
    uartPort_t port;
    const uartHardware_t *hardware;
    uartPinDef_t rx;
    uartPinDef_t tx;
    volatile uint8_t *rxBuffer;
    volatile uint8_t *txBuffer;
} uartDevice_t;


extern const uartHardware_t uartHardware[];
extern uartDevice_t *uartDevmap[];
extern const struct serialPortVTable uartVTable[];
void uartTryStartTxDMA(uartPort_t *s);
uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_e mode, portOptions_e options);
void uartIrqHandler(uartPort_t *s);
void uartReconfigure(uartPort_t *uartPort);
void uartConfigureDma(uartDevice_t *uartdev);
void uartDmaIrqHandler(dmaChannelDescriptor_t* descriptor);

