#pragma once

#include "drivers/dma.h" 

/* ---------------------------串口设备索引（ID）枚举--------------------------- */	
typedef enum {
    UARTDEV_1 = 0,
    UARTDEV_2 = 1,
    UARTDEV_3 = 2,
    UARTDEV_4 = 3,
    UARTDEV_5 = 4,
    UARTDEV_6 = 5,
    UARTDEV_7 = 6,
    UARTDEV_8 = 7,
    UARTDEV_9 = 8,
} UARTDevice_e;

/* -------------------------------串行端口结构体------------------------------- */	
typedef struct uartPort_s {
    serialPort_t port;						 // 串口		
#ifdef USE_DMA
    dmaResource_t *rxDMAResource;			 // RX_DMA控制器_数据流
    dmaResource_t *txDMAResource;			 // TX_DMA控制器_数据流
    uint32_t rxDMAChannel;					 // RX_DMA通道			
    uint32_t txDMAChannel;		        	 // TX_DMA通道
    uint32_t rxDMAIrq;						 // RX_DMA中断源					
    uint32_t txDMAIrq;						 // TX_DMA中断源	
    uint32_t rxDMAPos;						 // RX_DMAPos
    uint32_t txDMAPeripheralBaseAddr;		 // TX_DMA外设地址
    uint32_t rxDMAPeripheralBaseAddr;		 // RX_DMA外设地址
#endif // USE_DMA
    USART_TypeDef *USARTx;					 // USARTx
    bool txDMAEmpty;						 // 是否有数据需要DMA传输
} uartPort_t;

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig);
serialPort_t *uartOpen(UARTDevice_e device, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options);

