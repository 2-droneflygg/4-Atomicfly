#pragma once

#include "platform.h"

#include "drivers/dma.h"
#include "drivers/timer.h"

/* --------------------------DMA信息块结构体--------------------------- */	
typedef uint16_t dmaCode_t;
typedef struct dmaChannelSpec_s {
    dmaCode_t             code;				// DMA代码
    dmaResource_t         *ref;				// DMA控制器_数据流
    uint32_t              channel;			// DMA控制器_数据流_通道
} dmaChannelSpec_t;

/* ----------------------------外设DMA枚举----------------------------- */
typedef enum {
    DMA_PERIPH_SPI_TX,						// SPI	
    DMA_PERIPH_SPI_RX,
    DMA_PERIPH_ADC, 						// ADC
    DMA_PERIPH_UART_TX,						// USART
    DMA_PERIPH_UART_RX,
} dmaPeripheral_e;

// DMA代码设置宏
#define DMA_CODE(dma, stream, chanreq) ((dma << 12)|(stream << 8)|(chanreq << 0))
// DMA信息块设置宏
#define DMA(d, s, c) { DMA_CODE(d, s, c), (dmaResource_t *)DMA ## d ## _Stream ## s, DMA_Channel_ ## c }

// DMA选择
typedef int8_t dmaoptValue_t;
// DMA未使用
#define DMA_OPT_UNUSED (-1)
// DMA外设最大选择（可选用两种不同的DMA_数据流_通道）
#define MAX_PERIPHERAL_DMA_OPTIONS 2
// DMA定时器最大选择（可选用三种不同的DMA_数据流_通道）
#define MAX_TIMER_DMA_OPTIONS 3

struct timerHardware_s;
dmaoptValue_t dmaoptByTag(ioTag_t ioTag);
const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt);
const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(TIM_TypeDef *tim, uint8_t channel, dmaoptValue_t dmaopt);
const dmaChannelSpec_t *dmaGetChannelSpecByTimer(const struct timerHardware_s *timer);
dmaoptValue_t dmaGetOptionByTimer(const struct timerHardware_s *timer);
dmaoptValue_t dmaGetUpOptionByTimer(const struct timerHardware_s *timer);

