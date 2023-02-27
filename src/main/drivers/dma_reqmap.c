/*********************************************************************************
	1.列出外设所有可用的DMA控制器数据流通道信息；
	2.提供获取外设DMA信息块和外设DMA是否启用相关API。
*********************************************************************************/
#include <stdint.h>

#include "platform.h"

#ifdef USE_DMA_SPEC

#include "drivers/adc.h"
#include "drivers/bus_spi.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/timer_def.h"

#include "pg/timerio.h"

#include "dma_reqmap.h"

/* --------------------------外设DMA映射结构体-------------------------- */	
typedef struct dmaPeripheralMapping_s {
    dmaPeripheral_e device;										// 设备
    uint8_t index;											    // 索引
    dmaChannelSpec_t channelSpec[MAX_PERIPHERAL_DMA_OPTIONS];	// DMA信息块
} dmaPeripheralMapping_t;

/* -------------------------定时器DMA映射结构体-------------------------- */	
typedef struct dmaTimerMapping_s {
    TIM_TypeDef *tim;											// 定时器
    uint8_t channel;											// 定时器通道
    dmaChannelSpec_t channelSpec[MAX_TIMER_DMA_OPTIONS];		// DMA信息块	
} dmaTimerMapping_t;

/* ---------------------------外设DMA映射信息---------------------------- */	
static const dmaPeripheralMapping_t dmaPeripheralMapping[] = {
#ifdef USE_SPI                  
    { DMA_PERIPH_SPI_TX,  SPIDEV_1,  { DMA(2, 3, 3), DMA(2, 5, 3) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_1,  { DMA(2, 0, 3), DMA(2, 2, 3) } },
    { DMA_PERIPH_SPI_TX,  SPIDEV_2,  { DMA(1, 4, 0) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_2,  { DMA(1, 3, 0) } },
    { DMA_PERIPH_SPI_TX,  SPIDEV_3,  { DMA(1, 5, 0), DMA(1, 7, 0) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_3,  { DMA(1, 0, 0), DMA(1, 2, 0) } },
#endif // USE_SPI

#ifdef USE_ADC
    { DMA_PERIPH_ADC,     ADCDEV_1,  { DMA(2, 0, 0), DMA(2, 4, 0) } },
    { DMA_PERIPH_ADC,     ADCDEV_2,  { DMA(2, 2, 1), DMA(2, 3, 1) } },
    { DMA_PERIPH_ADC,     ADCDEV_3,  { DMA(2, 0, 2), DMA(2, 1, 2) } },
#endif

#ifdef USE_UART
    { DMA_PERIPH_UART_TX, UARTDEV_1, { DMA(2, 7, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_1, { DMA(2, 5, 4), DMA(2, 2, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_2, { DMA(1, 6, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_2, { DMA(1, 5, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_3, { DMA(1, 3, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_3, { DMA(1, 1, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_4, { DMA(1, 4, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_4, { DMA(1, 2, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_5, { DMA(1, 7, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_5, { DMA(1, 0, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_6, { DMA(2, 6, 5), DMA(2, 7, 5) } },
    { DMA_PERIPH_UART_RX, UARTDEV_6, { DMA(2, 1, 5), DMA(2, 2, 5) } },
#endif
};

/* --------------------------定时器DMA映射信息--------------------------- */
// 定时器通道定义宏
#define TC(chan) DEF_TIM_CHANNEL(CH_ ## chan)
static const dmaTimerMapping_t dmaTimerMapping[] = {
    // 由'timer def.h'生成
    { TIM1, TC(CH1), { DMA(2, 6, 0), DMA(2, 1, 6), DMA(2, 3, 6) } },
    { TIM1, TC(CH2), { DMA(2, 6, 0), DMA(2, 2, 6) } },
    { TIM1, TC(CH3), { DMA(2, 6, 0), DMA(2, 6, 6) } },
    { TIM1, TC(CH4), { DMA(2, 4, 6) } },

    { TIM2, TC(CH1), { DMA(1, 5, 3) } },
    { TIM2, TC(CH2), { DMA(1, 6, 3) } },
    { TIM2, TC(CH3), { DMA(1, 1, 3) } },
    { TIM2, TC(CH4), { DMA(1, 7, 3), DMA(1, 6, 3) } },

    { TIM3, TC(CH1), { DMA(1, 4, 5) } },
    { TIM3, TC(CH2), { DMA(1, 5, 5) } },
    { TIM3, TC(CH3), { DMA(1, 7, 5) } },
    { TIM3, TC(CH4), { DMA(1, 2, 5) } },

    { TIM4, TC(CH1), { DMA(1, 0, 2) } },
    { TIM4, TC(CH2), { DMA(1, 3, 2) } },
    { TIM4, TC(CH3), { DMA(1, 7, 2) } },

    { TIM5, TC(CH1), { DMA(1, 2, 6) } },
    { TIM5, TC(CH2), { DMA(1, 4, 6) } },
    { TIM5, TC(CH3), { DMA(1, 0, 6) } },
    { TIM5, TC(CH4), { DMA(1, 1, 6), DMA(1, 3, 6) } },

    { TIM8, TC(CH1), { DMA(2, 2, 0), DMA(2, 2, 7) } },
    { TIM8, TC(CH2), { DMA(2, 2, 0), DMA(2, 3, 7) } },
    { TIM8, TC(CH3), { DMA(2, 2, 0), DMA(2, 4, 7) } },
    { TIM8, TC(CH4), { DMA(2, 7, 7) } },
};
#undef TC
#undef DMA

/**********************************************************************
函数名称：dmaGetChannelSpecByPeripheral
函数功能：通过外设信息获取其DMA信息块
函数形参：外设，外设索引，外设是否使用DMA
函数返回值：channelSpec或NULL
函数描述：None
**********************************************************************/
const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt)
{
	// 如果未使用DMA则直接返回NULL
    if (opt < 0 || opt >= MAX_PERIPHERAL_DMA_OPTIONS) {
        return NULL;
    }
	// 遍历所有外设DMA映射信息
    for (unsigned i = 0 ; i < ARRAYLEN(dmaPeripheralMapping) ; i++) {
		// 获取外设DMA映射信息
        const dmaPeripheralMapping_t *periph = &dmaPeripheralMapping[i];
		// 如果获取到则返回该外设DMA映射信息
        if (periph->device == device && periph->index == index && periph->channelSpec[opt].ref) {
            return &periph->channelSpec[opt];
        }
    }
    return NULL;
}

/**********************************************************************
函数名称：dmaoptByTag
函数功能：通过IO标签获取其DMA是否启用
函数形参：ioTag
函数返回值：timerIOConfig(i)->dmaop（定时器是否启用DMA）
函数描述：None
**********************************************************************/
dmaoptValue_t dmaoptByTag(ioTag_t ioTag)
{
#ifdef USE_TIMER_MGMT
	// 遍历所有定时器引脚
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        if (timerIOConfig(i)->ioTag == ioTag) {
            return timerIOConfig(i)->dmaopt;
        }
    }
#else
    UNUSED(ioTag);
#endif
    return DMA_OPT_UNUSED;
}

/**********************************************************************
函数名称：dmaGetChannelSpecByTimerValue
函数功能：通过定时器信息获取其DMA信息块
函数形参：定时器，定时器通道，外设是否使用DMA
函数返回值：channelSpec
函数描述：None
**********************************************************************/
const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(TIM_TypeDef *tim, uint8_t channel, dmaoptValue_t dmaopt)
{
	// 如果未使用DMA则直接返回NULL
    if (dmaopt < 0 || dmaopt >= MAX_TIMER_DMA_OPTIONS) {
        return NULL;
    }
	// 遍历所有定时器DMA映射信息
    for (unsigned i = 0 ; i < ARRAYLEN(dmaTimerMapping) ; i++) {
		// 获取定时器DMA映射信息
        const dmaTimerMapping_t *timerMapping = &dmaTimerMapping[i];
		// 如果获取到则返回该定时器DMA映射信息
        if (timerMapping->tim == tim && timerMapping->channel == channel && timerMapping->channelSpec[dmaopt].ref) {
            return &timerMapping->channelSpec[dmaopt];
        }
    }
    return NULL;
}

/**********************************************************************
函数名称：dmaGetChannelSpecByTimer
函数功能：通过定时器获取其DMA信息块
函数形参：定时器
函数返回值：channelSpec
函数描述：None
**********************************************************************/
const dmaChannelSpec_t *dmaGetChannelSpecByTimer(const timerHardware_t *timer)
{
    if (!timer) {
        return NULL;
    }
	// 通过IO标签获取其DMA是否启用
    dmaoptValue_t dmaopt = dmaoptByTag(timer->tag);
	// 通过定时器信息获取其DMA信息块
    return dmaGetChannelSpecByTimerValue(timer->tim, timer->channel, dmaopt);
}

/**********************************************************************
函数名称：dmaGetOptionByTimer
函数功能：通过定时器其DMA是否启用
函数形参：定时器
函数返回值：该定时器是否启用DMA
函数描述：
	由pgResetFn_timerIOConfig调用，以找出预配置定时器是否启用DMA.
**********************************************************************/
dmaoptValue_t dmaGetOptionByTimer(const timerHardware_t *timer)
{
	// 遍历所有定时器DMA映射信息
    for (unsigned i = 0 ; i < ARRAYLEN(dmaTimerMapping); i++) {
		// 获取定时器DMA映射信息
        const dmaTimerMapping_t *timerMapping = &dmaTimerMapping[i];
        if (timerMapping->tim == timer->tim && timerMapping->channel == timer->channel) {
			// 遍历DMA通道DMA_OPTIONS
            for (unsigned j = 0; j < MAX_TIMER_DMA_OPTIONS; j++) {
				// 获取定时器通道DMA启用信息
                const dmaChannelSpec_t *dma = &timerMapping->channelSpec[j];
                if (dma->ref == timer->dmaRefConfigured && dma->channel == timer->dmaChannelConfigured) {
                    return j;
                }
            }
        }
    }
    return DMA_OPT_UNUSED;
}
#endif // USE_DMA_SPEC

