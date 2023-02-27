#pragma once

#include <stdbool.h>

#include "drivers/io_types.h"
#include "drivers/time.h"

/* -------------------------ADC实例设备配置-------------------------- */	
#ifndef ADC_INSTANCE
#define ADC_INSTANCE    	ADC1
#endif

/* -------------------------ADC_DMA数据流配置------------------------ */	
#ifndef ADC1_DMA_STREAM
// ST0 or ST4
#define ADC1_DMA_STREAM 	DMA2_Stream4 
#endif

/* --------------------------ADC_CFG_TO_DEV-------------------------- */	
#define ADC_CFG_TO_DEV(x) ((x) - 1)

/* --------------------------ADC_DEV_TO_CFG-------------------------- */	
#define ADC_DEV_TO_CFG(x) ((x) + 1)

/* --------------------------ADC设备索引（ID）----------------------- */	
typedef enum ADCDevice {
    ADCINVALID = -1,
    ADCDEV_1   =  0,
    ADCDEV_2,
    ADCDEV_3,
    ADCDEV_COUNT
} ADCDevice;

/* ----------------------------ADC通道枚举--------------------------- */	
typedef enum {
    ADC_BATTERY   = 0,  			// 电池电压
    ADC_CURRENT   = 1,				// 电流计
    ADC_CHANNEL_COUNT   			// ADC通道数量
} AdcChannel;

/* --------------------------ADC配置信息结构体----------------------- */	
typedef struct adcOperatingConfig_s {
    ioTag_t tag;					// IO引脚
    uint8_t adcChannel;         	// ADC通道
    uint8_t dmaIndex;           	// DMA缓冲区索引
    bool enabled;					// 使能状态
    uint8_t sampleTime;				// 采样时间
} adcOperatingConfig_t;

struct adcConfig_s;
void adcInit(const struct adcConfig_s *config);
uint16_t adcGetChannel(uint8_t channel);
ADCDevice adcDeviceByInstance(ADC_TypeDef *instance);

