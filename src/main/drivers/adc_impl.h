/*********************************************************************************
 提供ADC描述信息块结构体定义。
*********************************************************************************/
#pragma once

#include "drivers/adc.h"
#include "drivers/dma.h"
#include "drivers/io_types.h"
#include "drivers/rcc_types.h"

/* -------------------------------ADC设备结构体-------------------------------- */	
typedef struct adcDevice_s {
    ADC_TypeDef* ADCx;				// ADC设备
    rccPeriphTag_t rccADC;			// ADC时钟
} adcDevice_t;

/* -----------------------------ADC标签映射结构体------------------------------ */	
typedef struct adcTagMap_s {
    ioTag_t tag;					// IO标签
	uint8_t devices;				// ADC设备
    uint32_t channel;				// ADC通道
} adcTagMap_t;

// ADC标签映射数量
#define ADC_TAG_MAP_COUNT 16
// adc标签映射编码
#define ADC_DEVICES_12  ((1 << ADCDEV_1)|(1 << ADCDEV_2))
#define ADC_DEVICES_123 ((1 << ADCDEV_1)|(1 << ADCDEV_2)|(1 << ADCDEV_3))

extern const adcDevice_t adcHardware[];
extern const adcTagMap_t adcTagMap[ADC_TAG_MAP_COUNT];
extern adcOperatingConfig_t adcOperatingConfig[ADC_CHANNEL_COUNT];
extern volatile uint16_t adcValues[ADC_CHANNEL_COUNT];
uint8_t adcChannelByTag(ioTag_t ioTag);
ADCDevice adcDeviceByInstance(ADC_TypeDef *instance);
bool adcVerifyPin(ioTag_t tag, ADCDevice device);
void adcGetChannelValues(void);

