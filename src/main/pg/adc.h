#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "pg/pg.h"
#include "drivers/adc.h"
#include "drivers/io_types.h"
#include "drivers/dma_reqmap.h"

// ADC最大支持数
#define MAX_ADC_SUPPORTED 4

/* --------------------------ADC通道配置结构体-------------------------- */	
typedef struct adcChannelConfig_t {
    bool enabled;							// 使能状态
    ioTag_t ioTag;							// IO标签
} adcChannelConfig_t;

/* ---------------------------ADC配置结构体----------------------------- */	
typedef struct adcConfig_s {
    adcChannelConfig_t vbat;         		// 电池电压
    adcChannelConfig_t current;       		// 电流计
    int8_t device;   			      		// ADC设备
    int8_t dmaopt[MAX_ADC_SUPPORTED]; 		// DMA设备_数据流_通道选择
} adcConfig_t;
// 声明ADC配置结构体
PG_DECLARE(adcConfig_t, adcConfig);

