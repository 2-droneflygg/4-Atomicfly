/*********************************************************************************
 提供一系列ADC初始化API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ADC
#include "drivers/dma_reqmap.h"

#include "drivers/io.h"
#include "io_impl.h"
#include "rcc.h"
#include "dma.h"

#include "drivers/sensor.h"

#include "adc.h"
#include "adc_impl.h"

#include "pg/adc.h"

// ---------------------------------------------------------预定义ADC设备信息
const adcDevice_t adcHardware[] = {
    {
        .ADCx = ADC1,
        .rccADC = RCC_APB2(ADC1),
    },
    {
        .ADCx = ADC2,
        .rccADC = RCC_APB2(ADC2),
    },
    {
        .ADCx = ADC3,
        .rccADC = RCC_APB2(ADC3),
    }
};

// ---------------------------------------------------------预定义ADC标签映射
const adcTagMap_t adcTagMap[] = {
    { DEFIO_TAG_E__PC0, ADC_DEVICES_123, ADC_Channel_10 },
    { DEFIO_TAG_E__PC1, ADC_DEVICES_123, ADC_Channel_11 },
    { DEFIO_TAG_E__PC2, ADC_DEVICES_123, ADC_Channel_12 },
    { DEFIO_TAG_E__PC3, ADC_DEVICES_123, ADC_Channel_13 },
    { DEFIO_TAG_E__PC4, ADC_DEVICES_12,  ADC_Channel_14 },
    { DEFIO_TAG_E__PC5, ADC_DEVICES_12,  ADC_Channel_15 },
    { DEFIO_TAG_E__PB0, ADC_DEVICES_12,  ADC_Channel_8  },
    { DEFIO_TAG_E__PB1, ADC_DEVICES_12,  ADC_Channel_9  },
    { DEFIO_TAG_E__PA0, ADC_DEVICES_123, ADC_Channel_0  },
    { DEFIO_TAG_E__PA1, ADC_DEVICES_123, ADC_Channel_1  },
    { DEFIO_TAG_E__PA2, ADC_DEVICES_123, ADC_Channel_2  },
    { DEFIO_TAG_E__PA3, ADC_DEVICES_123, ADC_Channel_3  },
    { DEFIO_TAG_E__PA4, ADC_DEVICES_12,  ADC_Channel_4  },
    { DEFIO_TAG_E__PA5, ADC_DEVICES_12,  ADC_Channel_5  },
    { DEFIO_TAG_E__PA6, ADC_DEVICES_12,  ADC_Channel_6  },
    { DEFIO_TAG_E__PA7, ADC_DEVICES_12,  ADC_Channel_7  }
};

/**********************************************************************
函数名称：adcInitDevice
函数功能：ADC设备初始化
函数形参：ADCx，通道数量
函数返回值：None 
函数描述：None 
**********************************************************************/
void adcInitDevice(ADC_TypeDef *adcdev, int channelCount)
{
    ADC_InitTypeDef ADC_InitStructure;

	// 用默认值填充每个ADC_InitStruct成员
    ADC_StructInit(&ADC_InitStructure);
	// 配置ADC
    ADC_InitStructure.ADC_ContinuousConvMode       = ENABLE;							  // 开启连续转换
    ADC_InitStructure.ADC_Resolution               = ADC_Resolution_12b;				  // 12位模式
    ADC_InitStructure.ADC_ExternalTrigConv         = ADC_ExternalTrigConv_T1_CC1;         // 选择用于触发的外部事件
    ADC_InitStructure.ADC_ExternalTrigConvEdge     = ADC_ExternalTrigConvEdge_None;		  // 禁止边沿（转换由软件而不是外部触发启动）
    ADC_InitStructure.ADC_DataAlign                = ADC_DataAlign_Right;				  // 转换后的数据右对齐
    ADC_InitStructure.ADC_NbrOfConversion          = channelCount;						  // 指定ADC转换的个数
    ADC_InitStructure.ADC_ScanConvMode             = channelCount > 1 ? ENABLE : DISABLE; // 扫描模式
    // 初始化ADC
    ADC_Init(adcdev, &ADC_InitStructure);
}

/**********************************************************************
函数名称：adcInit
函数功能：ADC初始化
函数形参：config
函数返回值：None 
函数描述：None 
**********************************************************************/
void adcInit(const adcConfig_t *config)
{
    uint8_t i;
    uint8_t configuredAdcChannels = 0;

	// 清空ADC配置信息存储数组
    memset(&adcOperatingConfig, 0, sizeof(adcOperatingConfig));

	// 电压计引脚
    if (config->vbat.enabled) {
        adcOperatingConfig[ADC_BATTERY].tag = config->vbat.ioTag;
    }
	// 电流计引脚
    if (config->current.enabled) {
        adcOperatingConfig[ADC_CURRENT].tag = config->current.ioTag;  					
    }

	// 获取ADC设备 - ADCDevice
    ADCDevice device = ADC_CFG_TO_DEV(config->device);
    if (device == ADCINVALID) {
        return;
    }
	// 获取ADC设备信息
    adcDevice_t adc = adcHardware[device];

	// 初始化ADC设备通道激活状态
    bool adcActive = false;

	// 遍历ADC设备所有通道
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
		// 验证由IO标签指定的引脚是否映射到由ADC设备指定的ADC实例
        if (!adcVerifyPin(adcOperatingConfig[i].tag, device)) {
            continue;
        }
		// 验证无误则激活
        adcActive = true;
		// 初始化GPIO
        IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
		// 配置GPIO
        IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_Mode_AN, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL));
		// 注册ADC配置信息
        adcOperatingConfig[i].adcChannel = adcChannelByTag(adcOperatingConfig[i].tag);    // ADC通道
        adcOperatingConfig[i].dmaIndex = configuredAdcChannels++;                         // DMA索引
        adcOperatingConfig[i].sampleTime = ADC_SampleTime_480Cycles;					  // 采样时间
        adcOperatingConfig[i].enabled = true;											  // 使能状态
    }

	// 如果ADC设备通道未激活则直接返回
    if (!adcActive) {
        return;
    }

	// 使能ADC设备时钟
    RCC_ClockCmd(adc.rccADC, ENABLE);

    ADC_CommonInitTypeDef ADC_CommonInitStructure;
	// 配置ADC设备
    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;				  // 独立模式
    ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div8;					  // ADC时钟分频
    ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;			  // 配置直接内存访问 - 用于多ADC模式
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;		  // 配置采样周期
    // ADC设备公共配置初始化
    ADC_CommonInit(&ADC_CommonInitStructure);
	// ADC设备初始化
    adcInitDevice(adc.ADCx, configuredAdcChannels);

	// 初始化序列为1
    uint8_t rank = 1;
	// 遍历所有通道
    for (i = 0; i < ADC_CHANNEL_COUNT; i++) {
		// 检查通道是否使能
        if (!adcOperatingConfig[i].enabled) {
            continue;
        }
		// 配置ADC通道转换序列及采样时间
        ADC_RegularChannelConfig(adc.ADCx, adcOperatingConfig[i].adcChannel, rank++, adcOperatingConfig[i].sampleTime);
    }

	// 使能ADC_DMA传输请求
    ADC_DMARequestAfterLastTransferCmd(adc.ADCx, ENABLE);
	// 使能ADC_DMA
    ADC_DMACmd(adc.ADCx, ENABLE);
	// 使能ADC设备
    ADC_Cmd(adc.ADCx, ENABLE);

#ifdef USE_DMA_SPEC
	// 通过ADC外设信息获取其DMA信息块
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, device, config->dmaopt[device]);
	// 检查合法性
    if (!dmaSpec) {
        return;
    }
	// DMA设备初始化
    dmaInit(dmaGetIdentifier(dmaSpec->ref), OWNER_ADC, RESOURCE_INDEX(device));
	// DMA配置初始化为默认值
    xDMA_DeInit(dmaSpec->ref);
#endif
    DMA_InitTypeDef DMA_InitStructure;
	// DMA结构初始化位默认值
    DMA_StructInit(&DMA_InitStructure);
	// 配置DMA控制器
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&adc.ADCx->DR;            		  // 外设地址
#ifdef USE_DMA_SPEC
    DMA_InitStructure.DMA_Channel = dmaSpec->channel;                              		  // DMA通道
#endif
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adcValues;				   		  // 内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;						   		  // 外设到存储器
    DMA_InitStructure.DMA_BufferSize = configuredAdcChannels;                      		  // 缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			   		  // 外设地址固定
    DMA_InitStructure.DMA_MemoryInc = configuredAdcChannels > 1 ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable; // 内存地址是否自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	   		  // 半字
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;			   		  // 半字
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;								   		  // 循环传输
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;							   		  // 优先级
#ifdef USE_DMA_SPEC
	// DMA初始化
    xDMA_Init(dmaSpec->ref, &DMA_InitStructure);
	// 使能DMA
    xDMA_Cmd(dmaSpec->ref, ENABLE);
#endif
	// 开始转换
    ADC_SoftwareStartConv(adc.ADCx);
}
#endif

