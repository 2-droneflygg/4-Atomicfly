/*********************************************************************************
 提供一系列ADC配置操作API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "common/utils.h"

#ifdef USE_ADC
#include "build/build_config.h"

#include "drivers/adc_impl.h"
#include "drivers/io.h"

#include "pg/adc.h"

#include "adc.h"

// ---------------------------------------------------------定义ADC配置信息存储数组
adcOperatingConfig_t adcOperatingConfig[ADC_CHANNEL_COUNT];	 
// ---------------------------------------------------------定义ADC通道值缓存数组
volatile uint16_t adcValues[ADC_CHANNEL_COUNT];				 

/**********************************************************************
函数名称：adcChannelByTag
函数功能：根据IO标签获取ADC通道
函数形参：None  
函数返回值：成功返回ADC通道，失败返回0
函数描述：None 
**********************************************************************/
uint8_t adcChannelByTag(ioTag_t ioTag)
{
	// 遍历ADC标签映射
    for (uint8_t i = 0; i < ARRAYLEN(adcTagMap); i++) {
        if (ioTag == adcTagMap[i].tag)
            return adcTagMap[i].channel;
    }
    return 0;
}

/**********************************************************************
函数名称：adcDeviceByInstance
函数功能：实例化ADC设备
函数形参：ADCx 
函数返回值：ADC设备索引（ID）
函数描述：None 
**********************************************************************/
ADCDevice adcDeviceByInstance(ADC_TypeDef *instance)
{
    if (instance == ADC1) {
        return ADCDEV_1;
    }
    if (instance == ADC2) {
        return ADCDEV_2;
    }
    if (instance == ADC3) {
        return ADCDEV_3;
    }
    return ADCINVALID;
}

/**********************************************************************
函数名称：adcGetChannel
函数功能：获取ADC通道数据
函数形参：None  
函数返回值：adcValues
函数描述：None 
**********************************************************************/
uint16_t adcGetChannel(uint8_t channel)
{
	// 读取对应的ADC_DMA缓冲区
    return adcValues[adcOperatingConfig[channel].dmaIndex];
}

/**********************************************************************
函数名称：adcVerifyPin
函数功能：验证由IO标签指定的引脚是否映射到由ADC设备指定的ADC实例
函数形参：IO标签，ADC设备索引（ID）
函数返回值：result
函数描述：None 
**********************************************************************/
bool adcVerifyPin(ioTag_t tag, ADCDevice device)
{
	// 检查合法性
    if (!tag) {
        return false;
    }
	// 遍历所有标签映射
    for (int map = 0 ; map < ADC_TAG_MAP_COUNT ; map++) {
        if ((adcTagMap[map].tag == tag) && (adcTagMap[map].devices & (1 << device))) {
            return true;
        }
    }
    return false;
}
#endif

