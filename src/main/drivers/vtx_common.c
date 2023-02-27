/*********************************************************************************
 提供图传设备通用配置操作接口API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>

#include "platform.h"

#if defined(USE_VTX_COMMON)

#include "common/time.h"
#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"


static vtxDevice_t *vtxDevice = NULL;			// 图传设备
static uint8_t selectedBand = 0;				// 选择频组
static uint8_t selectedChannel = 0;				// 选择通道

/**********************************************************************
函数名称：vtxCommonSetDevice
函数功能：获取图传设备是否就绪
函数形参：图传设备
函数返回值：就绪状态
函数描述：None
**********************************************************************/
bool vtxCommonDeviceIsReady(const vtxDevice_t *vtxDevice)
{
    if (vtxDevice && vtxDevice->vTable->isReady) {
        return vtxDevice->vTable->isReady(vtxDevice);
    }
    return false;
}

/**********************************************************************
函数名称：vtxCommonSetDevice
函数功能：设置图传设备
函数形参：pDevice
函数返回值：None
函数描述：None
**********************************************************************/
void vtxCommonSetDevice(vtxDevice_t *pDevice)
{
    vtxDevice = pDevice;
}

/**********************************************************************
函数名称：vtxCommonDevice
函数功能：获取图传设备
函数形参：None
函数返回值：vtxDevice
函数描述：None
**********************************************************************/
vtxDevice_t *vtxCommonDevice(void)
{
    return vtxDevice;
}

/**********************************************************************
函数名称：vtxCommonGetDeviceType
函数功能：获取图传设备类型
函数形参：vtxDevice
函数返回值：vtxDevType_e
函数描述：None
**********************************************************************/
vtxDevType_e vtxCommonGetDeviceType(const vtxDevice_t *vtxDevice)
{
    if (!vtxDevice) {
        return VTXDEV_UNKNOWN;
    }
    return vtxDevice->vTable->getDeviceType(vtxDevice);
}

/**********************************************************************
函数名称：vtxCommonProcess
函数功能：图传线程
函数形参：vtxDevice，currentTimeUs
函数返回值：None
函数描述：None
**********************************************************************/
void vtxCommonProcess(vtxDevice_t *vtxDevice, timeUs_t currentTimeUs)
{
    if (vtxDevice) {
        vtxDevice->vTable->process(vtxDevice, currentTimeUs);
    }
}

/**********************************************************************
函数名称：vtxCommonSetBandAndChannel
函数功能：设置频组和通道
函数形参：图传设备，频组，通道
函数返回值：None
函数描述：None
**********************************************************************/
void vtxCommonSetBandAndChannel(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel)
{
	// 查找频率
    uint16_t freq = vtxCommonLookupFrequency(vtxDevice, band, channel);
    if (freq != 0) {
        selectedChannel = channel;
        selectedBand = band;
		// 设置频组和通道
        if (vtxTableIsFactoryBand[band - 1]) {
            vtxDevice->vTable->setBandAndChannel(vtxDevice, band, channel);
        } else {
            vtxDevice->vTable->setFrequency(vtxDevice, freq);
        }
    }
}

/**********************************************************************
函数名称：vtxCommonSetPowerByIndex
函数功能：通过索引设置功率
函数形参：vtxDevice，index
函数返回值：None
函数描述：
	0 = 未知功率。
**********************************************************************/
void vtxCommonSetPowerByIndex(vtxDevice_t *vtxDevice, uint8_t index)
{
    if (index <= vtxTablePowerLevels) {
        vtxDevice->vTable->setPowerByIndex(vtxDevice, index);
    }
}

/**********************************************************************
函数名称：vtxCommonSetPitMode
函数功能：设置为PIT模式
函数形参：vtxDevice，onOff
函数返回值：None
函数描述：
	on = 1, off = 0
**********************************************************************/
void vtxCommonSetPitMode(vtxDevice_t *vtxDevice, uint8_t onOff)
{
    vtxDevice->vTable->setPitMode(vtxDevice, onOff);
}

/**********************************************************************
函数名称：vtxCommonSetFrequency
函数功能：设置频率
函数形参：vtxDevice，frequency
函数返回值：None
函数描述：None
**********************************************************************/
void vtxCommonSetFrequency(vtxDevice_t *vtxDevice, uint16_t frequency)
{
    selectedBand = 0;
    selectedChannel = 0;
    vtxDevice->vTable->setFrequency(vtxDevice, frequency);
}

/**********************************************************************
函数名称：vtxCommonGetBandAndChannel
函数功能：获取频组和频点状态
函数形参：vtxDevice，pBand，pChannel
函数返回值：result
函数描述：None
**********************************************************************/
bool vtxCommonGetBandAndChannel(const vtxDevice_t *vtxDevice, uint8_t *pBand, uint8_t *pChannel)
{
    bool result = vtxDevice->vTable->getBandAndChannel(vtxDevice, pBand, pChannel);
    if ((!result || (*pBand == 0 && *pChannel == 0)) && selectedBand != 0 && selectedChannel != 0 && !vtxTableIsFactoryBand[selectedBand - 1]) {
        uint16_t freq;
        result = vtxCommonGetFrequency(vtxDevice, &freq);
        if (result) {
            vtxCommonLookupBandChan(vtxDevice, freq, pBand, pChannel);
        }
    }
    return result;
}

/**********************************************************************
函数名称：vtxCommonGetPowerIndex
函数功能：获取功率索引
函数形参：vtxDevice，pIndex
函数返回值：result
函数描述：None
**********************************************************************/
bool vtxCommonGetPowerIndex(const vtxDevice_t *vtxDevice, uint8_t *pIndex)
{
    return vtxDevice->vTable->getPowerIndex(vtxDevice, pIndex);
}

/**********************************************************************
函数名称：vtxCommonGetFrequency
函数功能：获取频率
函数形参：vtxDevice，pFrequency
函数返回值：result
函数描述：None
**********************************************************************/
bool vtxCommonGetFrequency(const vtxDevice_t *vtxDevice, uint16_t *pFrequency)
{
    return vtxDevice->vTable->getFrequency(vtxDevice, pFrequency);
}

/**********************************************************************
函数名称：vtxCommonGetStatus
函数功能：获取状态
函数形参：vtxDevice，status
函数返回值：result
函数描述：None
**********************************************************************/
bool vtxCommonGetStatus(const vtxDevice_t *vtxDevice, unsigned *status)
{
    return vtxDevice->vTable->getStatus(vtxDevice, status);
}

/**********************************************************************
函数名称：vtxCommonLookupBandLetter
函数功能：查找频组字母
函数形参：vtxDevice，band
函数返回值：result
函数描述：None
**********************************************************************/
char vtxCommonLookupBandLetter(const vtxDevice_t *vtxDevice, int band)
{
    if (vtxDevice && band > 0 && band <= vtxTableBandCount) {
        return vtxTableBandLetters[band];
    } else {
        return '?';
    }
}

/**********************************************************************
函数名称：vtxCommonLookupChannelName
函数功能：查找通道名称
函数形参：vtxDevice，channel
函数返回值：result
函数描述：None
**********************************************************************/
const char *vtxCommonLookupChannelName(const vtxDevice_t *vtxDevice, int channel)
{
    if (vtxDevice && channel > 0 && channel <= vtxTableChannelCount) {
        return vtxTableChannelNames[channel];
    } else {
        return "?";
    }
}

/**********************************************************************
函数名称：vtxCommonLookupBandChan
函数功能：查找频组通道
函数形参：vtxDevice，freq，pBand，pChannel
函数返回值：result
函数描述：
	将频率(MHz)转换为频带和信道值
	如果在vtxtable中找不到频率，那么band和channel将返回0
**********************************************************************/
void vtxCommonLookupBandChan(const vtxDevice_t *vtxDevice, uint16_t freq, uint8_t *pBand, uint8_t *pChannel)
{
    *pBand = 0;
    *pChannel = 0;
    if (vtxDevice) {
		// 使用反向查找的顺序，使5880Mhz，get Raceband 7而不是Fatshark 8。
        for (int band = vtxTableBandCount - 1 ; band >= 0 ; band--) {
            for (int channel = 0 ; channel < vtxTableChannelCount ; channel++) {
                if (vtxTableFrequency[band][channel] == freq) {
                    *pBand = band + 1;
                    *pChannel = channel + 1;
                    return;
                }
            }
        }
    }
}

/**********************************************************************
函数名称：vtxCommonLookupFrequency
函数功能：查找频率
函数形参：vtxDevice，freq，band，channel
函数返回值：返回频率值(MHz)，如果频带/信道超出范围则返回0
函数描述：
	将频带和信道值转换为频率(MHz)值：
		band: band值(1 ~ 5)
		channel: channel值(1 ~ 8)
**********************************************************************/
uint16_t vtxCommonLookupFrequency(const vtxDevice_t *vtxDevice, int band, int channel)
{
    if (vtxDevice) {
        if (band > 0 && band <= vtxTableBandCount && channel > 0 && channel <= vtxTableChannelCount) {
            return vtxTableFrequency[band - 1][channel - 1];
        }
    }

    return 0;
}

/**********************************************************************
函数名称：vtxCommonLookupPowerName
函数功能：查找功率标签
函数形参：vtxDevice，index
函数返回值：result
函数描述：None
**********************************************************************/
const char *vtxCommonLookupPowerName(const vtxDevice_t *vtxDevice, int index)
{
    if (vtxDevice && index > 0 && index <= vtxTablePowerLevels) {
        return vtxTablePowerLabels[index];
    } else {
        return "?";
    }
}

/**********************************************************************
函数名称：vtxCommonLookupPowerValue
函数功能：查找功率值
函数形参：vtxDevice，index，pPowerValue
函数返回值：状态
函数描述：None
**********************************************************************/
bool vtxCommonLookupPowerValue(const vtxDevice_t *vtxDevice, int index, uint16_t *pPowerValue)
{
    if (vtxDevice && index > 0 && index <= vtxTablePowerLevels) {
        *pPowerValue = vtxTablePowerValues[index - 1];
        return true;
    } else {
        return false;
    }
}
#endif

