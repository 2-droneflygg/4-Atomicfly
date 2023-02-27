/**********************************************************************
 SBUS通道数据解析：
**********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_SBUS_CHANNELS
#include "common/utils.h"

#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/sbus_channels.h"

#define SBUS_FLAG_CHANNEL_17        (1 << 0)		// SBUS通道17标志
#define SBUS_FLAG_CHANNEL_18        (1 << 1)		// SBUS通道18标志

#define SBUS_DIGITAL_CHANNEL_MIN 173				// SBUS数字通道范围最小值
#define SBUS_DIGITAL_CHANNEL_MAX 1812				// SBUS数字通道范围最大值

/**********************************************************************
函数名称：sbusChannelsDecode
函数功能：SBUS通道解码
函数形参：RX运行状态结构体,SBUS通道结构体
函数返回值：RX帧状态
函数描述：None
**********************************************************************/
uint8_t sbusChannelsDecode(rxRuntimeState_t *rxRuntimeState, const sbusChannels_t *channels)
{
	// 将通道数据放入RX运行状态结构体
    uint16_t *sbusChannelData = rxRuntimeState->channelData; 
    sbusChannelData[0] = channels->chan0;
    sbusChannelData[1] = channels->chan1;
    sbusChannelData[2] = channels->chan2;
    sbusChannelData[3] = channels->chan3;
    sbusChannelData[4] = channels->chan4;
    sbusChannelData[5] = channels->chan5;
    sbusChannelData[6] = channels->chan6;
    sbusChannelData[7] = channels->chan7;
    sbusChannelData[8] = channels->chan8;
    sbusChannelData[9] = channels->chan9;
    sbusChannelData[10] = channels->chan10;
    sbusChannelData[11] = channels->chan11;
    sbusChannelData[12] = channels->chan12;
    sbusChannelData[13] = channels->chan13;
    sbusChannelData[14] = channels->chan14;
    sbusChannelData[15] = channels->chan15;

    if (channels->flags & SBUS_FLAG_CHANNEL_17) {
        sbusChannelData[16] = SBUS_DIGITAL_CHANNEL_MAX;
    } else {
        sbusChannelData[16] = SBUS_DIGITAL_CHANNEL_MIN;
    }

    if (channels->flags & SBUS_FLAG_CHANNEL_18) {
        sbusChannelData[17] = SBUS_DIGITAL_CHANNEL_MAX;
    } else {
        sbusChannelData[17] = SBUS_DIGITAL_CHANNEL_MIN;
    }

	// 失控保护启用和rx故障安全标志设置
    if (channels->flags & SBUS_FLAG_FAILSAFE_ACTIVE) {
        return RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE;	
    }

	// 数据丢失
    if (channels->flags & SBUS_FLAG_SIGNAL_LOSS) {  
        return RX_FRAME_COMPLETE | RX_FRAME_DROPPED;
    }

    return RX_FRAME_COMPLETE;
}

/**********************************************************************
函数名称：sbusChannelsReadRawRC
函数功能：读取SBUS通道原始RC
函数形参：RX运行状态结构体,通道
函数返回值：从OpenTX-ppmus读取的线性拟合值，并与X4R接收的值进行比较
函数描述：None
**********************************************************************/
static uint16_t sbusChannelsReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    return (5 * rxRuntimeState->channelData[chan] / 8) + 880;
}

/**********************************************************************
函数名称：sbusChannelsReadRawRC
函数功能：SBUS通道初始化
函数形参：rxConfig,rxRuntimeState
函数返回值：None
函数描述：None
**********************************************************************/
void sbusChannelsInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
	// SBUS通道读取原始RC函数注册到RX运行状态结构体
    rxRuntimeState->rcReadRawFn = sbusChannelsReadRawRC;
	// 遍历SBUS所有通道 - 初始化SBU通道数据 
    for (int b = 0; b < SBUS_MAX_CHANNEL; b++) {
        rxRuntimeState->channelData[b] = (16 * rxConfig->midrc) / 10 - 1408; // 992
    }
}
#endif

