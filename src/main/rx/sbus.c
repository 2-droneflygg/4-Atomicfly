/**********************************************************************
协议解析：
	通信接口：USART（TTL）
	通信参数：1个起始位+8个数据位+偶校验位+2个停止位，波特率 = 100000bit/s,电平逻辑反转
	通信速率：每14ms（模拟模式）或7ms（高速模式）发送，即数据帧间隔为 11ms（模拟模式）或4ms（高速模式）
	数据帧格式：
		SBUS数据包的长度为25个字节：
			字节[0]：0x0F（SBUS头）
			字节[1-22]：16个数据通道，每个通道采用11位编码
			字节[23]：
	          位7：数字通道17（0x80）				10000000
	          位6：数字通道18（0x40）				10000000
	          位5：丢帧（0x20）					00100000
	          位4：用来激活故障安全（0x10） 00010000
	          位0-3：n/a
			字节[24]：SBUS结束字节，0x00 
	通道数据低位在前，高位在后，每个通道取11位，具体协议如下：
		22个字节就可以表示16通道（8 × 22 = 11 × 16）。11个bit可以表示的数值范围为0～2047，每帧25个字节，
		start byte = 0x0F
		CH1 = [data0]的8位 + [data1]的低3位
		CH2 = [data1]的高5位 + [data2]的低6位
		... ...
		end byte = 0x00
		以此类推。
**********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_SERIALRX_SBUS
#include "common/utils.h"

#include "drivers/time.h"

#include "io/serial.h"

#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/sbus.h"
#include "rx/sbus_channels.h"

// ---------------------------------------------------------SBUS波特率
#define SBUS_BAUDRATE                   100000					
// ---------------------------------------------------------SBUS RX刷新率
#define SBUS_RX_REFRESH_RATE            11000					// 
// ---------------------------------------------------------每帧所需的SBUS时间
#define SBUS_TIME_NEEDED_PER_FRAME      3000		    		

// ---------------------------------------------------------SBUS快速波特率
#define SBUS_FAST_BAUDRATE              200000					
// ---------------------------------------------------------SBUS快速RX刷新率
#define SBUS_FAST_RX_REFRESH_RATE       6000					

// ---------------------------------------------------------SBUS失控保护状态
#define SBUS_STATE_FAILSAFE   (1 << 0)								
// ---------------------------------------------------------SBUS信号丢失状态
#define SBUS_STATE_SIGNALLOSS (1 << 1)				    		

// ---------------------------------------------------------SBUS帧大小 - 起始字节1 + 通道字节22 + 标志位1 + 结束字节1
#define SBUS_FRAME_SIZE (SBUS_CHANNEL_DATA_LENGTH + 2)	

// ---------------------------------------------------------SBUS帧开始字节
#define SBUS_FRAME_BEGIN_BYTE 			0x0F						

// ---------------------------------------------------------SBUS端口选项
#if !defined(SBUS_PORT_OPTIONS)
#define SBUS_PORT_OPTIONS (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN)
#endif

// ---------------------------------------------------------SBUS数字通道最小值
#define SBUS_DIGITAL_CHANNEL_MIN 173						
// ---------------------------------------------------------SBUS数字通道最大值
#define SBUS_DIGITAL_CHANNEL_MAX 1812							

// ---------------------------------------------------------记录上一个RC帧时间戳
static timeUs_t lastRcFrameTimeUs = 0;			

/* -------------------------SBUS帧结构体------------------------- */	
struct sbusFrame_s {
    uint8_t syncByte;								// 字节起始
    sbusChannels_t channels;						// 通道字节+标志位
    uint8_t endByte;								// 结束字节
} __attribute__ ((__packed__));

/* -------------------------SBUS帧共用体------------------------- */	
// 成员共用一个内存空间.
typedef union sbusFrame_u {
    uint8_t bytes[SBUS_FRAME_SIZE];					// 字节
    struct sbusFrame_s frame;						// 帧
} sbusFrame_t;

/* -----------------------SBUS数据帧结构体体--------------------- */	
typedef struct sbusFrameData_s {
    sbusFrame_t frame;								// 帧			
    timeUs_t startAtUs;								// 起始时间(us)
    uint8_t position;								// 位置
    bool done;										// 完成
} sbusFrameData_t;

/**********************************************************************
函数名称：sbusDataReceive
函数功能：SBUS数据接收
函数形参：数据（USARTx->DR）,SBUS数据帧存储
函数返回值：None
函数描述：
	串口接收中断回调函数.
**********************************************************************/
static void sbusDataReceive(uint16_t c, void *data)
{
	// 获取SBUS数据帧存储
    sbusFrameData_t *sbusFrameData = data;                          

	// 获取当前时间节拍
    const timeUs_t nowUs = microsISR();							    

	// 获取SBUS帧时间 - 当前时间 - SBUS数据帧起始时间
    const timeDelta_t sbusFrameTime = cmpTimeUs(nowUs, sbusFrameData->startAtUs);
	
	// 如果SBUS帧时间 > 每帧所需的SBUS时间
    if (sbusFrameTime > (long)(SBUS_TIME_NEEDED_PER_FRAME + 500)) {	
		// 重置SBUS数据帧位置为0
        sbusFrameData->position = 0;								
    }

	// SBUS数据帧在起始位置
    if (sbusFrameData->position == 0) {
		// 判断SBUS帧开始字节
        if (c != SBUS_FRAME_BEGIN_BYTE) {
            return;
        }
		// SBUS数据帧起始时间 = 当前时间  
        sbusFrameData->startAtUs = nowUs;   						
    }

	// 判断SBUS帧是否接收完成
    if (sbusFrameData->position < SBUS_FRAME_SIZE) {
		// 缓存到数据帧存储
        sbusFrameData->frame.bytes[sbusFrameData->position++] = (uint8_t)c;
        if (sbusFrameData->position < SBUS_FRAME_SIZE) {
			// SBUS数据帧接收未完毕
            sbusFrameData->done = false;							
        } else {
        	// SBUS数据帧接收完毕
            sbusFrameData->done = true;							    
        }
    }
}

/**********************************************************************
函数名称：sbusFrameStatus
函数功能：SBUS帧状态
函数形参：rxChannelRangeConfig
函数返回值：帧状态
函数描述：
	ISR接收回调 - rxRuntimeState_t回调函数.
**********************************************************************/
static uint8_t sbusFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
	// 获取SBUS数据帧
    sbusFrameData_t *sbusFrameData = rxRuntimeState->frameData;    	
    // 判断RX数据帧是否接收完成
    if (!sbusFrameData->done) {
		// RX帧等待
        return RX_FRAME_PENDING;									
    }
	// RX帧重置为未完成，为下一帧数据做准备
    sbusFrameData->done = false;									

	// SBUS通道解码并获取帧状态
    const uint8_t frameStatus = sbusChannelsDecode(rxRuntimeState, &sbusFrameData->frame.frame.channels);
	// 未发生失控保护和丢失帧
    if (!(frameStatus & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED))) {
		// 更新RC帧时间 - 本次帧起始时间（us）
        lastRcFrameTimeUs = sbusFrameData->startAtUs;
    }
    return frameStatus;
}

/**********************************************************************
函数名称：sbusFrameTimeUs
函数功能：SBUS帧时间
函数形参：None
函数返回值：上一个RC帧时间戳
函数描述：None
**********************************************************************/
static timeUs_t sbusFrameTimeUs(void)
{
    return lastRcFrameTimeUs;
}

/**********************************************************************
函数名称：sbusInit
函数功能：SBUS初始化
函数形参：rxConfig,rxRuntimeState
函数返回值：SBUS端口是否存在
函数描述：None
**********************************************************************/
bool sbusInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
	// SBUS通道数据
    static uint16_t sbusChannelData[SBUS_MAX_CHANNEL];			
	// SBUS数据帧
    static sbusFrameData_t sbusFrameData;			
	// SBUS波特率
    static uint32_t sbusBaudRate;	
	
	// SBUS通道数据数组注册到RX运行状态结构体
    rxRuntimeState->channelData = sbusChannelData;			
	// SBUS数据帧注册到RX运行状态结构体
    rxRuntimeState->frameData = &sbusFrameData;						

	// SBUS通道初始化
    sbusChannelsInit(rxConfig, rxRuntimeState);						

	// 配置最大通道数
    rxRuntimeState->channelCount = SBUS_MAX_CHANNEL;				

	// 配置SBUS刷新率和波特率
    if (rxConfig->sbus_baud_fast) {						
		// 快速波特率
        rxRuntimeState->rxRefreshRate = SBUS_FAST_RX_REFRESH_RATE;  
        sbusBaudRate  = SBUS_FAST_BAUDRATE;							
    } else {												
		// 非快速波特率
        rxRuntimeState->rxRefreshRate = SBUS_RX_REFRESH_RATE;
        sbusBaudRate  = SBUS_BAUDRATE;
    }

	// 注册RC帧状态函数
    rxRuntimeState->rcFrameStatusFn = sbusFrameStatus;		
	// 注册检索最后一个通道数据帧的时间戳(以微秒为单位)函数
    rxRuntimeState->rcFrameTimeUsFn = sbusFrameTimeUs;				

	// 搜索配置为串行接收机功能的串口
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

	// 端口不共享 - MODE_RX
    bool portShared = false;										

	// 打开串口并进行初始化
    serialPort_t *sBusPort = openSerialPort(portConfig->identifier,
	        FUNCTION_RX_SERIAL,
	        sbusDataReceive,
	        &sbusFrameData,
	        sbusBaudRate,
	        portShared ? MODE_RXTX : MODE_RX,
	        SBUS_PORT_OPTIONS | (rxConfig->serialrx_inverted ? 0 : SERIAL_INVERTED) | (rxConfig->halfDuplex ? SERIAL_BIDIR : 0)
        );

    return sBusPort != NULL;
}
#endif

