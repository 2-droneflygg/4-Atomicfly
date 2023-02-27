/*********************************************************************************
 提供一系列串口操作API（标准串行驱动程序API）。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/system.h"

#include "serial.h"

/**********************************************************************
函数名称：serialSetBaudRate
函数功能：设置串口波特率
函数形参：串口标识，波特率
函数返回值：None
函数描述：None
**********************************************************************/
void serialSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    instance->vTable->serialSetBaudRate(instance, baudRate);
}

/**********************************************************************
函数名称：serialTxBytesFree
函数功能：获取串口发送缓冲区空闲字节数
函数形参：串口标识
函数返回值：发送缓冲区空闲字节数
函数描述：
	如果缓冲区已满，则进行发送释放。
**********************************************************************/
uint32_t serialTxBytesFree(const serialPort_t *instance)
{
    return instance->vTable->serialTotalTxFree(instance);
}

/**********************************************************************
函数名称：serialRxBytesWaiting
函数功能：串口接收缓冲区数量
函数形参：串口标识
函数返回值：接收缓冲区数量
函数描述：
	用于获取接收缓冲区是否有数据（是否接收到数据）。
**********************************************************************/
uint32_t serialRxBytesWaiting(const serialPort_t *instance)
{
    return instance->vTable->serialTotalRxWaiting(instance);
}

/**********************************************************************
函数名称：isSerialTransmitBufferEmpty
函数功能：串行传输缓冲区是否为空(是否传输完成)
函数形参：串口标识
函数返回值：状态
函数描述：None
**********************************************************************/
bool isSerialTransmitBufferEmpty(const serialPort_t *instance)
{
    return instance->vTable->isSerialTransmitBufferEmpty(instance);
}

/**********************************************************************
函数名称：waitForSerialPortToFinishTransmitting
函数功能：等待串行端口完成传输
函数形参：serialPort_t
函数返回值：None
函数描述：None 
**********************************************************************/
void waitForSerialPortToFinishTransmitting(serialPort_t *serialPort)
{
    while (!isSerialTransmitBufferEmpty(serialPort)) {
        delay(10);
    };
}

/**********************************************************************
函数名称：serialWrite
函数功能：写串口
函数形参：串口标识，字符
函数返回值：None
函数描述：None
**********************************************************************/
void serialWrite(serialPort_t *instance, uint8_t ch)
{
    instance->vTable->serialWrite(instance, ch);
}

/**********************************************************************
函数名称：serialRead
函数功能：读串口
函数形参：串口标识
函数返回值：数据
函数描述：None
**********************************************************************/
uint8_t serialRead(serialPort_t *instance)
{
    return instance->vTable->serialRead(instance);
}

/**********************************************************************
函数名称：serialWriteBuf
函数功能：串口写缓存
函数形参：串口标识，数据，数据长度
函数返回值：None
函数描述：None
**********************************************************************/
void serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count)
{
    for (const uint8_t *p = data; count > 0; count--, p++) {
        while (!serialTxBytesFree(instance)) {};
        serialWrite(instance, *p);
    }
}

/**********************************************************************
函数名称：serialBeginWrite
函数功能：串口开始大容量写模式
函数形参：串口标识
函数返回值：None
函数描述：None
**********************************************************************/
void serialBeginWrite(serialPort_t *instance)
{
    if (instance->vTable->beginWrite)
        instance->vTable->beginWrite(instance);
}

/**********************************************************************
函数名称：serialEndWrite
函数功能：串口结束大容量写模式
函数形参：串口标识
函数返回值：None
函数描述：None
**********************************************************************/
void serialEndWrite(serialPort_t *instance)
{
    if (instance->vTable->endWrite)
        instance->vTable->endWrite(instance);
}

/**********************************************************************
函数名称：serialPrint
函数功能：串口打印
函数形参：串口标识，字符串
函数返回值：None
函数描述：None
**********************************************************************/
void serialPrint(serialPort_t *instance, const char *str)
{
    uint8_t ch;
    while ((ch = *(str++)) != 0) {
        serialWrite(instance, ch);
    }
}

