/*********************************************************************************
 提供一系列IIC总线API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"

// 使用硬件IIC
#if defined(USE_I2C)
/**********************************************************************
函数名称：i2cBusWriteRegister
函数功能：IIC总线写寄存器
函数形参：IIC设备，寄存器，数据
函数返回值：成功：true，失败：false
函数描述：None 
**********************************************************************/
bool i2cBusWriteRegister(const busDevice_t *busdev, uint8_t reg, uint8_t data)
{
    return i2cWrite(busdev->busdev_u.i2c.device, busdev->busdev_u.i2c.address, reg, data);
}

/**********************************************************************
函数名称：i2cBusWriteRegisterStart
函数功能：IIC总线开始写寄存器
函数形参：IIC设备，寄存器，数据
函数返回值：成功：true，失败：false
函数描述：None 
**********************************************************************/
bool i2cBusWriteRegisterStart(const busDevice_t *busdev, uint8_t reg, uint8_t data)
{
    static uint8_t byte;
    byte = data;
    return i2cWriteBuffer(busdev->busdev_u.i2c.device, busdev->busdev_u.i2c.address, reg, sizeof (byte), &byte);
}

/**********************************************************************
函数名称：i2cBusReadRegisterBuffer
函数功能：IIC总线读寄存器缓存
函数形参：IIC设备，寄存器，数据，长度
函数返回值：成功：true，失败：false
函数描述：None 
**********************************************************************/
bool i2cBusReadRegisterBuffer(const busDevice_t *busdev, uint8_t reg, uint8_t *data, uint8_t length)
{
    return i2cRead(busdev->busdev_u.i2c.device, busdev->busdev_u.i2c.address, reg, length, data);
}

/**********************************************************************
函数名称：i2cBusReadRegister
函数功能：IIC总线读寄存器
函数形参：IIC设备，寄存器
函数返回值：数据
函数描述：None 
**********************************************************************/
uint8_t i2cBusReadRegister(const busDevice_t *busdev, uint8_t reg)
{
    uint8_t data;
    i2cRead(busdev->busdev_u.i2c.device, busdev->busdev_u.i2c.address, reg, 1, &data);
    return data;
}

/**********************************************************************
函数名称：i2cBusReadRegisterBufferStart
函数功能：IIC总线开始读寄存器
函数形参：IIC设备，寄存器，数据，长度
函数返回值：成功：true，失败：false
函数描述：None 
**********************************************************************/
bool i2cBusReadRegisterBufferStart(const busDevice_t *busdev, uint8_t reg, uint8_t *data, uint8_t length)
{
    return i2cReadBuffer(busdev->busdev_u.i2c.device, busdev->busdev_u.i2c.address, reg, length, data);
}

/**********************************************************************
函数名称：i2cBusBusy
函数功能：判断IIC总线是否忙碌
函数形参：IIC设备，错误指针
函数返回值：IIC总线是否忙碌
函数描述：None 
**********************************************************************/
bool i2cBusBusy(const busDevice_t *busdev, bool *error)
{
    return i2cBusy(busdev->busdev_u.i2c.device, error);
}
#endif

