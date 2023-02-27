/*********************************************************************************
 提供一系列总线接口API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"

/**********************************************************************
函数名称：busWriteRegister
函数功能：总线写寄存器
函数形参：总线设备，寄存器，数据
函数返回值：是否写成功
函数描述：None
**********************************************************************/
bool busWriteRegister(const busDevice_t *busdev, uint8_t reg, uint8_t data)
{
	// 判断合法性
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
#endif
	// 判断总线类型
    switch (busdev->bustype) {
#ifdef USE_SPI
	    case BUSTYPE_SPI:
	        return spiBusWriteRegister(busdev, reg & 0x7f, data);
#endif
#ifdef USE_I2C
	    case BUSTYPE_I2C:
	        return i2cBusWriteRegister(busdev, reg, data);
#endif
	    default:
	        return false;
    }
}

/**********************************************************************
函数名称：busWriteRegisterStart
函数功能：总线开始写寄存器
函数形参：总线设备，寄存器，数据
函数返回值：如果总线仍然繁忙，则返回true
函数描述：
	针对IIC设备。
**********************************************************************/
bool busWriteRegisterStart(const busDevice_t *busdev, uint8_t reg, uint8_t data)
{
	// 判断合法性
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
#endif
	// 判断总线类型
    switch (busdev->bustype) {
#ifdef USE_SPI
	    case BUSTYPE_SPI:
	        return spiBusWriteRegister(busdev, reg & 0x7f, data);
#endif
#ifdef USE_I2C
	    case BUSTYPE_I2C:
	        return i2cBusWriteRegisterStart(busdev, reg, data);
#endif
	    default:
	        return false;
    }
}

/**********************************************************************
函数名称：busReadRegisterBuffer
函数功能：总线读寄存器
函数形参：总线设备，寄存器，数据，长度
函数返回值：是否读成功
函数描述：None
**********************************************************************/
bool busReadRegisterBuffer(const busDevice_t *busdev, uint8_t reg, uint8_t *data, uint8_t length)
{
	// 判断合法性
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
    UNUSED(length);
#endif
    switch (busdev->bustype) {
#ifdef USE_SPI
	    case BUSTYPE_SPI:
	        return spiBusReadRegisterBuffer(busdev, reg | 0x80, data, length);
#endif
#ifdef USE_I2C
	    case BUSTYPE_I2C:
	        return i2cBusReadRegisterBuffer(busdev, reg, data, length);
#endif
	    default:
	        return false;
    }
}

/**********************************************************************
函数名称：busReadRegisterBufferStart
函数功能：总线开始读寄存器
函数形参：busdev，reg，data，length
函数返回值：如果总线仍然繁忙，则返回true
函数描述：
	针对IIC设备。
**********************************************************************/
bool busReadRegisterBufferStart(const busDevice_t *busdev, uint8_t reg, uint8_t *data, uint8_t length)
{
	// 判断合法性
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
    UNUSED(length);
#endif
    switch (busdev->bustype) {
#ifdef USE_SPI
	    case BUSTYPE_SPI:
	        // 对于SPI，允许事务完成
	        return spiBusReadRegisterBuffer(busdev, reg | 0x80, data, length);
#endif
#ifdef USE_I2C
	    case BUSTYPE_I2C: 
			// 启动I2C读取，但不要等待完成
	        // 发起读访问
	        return i2cBusReadRegisterBufferStart(busdev, reg, data, length);
#endif
	    default:
	        return false;
    }
}

/**********************************************************************
函数名称：busBusy
函数功能：判断总线是否忙碌
函数形参：总线设备，获取错误状态
函数返回值：如果总线仍然繁忙，则返回true
函数描述：
	针对IIC设备。
**********************************************************************/
bool busBusy(const busDevice_t *busdev, bool *error)
{
	// 判断合法性
#if !defined(USE_I2C)
    UNUSED(error);
#endif
    switch (busdev->bustype) {
#ifdef USE_SPI
    case BUSTYPE_SPI:
        // 不等待SPI
        return false;
#endif

#ifdef USE_I2C
    case BUSTYPE_I2C:
        return i2cBusBusy(busdev, error);
#endif

    default:
        return false;
    }
}

