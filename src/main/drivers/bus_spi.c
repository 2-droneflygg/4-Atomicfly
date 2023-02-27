/*********************************************************************************
 提供SPI总线操作API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SPI
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"

// ---------------------------------------------------------SPI设备信息存储数组
spiDevice_t spiDevice[SPIDEV_COUNT];

/**********************************************************************
函数名称：spiDeviceByInstance
函数功能：SPI设备实例化
函数形参：SPIx
函数返回值：实例化SPI设备（ID）
函数描述：None
**********************************************************************/
SPIDevice spiDeviceByInstance(SPI_TypeDef *instance)
{
#ifdef USE_SPI_DEVICE_1
    if (instance == SPI1) return SPIDEV_1;
#endif
#ifdef USE_SPI_DEVICE_2
    if (instance == SPI2) return SPIDEV_2;
#endif
#ifdef USE_SPI_DEVICE_3
    if (instance == SPI3) return SPIDEV_3;
#endif
    return SPIINVALID;
}

/**********************************************************************
函数名称：spiInstanceByDevice
函数功能：获取SPIx设备信息
函数形参：SPI设备（ID）
函数返回值：SPI设备信息
函数描述：None
**********************************************************************/
SPI_TypeDef *spiInstanceByDevice(SPIDevice device)
{
	// 检查合法性
    if (device == SPIINVALID || device >= SPIDEV_COUNT) {
        return NULL;
    }
    return spiDevice[device].dev;
}

/**********************************************************************
函数名称：spiInit
函数功能：SPI初始化
函数形参：SPI设备（ID），边沿模式
函数返回值：状态
函数描述：None
**********************************************************************/
bool spiInit(SPIDevice device, bool leadingEdge)
{
    switch (device) {
	    case SPIINVALID:
	        return false;
	    case SPIDEV_1:
#ifdef USE_SPI_DEVICE_1
	        spiInitDevice(device, leadingEdge);
	        return true;
#else
       		break;
#endif
   		 case SPIDEV_2:
#ifdef USE_SPI_DEVICE_2
        	spiInitDevice(device, leadingEdge);
        	return true;
#else
        	break;
#endif
    	case SPIDEV_3:
#if defined(USE_SPI_DEVICE_3) && !defined(STM32F1)
      	  spiInitDevice(device, leadingEdge);
      	  return true;
#else
       	 break;
#endif
    }
    return false;
}

/**********************************************************************
函数名称：SPITimeoutUserCallback
函数功能：SPI设备传输超时用户回调
函数形参：SPIx
函数返回值：SPI设备错误数量
函数描述：None
**********************************************************************/
uint32_t spiTimeoutUserCallback(SPI_TypeDef *instance)
{
	// 获取SPI设备（ID）
    SPIDevice device = spiDeviceByInstance(instance);
	// 判断合法性
    if (device == SPIINVALID) {
        return -1;
    }
	// SPI设备错误数量计数
    spiDevice[device].errorCount++;
    return spiDevice[device].errorCount;
}

/**********************************************************************
函数名称：spiBusTransfer
函数功能：spi总线传输
函数形参：总线，要发送的字节，接收数据缓存地址，数据长度
函数返回值：传输完成返回true
函数描述：None
**********************************************************************/
bool spiBusTransfer(const busDevice_t *bus, const uint8_t *txData, uint8_t *rxData, int length)
{
	// 拉低片选
    IOLo(bus->busdev_u.spi.csnPin);
	// 数据传输
    spiTransfer(bus->busdev_u.spi.instance, txData, rxData, length);
	// 拉高片选
    IOHi(bus->busdev_u.spi.csnPin);
    return true;
}

/**********************************************************************
函数名称：spiBusWriteRegister
函数功能：spi总线写寄存器数据
函数形参：总线，寄存器，数据
函数返回值：传输完成返回true
函数描述：None
**********************************************************************/
bool spiBusWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
	// 拉低片选
    IOLo(bus->busdev_u.spi.csnPin);
	// 写寄存器
    spiTransferByte(bus->busdev_u.spi.instance, reg);
	// 发送传输
    spiTransferByte(bus->busdev_u.spi.instance, data);
	// 拉高片选
    IOHi(bus->busdev_u.spi.csnPin);
    return true;
}

/**********************************************************************
函数名称：spiBusRawReadRegisterBuffer
函数功能：spi总线原始读寄存器缓存
函数形参：bus，reg，data，length
函数返回值：传输完成返回true
函数描述：None
**********************************************************************/
bool spiBusRawReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length)
{
	// 拉低片选
    IOLo(bus->busdev_u.spi.csnPin);
	// 写寄存器
    spiTransferByte(bus->busdev_u.spi.instance, reg);
	// 读取数据
    spiTransfer(bus->busdev_u.spi.instance, NULL, data, length);
	// 拉高片选
    IOHi(bus->busdev_u.spi.csnPin);
    return true;
}

/**********************************************************************
函数名称：spiBusReadRegisterBuffer
函数功能：spi总线读寄存器缓存
函数形参：bus，reg，data，length
函数返回值：传输完成返回true
函数描述：None
**********************************************************************/
bool spiBusReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length)
{
    return spiBusRawReadRegisterBuffer(bus, reg | 0x80, data, length);
}

/**********************************************************************
函数名称：spiBusRawReadRegister
函数功能：spi总线原始读寄存器
函数形参：总线，寄存器
函数返回值：数据
函数描述：None
**********************************************************************/
uint8_t spiBusRawReadRegister(const busDevice_t *bus, uint8_t reg)
{
    uint8_t data;
	// 拉低片选
    IOLo(bus->busdev_u.spi.csnPin);
	// 写寄存器
    spiTransferByte(bus->busdev_u.spi.instance, reg);
	// 读取数据
    spiTransfer(bus->busdev_u.spi.instance, NULL, &data, 1);
	// 拉高片选
    IOHi(bus->busdev_u.spi.csnPin);
    return data;
}

/**********************************************************************
函数名称：spiBusReadRegister
函数功能：spi总线读寄存器
函数形参：总线，寄存器
函数返回值：数据
函数描述：None
**********************************************************************/
uint8_t spiBusReadRegister(const busDevice_t *bus, uint8_t reg)
{
    return spiBusRawReadRegister(bus, reg | 0x80);
}

/**********************************************************************
函数名称：spiBusSetInstance
函数功能：实例化SPI设备
函数形参：总线，SPIx
函数返回值：None
函数描述：None
**********************************************************************/
void spiBusSetInstance(busDevice_t *bus, SPI_TypeDef *instance)
{
    bus->bustype = BUSTYPE_SPI;   			 // 设备总线类型设置为SPI
    bus->busdev_u.spi.instance = instance;   // 设备实例化SPI
}

/**********************************************************************
函数名称：spiBusSetDivisor
函数功能：设置SPI总线分频 
函数形参：总线，分频系数
函数返回值：None
函数描述：None
**********************************************************************/
void spiBusSetDivisor(busDevice_t *bus, uint16_t divisor)
{
    spiSetDivisor(bus->busdev_u.spi.instance, divisor);
}
#endif

