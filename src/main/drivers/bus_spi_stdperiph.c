/*********************************************************************************
 提供SPI硬件操作API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SPI
#include "common/maths.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"

// ---------------------------------------------------------SPI默认初始化配置
static SPI_InitTypeDef defaultInit = {
    .SPI_Mode = SPI_Mode_Master,
    .SPI_Direction = SPI_Direction_2Lines_FullDuplex,
    .SPI_DataSize = SPI_DataSize_8b,
    .SPI_NSS = SPI_NSS_Soft,
    .SPI_FirstBit = SPI_FirstBit_MSB,
    .SPI_CRCPolynomial = 7,
    .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8,
};

/**********************************************************************
函数名称：spiInitDevice
函数功能：spi设备初始化
函数形参：SPI设备(ID)，边沿
函数返回值：None
函数描述：None
**********************************************************************/
void spiInitDevice(SPIDevice device, bool leadingEdge)
{
	// 获取SPI设备信息
    spiDevice_t *spi = &(spiDevice[device]);
	// 判断合法性
    if (!spi->dev) {
        return;
    }
	// 获取边沿
    spi->leadingEdge = leadingEdge;
    // 使能SPI时钟
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);
	// IO初始化
    IOInit(IOGetByTag(spi->sck),  OWNER_SPI_SCK,  RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_MISO, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_MOSI, RESOURCE_INDEX(device));
	// IO复用配置
    IOConfigGPIOAF(IOGetByTag(spi->sck),  SPI_IO_AF_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->af);
    // 初始化SPI为默认值
    SPI_I2S_DeInit(spi->dev);
	// 配置时钟极性和时钟相位
	// 1.时钟极性（CPOL）：时钟极性决定了SPI通信在空闲状态时，时钟线的电平状态。
	// 		1）	CPOL = 0：SCLK时钟线的在空闲状态下位低电平状态
	//      2）	CPOL = 1：SCLK时钟线的在空闲状态下位高电平状态
	// 2.时钟相位（CPHA）：时钟相位决定了数据线上第一个数据被采集的时钟。
	//      1）	CPHA = 0：MOSI或MISO数据线上的数据会在SCLK时钟线的“奇数跳变沿”被采集(读数据)
	//		2）	CPHA = 1：MOSI或MISO数据线上的数据会在SCLK时钟线的“偶数跳变沿”被采集(读数据)
    if (spi->leadingEdge) {
        defaultInit.SPI_CPOL = SPI_CPOL_Low;
        defaultInit.SPI_CPHA = SPI_CPHA_1Edge;
    } else
    {
        defaultInit.SPI_CPOL = SPI_CPOL_High;
        defaultInit.SPI_CPHA = SPI_CPHA_2Edge;
    }
	// SPI初始化
    SPI_Init(spi->dev, &defaultInit);
	// 使能SPI
    SPI_Cmd(spi->dev, ENABLE);
}

/**********************************************************************
函数名称：spiTransferByte
函数功能：SPI数据传输（收发）
函数形参：SPIx，要发送的字节
函数返回值：数据或-1
函数描述：None
**********************************************************************/
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t txByte)
{
	// 定义超时时间
    uint16_t spiTimeout = 1000;
	// 强转为void - 清空数据寄存器
    DISCARD(instance->DR);

	// 等待发送完毕 - 防止有正在传输的事务
    while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET) 
		// 判断是否超时 - 超时则进行SPI设备传输超时用户回调
        if ((spiTimeout--) == 0)
            return spiTimeoutUserCallback(instance);

	// 发送数据
    SPI_I2S_SendData(instance, txByte);
	// 重新设置超时时间
    spiTimeout = 1000;
	// 等待接收完毕
    while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET)
		// 判断是否超时 - 超时则进行SPI设备传输超时用户回调
        if ((spiTimeout--) == 0)
            return spiTimeoutUserCallback(instance);
	// 接收数据
    return ((uint8_t)SPI_I2S_ReceiveData(instance));
}

/**********************************************************************
函数名称：spiTransfer
函数功能：SPI数据传输（收发多个字节）
函数形参：SPIx，要发送的字节，接收缓存地址，数据长度
函数返回值：传输完成返回true，超时返回errorcount
函数描述：None
**********************************************************************/
bool spiTransfer(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
	// 定义超时时间
    uint16_t spiTimeout = 1000;
	// 定义字节缓存变量
    uint8_t b;
	// 强转为void - 清空数据寄存器
    DISCARD(instance->DR);
	// 循环收发字节
    while (len--) {
		// 获取字节 - 判断要发送的字节是否为空
        b = txData ? *(txData++) : 0xFF;
		// 等待发送完毕 - 防止有正在传输的事务
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET) {
			// 判断是否超时 - 超时则进行SPI设备传输超时用户回调
            if ((spiTimeout--) == 0)  
                return spiTimeoutUserCallback(instance);
        }
		// 发送数据
        SPI_I2S_SendData(instance, b);
		// 重新设置超时时间
        spiTimeout = 1000;
		// 等待接收完毕
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET) {
			// 判断是否超时 - 超时则进行SPI设备传输超时用户回调
            if ((spiTimeout--) == 0) 
                return spiTimeoutUserCallback(instance);
        }
		// 接收数据
        b = SPI_I2S_ReceiveData(instance);
		// 保存接收数据
        if (rxData)
            *(rxData++) = b;
    }
    return true;
}

/**********************************************************************
函数名称：spiIsBusBusy
函数功能：获取SPI总线是否空闲
函数形参：SPIx
函数返回值：如果总线当前正在传输，则返回true
函数描述：None
**********************************************************************/
bool spiIsBusBusy(SPI_TypeDef *instance)
{
    return SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET || SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_BSY) == SET;
}

/**********************************************************************
函数名称：spiDivisorToBRbits
函数功能：SPI分频转BR位（波特率控制）
函数形参：SPIx，分频系数
函数返回值：SPI_CR1_BR_Pos
函数描述：
	比如8分频。
**********************************************************************/
static uint16_t spiDivisorToBRbits(SPI_TypeDef *instance, uint16_t divisor)
{
    // SPI2和SPI3在APB1/AHB1上，PCLK是APB2/AHB2的一半
    if (instance == SPI2 || instance == SPI3) {
        divisor /= 2; 
    }
	// 约束限制（2 - 256）
    divisor = constrain(divisor, 2, 256);
	// ffs()函数用于查找一个整数中的第一个置位值(也就是bit为1的位)
	// 1 << 3 = 8
    return (ffs(divisor) - 2) << 3; 
}

/**********************************************************************
函数名称：spiSetDivisorBRreg
函数功能：设置SPI->BR寄存器分频
函数形参：SPIx，分频系数
函数返回值：None
函数描述：
	CR1 - 位5:3 波特率控制。
**********************************************************************/
static void spiSetDivisorBRreg(SPI_TypeDef *instance, uint16_t divisor)
{
#define BR_BITS ((BIT(5) | BIT(4) | BIT(3)))
	// 清空BR寄存器
    const uint16_t tempRegister = (instance->CR1 & ~BR_BITS);
	// 重新设置BR寄存器
    instance->CR1 = tempRegister | spiDivisorToBRbits(instance, divisor);
#undef BR_BITS
}

/**********************************************************************
函数名称：spiSetDivisor
函数功能：设置SPI分频系数
函数形参：SPIx，分频系数
函数返回值：None
函数描述：None
**********************************************************************/
void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor)
{
	// 失能SPIx - 正在通信时不应更改这些位
    SPI_Cmd(instance, DISABLE);	
	// 设置SPI->BR寄存器分频
    spiSetDivisorBRreg(instance, divisor);
	// 使能SPIx
    SPI_Cmd(instance, ENABLE);
}
#endif

