/*********************************************************************************
 提供SPI硬件配置和引脚初始化API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SPI
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"

#include "pg/bus_spi.h"

// ---------------------------------------------------------SPI硬件预配置
const spiHardware_t spiHardware[] = {
    {
        .device = SPIDEV_1,
        .reg = SPI1,
        .sckPins = {
            { DEFIO_TAG_E(PA5) },
            { DEFIO_TAG_E(PB3) },
        },
        .misoPins = {
            { DEFIO_TAG_E(PA6) },
            { DEFIO_TAG_E(PB4) },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA7) },
            { DEFIO_TAG_E(PB5) },
        },
        .af = GPIO_AF_SPI1,
        .rcc = RCC_APB2(SPI1),
    },
    {
        .device = SPIDEV_2,
        .reg = SPI2,
        .sckPins = {
            { DEFIO_TAG_E(PB10) },
            { DEFIO_TAG_E(PB13) },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB14) },
            { DEFIO_TAG_E(PC2) },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB15) },
            { DEFIO_TAG_E(PC3) },
        },
        .af = GPIO_AF_SPI2,
        .rcc = RCC_APB1(SPI2),
    },
    {
        .device = SPIDEV_3,
        .reg = SPI3,
        .sckPins = {
            { DEFIO_TAG_E(PB3) },
            { DEFIO_TAG_E(PC10) },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB4) },
            { DEFIO_TAG_E(PC11) },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB5) },
            { DEFIO_TAG_E(PC12) },
        },
        .af = GPIO_AF_SPI3,
        .rcc = RCC_APB1(SPI3),
    },
};

/**********************************************************************
函数名称：spiPinConfigure
函数功能：spi引脚初始化
函数形参：pConfig
函数返回值：None
函数描述：None 
**********************************************************************/
void spiPinConfigure(const spiPinConfig_t *pConfig)
{
	// 遍历SPI硬件配置数组
    for (size_t hwindex = 0 ; hwindex < ARRAYLEN(spiHardware) ; hwindex++) {
		// 获取当前SPI硬件配置
        const spiHardware_t *hw = &spiHardware[hwindex];
		// 判断合法性
        if (!hw->reg) {
            continue;
        }
		// 获取SPI设备
        SPIDevice device = hw->device;    		
		// 获取SPI设备信息存储数组
        spiDevice_t *pDev = &spiDevice[device]; 
		// 遍历所有引脚方案
        for (int pindex = 0 ; pindex < MAX_SPI_PIN_SEL ; pindex++) {
            if (pConfig[device].ioTagSck == hw->sckPins[pindex].pin) {
                pDev->sck = hw->sckPins[pindex].pin;
            }
            if (pConfig[device].ioTagMiso == hw->misoPins[pindex].pin) {
                pDev->miso = hw->misoPins[pindex].pin;
            }
            if (pConfig[device].ioTagMosi == hw->mosiPins[pindex].pin) {
                pDev->mosi = hw->mosiPins[pindex].pin;
            }
        }
		// 将硬件配置注册到SPI设备信息存储数组
        if (pDev->sck && pDev->miso && pDev->mosi) {
            pDev->dev = hw->reg;
            pDev->af = hw->af;
            pDev->rcc = hw->rcc;
            pDev->leadingEdge = false; // XXX Should be part of transfer context
        }
    }
}
#endif

