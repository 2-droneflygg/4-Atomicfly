/*********************************************************************************
 提供IIC设备硬件初始化API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C)
#include "build/build_config.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/io.h"

#include "pg/bus_i2c.h"

/**********************************************************************
函数名称：i2cHardwareConfigure
函数功能：IIC硬件初始化 - 注册硬件配置到IIC设备信息
函数形参：i2cConfig
函数返回值：None
函数描述：None
**********************************************************************/
void i2cHardwareConfigure(const i2cConfig_t *i2cConfig)
{
	// 遍历IIC设备
    for (int index = 0 ; index < I2CDEV_COUNT ; index++) {
		// 获取IIC设备硬件配置
        const i2cHardware_t *hardware = &i2cHardware[index];
		// 判断合法性
        if (!hardware->reg) {
            continue;
        }
		// 获取IIC设备
        I2CDevice device = hardware->device;   
		// 获取IIC设备结构体
        i2cDevice_t *pDev = &i2cDevice[device];	 	
		// 清空IIC设备结构体
        memset(pDev, 0, sizeof(*pDev));
		// 遍历IIC设备
        for (int pindex = 0 ; pindex < I2C_PIN_SEL_MAX ; pindex++) {
            if (i2cConfig[device].ioTagScl == hardware->sclPins[pindex].ioTag) {
                pDev->scl = IOGetByTag(i2cConfig[device].ioTagScl);
                pDev->sclAF = hardware->sclPins[pindex].af;
            }
            if (i2cConfig[device].ioTagSda == hardware->sdaPins[pindex].ioTag) {
                pDev->sda = IOGetByTag(i2cConfig[device].ioTagSda);
                pDev->sdaAF = hardware->sdaPins[pindex].af;
            }
        }
		// 注册IIC设备信息
        if (pDev->scl && pDev->sda) {
            pDev->hardware = hardware;
            pDev->reg = hardware->reg;
            pDev->overClock = i2cConfig[device].overClock;
            pDev->pullUp = i2cConfig[device].pullUp;
        }
    }
}
#endif // defined(USE_I2C) 

