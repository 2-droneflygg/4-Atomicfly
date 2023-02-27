/*********************************************************************************
 提供MPU驱动接口相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "build/atomic.h"
#include "build/build_config.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/accgyro/accgyro_mpu.h"

#include "pg/pg.h"
#include "pg/gyrodev.h"

#ifndef MPU_ADDRESS
#define MPU_ADDRESS             0x68
#endif
#define MPU_INQUIRY_MASK        0x7E

/**********************************************************************
函数名称：mpuIntExtiHandler
函数功能：陀螺中断服务程序（回调函数）
函数形参：extiChannelRecs[idx].handler(具体是哪一个中断线的回调函数)
函数返回值：None  
函数描述：None 
**********************************************************************/
#ifdef USE_GYRO_EXTI
static void mpuIntExtiHandler(extiCallbackRec_t *cb)
{
	// 获取宿主结构体的地址
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
	// MPU数据就绪
    gyro->dataReady = true;   
}

/**********************************************************************
函数名称：mpuIntExtiInit
函数功能：MPU外部中断初始化
函数形参：陀螺仪设备结构体
函数返回值：None  
函数描述：
	EXTI用于判断陀螺仪数据是否就绪。
**********************************************************************/
static void mpuIntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    const IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

	// 初始化IO
    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
	// EXTI中断服务函数初始化 - 回调函数形式
    EXTIHandlerInit(&gyro->exti, mpuIntExtiHandler);
    // EXTI配置
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, EXTI_TRIGGER_RISING);
	// 使能EXTI
    EXTIEnable(mpuIntIO, true);
}
#endif // USE_GYRO_EXTI

/**********************************************************************
函数名称：mpuGyroRead
函数功能：读取陀螺仪原始数据
函数形参：陀螺仪设备结构体
函数返回值：成功：true,失败：false
函数描述：None 
**********************************************************************/
bool mpuGyroRead(gyroDev_t *gyro)
{
    uint8_t data[6];
    const bool ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_GYRO_XOUT_H, data, 6);
    if (!ack) {
        return false;
    }
    gyro->gyroADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);
    return true;
}

/**********************************************************************
函数名称：mpuAccRead
函数功能：读取加速度原始数据
函数形参：加速度计设备结构体
函数返回值：成功：true,失败：false
函数描述：None 
**********************************************************************/
bool mpuAccRead(accDev_t *acc)
{
    uint8_t data[6];
    const bool ack = busReadRegisterBuffer(&acc->bus, MPU_RA_ACCEL_XOUT_H, data, 6);
    if (!ack) {
        return false;
    }
    acc->ADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    acc->ADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    acc->ADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);
    return true;
}

/**********************************************************************
函数名称：mpuGyroReadSPI
函数功能：读取陀螺仪原始数据（SPI）
函数形参：陀螺仪设备结构体 
函数返回值：成功：true,失败：false
函数描述：None 
**********************************************************************/
#ifdef USE_SPI_GYRO
bool mpuGyroReadSPI(gyroDev_t *gyro)
{
    static const uint8_t dataToSend[7] = {MPU_RA_GYRO_XOUT_H | 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t data[7];
    const bool ack = spiBusTransfer(&gyro->bus, dataToSend, data, 7);
    if (!ack) {
        return false;
    }
    gyro->gyroADCRaw[X] = (int16_t)((data[1] << 8) | data[2]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[3] << 8) | data[4]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[5] << 8) | data[6]);
    return true;
}

/**********************************************************************
函数名称：detectSPISensorsAndUpdateDetectionResult
函数功能：检测SPI传感器并更新检测结果
函数形参：陀螺仪设备结构体 ， 陀螺仪设备配置结构体
函数返回值：成功：true,失败：false
函数描述：None 
**********************************************************************/
typedef uint8_t (*gyroSpiDetectFn_t)(const busDevice_t *bus);
static gyroSpiDetectFn_t gyroSpiDetectFnTable[] = {
#ifdef USE_GYRO_SPI_MPU6000
    mpu6000SpiDetect,
#endif
	// 避免使用空数组
    NULL     
};
static bool detectSPISensorsAndUpdateDetectionResult(gyroDev_t *gyro, const gyroDeviceConfig_t *config)
{
    SPI_TypeDef *instance = spiInstanceByDevice(SPI_CFG_TO_DEV(config->spiBus));

    if (!instance || !config->csnTag) {
        return false;
    }
    spiBusSetInstance(&gyro->bus, instance);

    gyro->bus.busdev_u.spi.csnPin = IOGetByTag(config->csnTag);
    IOInit(gyro->bus.busdev_u.spi.csnPin, OWNER_GYRO_CS, RESOURCE_INDEX(config->index));
    IOConfigGPIO(gyro->bus.busdev_u.spi.csnPin, SPI_IO_CS_CFG);
	// 当两个设备在同一总线上时确保设备被禁用
    IOHi(gyro->bus.busdev_u.spi.csnPin); 	

    uint8_t sensor = MPU_NONE;

    for (size_t index = 0 ; gyroSpiDetectFnTable[index] ; index++) {
        sensor = (gyroSpiDetectFnTable[index])(&gyro->bus);
        if (sensor != MPU_NONE) {
            gyro->mpuDetectionResult.sensor = sensor;

            return true;
        }
    }

    // 检测失败，再次禁用CS引脚
    spiPreinitByTag(config->csnTag);

    return false;
}
#endif

/**********************************************************************
函数名称：mpuDetect
函数功能：MPU检测
函数形参：陀螺仪设备结构体，陀螺仪设备配置结构体
函数返回值：成功：true,失败：false
函数描述：None 
**********************************************************************/
bool mpuDetect(gyroDev_t *gyro, const gyroDeviceConfig_t *config)
{
    // MPU datasheet 指定 30ms.
    delay(35);

    if (config->bustype == BUSTYPE_NONE) {
        return false;
    }

    if (config->bustype == BUSTYPE_GYRO_AUTO) {
        gyro->bus.bustype = BUSTYPE_I2C;
    } else {
        gyro->bus.bustype = config->bustype;
    }

#ifdef USE_SPI_GYRO
    gyro->bus.bustype = BUSTYPE_SPI;
    // 检测SPI传感器并更新检测结果
    return detectSPISensorsAndUpdateDetectionResult(gyro, config);
#else
    return false;
#endif
}

/**********************************************************************
函数名称：mpuPreInit
函数功能：MPU预初始化
函数形参：陀螺仪设备配置结构体
函数返回值：None
函数描述：None 
**********************************************************************/
void mpuPreInit(const struct gyroDeviceConfig_s *config)
{
#ifdef USE_SPI_GYRO
    spiPreinitRegister(config->csnTag, IOCFG_IPU, 1);
#else
    UNUSED(config);
#endif
}

/**********************************************************************
函数名称：mpuGyroInit
函数功能：MPU陀螺仪初始化
函数形参：陀螺仪设备结构体
函数返回值：None
函数描述：None 
**********************************************************************/
void mpuGyroInit(gyroDev_t *gyro)
{
#ifdef USE_GYRO_EXTI
    mpuIntExtiInit(gyro);
#else
    UNUSED(gyro);
#endif
}

/**********************************************************************
函数名称：mpuGyroDLPF
函数功能：MPU陀螺仪数字低通滤波器
函数形参：陀螺仪设备结构体
函数返回值：ret
函数描述：None 
**********************************************************************/
uint8_t mpuGyroDLPF(gyroDev_t *gyro)
{
    uint8_t ret = 0;

    // 如果陀螺仪处于32KHz模式，那么DLPF位不会被使用
    if (gyro->gyroRateKHz <= GYRO_RATE_8_kHz) {
        switch (gyro->hardware_lpf) {
            case GYRO_HARDWARE_LPF_NORMAL:
            default:
                ret = 0;
                break;
        }
    }
    return ret;
}

