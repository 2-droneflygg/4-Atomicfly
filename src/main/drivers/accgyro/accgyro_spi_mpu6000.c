/*********************************************************************************
 提供MPU6000驱动相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_GYRO_SPI_MPU6000) || defined(USE_ACC_SPI_MPU6000)
#include "common/axis.h"
#include "common/maths.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/accgyro/accgyro_spi_icm426xx.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/sensor.h"
#include "drivers/system.h"

/* ---------------------------寄存器地址---------------------------- */	
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA         0x01
#define BIT_GYRO                    0x04
#define BIT_ACC                     0x02
#define BIT_TEMP                    0x01
#define MPU6000ES_REV_C4 			0x14
#define MPU6000ES_REV_C5 			0x15
#define MPU6000ES_REV_D6 			0x16
#define MPU6000ES_REV_D7 			0x17
#define MPU6000ES_REV_D8 			0x18
#define MPU6000_REV_C4 				0x54
#define MPU6000_REV_C5 				0x55
#define MPU6000_REV_D6 				0x56
#define MPU6000_REV_D7 				0x57
#define MPU6000_REV_D8 				0x58
#define MPU6000_REV_D9 				0x59
#define MPU6000_REV_D10 			0x5A

/**********************************************************************
函数名称：mpu6000SpiGyroInit
函数功能：陀螺仪初始化
函数形参：陀螺仪设备结构体
函数返回值：None
函数描述：None 
**********************************************************************/
void mpu6000SpiGyroInit(gyroDev_t *gyro)
{
	// 陀螺仪初始化
    mpuGyroInit(gyro);
	// 陀螺仪&&加速度计初始化
    mpu6000AccAndGyroInit(gyro);
	// 设置SPI分频系数
    spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_INITIALIZATION);
    // 加速和陀螺DLPF设置
    spiBusWriteRegister(&gyro->bus, MPU6000_CONFIG, mpuGyroDLPF(gyro));
	// 延时
    delayMicroseconds(1);
	// 设置SPI分频系数
    spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_FAST);  // 18 MHz SPI clock
	// 读取陀螺仪原始数据
    mpuGyroRead(gyro);
	// 验证陀螺仪数据合法性
    if (((int8_t)gyro->gyroADCRaw[1]) == -1 && ((int8_t)gyro->gyroADCRaw[0]) == -1) {
		// 失败模式
        failureMode(FAILURE_GYRO_INIT_FAILED);
    }
}

/**********************************************************************
函数名称：mpu6000SpiAccInit
函数功能：加速度计初始化
函数形参：加速度计设备结构体
函数返回值：None
函数描述：None 
**********************************************************************/
void mpu6000SpiAccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

/**********************************************************************
函数名称：mpu6000AccAndGyroInit
函数功能：陀螺仪&&加速度计初始化
函数形参：陀螺仪设备结构体
函数返回值：None
函数描述：None 
**********************************************************************/
static void mpu6000AccAndGyroInit(gyroDev_t *gyro)
{
	// 设置SPI分频系数
    spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_INITIALIZATION);
    // 设备复位
    spiBusWriteRegister(&gyro->bus, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
    delay(150);
    spiBusWriteRegister(&gyro->bus, MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
    delay(150);
    // 时钟源PPL与Z轴陀螺参考
    spiBusWriteRegister(&gyro->bus, MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    delayMicroseconds(15);
    // 禁用I2C接口
    spiBusWriteRegister(&gyro->bus, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
    delayMicroseconds(15);
    spiBusWriteRegister(&gyro->bus, MPU_RA_PWR_MGMT_2, 0x00);
    delayMicroseconds(15);
    // 加速度采样率1kHz
    // 陀螺仪输出速率= 1kHz时DLPF是启用的
    spiBusWriteRegister(&gyro->bus, MPU_RA_SMPLRT_DIV, gyro->mpuDividerDrops);
    delayMicroseconds(15);
    // 陀螺仪满量程 +/- 2000DPS
    spiBusWriteRegister(&gyro->bus, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    delayMicroseconds(15);
    // 加速度满量程 +/- 16g
    spiBusWriteRegister(&gyro->bus, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    delayMicroseconds(15);
    spiBusWriteRegister(&gyro->bus, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR
    delayMicroseconds(15);
	// 使能中断
#ifdef USE_MPU_DATA_READY_SIGNAL
    spiBusWriteRegister(&gyro->bus, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
    delayMicroseconds(15);
#endif
	// 快速始终
    spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_FAST);
    delayMicroseconds(1);
}

/**********************************************************************
函数名称：mpu6000SpiAccInit
函数功能：陀螺仪检测
函数形参：总线设备结构体
函数返回值：传感器ID
函数描述：None 
**********************************************************************/
uint8_t mpu6000SpiDetect(const busDevice_t *bus)
{
	// 设置SPI分频系数
    spiSetDivisor(bus->busdev_u.spi.instance, SPI_CLOCK_SLOW);
	// 写寄存器数据
    spiBusWriteRegister(bus, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
	// 判断传感器是否在线
    uint8_t attemptsRemaining = 5;
    do {
        delay(150);
        const uint8_t whoAmI = spiBusReadRegister(bus, MPU_RA_WHO_AM_I);
        if (whoAmI == MPU6000_WHO_AM_I_CONST) {
            break;
        }
        if (!attemptsRemaining) {
            return MPU_NONE;
        }
    } while (attemptsRemaining--);
   // 读取产品ID
   const uint8_t productID = spiBusReadRegister(bus, MPU_RA_PRODUCT_ID);
   // 验证产品ID
   switch (productID) {
	    case MPU6000ES_REV_C4:
	    case MPU6000ES_REV_C5:
	    case MPU6000_REV_C4:
	    case MPU6000_REV_C5:
	    case MPU6000ES_REV_D6:
	    case MPU6000ES_REV_D7:
	    case MPU6000ES_REV_D8:
	    case MPU6000_REV_D6:
	    case MPU6000_REV_D7:
	    case MPU6000_REV_D8:
	    case MPU6000_REV_D9:
	    case MPU6000_REV_D10:
        return MPU_60x0_SPI;
    }
    return MPU_NONE;
}

/**********************************************************************
函数名称：mpu6000SpiAccDetect
函数功能：加速度计检测
函数形参：加速度计设备结构体
函数返回值：成功：true,失败：false
函数描述：None 
**********************************************************************/
bool mpu6000SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != MPU_60x0_SPI) {
        return false;
    }
	// 注册初始化回调函数
    acc->initFn = mpu6000SpiAccInit;
	// 注册读数据回调函数
    acc->readFn = mpuAccRead;
    return true;
}

/**********************************************************************
函数名称：mpu6000SpiGyroDetect
函数功能：陀螺仪检测
函数形参：陀螺仪设备结构体
函数返回值：成功：true,失败：false
函数描述：None 
**********************************************************************/
bool mpu6000SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != MPU_60x0_SPI) {
        return false;
    }
	// 注册初始化回调函数
    gyro->initFn = mpu6000SpiGyroInit;
	// 注册读数据回调函数
    gyro->readFn = mpuGyroReadSPI;
    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;
    return true;
}
#endif

