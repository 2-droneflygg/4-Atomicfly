/*********************************************************************************
 提供IIC设备类型和IIC相关API函数声明。
*********************************************************************************/
#pragma once

#include "platform.h"

#include "drivers/io_types.h"
#include "drivers/rcc_types.h"

#ifndef I2C_DEVICE
#define I2C_DEVICE I2CINVALID
#endif

/* --------------------------IIC设备枚举-------------------------- */	
typedef enum I2CDevice {
    I2CINVALID = -1,
    I2CDEV_1   = 0,
    I2CDEV_2,
    I2CDEV_3,
    I2CDEV_4,
} I2CDevice;
// IIC设备数量
#define I2CDEV_COUNT 3

// 配置设备转换宏
#define I2C_CFG_TO_DEV(x)   ((x) - 1)
#define I2C_DEV_TO_CFG(x)   ((x) + 1)

struct i2cConfig_s;
void i2cHardwareConfigure(const struct i2cConfig_s *i2cConfig);
void i2cInit(I2CDevice device);
bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg, uint8_t data);
bool i2cReadBuffer(I2CDevice device, uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf);
bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf);
bool i2cBusy(I2CDevice device, bool *error);
uint16_t i2cGetErrorCounter(void);

