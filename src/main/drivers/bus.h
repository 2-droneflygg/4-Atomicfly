#pragma once

#include "platform.h"

#include "drivers/bus_i2c.h"
#include "drivers/io_types.h"

/* -------------------------总线类型枚举-------------------------- */	
typedef enum {
    BUSTYPE_NONE = 0,							// 无类型
    BUSTYPE_I2C,								// IIC总线
    BUSTYPE_SPI,								// SPI总线
    BUSTYPE_GYRO_AUTO,  				    	// 仅用加速度计/陀螺仪总线自动检测码
} busType_e;

/* -------------------------总线设备结构体------------------------ */	
typedef struct busDevice_s {
    busType_e bustype;
    union {
        struct deviceSpi_s {					// SPI结构体
            SPI_TypeDef *instance;				// SPI设备
            IO_t csnPin;						// 片选线
        } spi;
        struct deviceI2C_s {					// IIC结构体
            I2CDevice device;					// IIC设备
            uint8_t address;					// 地址
        } i2c;
    } busdev_u;
} busDevice_t;

bool busRawWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data);
bool busWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data);
bool busRawWriteRegisterStart(const busDevice_t *bus, uint8_t reg, uint8_t data);
bool busWriteRegisterStart(const busDevice_t *bus, uint8_t reg, uint8_t data);
bool busRawReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length);
bool busReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length);
uint8_t busReadRegister(const busDevice_t *bus, uint8_t reg);
bool busRawReadRegisterBufferStart(const busDevice_t *busdev, uint8_t reg, uint8_t *data, uint8_t length);
bool busReadRegisterBufferStart(const busDevice_t *busdev, uint8_t reg, uint8_t *data, uint8_t length);
bool busBusy(const busDevice_t *busdev, bool *error);

