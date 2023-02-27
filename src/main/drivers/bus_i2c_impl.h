/*********************************************************************************
 提供一系列IIC总线配置信息块。
*********************************************************************************/
#pragma once

#include "platform.h"

#include "drivers/io_types.h"
#include "drivers/rcc_types.h"

// 定义IIC超时时间
#define I2C_SHORT_TIMEOUT            ((uint32_t)0x1000)
#define I2C_DEFAULT_TIMEOUT          I2C_SHORT_TIMEOUT

/* ---------------------------IIC状态结构体----------------------------- */	
typedef struct i2cState_s {
    volatile bool error;					// 错误状态
    volatile bool busy;						// 空闲状态
    volatile uint8_t addr;					// 地址
    volatile uint8_t reg;					// 寄存器
    volatile uint8_t bytes;					// 字节
    volatile uint8_t writing;				// 正在写入
    volatile uint8_t reading;				// 正在读取
    volatile uint8_t* write_p;				// 写缓存
    volatile uint8_t* read_p;				// 读缓存
} i2cState_t;

/* --------------------------IIC引脚定义结构体-------------------------- */	
typedef struct i2cPinDef_s {
    ioTag_t ioTag;							// IO引脚
    uint8_t af;								// 引脚复用
} i2cPinDef_t;
// IIC引脚定义宏 - DEFIO_TAG_E__pin
#define I2CPINDEF(pin, af) { DEFIO_TAG_E(pin), af }

/* -------------------------IIC硬件信息结构体--------------------------- */	
#define I2C_PIN_SEL_MAX 	4
typedef struct i2cHardware_s {
    I2CDevice device;						// IIC设备
    I2C_TypeDef *reg;						// IICx
    i2cPinDef_t sclPins[I2C_PIN_SEL_MAX];   // 时钟线引脚
    i2cPinDef_t sdaPins[I2C_PIN_SEL_MAX];	// 数据线引脚
    rccPeriphTag_t rcc;						// 时钟
    uint8_t ev_irq;						 	// 事件中断
    uint8_t er_irq;							// 错误中断
} i2cHardware_t;
// 声明硬件IIC结构体
extern const i2cHardware_t i2cHardware[];

/* -------------------------IIC设备信息结构体--------------------------- */	
typedef struct i2cDevice_s {
    const i2cHardware_t *hardware;			// IIC硬件
    I2C_TypeDef *reg;						// IICx
    IO_t scl;								// 时钟线
    IO_t sda;								// 数据线
    uint8_t sclAF;							// 复用时钟线
    uint8_t sdaAF;							// 复用数据线
    bool overClock;							// 超频
    bool pullUp;							// 上拉
    i2cState_t state;						// 状态
} i2cDevice_t;
// 声明IIC设备结构体
extern i2cDevice_t i2cDevice[];

