#pragma once

#include "drivers/bus_i2c.h"
#include "drivers/io_types.h"
#include "drivers/rcc_types.h"

#include "pg/pg.h"

/* --------------------------IIC配置结构体-------------------------- */	
typedef struct i2cConfig_s {
    ioTag_t ioTagScl;	  // SCL
    ioTag_t ioTagSda;	  // SDA
    bool overClock;		  // 超频
    bool pullUp;			
} i2cConfig_t;
// 声明IIC配置结构体
PG_DECLARE_ARRAY(i2cConfig_t, I2CDEV_COUNT, i2cConfig);

