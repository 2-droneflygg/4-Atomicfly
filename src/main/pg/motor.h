#pragma once

#include "pg/pg.h"

#include "drivers/io.h"
	
/* --------------------------------dshotDMA枚举-------------------------------- */	
// 是否使用DMA驱动Dshot
typedef enum {
    DSHOT_DMAR_OFF,
    DSHOT_DMAR_ON,
    DSHOT_DMAR_AUTO
} dshotDmar_e;

/* ------------------------------电机设备配置结构体---------------------------- */	
typedef struct motorDevConfig_s {
    uint8_t  motorPwmProtocol;              // PWM协议
    uint8_t  useBurstDshot;			        // 使用DMA驱动Dshot
    ioTag_t  ioTags[MAX_SUPPORTED_MOTORS];  // IO引脚
} motorDevConfig_t;

/* --------------------------------电机配置结构体------------------------------- */	
typedef struct motorConfig_s {	
    motorDevConfig_t dev;					// 电机设备
    uint16_t digitalIdleOffsetValue;        // DShot协议的空闲值，全电机输出 = 10000
    uint16_t minthrottle;                   // 设置发送给ESC(电子调速器)的最小油门命令，这是允许电动机以怠速运行的最小值
    uint16_t maxthrottle;                   // ESC在全功率时的最大值，这个值可以增加到2000
} motorConfig_t;
// 声明电机配置结构体
PG_DECLARE(motorConfig_t, motorConfig);

