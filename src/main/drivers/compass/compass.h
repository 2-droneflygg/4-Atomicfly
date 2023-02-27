/*********************************************************************************
 提供磁力机驱动相关信息。
*********************************************************************************/
#pragma once

#include "sensors/boardalignment.h"

#include "drivers/bus.h"
#include "drivers/sensor.h"
#include "drivers/exti.h"

/* -----------------------------磁力计设备结构体------------------------------ */	
typedef struct magDev_s {
    sensorMagInitFuncPtr init;                              // 初始化函数
    sensorMagReadFuncPtr read;                              // 读取3轴数据功能函数
    extiCallbackRec_t exti;					
    busDevice_t busdev;					
    sensor_align_e magAlignment;
    fp_rotationMatrix_t rotationMatrix;
    int16_t magGain[3];
} magDev_t;

