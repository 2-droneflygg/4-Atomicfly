#pragma once

#include "common/axis.h"
#include "common/maths.h"

#include "pg/pg.h"

/* ---------------------------传感器对齐枚举------------------------ */	
typedef enum {
    ALIGN_DEFAULT = 0,      // driver-provided alignment
    // 这8个值的顺序与ALIGNMENT_TO_BITMASK中相应的代码相关
                            // R, P, Y
    CW0_DEG = 1,            // 00,00,00
    CW90_DEG = 2,           // 00,00,01
    CW180_DEG = 3,          // 00,00,10
    CW270_DEG = 4,          // 00,00,11
    CW0_DEG_FLIP = 5,       // 00,10,00 // 翻转= 2x90度俯仰旋转
    CW90_DEG_FLIP = 6,      // 00,10,01
    CW180_DEG_FLIP = 7,     // 00,10,10
    CW270_DEG_FLIP = 8,     // 00,10,11
} sensor_align_e;

/* --------------------------板对齐结构体----------------------------- */	
typedef struct boardAlignment_s {
    int32_t rollDegrees;
    int32_t pitchDegrees;
    int32_t yawDegrees;
} boardAlignment_t;
PG_DECLARE(boardAlignment_t, boardAlignment);

void alignSensorViaRotation(float *dest, uint8_t rotation);

