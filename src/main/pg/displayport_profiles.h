#pragma once

#include "pg/pg.h"

/* --------------------------显示端口结构体结构体-------------------------- */	
typedef struct displayPortProfile_s {
    int8_t colAdjust;
    int8_t rowAdjust;
    bool invert;
    uint8_t blackBrightness;
    uint8_t whiteBrightness;
    int8_t displayPortSerial;  // serialPortIdentifier_e

    // For attribute-rich OSDs
    uint8_t attrValues[4];     // NORMAL, INFORMATIONAL, WARNING, CRITICAL
    uint8_t useDeviceBlink;    // Use device local blink capability
} displayPortProfile_t;
// 声明显示端口结构体
PG_DECLARE(displayPortProfile_t, displayPortProfileMax7456);

