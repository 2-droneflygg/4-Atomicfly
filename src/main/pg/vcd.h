#pragma once

#include "pg/pg.h"

/* ------------------------------视频字符显示参数结构体------------------------------ */	
typedef struct vcdProfile_s {
    uint8_t video_system;
    int8_t h_offset;
    int8_t v_offset;
} vcdProfile_t;
// 声明视频字符显示参数结构体
PG_DECLARE(vcdProfile_t, vcdProfile);

