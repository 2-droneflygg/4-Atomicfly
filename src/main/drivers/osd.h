/*******************************************************************************************
 字库地址（0-255），54字节字库数据
	每个字符需要12*18=216个像素，一个8位字节表示4个像素，所以每个字符需要216/4=54字节.
	每次只能从非易失性存储器中读写一个字符数据（54 个字节），这一步通过影子存储器来完成.
	像素：透明01 ，灰色11.(一个像素为2bit)
*******************************************************************************************/
#pragma once

#include <stdint.h>

#include "common/utils.h"

// OSD字符宽度
#define OSD_CHAR_WIDTH 12

// OSD字符高度
#define OSD_CHAR_HEIGHT 18

// 每个像素的OSD字符位
#define OSD_CHAR_BITS_PER_PIXEL 2

// 字符可见字节 - 54
#define OSD_CHAR_VISIBLE_BYTES (OSD_CHAR_WIDTH * OSD_CHAR_HEIGHT * OSD_CHAR_BITS_PER_PIXEL / 8)

// 只有字符的前54个字节表示可见数据,一些OSD驱动程序接受64字节，额外的10字节作为元数据
#define OSD_CHAR_BYTES 64

/* --------------------------视频制式枚举-------------------------- */	
typedef enum {
    VIDEO_SYSTEM_AUTO = 0,
    VIDEO_SYSTEM_PAL,
    VIDEO_SYSTEM_NTSC
} videoSystem_e;

/* --------------------------OSD字符结构体-------------------------- */	
// osdCharacter_t表示OSD的二进制数据字符
// 所有OSD驱动都使用相同的字符格式定义:
//     OSD_CHARACTER_WIDTH、OSD_CHARACTER_HEIGHT、OSD_CHARACTER_BITS_PER_PIXEL
typedef struct osdCharacter_s {
    uint8_t data[OSD_CHAR_BYTES];
} osdCharacter_t;

