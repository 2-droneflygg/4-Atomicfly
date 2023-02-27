/*********************************************************************************
 根据target.h中配置的引脚（如PB5）
 	转换为： io_def_generated.h中引脚宏定义（如DEFIO_TAG__PB5或DEFIO_TAG_E__PB5）
*********************************************************************************/
#pragma once

#include "common/utils.h"
#include "target.h"
#include "io_def_generated.h"


// 定义IO标签宏 - DEFIO_TAG__pinid
#define DEFIO_TAG(pinid)   CONCAT(DEFIO_TAG__, pinid)
#define DEFIO_TAG__NONE 0

// 定义复用IO标签宏 - DEFIO_TAG_E__pinid
#define DEFIO_TAG_E(pinid) CONCAT(DEFIO_TAG_E__, pinid)
#define DEFIO_TAG_E__NONE 0

// IO标签转换宏
#define DEFIO_TAG_MAKE(gpioid, pin) ((ioTag_t)((((gpioid) + 1) << 4) | (pin)))  // 生成IO标签
#define DEFIO_TAG_GPIOID(tag)       (((tag) >> 4) - 1)                               // 将IO标签转换为端口索引（GPIO端口ID）[0-7]
#define DEFIO_TAG_PIN(tag) 			((tag) & 0x0f)                                   // 将IO标签转换为引脚索引[0-15]

