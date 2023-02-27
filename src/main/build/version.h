/*********************************************************************************
 提供固件相关信息。
*********************************************************************************/
#pragma once

#include "common/utils.h"

// 固件标识
#define FC_FIRMWARE_NAME            "Dflight"         // 固件名称	
#define FC_FIRMWARE_IDENTIFIER      "DFLIGHT"         // 固件标识符
// 固件版本
#define FC_VERSION_MAJOR            1  				  // 重大发布增量(大的新特性等)
#define FC_VERSION_MINOR            0 				  // 次要发布增量(小的新特性、变更等)
#define FC_VERSION_PATCH_LEVEL      0 				  // 修正错误增量
#define FC_VERSION_STRING STR(FC_VERSION_MAJOR) "." STR(FC_VERSION_MINOR) "." STR(FC_VERSION_PATCH_LEVEL)

