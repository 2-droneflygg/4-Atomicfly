/*********************************************************************************
 提供IO引脚相关类型。
*********************************************************************************/
#pragma once

#include <stdint.h>

typedef uint8_t ioTag_t;          	 // 指定IO引脚标签类型
typedef void* IO_t;             	 // 指定IO引脚类型 

#define IO_TAG_NONE 0			  	 // 空IO引脚标签
#define IO_NONE ((IO_t)0)         	 // 空IO引脚

typedef uint8_t ioConfig_t;  	     // IO配置封装 - IO模式（输入、输出、复用）

