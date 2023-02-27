/**********************************************************************
平台配置：
**********************************************************************/
#pragma once

/* --------声明非内联函数宏 -------- */
#define NOINLINE __attribute__((noinline))

/* ------------目标配置 ------------ */
#if defined(STM32F40_41xxx)
// 包含所有外设寄存器的定义
#include "stm32f4xx.h"
// 芯片目标宏，确保定义STM32F4目标
#ifndef STM32F4
	#define STM32F4
#endif
// 指定无效的芯片组
#else
	#error "Invalid chipset specified. Update platform.h"
#endif /* defined(STM32F40_41xxx) */

/* -------包含目标配置文件 --------- */
#include "target/common_pre.h"               // 功能宏预配置
#include "target.h"                          // 目标功能宏&&引脚宏配置
#include "target/common_post.h"              // 未启用功能宏 -> #undef（取消定义）
#include "target/common_defaults_post.h"     // 未启用引脚宏 -> None（空）

