#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "platform.h"
#include "resource.h"
#include "drivers/io_types.h"
#include "io_def.h"

// 定义IO标签宏 - DEFIO_TAG__pinid
#define IO_TAG(pinid) DEFIO_TAG(pinid)

// 通用IO配置宏 - 模式、速度、输出类型、上下拉
#define IO_CONFIG(mode, speed, otype, pupd) ((mode) | ((speed) << 2) | ((otype) << 4) | ((pupd) << 5))

// 预定义IO配置宏 - ioConfig_t
#define IOCFG_OUT_PP         IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_PP, GPIO_PuPd_NOPULL)    // 推挽输出
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL)    // 开漏输出
#define IOCFG_AF_PP          IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_NOPULL)    // 复用推挽输出
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_DOWN)      // 复用下拉推挽输出
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_UP)        // 复用上拉推挽输出
#define IOCFG_AF_OD          IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_OD, GPIO_PuPd_NOPULL)    // 复用开漏输出
#define IOCFG_IPD            IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_DOWN)      // 下拉输入
#define IOCFG_IPU            IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_UP)    	 // 上拉输入
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_NOPULL)    // 浮空输入

// 回调函数
typedef void (*IOTraverseFuncPtr_t)(IO_t io);

bool IORead(IO_t io);
void IOWrite(IO_t io, bool value);
void IOHi(IO_t io);
void IOLo(IO_t io);
void IOToggle(IO_t io);
void IOInit(IO_t io, resourceOwner_e owner, uint8_t index);
void IORelease(IO_t io);  // unimplemented
resourceOwner_e IOGetOwner(IO_t io);
bool IOIsFreeOrPreinit(IO_t io);
IO_t IOGetByTag(ioTag_t tag);
void IOConfigGPIO(IO_t io, ioConfig_t cfg);
void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af);
void IOInitGlobal(void);
void IOTraversePins(IOTraverseFuncPtr_t func);

