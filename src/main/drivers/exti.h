#pragma once

#include <stdbool.h>

#include "drivers/io_types.h"

/* -----------------------------外部中断触发方式枚举------------------------------ */	
typedef enum {
    EXTI_TRIGGER_RISING = 0,	// 上升沿触发
    EXTI_TRIGGER_FALLING = 1,   // 下降沿触发
    EXTI_TRIGGER_BOTH = 2       // 边沿触发
} extiTrigger_t;

/* -----------------------------EXTI中断回调函数记录------------------------------ */	
// 命名重定义
typedef struct extiCallbackRec_s extiCallbackRec_t;
// EXTI中断服务回调函数
typedef void extiHandlerCallback(extiCallbackRec_t *self); 
struct extiCallbackRec_s {
    extiHandlerCallback *fn;
};

void EXTIInit(void);
void EXTIHandlerInit(extiCallbackRec_t *cb, extiHandlerCallback *fn);
void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, ioConfig_t config, extiTrigger_t trigger);
void EXTIEnable(IO_t io, bool enable);

