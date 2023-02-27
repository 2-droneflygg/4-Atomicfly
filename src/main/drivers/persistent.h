#pragma once

/* -----------------------------RTC备份寄存器ID枚举------------------------------ */	
// 每个MCU类型的可用RTC备份寄存器(4字节字)
// F4: 20 words
typedef enum {
    PERSISTENT_OBJECT_MAGIC = 0,		  // 固定值
    PERSISTENT_OBJECT_HSE_VALUE,		  // HSE值
    PERSISTENT_OBJECT_OVERCLOCK_LEVEL,	  // 超频级别
    PERSISTENT_OBJECT_RESET_REASON,		  // 重启原因
    PERSISTENT_OBJECT_COUNT,
} persistentObjectId_e;

// 重置原因的值
#define RESET_NONE                      0  // 无复位 
#define RESET_BOOTLOADER_REQUEST_ROM    1  // 请求了引导加载程序调用

uint32_t persistentObjectRead(persistentObjectId_e id);
void persistentObjectWrite(persistentObjectId_e id, uint32_t value);
void persistentObjectInit(void);

