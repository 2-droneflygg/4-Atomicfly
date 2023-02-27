#pragma once

#include <stdint.h>
#include <stdbool.h>

// 配置流大小 - 需要将数据流输出到EEPROM，填充到写入大小,并更新校验和
// 对于32位单片机系统来说，一个字是4个字节（32位）
#define CONFIG_STREAMER_BUFFER_SIZE 4
typedef uint32_t config_streamer_buffer_align_type_t;

/* --------------------------配置流结构体-------------------------- */	
typedef struct config_streamer_s {
    uintptr_t address;                             // 地址
    int size;									   // 大小
    union {
        uint8_t b[CONFIG_STREAMER_BUFFER_SIZE];
        config_streamer_buffer_align_type_t w;     // 字
    } buffer;									   // 缓存
    int at;
    int err;									   // 流状态
    bool unlocked;								   // FLASH解锁状态
} config_streamer_t;

void config_streamer_init(config_streamer_t *c);
void config_streamer_start(config_streamer_t *c, uintptr_t base, int size);
int config_streamer_write(config_streamer_t *c, const uint8_t *p, uint32_t size);
int config_streamer_flush(config_streamer_t *c);
int config_streamer_finish(config_streamer_t *c);

