#pragma once

#include <stdint.h>

/* -------------------------sbuf结构体------------------------ */	
// 简单的基于缓冲区的序列化/反序列化程序，没有隐式的大小检查
typedef struct sbuf_s {
    uint8_t *ptr;    // 数据指针必须优先(sbuf_t*等效于uint8_t **)
    uint8_t *end;
}sbuf_t;

sbuf_t *sbufInit(sbuf_t *sbuf, uint8_t *ptr, uint8_t *end);
void sbufWriteU8(sbuf_t *dst, uint8_t val);
void sbufWriteU16(sbuf_t *dst, uint16_t val);
void sbufWriteU32(sbuf_t *dst, uint32_t val);
void sbufWriteData(sbuf_t *dst, const void *data, int len);
void sbufWriteString(sbuf_t *dst, const char *string);
uint8_t sbufReadU8(sbuf_t *src);
uint16_t sbufReadU16(sbuf_t *src);
uint32_t sbufReadU32(sbuf_t *src);
void sbufReadData(sbuf_t *dst, void *data, int len);
int sbufBytesRemaining(sbuf_t *buf);
uint8_t* sbufPtr(sbuf_t *buf);
void sbufAdvance(sbuf_t *buf, int size);
void sbufSwitchToReader(sbuf_t *buf, uint8_t * base);

