#pragma once

#include "drivers/serial.h"

/* -------------------------USB配置结构体------------------------- */	
typedef struct {
	// 端口
    serialPort_t port;
    // 发送缓冲区
    uint8_t txBuf[20];
    uint8_t txAt;
    // 如果端口处于大容量写模式并且可以缓冲则使能  
    bool buffering;
} vcpPort_t;

struct serialPort_s;
serialPort_t *usbVcpOpen(void);

