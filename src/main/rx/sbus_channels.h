#pragma once

#include <stdint.h>

#define SBUS_MAX_CHANNEL 18					  			 // SBUS最大通道数

#define SBUS_FLAG_SIGNAL_LOSS       (1 << 2)  			 // SBUS信号丢失标志
#define SBUS_FLAG_FAILSAFE_ACTIVE   (1 << 3)  			 // SBUS失控保护激活标志

/* --------------------------SBUS通道结构体-------------------------- */	
typedef struct sbusChannels_s {
    // 176位数据(每个通道11位* 16个通道)            - 22字节
    // flags标志位用来检测丢帧和失控保护事件 - 1字节
    // 位域是指为位为单位来定义某结构体的成员变量 它和结构体是分不开的
    // C99规定int、unsigned int和bool可以作为位域类型，使用位域的主要目的是压缩存储
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
    uint8_t flags;										 // 通道标志
} __attribute__((__packed__)) sbusChannels_t;
#define SBUS_CHANNEL_DATA_LENGTH sizeof(sbusChannels_t)  // SBUS通道数据长度

uint8_t sbusChannelsDecode(rxRuntimeState_t *rxRuntimeState, const sbusChannels_t *channels);
void sbusChannelsInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState);

