#pragma once

#include "drivers/serial.h"

#define SMARTAUDIO_SERIAL_OPTIONS   SERIAL_NOT_INVERTED | SERIAL_BIDIR_NOPULL | SERIAL_STOPBITS_2  // 端口选项
#define SMARTAUDIO_DEFAULT_BAUD     4900														   // 默认波特率
#define SMARTAUDIO_MIN_BAUD         4800														   // 最小波特率
#define SMARTAUDIO_MAX_BAUD         4950														   // 最大波特率

/* --------------------------Smartaudio设置结构体-------------------------- */	
typedef struct smartaudioSettings_s {
    uint8_t version;                      // 版本
    uint8_t unlocked; 					  // 解锁
    uint8_t channel;					  // 通道
    uint8_t power;						  // 功率
    uint16_t frequency;					  // 频率
    uint16_t pitmodeFrequency;			  // PIT模式频率
    bool userFrequencyMode;				  // 用户频率模式
    bool pitmodeEnabled;				  // PIT模式使能状态
    bool pitmodeInRangeActive;			  // PIT模式激活输入范围
    bool pitmodeOutRangeActive;		      // PIT模式激活输出范围			
} smartaudioSettings_t;

/* --------------------------Smartaudio帧头结构体-------------------------- */	
typedef struct smartaudioFrameHeader_s {
    uint16_t startCode;					  // 起始代码
    uint8_t length;						  // 长度
    uint8_t command;					  // 命令
} __attribute__((packed)) smartaudioFrameHeader_t;

/* -------------------------Smartaudio命令帧结构体------------------------- */	
typedef struct smartaudioCommandOnlyFrame_s {
    smartaudioFrameHeader_t header;		  // 帧头
    uint8_t crc;						  // CRC
} __attribute__((packed)) smartaudioCommandOnlyFrame_t;

/* --------------------------SmartaudioU8帧结构体-------------------------- */	
typedef struct smartaudioU8Frame_s {
    smartaudioFrameHeader_t header;		  // 帧头
    uint8_t payload;					  // 有效负载
    uint8_t crc;						  // CRC
} __attribute__((packed)) smartaudioU8Frame_t;

/* -------------------------SmartaudioU16帧结构体-------------------------- */	
typedef struct smartaudioU16Frame_s {
    smartaudioFrameHeader_t header;		  // 帧头
    uint16_t payload;					  // 有效负载
    uint8_t crc;						  // CRC
} __attribute__((packed)) smartaudioU16Frame_t;

/* ------------------------SmartaudioU8响应帧结构体------------------------ */	
typedef struct smartaudioU8ResponseFrame_s {
    smartaudioFrameHeader_t header;		  // 帧头
    uint8_t payload;					  // 有效负载
    uint8_t reserved;					  // 响应
    uint8_t crc;						  // CRC
} __attribute__((packed)) smartaudioU8ResponseFrame_t;

/* ------------------------SmartaudioU16响应帧结构体----------------------- */	
typedef struct smartaudioU16ResponseFrame_s {
    smartaudioFrameHeader_t header;		  // 帧头
    uint16_t payload;					  // 有效负载
    uint8_t reserved;					  // 响应
    uint8_t crc;						  // CRC
} __attribute__((packed)) smartaudioU16ResponseFrame_t;

/* -----------------------Smartaudio设置响应帧结构体----------------------- */	
typedef struct smartaudioSettingsResponseFrame_s {
    smartaudioFrameHeader_t header;		  // 帧头
    uint8_t channel;					  // 通道
    uint8_t power;						  // 功率
    uint8_t operationMode;				  // 操作模式
    uint16_t frequency;					  // 频率
    uint8_t crc;						  // CRC
} __attribute__((packed)) smartaudioSettingsResponseFrame_t;

/* ---------------------------Smartaudio帧共用体--------------------------- */	
typedef union smartaudioFrame_u {
    smartaudioCommandOnlyFrame_t commandOnlyFrame;   // 命令帧
    smartaudioU8Frame_t u8RequestFrame;				 // U8帧
    smartaudioU16Frame_t u16RequestFrame;			 // U16帧
} __attribute__((packed)) smartaudioFrame_t;

size_t smartaudioFrameGetSettings(smartaudioFrame_t *smartaudioFrame);
size_t smartaudioFrameGetPitmodeFrequency(smartaudioFrame_t *smartaudioFrame);
size_t smartaudioFrameSetPower(smartaudioFrame_t *smartaudioFrame, const uint8_t power);
size_t smartaudioFrameSetBandChannel(smartaudioFrame_t *smartaudioFrame, const uint8_t band, const uint8_t channel);
size_t smartaudioFrameSetFrequency(smartaudioFrame_t *smartaudioFrame, const uint16_t frequency, const bool pitmodeFrequency);
size_t smartaudioFrameSetOperationMode(smartaudioFrame_t *smartaudioFrame, const smartaudioSettings_t *settings);
bool smartaudioParseResponseBuffer(smartaudioSettings_t *settings, const uint8_t *buffer);

