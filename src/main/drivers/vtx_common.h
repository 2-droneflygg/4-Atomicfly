#pragma once

#include <stdint.h>

#include "platform.h"
#include "common/time.h"

/* -----------------------------图传设备类型枚举--------------------------- */	
typedef enum {
    VTXDEV_UNSUPPORTED = 0, 		
    VTXDEV_SMARTAUDIO  = 3,      
    VTXDEV_UNKNOWN     = 0xFF,
} vtxDevType_e;

/* -------------------------------图传状态枚举----------------------------- */	
enum {
    VTX_STATUS_PIT_MODE = 1 << 0,
    VTX_STATUS_LOCKED = 1 << 1,
};
	
/* ----------------------------图传设备配置结构体-------------------------- */	
struct vtxVTable_s;
typedef struct vtxDevice_s {
    const struct vtxVTable_s *const vTable;
} vtxDevice_t;

/* -------------------------------图传表结构体----------------------------- */	
// 由图传设备进行注册对应的API函数
// {set,get}BandAndChannel: band and channel are 1 origin
// {set,get}PowerByIndex: 0 = Power OFF, 1 = device dependent
// {set,get}PitMode: 0 = OFF, 1 = ON
typedef struct vtxVTable_s {
    void (*process)(vtxDevice_t *vtxDevice, timeUs_t currentTimeUs);                             // 图传线程
    vtxDevType_e (*getDeviceType)(const vtxDevice_t *vtxDevice);								 // 获取图传设备类型
    bool (*isReady)(const vtxDevice_t *vtxDevice);												 // 是否就绪
    void (*setBandAndChannel)(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel);			 // 设置频组和通道
    void (*setPowerByIndex)(vtxDevice_t *vtxDevice, uint8_t level);								 // 通过索引设置功率
    void (*setPitMode)(vtxDevice_t *vtxDevice, uint8_t onoff);									 // 设置为PIT模式
    void (*setFrequency)(vtxDevice_t *vtxDevice, uint16_t freq);								 // 设置频率
    bool (*getBandAndChannel)(const vtxDevice_t *vtxDevice, uint8_t *pBand, uint8_t *pChannel);	 // 获取频组和通道
    bool (*getPowerIndex)(const vtxDevice_t *vtxDevice, uint8_t *pIndex);						 // 获取功率索引
    bool (*getFrequency)(const vtxDevice_t *vtxDevice, uint16_t *pFreq);						 // 获取频率
    bool (*getStatus)(const vtxDevice_t *vtxDevice, unsigned *status);							 // 获取状态
    uint8_t (*getPowerLevels)(const vtxDevice_t *vtxDevice, uint16_t *levels, uint16_t *powers); // 获取功率等级
} vtxVTable_t;

bool vtxCommonDeviceIsReady(const vtxDevice_t *vtxDevice);
void vtxCommonSetDevice(vtxDevice_t *vtxDevice);
vtxDevice_t *vtxCommonDevice(void);
void vtxCommonProcess(vtxDevice_t *vtxDevice, timeUs_t currentTimeUs);
vtxDevType_e vtxCommonGetDeviceType(const vtxDevice_t *vtxDevice);
void vtxCommonSetBandAndChannel(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel);
void vtxCommonSetPowerByIndex(vtxDevice_t *vtxDevice, uint8_t level);
void vtxCommonSetPitMode(vtxDevice_t *vtxDevice, uint8_t onoff);
void vtxCommonSetFrequency(vtxDevice_t *vtxDevice, uint16_t freq);
bool vtxCommonGetBandAndChannel(const vtxDevice_t *vtxDevice, uint8_t *pBand, uint8_t *pChannel);
bool vtxCommonGetPowerIndex(const vtxDevice_t *vtxDevice, uint8_t *pIndex);
bool vtxCommonGetFrequency(const vtxDevice_t *vtxDevice, uint16_t *pFreq);
bool vtxCommonGetStatus(const vtxDevice_t *vtxDevice, unsigned *status);
char vtxCommonGetBandLetter(const vtxDevice_t *vtxDevice, int band);
const char *vtxCommonLookupChannelName(const vtxDevice_t *vtxDevice, int channel);
uint16_t vtxCommonLookupFrequency(const vtxDevice_t *vtxDevice, int band, int channel);
void vtxCommonLookupBandChan(const vtxDevice_t *vtxDevice, uint16_t freq, uint8_t *pBand, uint8_t *pChannel);
const char *vtxCommonLookupPowerName(const vtxDevice_t *vtxDevice, int index);
bool vtxCommonLookupPowerValue(const vtxDevice_t *vtxDevice, int index, uint16_t *pPowerValue);
char vtxCommonLookupBandLetter(const vtxDevice_t *vtxDevice, int band);

