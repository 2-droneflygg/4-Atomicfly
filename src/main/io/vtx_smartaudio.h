#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#define dprintf(x)

#define VTX_SMARTAUDIO_MIN_BAND    1
#define VTX_SMARTAUDIO_MIN_CHANNEL 1

#define VTX_SMARTAUDIO_MIN_FREQUENCY_MHZ 5000       // min freq in MHz
#define VTX_SMARTAUDIO_MAX_FREQUENCY_MHZ 5999       // max freq in MHz

// opmode flags, GET side
#define SA_MODE_GET_FREQ_BY_FREQ            1
#define SA_MODE_GET_PITMODE                 2
#define SA_MODE_GET_IN_RANGE_PITMODE        4
#define SA_MODE_GET_OUT_RANGE_PITMODE       8
#define SA_MODE_GET_UNLOCK                 16
#define SA_MODE_GET_DEFERRED_FREQ          32

// opmode flags, SET side
#define SA_MODE_SET_IN_RANGE_PITMODE        1 		// Immediate
#define SA_MODE_SET_OUT_RANGE_PITMODE       2 		// Immediate
#define SA_MODE_CLR_PITMODE                 4 		// Immediate
#define SA_MODE_SET_UNLOCK                  8
#define SA_MODE_SET_LOCK                    0 		// ~UNLOCK
#define SA_MODE_SET_DEFERRED_FREQ          16

// SetFrequency标志，用于PIT模式频率操作
#define SA_FREQ_GETPIT                      (1 << 14)
#define SA_FREQ_SETPIT                      (1 << 15)
#define SA_FREQ_MASK                        (~(SA_FREQ_GETPIT|SA_FREQ_SETPIT))

/* --------------------------SmartAudio设备结构体-------------------------- */	
// 用于通用API的使用，但这里只是暂时的
typedef struct smartAudioDevice_s {
    int8_t version;
    int8_t channel;
    int8_t power;
    int8_t mode;
    uint16_t freq;
    uint16_t orfreq;
    bool willBootIntoPitMode;
} smartAudioDevice_t;

/* --------------------------SmartAudio状态结构体-------------------------- */	
typedef struct smartAudioStat_s {
    uint16_t pktsent;
    uint16_t pktrcvd;
    uint16_t badpre;
    uint16_t badlen;
    uint16_t crc;
    uint16_t ooopresp;
    uint16_t badcode;
} smartAudioStat_t;

extern smartAudioDevice_t saDevice;
extern smartAudioStat_t saStat;
extern uint16_t sa_smartbaud;
extern bool saDeferred;

void saSetMode(int mode);
void saSetFreq(uint16_t freq);
void saSetPitFreq(uint16_t freq);
bool vtxSmartAudioInit(void);

