#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "pg/pg.h"
#include "fc/rc_modes.h"

#define MAX_CHANNEL_ACTIVATION_CONDITION_COUNT  10

/* --------------------------vtx通道激活情况结构体-------------------------- */	
typedef struct vtxChannelActivationCondition_s {
    uint8_t auxChannelIndex;
    uint8_t band;
    uint8_t channel;
    uint8_t power;
    channelRange_t range;
} vtxChannelActivationCondition_t;

/* ------------------------------vtx配置结构体------------------------------ */	
typedef struct vtxConfig_s {
    vtxChannelActivationCondition_t vtxChannelActivationConditions[MAX_CHANNEL_ACTIVATION_CONDITION_COUNT];
    uint8_t halfDuplex;
} vtxConfig_t;
// 声明VTX配置结构体
PG_DECLARE(vtxConfig_t, vtxConfig);

void vtxControlInputPoll(void);
void vtxIncrementBand(void);
void vtxDecrementBand(void);
void vtxIncrementChannel(void);
void vtxDecrementChannel(void);
void vtxCyclePower(const uint8_t powerStep);
void vtxCycleBandOrChannel(const uint8_t bandStep, const uint8_t channelStep);
void vtxUpdateActivatedChannel(void);

