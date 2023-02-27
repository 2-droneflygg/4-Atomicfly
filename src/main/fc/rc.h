#pragma once

#include "drivers/time.h"

#include "fc/rc_controls.h"

/* ---------------------------RC平滑-插值法类型枚举--------------------------- */	
typedef enum {
    INTERPOLATION_CHANNELS_RP,
    INTERPOLATION_CHANNELS_RPY,
    INTERPOLATION_CHANNELS_RPYT,
    INTERPOLATION_CHANNELS_T,
    INTERPOLATION_CHANNELS_RPT,
} interpolationChannels_e;

void processRcCommand(void);
float getSetpointRate(int axis);
float getRcDeflection(int axis);
float getRcDeflectionAbs(int axis);
float getThrottlePIDAttenuation(void);
void updateRcCommands(void);
void initRcProcessing(void);
bool rcSmoothingIsEnabled(void);
rcSmoothingFilter_t *getRcSmoothingData(void);
bool rcSmoothingAutoCalculate(void);
bool rcSmoothingInitializationComplete(void);
float getRawSetpoint(int axis);
float applyCurve(int axis, float deflection);
uint32_t getRcFrameNumber();
void updateRcRefreshRate(timeUs_t currentTimeUs);
uint16_t getCurrentRxRefreshRate(void);

