#pragma once

#include "common/time.h"
#include "pg/pg.h"

#if defined(USE_GPS) || defined(USE_MAG)
extern int16_t magHold;
#endif

/* ---------------------------飞行日志禁止解锁原因枚举--------------------------- */	
typedef enum {
    DISARM_REASON_ARMING_DISABLED   = 0,
    DISARM_REASON_FAILSAFE          = 1,
    DISARM_REASON_THROTTLE_TIMEOUT  = 2,
    DISARM_REASON_STICKS            = 3,
    DISARM_REASON_SWITCH            = 4,
    DISARM_REASON_CRASH_PROTECTION  = 5,
    DISARM_REASON_RUNAWAY_TAKEOFF   = 6,
    DISARM_REASON_GPS_RESCUE        = 7,
    DISARM_REASON_SERIAL_COMMAND    = 8,
} flightLogDisarmReason_e;

union rollAndPitchTrims_u;
void resetArmingDisabled(void);
void disarm(flightLogDisarmReason_e reason);
void tryArm(void);
bool processRx(timeUs_t currentTimeUs);
void updateArmingStatus(void);
void taskGyroSample(timeUs_t currentTimeUs);
bool gyroFilterReady(void);
bool pidLoopReady(void);
void taskFiltering(timeUs_t currentTimeUs);
void taskMainPidLoop(timeUs_t currentTimeUs);
bool isFlipOverAfterCrashActive(void);
int8_t calculateThrottlePercent(void);
uint8_t calculateThrottlePercentAbs(void);
bool areSticksActive(uint8_t stickPercentLimit);
void runawayTakeoffTemporaryDisable(uint8_t disableFlag);
bool isAirmodeActivated();
timeUs_t getLastDisarmTimeUs(void);
bool isTryingToArm();
void resetTryingToArm();
void subTaskTelemetryPollSensors(timeUs_t currentTimeUs);

