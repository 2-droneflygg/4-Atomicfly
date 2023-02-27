#pragma once

#include <stdint.h>

#include "common/axis.h"
#include "flight/pid.h"

/* --------------------------前馈插值类型枚举-------------------------- */
// 滞后平均移动滤波器窗口大小
typedef enum ffInterpolationType_e {
    FF_INTERPOLATE_OFF,
    FF_INTERPOLATE_ON,       
    FF_INTERPOLATE_AVG2,
    FF_INTERPOLATE_AVG3,
    FF_INTERPOLATE_AVG4
} ffInterpolationType_t;

void interpolatedSpInit(const pidProfile_t *pidProfile);
float interpolatedSpApply(int axis, bool newRcFrame, ffInterpolationType_t type);
float applyFfLimit(int axis, float value, float Kp, float currentPidSetpoint);
bool shouldApplyFfLimits(int axis);

