#pragma once

#include "pg/gyrodev.h"
#include "sensors/gyro.h"

int16_t gyroRateDps(int axis);
void gyroSetTargetLooptime(uint8_t pidDenom);
void gyroPreInit(void);
bool gyroInit(void);
void gyroInitFilters(void);
void gyroInitSensor(gyroSensor_t *gyroSensor, const gyroDeviceConfig_t *config);
const busDevice_t *gyroSensorBus(void);
struct mpuDetectionResult_s;
const struct mpuDetectionResult_s *gyroMpuDetectionResult(void);
uint8_t gyroReadRegister(uint8_t whichSensor, uint8_t reg);

