#pragma once

#include "drivers/bus.h"

#define MPU6000_CONFIG              0x1A
// (4/131) * pi/180   (32.75 LSB = 1 DPS)
#define GYRO_SCALE_FACTOR  0.00053292f  
// RF = Register Flag
#define MPU_RF_DATA_RDY_EN (1 << 0)

uint8_t mpu6000SpiDetect(const busDevice_t *bus);
static void mpu6000AccAndGyroInit(gyroDev_t *gyro);
bool mpu6000SpiAccDetect(accDev_t *acc);
bool mpu6000SpiGyroDetect(gyroDev_t *gyro);

