/*********************************************************************************
 提供陀螺仪同步相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/gyro_sync.h"

/**********************************************************************
函数名称：gyroSetSampleRate
函数功能：陀螺设定采样率
函数形参：陀螺仪设备结构体
函数返回值：gyroSampleRateHz
函数描述：None 
**********************************************************************/
uint16_t gyroSetSampleRate(gyroDev_t *gyro)
{
    uint16_t gyroSampleRateHz;
    uint16_t accSampleRateHz;

	// 设置陀螺仪采样速率
    switch (gyro->mpuDetectionResult.sensor) {
        default:
            gyro->gyroRateKHz = GYRO_RATE_8_kHz;
            gyroSampleRateHz = 8000;
            accSampleRateHz = 1000;
            break;
    }
	// 不再使用陀螺仪的采样分配器
    gyro->mpuDividerDrops  = 0;      
    // 设置加速度计采样速率
    gyro->accSampleRateHz = accSampleRateHz;
    return gyroSampleRateHz;
}

