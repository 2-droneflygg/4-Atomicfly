/*********************************************************************************
 提供传感器初始化相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/runtime_config.h"

#include "flight/pid.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/initialisation.h"
#include "sensors/sensors.h"

// ---------------------------------------------------------传感器检测状态
uint8_t detectedSensors[SENSOR_INDEX_COUNT]  = { GYRO_NONE, ACC_NONE, BARO_NONE, MAG_NONE };

/**********************************************************************
函数名称：sensorsPreInit
函数功能：传感器预初始化
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void sensorsPreInit(void)
{
	// ---------------------------陀螺仪预初始化
    gyroPreInit();
#ifdef USE_BARO
	// ---------------------------加速度计预初始化
    baroPreInit();
#endif
}

/**********************************************************************
函数名称：sensorsAutodetect
函数功能：传感器自动检测并进行初始化
函数形参：None 
函数返回值：陀螺仪检测状态 - 陀螺仪时必要传感器
函数描述：None 
**********************************************************************/
bool sensorsAutodetect(void)
{
	// ---------------------------陀螺仪初始化
	// 陀螺仪必须在加速度计之前初始化
    bool gyroDetected = gyroInit();
	// ---------------------------加速度计初始化
#ifdef USE_ACC
    if (gyroDetected) {
        accInit(gyro.accSampleRateHz);
    }
#endif
	// ---------------------------磁力计初始化
#ifdef USE_MAG
    compassInit();
#endif
	// ---------------------------气压计初始化
#ifdef USE_BARO
    baroDetect(&baro.dev, barometerConfig()->baro_hardware);
#endif
    return gyroDetected;
}

