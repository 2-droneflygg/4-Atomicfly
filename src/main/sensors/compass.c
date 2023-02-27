/*********************************************************************************
 提供磁力计相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_MAG)
#include "common/axis.h"

#include "config/config.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/compass/compass.h"
#include "drivers/compass/compass_hmc5883l.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/sensors.h"

#include "compass.h"

// ---------------------------------------------------------磁力计校准周期
static timeUs_t tCal = 0;
// ---------------------------------------------------------磁力计零偏
static flightDynamicsTrims_t magZeroTempMin;
static flightDynamicsTrims_t magZeroTempMax;

// ---------------------------------------------------------磁力计设备
magDev_t magDev;
// ---------------------------------------------------------磁力计访问
mag_t mag;              

// ---------------------------------------------------------磁力计原始数据
static int16_t magADCRaw[XYZ_AXIS_COUNT];
// ---------------------------------------------------------磁力计初始化状态
static uint8_t magInit = 0;

PG_REGISTER_WITH_RESET_FN(compassConfig_t, compassConfig, PG_COMPASS_CONFIG, 2);
void pgResetFn_compassConfig(compassConfig_t *compassConfig)
{
    compassConfig->mag_alignment = CW90_DEG_FLIP;			// 板对齐
    compassConfig->mag_declination = 0;						// 偏差
    compassConfig->mag_hardware = MAG_HMC5883;				// 磁力计硬件：HMC5883

#if defined(USE_MAG_HMC5883) 
    compassConfig->mag_bustype = BUSTYPE_I2C;			    // IIC通讯
    compassConfig->mag_i2c_device = I2C_DEV_TO_CFG(MAG_I2C_INSTANCE);
    compassConfig->mag_i2c_address = 0;
    compassConfig->mag_spi_device = SPI_DEV_TO_CFG(SPIINVALID);
    compassConfig->mag_spi_csn = IO_TAG_NONE;
#endif
    compassConfig->interruptTag = IO_TAG(MAG_INT_EXTI);
}

//-------------------------------------------------------------------------------------磁力计校准相关API

/**********************************************************************
函数名称：compassStartCalibration
函数功能：磁力计开始校准
函数形参：None  
函数返回值：None 
函数描述：None 
**********************************************************************/
void compassStartCalibration(void)
{
    tCal = micros();
    flightDynamicsTrims_t *magZero = &compassConfigMutable()->magZero;
    for (int axis = 0; axis < 3; axis++) {
        magZero->raw[axis] = 0;
        magZeroTempMin.raw[axis] = mag.magADC[axis];
        magZeroTempMax.raw[axis] = mag.magADC[axis];
    }
}

/**********************************************************************
函数名称：compassIsCalibrationComplete
函数功能：磁力计是否校准完毕
函数形参：None  
函数返回值：状态
函数描述：None 
**********************************************************************/
bool compassIsCalibrationComplete(void)
{
    return tCal == 0;
}

//-------------------------------------------------------------------------------------磁力计检测相关API

/**********************************************************************
函数名称：compassDetect
函数功能：磁力计检测
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
bool compassDetect(magDev_t *dev, uint8_t *alignment)
{
	// 如果目标指定MAG_*_ALIGN可能被覆盖
    *alignment = ALIGN_DEFAULT;  

    magSensor_e magHardware = MAG_NONE;
	// 获取总线设备
    busDevice_t *busdev = &dev->busdev;
	// 	IIC通讯
    switch (compassConfig()->mag_bustype) {			
#ifdef USE_I2C
	    case BUSTYPE_I2C:
	        busdev->bustype = BUSTYPE_I2C;
	        busdev->busdev_u.i2c.device = I2C_CFG_TO_DEV(compassConfig()->mag_i2c_device);
	        busdev->busdev_u.i2c.address = compassConfig()->mag_i2c_address;
	        break;
#endif

	    default:
	        return false;
    }

    switch (compassConfig()->mag_hardware) {
	    case MAG_DEFAULT:
	        FALLTHROUGH;
	    case MAG_HMC5883:
#if defined(USE_MAG_HMC5883) 
	        if (busdev->bustype == BUSTYPE_I2C) {
	            busdev->busdev_u.i2c.address = compassConfig()->mag_i2c_address;
	        }

	        if (hmc5883lDetect(dev)) {
#ifdef MAG_HMC5883_ALIGN
				// 板对齐
	            *alignment = MAG_HMC5883_ALIGN;
#endif			
	            magHardware = MAG_HMC5883;
	            break;
        }
#endif
        FALLTHROUGH;

    case MAG_NONE:
        magHardware = MAG_NONE;
        break;
    }

    if (magHardware == MAG_NONE) {
        return false;
    }
	// 磁力计在线
    detectedSensors[SENSOR_INDEX_MAG] = magHardware;
	// 使能磁力计
    sensorsSet(SENSOR_MAG);
    return true;
}

//-------------------------------------------------------------------------------------磁力计初始化相关API

/**********************************************************************
函数名称：compassInit
函数功能：磁力计初始化
函数形参：None  
函数返回值：状态
函数描述：None 
**********************************************************************/
bool compassInit(void)
{
	// 初始化和校准，mag校准时打开led(校准程序会使其闪烁)
	// 计算磁偏角
	// 如果没有mag传感器，或者配置中存储的值应该被使用，请调查是否需要这样做
    mag.magneticDeclination = 0.0f;    

    sensor_align_e alignment;
	// 磁力计检测并获取板对齐
    if (!compassDetect(&magDev, &alignment)) {
        return false;
    }

    const int16_t deg = compassConfig()->mag_declination / 100;
    const int16_t min = compassConfig()->mag_declination % 100;
	// heading 以0.1度为单位
    mag.magneticDeclination = (deg + ((float)min * (1.0f / 60.0f))) * 10; 
    magDev.init(&magDev);
    magInit = 1;

	// 磁力计板对齐
    magDev.magAlignment = alignment;

    if (compassConfig()->mag_alignment != ALIGN_DEFAULT) {
        magDev.magAlignment = compassConfig()->mag_alignment;
    }

    return true;
}

//-------------------------------------------------------------------------------------磁力计监测相关API

/**********************************************************************
函数名称：compassIsHealthy
函数功能：获取磁力计健康状态
函数形参：None  
函数返回值：状态
函数描述：
	三轴磁力计数据各个分量都不为0。
**********************************************************************/
bool compassIsHealthy(void)
{
    return (mag.magADC[X] != 0) && (mag.magADC[Y] != 0) && (mag.magADC[Z] != 0);
}

//-------------------------------------------------------------------------------------磁力计更新相关API

/**********************************************************************
函数名称：compassUpdate
函数功能：磁力计更新
函数形参：currentTimeUs
函数返回值：None 
函数描述：
	由调度器以任务形式调用。
**********************************************************************/
void compassUpdate(timeUs_t currentTimeUs)
{
	// 读取磁力计原始数据
    magDev.read(&magDev, magADCRaw);

	// 保存磁力计数据
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        mag.magADC[axis] = magADCRaw[axis];
    }
	
   	// 磁力计数据板对齐
    alignSensorViaRotation(mag.magADC, magDev.magAlignment);
	
	// 应用磁力计零偏
    flightDynamicsTrims_t *magZero = &compassConfigMutable()->magZero;
	// 只在mag校准完成后应用偏移量
    if (magInit) {            					    
        mag.magADC[X] -= magZero->raw[X];
        mag.magADC[Y] -= magZero->raw[Y];
        mag.magADC[Z] -= magZero->raw[Z];
    }

	// 磁力计校准
    if (tCal != 0) {
        if ((currentTimeUs - tCal) < 30000000) {    
			// 30秒校准:有30秒的时间可以将机体转向各个方向
            LED0_TOGGLE;
            for (int axis = 0; axis < 3; axis++) {
                if (mag.magADC[axis] < magZeroTempMin.raw[axis])
                    magZeroTempMin.raw[axis] = mag.magADC[axis];
                if (mag.magADC[axis] > magZeroTempMax.raw[axis])
                    magZeroTempMax.raw[axis] = mag.magADC[axis];
            }
        } else {	
        	// 校准完成
            tCal = 0;
            for (int axis = 0; axis < 3; axis++) {
				// 计算偏移量
                magZero->raw[axis] = (magZeroTempMin.raw[axis] + magZeroTempMax.raw[axis]) / 2;  
            }
			// 保存配置和通知
            saveConfigAndNotify();
        }
    }
}
#endif // USE_MAG

