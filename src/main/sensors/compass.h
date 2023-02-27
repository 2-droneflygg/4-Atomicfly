#pragma once

#include "common/time.h"
#include "sensors/boardalignment.h"
#include "drivers/io_types.h"
#include "drivers/sensor.h"
#include "pg/pg.h"
#include "sensors/sensors.h"

/* --------------------------使用/检测的磁力计类型枚举-------------------------- */	
typedef enum {
    MAG_DEFAULT = 0,
    MAG_NONE = 1,
    MAG_HMC5883 = 2,
} magSensor_e;
	
/* --------------------------------磁力计结构体--------------------------------- */	
typedef struct mag_s {
    float magADC[XYZ_AXIS_COUNT];
    float magneticDeclination;
} mag_t;
// 声明磁力计结构体
extern mag_t mag;

/* -------------------------------磁力计配置结构体------------------------------ */	
typedef struct compassConfig_s {
    int16_t mag_declination;                // 从这里获取你的磁偏角 : http://magnetic-declination.com/
                                            // 例如-6deg 37min， = -637 Japan，格式为[sign]dddmm (degreesminutes)默认值为0。
    uint8_t mag_alignment;                  // 磁力计板对齐
    uint8_t mag_hardware;                   // 磁力计硬件设备
    uint8_t mag_bustype;					// 总线类型
    uint8_t mag_i2c_device;					// IIC设备
    uint8_t mag_i2c_address;				// IIC地址
    uint8_t mag_spi_device;					// SPI设备
    ioTag_t mag_spi_csn;					// SPI片选线
    ioTag_t interruptTag;					// 中断线
    flightDynamicsTrims_t magZero;			// 磁力计零偏
} compassConfig_t;
// 声明磁力计配置结构体
PG_DECLARE(compassConfig_t, compassConfig);

bool compassIsHealthy(void);
void compassUpdate(timeUs_t currentTime);
bool compassInit(void);
void compassStartCalibration(void);
bool compassIsCalibrationComplete(void);

