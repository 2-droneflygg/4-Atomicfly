/*********************************************************************************
 提供BMP280驱动相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/build_config.h"

#include "barometer.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "barometer_bmp280.h"

#if defined(USE_BARO) && (defined(USE_BARO_BMP280) || defined(USE_BARO_SPI_BMP280))
/* -----------------------------寄存器地址------------------------------ */	
#define BMP280_I2C_ADDR                      (0x76)
#define BMP280_DEFAULT_CHIP_ID               (0x58)

#define BMP280_CHIP_ID_REG                   (0xD0)  /* Chip ID Register */
#define BMP280_RST_REG                       (0xE0)  /* Softreset Register */
#define BMP280_STAT_REG                      (0xF3)  /* Status Register */
#define BMP280_CTRL_MEAS_REG                 (0xF4)  /* Ctrl Measure Register */
#define BMP280_CONFIG_REG                    (0xF5)  /* Configuration Register */
#define BMP280_PRESSURE_MSB_REG              (0xF7)  /* Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG              (0xF8)  /* Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG             (0xF9)  /* Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG           (0xFA)  /* Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG           (0xFB)  /* Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG          (0xFC)  /* Temperature XLSB Reg */
#define BMP280_FORCED_MODE                   (0x01)

#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x88)
#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)
#define BMP280_DATA_FRAME_SIZE               (6)

#define BMP280_OVERSAMP_SKIPPED              (0x00)
#define BMP280_OVERSAMP_1X               	 (0x01)
#define BMP280_OVERSAMP_2X               	 (0x02)
#define BMP280_OVERSAMP_4X               	 (0x03)
#define BMP280_OVERSAMP_8X               	 (0x04)
#define BMP280_OVERSAMP_16X               	 (0x05)

// 配置压力和温度采样，强制采样模式
#define BMP280_PRESSURE_OSR              (BMP280_OVERSAMP_8X)
#define BMP280_TEMPERATURE_OSR           (BMP280_OVERSAMP_1X)
#define BMP280_MODE                      (BMP280_PRESSURE_OSR << 2 | BMP280_TEMPERATURE_OSR << 5 | BMP280_FORCED_MODE)

#define T_INIT_MAX                       (20)
// 20/16 = 1.25 ms
#define T_MEASURE_PER_OSRS_MAX           (37)
// 37/16 = 2.3125 ms
#define T_SETUP_PRESSURE_MAX             (10)
// 10/16 = 0.625 ms

/* ------------------------BMP280校准参数结构体------------------------- */
typedef struct bmp280_calib_param_s {
    uint16_t dig_T1; /* calibration T1 data */
    int16_t dig_T2;  /* calibration T2 data */
    int16_t dig_T3;  /* calibration T3 data */
    uint16_t dig_P1; /* calibration P1 data */
    int16_t dig_P2;  /* calibration P2 data */
    int16_t dig_P3;  /* calibration P3 data */
    int16_t dig_P4;  /* calibration P4 data */
    int16_t dig_P5;  /* calibration P5 data */
    int16_t dig_P6;  /* calibration P6 data */
    int16_t dig_P7;  /* calibration P7 data */
    int16_t dig_P8;  /* calibration P8 data */
    int16_t dig_P9;  /* calibration P9 data */
} __attribute__((packed)) bmp280_calib_param_t; 

STATIC_ASSERT(sizeof(bmp280_calib_param_t) == BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH, bmp280_calibration_structure_incorrectly_packed);
 /* calibration t_fine data */
STATIC_UNIT_TESTED int32_t t_fine;
static uint8_t bmp280_chip_id = 0;
STATIC_UNIT_TESTED bmp280_calib_param_t bmp280_cal;
// 未补偿的压力和温度
int32_t bmp280_up = 0;
int32_t bmp280_ut = 0;
static uint8_t sensor_data[BMP280_DATA_FRAME_SIZE];

static void bmp280StartUT(baroDev_t *baro);
static bool bmp280ReadUT(baroDev_t *baro);
static bool bmp280GetUT(baroDev_t *baro);
static void bmp280StartUP(baroDev_t *baro);
static bool bmp280ReadUP(baroDev_t *baro);
static bool bmp280GetUP(baroDev_t *baro);
STATIC_UNIT_TESTED void bmp280Calculate(int32_t *pressure, int32_t *temperature);

/**********************************************************************
函数名称：bmp280BusInit
函数功能：BMP280总线初始化
函数形参：总线设备结构体
函数返回值：None  
函数描述：None 
**********************************************************************/
void bmp280BusInit(busDevice_t *busdev)
{
#ifdef USE_BARO_SPI_BMP280
    if (busdev->bustype == BUSTYPE_SPI) {
		// 片选拉高
        IOHi(busdev->busdev_u.spi.csnPin); 
		// IO初始化
        IOInit(busdev->busdev_u.spi.csnPin, OWNER_BARO_CS, 0);
		// 配置IO
        IOConfigGPIO(busdev->busdev_u.spi.csnPin, IOCFG_OUT_PP);
		// 设置SPI总线分频      	- 8   10.50000 MHz
        spiBusSetDivisor(busdev, SPI_CLOCK_STANDARD);
    }
#else
    UNUSED(busdev);
#endif
}

/**********************************************************************
函数名称：bmp280BusDeinit
函数功能：SPI引脚所有者为预初始化
函数形参：总线设备结构体
函数返回值：None  
函数描述：None 
**********************************************************************/
void bmp280BusDeinit(busDevice_t *busdev)
{
#ifdef USE_BARO_SPI_BMP280
    if (busdev->bustype == BUSTYPE_SPI) {
		// SPI引脚预初始化
        spiPreinitByIO(busdev->busdev_u.spi.csnPin);
    }
#else
    UNUSED(busdev);
#endif
}

/**********************************************************************
函数名称：bmp280Detect
函数功能：BMP280检测
函数形参：气压计设备结构体
函数返回值：成功：true,失败：flase 
函数描述：None 
**********************************************************************/
bool bmp280Detect(baroDev_t *baro)
{
    delay(20);
	// 获取气压计总线
    busDevice_t *busdev = &baro->busdev;
    bool defaultAddressApplied = false;
	// BMP280总线初始化
    bmp280BusInit(busdev);
	// IIC形式
    if ((busdev->bustype == BUSTYPE_I2C) && (busdev->busdev_u.i2c.address == 0)) {
        // BMP280的默认地址
        busdev->busdev_u.i2c.address = BMP280_I2C_ADDR;
        defaultAddressApplied = true;
    }
	// 读取芯片ID
    busReadRegisterBuffer(busdev, BMP280_CHIP_ID_REG, &bmp280_chip_id, 1);  
    if (bmp280_chip_id != BMP280_DEFAULT_CHIP_ID) {
        bmp280BusDeinit(busdev);
        if (defaultAddressApplied) {
            busdev->busdev_u.i2c.address = 0;
        }
        return false;
    }
    // 读校准
    busReadRegisterBuffer(busdev, BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG, (uint8_t *)&bmp280_cal, sizeof(bmp280_calib_param_t));
    // 设置过采样+电源模式(强制)，并开始采样
    busWriteRegister(busdev, BMP280_CTRL_MEAS_REG, BMP280_MODE);
    // 当温度作为压力的一部分被测量时，这些都是假的
    baro->combined_read = true;
    baro->ut_delay = 0;
    baro->start_ut = bmp280StartUT;
    baro->get_ut = bmp280GetUT;
    baro->read_ut = bmp280ReadUT;
    // 只执行_up部分，得到温度和压力
    baro->start_up = bmp280StartUP;
    baro->get_up = bmp280GetUP;
    baro->read_up = bmp280ReadUP;
    baro->up_delay = ((T_INIT_MAX + T_MEASURE_PER_OSRS_MAX * (((1 << BMP280_TEMPERATURE_OSR) >> 1) + ((1 << BMP280_PRESSURE_OSR) >> 1)) + (BMP280_PRESSURE_OSR ? T_SETUP_PRESSURE_MAX : 0) + 15) / 16) * 1000;
    baro->calculate = bmp280Calculate;
    return true;
}

/**********************************************************************
函数名称：bmp280StartUT
函数功能：开始UT
函数形参：气压计设备结构体
函数返回值：成功：true,失败：flase 
函数描述：None 
**********************************************************************/
static void bmp280StartUT(baroDev_t *baro)
{
    UNUSED(baro);
}

/**********************************************************************
函数名称：bmp280ReadUT
函数功能：读取UT
函数形参：气压计设备结构体
函数返回值：成功：true,失败：flase 
函数描述：None 
**********************************************************************/
static bool bmp280ReadUT(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

/**********************************************************************
函数名称：bmp280GetUT
函数功能：获取UT
函数形参：气压计设备结构体
函数返回值：成功：true,失败：flase 
函数描述：None 
**********************************************************************/
static bool bmp280GetUT(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

/**********************************************************************
函数名称：bmp280StartUP
函数功能：开始UP
函数形参：气压计设备结构体
函数返回值：成功：true,失败：flase 
函数描述：None 
**********************************************************************/
static void bmp280StartUP(baroDev_t *baro)
{
	// 开始测量
	// 设置采样采样+电源模式(强制)，并开始采样
    busWriteRegisterStart(&baro->busdev, BMP280_CTRL_MEAS_REG, BMP280_MODE);
}

/**********************************************************************
函数名称：bmp280ReadUP
函数功能：获取UP
函数形参：气压计设备结构体
函数返回值：成功：true,失败：flase 
函数描述：None 
**********************************************************************/
static bool bmp280ReadUP(baroDev_t *baro)
{
    if (busBusy(&baro->busdev, NULL)) {
        return false;
    }
    // 读取数据
    busReadRegisterBufferStart(&baro->busdev, BMP280_PRESSURE_MSB_REG, sensor_data, BMP280_DATA_FRAME_SIZE);
    return true;
}

/**********************************************************************
函数名称：bmp280GetUP
函数功能：获取UP
函数形参：气压计设备结构体
函数返回值：成功：true,失败：flase 
函数描述：None 
**********************************************************************/
static bool bmp280GetUP(baroDev_t *baro)
{
    if (busBusy(&baro->busdev, NULL)) {
        return false;
    }
    bmp280_up = (int32_t)(sensor_data[0] << 12 | sensor_data[1] << 4 | sensor_data[2] >> 4);
    bmp280_ut = (int32_t)(sensor_data[3] << 12 | sensor_data[4] << 4 | sensor_data[5] >> 4);
    return true;
}

/**********************************************************************
函数名称：bmp280CompensateTemperature
函数功能：BMP280温度补偿
函数形参：ADC温度
函数返回值：返回温度，分辨率为0.01℃。“5123”的产值等于51.23度
函数描述：
 	t_fine将fine温度作为全局值.
**********************************************************************/
static int32_t bmp280CompensateTemperature(int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_cal.dig_T1 << 1))) * ((int32_t)bmp280_cal.dig_T2)) >> 11;
    var2  = (((((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1))) >> 12) * ((int32_t)bmp280_cal.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

/**********************************************************************
函数名称：bmp280CompensatePressure
函数功能：BMP280压力补偿
函数形参：ADC压力
函数返回值：返回压力Pa为无符号32位整数Q24.8格式(24个整数位和8个小数位)
函数描述：
 	t_fine将fine温度作为全局值。
**********************************************************************/
static uint32_t bmp280CompensatePressure(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_cal.dig_P6;
    var2 = var2 + ((var1*(int64_t)bmp280_cal.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_cal.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_cal.dig_P3) >> 8) + ((var1 * (int64_t)bmp280_cal.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_cal.dig_P1) >> 33;
    if (var1 == 0)
        return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280_cal.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_cal.dig_P7) << 4);
    return (uint32_t)p;
}

/**********************************************************************
函数名称：bmp280Calculate
函数功能：BMP280校准
函数形参：大气压，温度
函数返回值：None
函数描述：None
**********************************************************************/
STATIC_UNIT_TESTED void bmp280Calculate(int32_t *pressure, int32_t *temperature)
{
    int32_t t;
    uint32_t p;
	// 获取BMP280温度补偿
    t = bmp280CompensateTemperature(bmp280_ut);
	// 获取BMP280压力补偿
    p = bmp280CompensatePressure(bmp280_up);
	// 进行气压补偿
    if (pressure)
        *pressure = (int32_t)(p / 256);
	// 进行温度补偿
    if (temperature)
        *temperature = t;
}
#endif

