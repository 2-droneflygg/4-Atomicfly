/*********************************************************************************
 提供气压计相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_BARO
#include "common/maths.h"
#include "common/filter.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_bmp280.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "sensors/sensors.h"

#include "barometer.h"

// ---------------------------------------------------------气压计访问
baro_t baro;

// ---------------------------------------------------------气压计校准周期
static uint16_t calibratingB   = 0;    

// ---------------------------------------------------------气压值
static int32_t baroPressure    = 0;
// ---------------------------------------------------------温度值
static int32_t baroTemperature = 0;

// ---------------------------------------------------------地面高度
static int32_t baroGroundAltitude = 0;
// ---------------------------------------------------------地面压力
static int32_t baroGroundPressure = 8*101325;
// ---------------------------------------------------------压力和
static uint32_t baroPressureSum   = 0;

// ---------------------------------------------------------气压计就绪状态
static bool baroReady = false;

// ---------------------------------------------------------相关宏定义
// 10秒init_delay + 200 * 25 ms = 15秒后地面压力稳定
#define CALIBRATING_BARO_CYCLES 200 	
// 校准气压至新的地面高度(10 * 25ms = ~ 250ms无阻塞)
#define SET_GROUND_LEVEL_BARO_CYCLES 10  
// 压力采样数量
#define PRESSURE_SAMPLE_COUNT (barometerConfig()->baro_sample_count - 1)

PG_REGISTER_WITH_RESET_FN(barometerConfig_t, barometerConfig, PG_BAROMETER_CONFIG, 1);
void pgResetFn_barometerConfig(barometerConfig_t *barometerConfig)
{
    barometerConfig->baro_sample_count = 21;
    barometerConfig->baro_noise_lpf = 600;
    barometerConfig->baro_cf_vel = 985;
    barometerConfig->baro_hardware = BARO_DEFAULT;

	// 默认使用BMP280 - SPI
#if defined(DEFAULT_BARO_SPI_BMP280) 
#if defined(USE_BARO_BMP280) || defined(USE_BARO_SPI_BMP280)
#if defined(USE_BARO_SPI_BMP280)
#define DEFAULT_BARO_SPI_BMP280
#endif
#endif
#endif
	// 气压计相关配置
#if defined(DEFAULT_BARO_SPI_BMP280) 
    barometerConfig->baro_bustype 	  = BUSTYPE_SPI;
    barometerConfig->baro_spi_device  = SPI_DEV_TO_CFG(spiDeviceByInstance(BARO_SPI_INSTANCE));
    barometerConfig->baro_spi_csn 	  = IO_TAG(BARO_CS_PIN);
    barometerConfig->baro_i2c_device  = I2C_DEV_TO_CFG(I2CINVALID);
    barometerConfig->baro_i2c_address = 0;
#endif
}

//-------------------------------------------------------------------------------------气压计校准相关API

/**********************************************************************
函数名称：baroIsCalibrationComplete
函数功能：获取气压是否校准完成
函数形参：None 
函数返回值：状态
函数描述：None 
**********************************************************************/
bool baroIsCalibrationComplete(void)
{
    return calibratingB == 0;
}

/**********************************************************************
函数名称：baroSetCalibrationCycles
函数功能：设置气压计校准周期
函数形参：calibrationCyclesRequired
函数返回值：None 
函数描述：None 
**********************************************************************/
static void baroSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingB = calibrationCyclesRequired;
}

/**********************************************************************
函数名称：baroStartCalibration
函数功能：气压计开始校准
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void baroStartCalibration(void)
{
    baroSetCalibrationCycles(CALIBRATING_BARO_CYCLES);
}

/**********************************************************************
函数名称：baroSetGroundLevel
函数功能：校准气压至新的地面高度
函数形参：None 
函数返回值：None 
函数描述：
	用于摇杆快捷功能设置。
**********************************************************************/
void baroSetGroundLevel(void)
{
    baroSetCalibrationCycles(SET_GROUND_LEVEL_BARO_CYCLES);
}

/**********************************************************************
函数名称：performBaroCalibrationCycle
函数功能：执行气压校准循环
函数形参：None 
函数返回值：None 
函数描述：
	收敛校准 - 1个标准大气压校准。
**********************************************************************/
void performBaroCalibrationCycle(void)
{
    static int32_t savedGroundPressure = 0;

	// 收敛压力
    baroGroundPressure -= baroGroundPressure / 8;
	// 对压力采样值进行均值累加
    baroGroundPressure += baroPressureSum / PRESSURE_SAMPLE_COUNT;
	// 计算地面高度
    baroGroundAltitude = (1.0f - pow_approx((baroGroundPressure / 8) / 101325.0f, 0.190259f)) * 4433000.0f;
	// 前后两次地面压力做比对
    if (baroGroundPressure == savedGroundPressure) {
		// 如果一致 - 校准完毕
        calibratingB = 0;
    } else {
    	// 校准周期递减
        calibratingB--;
		// 保存压力值
        savedGroundPressure = baroGroundPressure;
    }
}

//-------------------------------------------------------------------------------------气压计检测相关API

/**********************************************************************
函数名称：baroDetect
函数功能：气压计检测
函数形参：dev，baroHardwareToUse
函数返回值：气压计是否存在
函数描述：None 
**********************************************************************/
bool baroDetect(baroDev_t *dev, baroSensor_e baroHardwareToUse)
{
    // ---------------------------检测可用的压力传感器，baro->update()设置为传感器特定的更新功能
    baroSensor_e baroHardware = baroHardwareToUse;
#if !defined(USE_BARO_BMP280) && !defined(USE_BARO_SPI_BMP280)
    UNUSED(dev);
#endif
	// ---------------------------配置总线
    switch (barometerConfig()->baro_bustype) {
#ifdef USE_SPI
	    case BUSTYPE_SPI:
	        {
	            SPI_TypeDef *instance = spiInstanceByDevice(SPI_CFG_TO_DEV(barometerConfig()->baro_spi_device));
	            if (!instance) {
	                return false;
	            }
	            dev->busdev.bustype = BUSTYPE_SPI;
	            spiBusSetInstance(&dev->busdev, instance);
	            dev->busdev.busdev_u.spi.csnPin = IOGetByTag(barometerConfig()->baro_spi_csn);
	        }
	        break;
#endif
	    default:
	        return false;
    }
	// ---------------------------配置硬件
    switch (baroHardware) {
	    case BARO_DEFAULT:
	        FALLTHROUGH;
	    case BARO_BMP280:
#if defined(USE_BARO_BMP280) || defined(USE_BARO_SPI_BMP280)
	        if (bmp280Detect(dev)) {
	            baroHardware = BARO_BMP280;
	            break;
	        }
#endif
	        FALLTHROUGH;
	    case BARO_NONE:
	        baroHardware = BARO_NONE;
	        break;
	   }
    if (baroHardware == BARO_NONE) {
        return false;
    }
	// ---------------------------气压计在线
    detectedSensors[SENSOR_INDEX_BARO] = baroHardware;
	// ---------------------------传感器使能
    sensorsSet(SENSOR_BARO);
    return true;
}

//-------------------------------------------------------------------------------------气压计初始化相关API

/**********************************************************************
函数名称：baroPreInit
函数功能：气压计预初始化
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void baroPreInit(void)
{
#ifdef USE_SPI
    if (barometerConfig()->baro_bustype == BUSTYPE_SPI) {
        spiPreinitRegister(barometerConfig()->baro_spi_csn, IOCFG_IPU, 1);
    }
#endif
}

//-------------------------------------------------------------------------------------气压计滤波相关API

/**********************************************************************
函数名称：applyBarometerMedianFilter
函数功能：气压计应用中值滤波
函数形参：气压数据
函数返回值：窗口为3,  1[未滤波] 2[未滤波] 3[滤波]
函数描述：None 
**********************************************************************/
// 气压样本数量 - 中值滤波窗口为3
#define PRESSURE_SAMPLES_MEDIAN 3		
static int32_t applyBarometerMedianFilter(int32_t newPressureReading)
{
	// 采样缓冲区 - 中值滤波窗口大小
    static int32_t barometerFilterSamples[PRESSURE_SAMPLES_MEDIAN];
	// 当前滤波采样索引
    static int currentFilterSampleIndex = 0;
	// 中值滤波就绪状态
    static bool medianFilterReady = false;
	// 下一个采样索引
    int nextSampleIndex;

	// 计算下一个采样索引
    nextSampleIndex = (currentFilterSampleIndex + 1);
	// 判断是否达到中值滤波窗口大小
    if (nextSampleIndex == PRESSURE_SAMPLES_MEDIAN) {
        nextSampleIndex = 0;
        medianFilterReady = true;
    }
	// 记录气压数据
    barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
    currentFilterSampleIndex = nextSampleIndex;
	
	// 判断中值滤波就绪状态
    if (medianFilterReady)
		// 中值滤波
        return quickMedianFilter3(barometerFilterSamples);
    else
        return newPressureReading;
}

//-------------------------------------------------------------------------------------气压计数据更新相关API

/**********************************************************************
函数名称：isBaroReady
函数功能：气压计是否就绪
函数形参：None 
函数返回值：状态
函数描述：None 
**********************************************************************/
bool isBaroReady(void) {
    return baroReady;
}

/**********************************************************************
函数名称：recalculateBarometerTotal
函数功能：重新计算气压计总压力
函数形参：baroSampleCount，pressureTotal，newPressureReading
函数返回值：pressureTotal
函数描述：None 
**********************************************************************/
static uint32_t recalculateBarometerTotal(uint8_t baroSampleCount, uint32_t pressureTotal, int32_t newPressureReading)
{
    static int32_t barometerSamples[BARO_SAMPLE_COUNT_MAX];
    static int currentSampleIndex = 0;
    int nextSampleIndex;

    // 将当前压力存储在气压计样品中
    nextSampleIndex = (currentSampleIndex + 1);
    if (nextSampleIndex == baroSampleCount) {
        nextSampleIndex = 0;
        baroReady = true;
    }
    barometerSamples[currentSampleIndex] = applyBarometerMedianFilter(newPressureReading);

	// 重新计算总压力
	// 注意，总压力由baroSampleCount - 1个样品组成，参见PRESSURE_SAMPLE_COUNT
    pressureTotal += barometerSamples[currentSampleIndex];
    pressureTotal -= barometerSamples[nextSampleIndex];

    currentSampleIndex = nextSampleIndex;

    return pressureTotal;
}	

/**********************************************************************
函数名称：baroUpdate
函数功能：气压计更新
函数形参：None 
函数返回值：sleepTime
函数描述：None 
**********************************************************************/
uint32_t baroUpdate(void)
{
    static barometerState_e state = BAROMETER_NEEDS_PRESSURE_START;
	// 在状态之间等待1毫秒
    timeUs_t sleepTime = 1000; 		

    switch (state) {
        default:
		// 开始采集温度
        case BAROMETER_NEEDS_TEMPERATURE_START:
            baro.dev.start_ut(&baro.dev);
            state = BAROMETER_NEEDS_TEMPERATURE_READ;
			// 睡眠
            sleepTime = baro.dev.ut_delay;
            break;
		// 读取温度
        case BAROMETER_NEEDS_TEMPERATURE_READ:
            if (baro.dev.read_ut(&baro.dev)) {
                state = BAROMETER_NEEDS_TEMPERATURE_SAMPLE;
            }
        break;
		// 温度采样
        case BAROMETER_NEEDS_TEMPERATURE_SAMPLE:
            if (baro.dev.get_ut(&baro.dev)) {
                state = BAROMETER_NEEDS_PRESSURE_START;
            }
        break;
		// 开始采集压力
        case BAROMETER_NEEDS_PRESSURE_START:
            baro.dev.start_up(&baro.dev);
            state = BAROMETER_NEEDS_PRESSURE_READ;
			// 睡眠
            sleepTime = baro.dev.up_delay;
        break;
		// 读取压力
        case BAROMETER_NEEDS_PRESSURE_READ:
            if (baro.dev.read_up(&baro.dev)) {
                state = BAROMETER_NEEDS_PRESSURE_SAMPLE;
            }
        break;
		// 压力采样
        case BAROMETER_NEEDS_PRESSURE_SAMPLE:
            if (!baro.dev.get_up(&baro.dev)) {
                break;
            }
            baro.dev.calculate(&baroPressure, &baroTemperature);
            baro.baroPressure = baroPressure;
            baro.baroTemperature = baroTemperature;
            baroPressureSum = recalculateBarometerTotal(barometerConfig()->baro_sample_count, baroPressureSum, baroPressure);
            if (baro.dev.combined_read) {
                state = BAROMETER_NEEDS_PRESSURE_START;
            } else {
                state = BAROMETER_NEEDS_TEMPERATURE_START;
            }
			// 睡眠
            sleepTime = baro.dev.ut_delay;
        break;
    }

    return sleepTime;
}

//-------------------------------------------------------------------------------------气压计数据转换相关API

/**********************************************************************
函数名称：pressureToAltitude
函数功能：将压力转换高度
函数形参：压力
函数返回值：高度
函数描述：None 
**********************************************************************/
static float pressureToAltitude(const float pressure)
{
    return (1.0f - powf(pressure / 101325.0f, 0.190295f)) * 4433000.0f;
}

/**********************************************************************
函数名称：baroCalculateAltitude
函数功能：气压计高度计算
函数形参：None 
函数返回值：高度
函数描述：None 
**********************************************************************/
int32_t baroCalculateAltitude(void)
{
    int32_t BaroAlt_tmp;

    // 通过气压数据计算地面高度 - 前提：气压计是否校准完成
    if (baroIsCalibrationComplete()) {
		// 四舍五入
        BaroAlt_tmp = lrintf(pressureToAltitude((float)(baroPressureSum / PRESSURE_SAMPLE_COUNT)));
        BaroAlt_tmp -= baroGroundAltitude;
		// 计算高度附加LPF以降低气压噪声
        baro.BaroAlt = lrintf((float)baro.BaroAlt * CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_noise_lpf) 
        	+ (float)BaroAlt_tmp * (1.0f - CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_noise_lpf))); 
    }
    else {
        baro.BaroAlt = 0;
    }
    return baro.BaroAlt;
}
#endif /* BARO */

