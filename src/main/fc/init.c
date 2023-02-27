/**********************************************************************
 系统相关初始化：
**********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/config_eeprom.h"
#include "config/feature.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/adc.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/compass/compass.h"
#include "drivers/dma.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/nvic.h"
#include "drivers/persistent.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"
#include "drivers/motor.h"

#include "config/config.h"
#include "fc/init.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/tasks.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/beeper.h"
#include "io/displayport_max7456.h"
#include "io/gps.h"
#include "io/serial.h"
#include "io/vtx.h"
#include "io/vtx_control.h"
#include "io/vtx_smartaudio.h"

#include "osd/osd.h"

#include "pg/adc.h"
#include "pg/beeper.h"
#include "pg/beeper_dev.h"
#include "pg/bus_i2c.h"
#include "pg/bus_spi.h"
#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/rx.h"
#include "pg/vcd.h"

#include "rx/rx.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/initialisation.h"

// ----------------------------------------------------------------------------------------系统状态：未初始化状态
uint8_t systemState = SYSTEM_STATE_INITIALISING;

/**********************************************************************
函数名称：configureSPIAndQuadSPI
函数功能：配置SPI
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
static void configureSPI(void)
{
#ifdef USE_SPI
    spiPinConfigure(spiPinConfig(0));					// 配置SPI引脚
#endif
    sensorsPreInit();									// 传感器预初始化
#ifdef USE_SPI
    spiPreinit();										// SPI预初始化
#ifdef USE_SPI_DEVICE_1
    spiInit(SPIDEV_1, false);							// SPI设备1初始化
#endif
#ifdef USE_SPI_DEVICE_2
    spiInit(SPIDEV_2, false);							// SPI设备2初始化
#endif
#ifdef USE_SPI_DEVICE_3
    spiInit(SPIDEV_3, false);							// SPI设备3初始化
#endif
#endif // USE_SPI
}

/**********************************************************************
函数名称：swdPinsInit
函数功能：SWD引脚初始化
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
static void swdPinsInit(void)
{
    IO_t io = IOGetByTag(DEFIO_TAG_E(PA13)); 			// SWDIO
    if (IOGetOwner(io) == OWNER_FREE) {		 			// 确保IO所有者为OWNER_SWD
        IOInit(io, OWNER_SWD, 0);
    }
    io = IOGetByTag(DEFIO_TAG_E(PA14));      			// SWCLK
    if (IOGetOwner(io) == OWNER_FREE) {		 	  		// 确保IO所有者为OWNER_SWD
        IOInit(io, OWNER_SWD, 0);
    }
}

/**********************************************************************
函数名称：init
函数功能：初始化
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void init(void)
{
// -----------------------------------------------------------------系统时钟初始化
    systemInit();

// -----------------------------------------------------------------初始化全部IO
// 使用位掩码从ROM中初始化所有ioRec_t结构（IO描述信息块）
    IOInitGlobal();

// -----------------------------------------------------------------存储设备初始化状态枚举
	// SPI初始化标志 
    enum {
        SPI_AND_QSPI_INIT_ATTEMPTED = (1 << 2),      
    };
	// 初始化标志位
    uint8_t initFlags = 0;    							

// -----------------------------------------------------------------初始化EEPROM && 配置加载
	// 初始化EEPROM - 验证编译环境是否正确
    initEEPROM();
	// 确保EEPROM配置结构是有效的
    ensureEEPROMStructureIsValid();
	// 读取EEPROM - 完整性检查
    bool readSuccess = readEEPROM();
	// 如果读取EEPROM配置失败 || EEPROM版本无效 || 飞控板标识错误
    if (!readSuccess || !isEEPROMVersionValid() || strncasecmp(systemConfig()->boardIdentifier, TARGET_BOARD_IDENTIFIER, sizeof(TARGET_BOARD_IDENTIFIER))) {
		// 使用PG参数组默认配置
		resetEEPROM(false);
    }
	
// -----------------------------------------------------------------系统状态：配置加载完成
    systemState |= SYSTEM_STATE_CONFIG_LOADED;

// -----------------------------------------------------------------LED板载状态灯初始化
    ledInit(statusLedConfig());

// -----------------------------------------------------------------EXTI初始化
#ifdef USE_EXTI
	// 启用SYSCFG时钟并进行相关初始化
    EXTIInit();   										
#endif

// -----------------------------------------------------------------设置系统HSE时钟：只有F4有非8mhz的主板
	// 设置系统时钟HSE值 - 8000000
    systemClockSetHSEValue(systemConfig()->hseMhz * 1000000U);

// -----------------------------------------------------------------定时器初始化
#ifdef USE_TIMER
    // 在分配任何通道之前，必须初始化计时器
    // 时钟、通道、优先级
    timerInit();     
#endif

// -----------------------------------------------------------------串口初始化
	// 串口引脚配置
#if defined(USE_UART) 
    uartPinConfigure(serialPinConfig());
#endif
	// 串口初始化
    serialInit(featureIsEnabled(FEATURE_SOFTSERIAL), SERIAL_PORT_NONE);
	// 用于调试的串口初始化 - 软串口2
	//SerialDebug_Init();
    // USB初始化
    USB_VCP_Init();

// -----------------------------------------------------------------混控器初始化
	// 初始化混控模式和电调结束点
    mixerInit(mixerConfig()->mixerMode);

// -----------------------------------------------------------------配置混控器输出
	// 应用混控表和设置锁定的马达值
    mixerConfigureOutput();

// -----------------------------------------------------------------电机设备初始化
#ifdef USE_MOTOR
	// 电机需要尽快初始化，因为硬件初始化可能发送假脉冲到电调，导致电调提前初始化
    motorDevInit(&motorConfig()->dev, getMotorCount());
// ----------------------------------------------------------------------------------------系统状态：电机已就绪状态
    systemState |= SYSTEM_STATE_MOTORS_READY;
#endif

// -----------------------------------------------------------------蜂鸣器初始化
#ifdef USE_BEEPER
    beeperInit(beeperDevConfig());
#endif

// -----------------------------------------------------------------总线初始化
// 配置SPI - 根据编译选项，SPI初始化可能已经完成
    if (!(initFlags & SPI_AND_QSPI_INIT_ATTEMPTED)) {
        configureSPI();
        initFlags |= SPI_AND_QSPI_INIT_ATTEMPTED;
    }

// 配置IIC - 硬件IIC，传输速度快，减少MCU负担
#ifdef USE_I2C
    i2cHardwareConfigure(i2cConfig(0));
    i2cInit(I2CDEV_2);
#endif // USE_I2C

// -----------------------------------------------------------------ADC初始化
#ifdef USE_ADC
    adcInit(adcConfig());
#endif

// -----------------------------------------------------------------传感器初始化 - 自动检测
    if (!sensorsAutodetect()) {
		// 若未检测到陀螺仪，应通知，不要解锁
        indicateFailure(FAILURE_MISSING_ACC, 2);
        setArmingDisabled(ARMING_DISABLED_NO_GYRO);
    }

// ----------------------------------------------------------------------------------------系统状态：传感器已就绪状态
    systemState |= SYSTEM_STATE_SENSORS_READY;

// -----------------------------------------------------------------根据检测到的陀螺仪sampleRateHz和pid_process_denom设置目标循环时间
    gyroSetTargetLooptime(pidConfig()->pid_process_denom);

// -----------------------------------------------------------------验证和修复陀螺仪配置
    validateAndFixGyroConfig();

// -----------------------------------------------------------------配置陀螺仪循环时间
    gyroSetTargetLooptime(pidConfig()->pid_process_denom);

// -----------------------------------------------------------------陀螺仪滤波初始化 
    gyroInitFilters();

// -----------------------------------------------------------------PID初始化 
    pidInit(currentPidProfile);

// -----------------------------------------------------------------系统初始化 - LED && 蜂鸣器通知
	// LED闪烁
    LED0_OFF;
	// 循环
    for (int i = 0; i < 10; i++) {
		// LED
        LED0_TOGGLE;
		// 蜂鸣器
#if defined(USE_BEEPER)
        delay(25);
        //if (!(beeperConfig()->beeper_off_flags & BEEPER_GET_FLAG(BEEPER_SYSTEM_INIT))) {
        if(!(beeperConfig()->beeper_off_flags)) {
            BEEP_ON;
        }
        delay(25);
        BEEP_OFF;
#else
        delay(50);
#endif
    }
    LED0_OFF;

// -----------------------------------------------------------------IMU初始化
    imuInit();

// -----------------------------------------------------------------失控保护初始化
    failsafeInit();

// -----------------------------------------------------------------RX初始化
    rxInit();

// -----------------------------------------------------------------GPS初始化
#ifdef USE_GPS
    if (featureIsEnabled(FEATURE_GPS)) {
        gpsInit();
    }
#endif

// -----------------------------------------------------------------陀螺仪开始校准
    gyroStartCalibration(false);

// -----------------------------------------------------------------气压计开始校准
#ifdef USE_BARO
    baroStartCalibration();
#endif

// -----------------------------------------------------------------图传表初始化
#if defined(USE_VTX_COMMON) || defined(USE_VTX_CONTROL)
    vtxTableInit();
#endif

// -----------------------------------------------------------------图传控制初始化
#ifdef USE_VTX_CONTROL
#ifdef USE_VTX_SMARTAUDIO
    vtxSmartAudioInit();
#endif
#endif // VTX_CONTROL

// -----------------------------------------------------------------电池电压检测初始化
    batteryInit(); 

// -----------------------------------------------------------------CMS初始化
#ifdef USE_CMS
    cmsInit();
#endif

// -----------------------------------------------------------------配置OSD端口（初始化为空）
#if defined(USE_OSD) 
    displayPort_t *osdDisplayPort = NULL;
    osdDisplayPortDevice_e osdDisplayPortDevice = OSD_DISPLAYPORT_DEVICE_NONE;
#endif

// -----------------------------------------------------------------配置OSD并配置显示端口
#if defined(USE_OSD)
    // OSD需要在陀螺之后初始化，以避免陀螺初始化在某些目标上失败
    if (featureIsEnabled(FEATURE_OSD)) {
		// 获取OSD显示端口设备
        osdDisplayPortDevice_e device = osdConfig()->displayPortDevice;
		// OSD显示端口设备配置
        switch(device) {
	        case OSD_DISPLAYPORT_DEVICE_AUTO:
	            FALLTHROUGH;
			// MAX7456设备
#if defined(USE_MAX7456)
	        case OSD_DISPLAYPORT_DEVICE_MAX7456:
				// max7456显示端口初始化 - 视频制式
	            osdDisplayPort = max7456DisplayPortInit(vcdProfile());
				// 注册显示端口设备
	            if (osdDisplayPort || device == OSD_DISPLAYPORT_DEVICE_MAX7456) {
	                osdDisplayPortDevice = OSD_DISPLAYPORT_DEVICE_MAX7456;
	                break;
	            }
	            FALLTHROUGH;
#endif
	        // 其他设备案例可以在这里添加
	        case OSD_DISPLAYPORT_DEVICE_NONE:
	        default:
	            break;
        }

        // OSD初始化
        osdInit(osdDisplayPort, osdDisplayPortDevice);
    }
#endif // USE_OSD

// -----------------------------------------------------------------设置解锁禁用
    setArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);

// -----------------------------------------------------------------使能电机
#ifdef USE_MOTOR
    motorEnable();
#endif

// -----------------------------------------------------------------SWD引脚初始化
    swdPinsInit();

// -----------------------------------------------------------------未使用的引脚初始化
    unusedPinsInit();
	
// -----------------------------------------------------------------任务初始化
    tasksInit();
	
// ----------------------------------------------------------------------------------------系统状态：系统已就绪状态
    systemState |= SYSTEM_STATE_READY;
}

