/**********************************************************************
 验证飞控相关功能预定义正确性 - 某些功能具有连锁性：
**********************************************************************/
#pragma once

#include "build/version.h"

//-----------------------------------------------------------设置晶振频率
#define SYSTEM_HSE_VALUE (HSE_VALUE/1000000U)

//-----------------------------------------------------------设置配置存储位置
// 配置存储在STM32内部FLASH
#ifndef CONFIG_IN_FLASH
#define CONFIG_IN_FLASH         
#endif
// 定义配置配置地址并外部声明 - 地址由FLASH链接文件定义
extern uint8_t __config_start;   
extern uint8_t __config_end;

//-----------------------------------------------------------验证是否使用蜂鸣器[如果未使用则需要禁用蜂鸣器引脚]
#ifndef USE_BEEPER
#undef BEEPER_PIN 
#endif

//-----------------------------------------------------------验证是否定义SPI预初始化引脚数
#ifdef USE_SPI
#ifndef SPI_PREINIT_COUNT
// 2 x 8 (GYROx2, BARO, MAG, MAX, FLASHx2, RX)
#define SPI_PREINIT_COUNT 16            
#endif
#endif

//-----------------------------------------------------------验证是否使用SPI驱动MPU6000[如果使用则必须开启USE_SPI_GYRO]
#if defined(USE_GYRO_SPI_MPU6000)
#define USE_SPI_GYRO
#endif

//-----------------------------------------------------------验证是否使用IIC[未使用则禁用BMP280]
#if !defined(USE_I2C)
#if defined(USE_BARO_BMP280)
#undef USE_BARO_BMP280
#endif
#endif

//-----------------------------------------------------------验证是否使用气压计和GPS[未使用则禁用垂直速度表]
#if !defined(USE_BARO) && !defined(USE_GPS)
#undef USE_VARIO
#endif

//-----------------------------------------------------------验证是否使用串行接收机[未使用则禁用SBUS协议及通道]
#if !defined(USE_SERIAL_RX)
#undef USE_SERIALRX_SBUS
#endif
#if defined(USE_SERIALRX_SBUS) 
#define USE_SBUS_CHANNELS
#endif

//-----------------------------------------------------------验证是否使用OSD[未使用则RX链接质量功能]
#if !defined(USE_OSD)
#undef USE_RX_LINK_QUALITY_INFO
#endif

//-----------------------------------------------------------验证是否使用图传控制[未使用则禁用VTX]
#if !defined(USE_VTX_COMMON) || !defined(USE_VTX_CONTROL)
#undef USE_VTX_COMMON
#undef USE_VTX_CONTROL
#undef USE_VTX_SMARTAUDIO
#endif

//-----------------------------------------------------------验证是否使用救援模式[如果使用则必须开启GPS功能]
#if defined(USE_GPS_RESCUE)
#define USE_GPS
#endif

//-----------------------------------------------------------验证是否使用加速度计[如果未使用则需要禁用GPS救援]
#if !defined(USE_ACC)
#undef USE_GPS_RESCUE
#endif

//-----------------------------------------------------------验证是否使用救援模式[如果未使用则需要禁用失控保护相关菜单]
#ifndef USE_GPS_RESCUE
#undef USE_CMS_GPS_RESCUE_MENU
#endif

//-----------------------------------------------------------验证是否使用失控保护相关菜单
#ifndef USE_CMS
#undef USE_CMS_FAILSAFE_MENU
#endif
#if (!defined(USE_GPS_RESCUE) || !defined(USE_CMS_FAILSAFE_MENU))
#undef USE_CMS_GPS_RESCUE_MENU
#endif

//-----------------------------------------------------------验证是否使用Ddhot电调协议[如果使用则需要启用定时器DMA]
#if defined(USE_DSHOT)  
#define USE_TIMER_DMA
#else
#undef USE_DMA_SPEC
#endif

//-----------------------------------------------------------验证是否使用DMA信息块管理[未使用则需要禁用定时器管理]
#if !defined(USE_DMA_SPEC)
#undef USE_TIMER_MGMT
#endif

//-----------------------------------------------------------验证是否使用定时器管理[未使用则需要禁用定时器]
#if defined(USE_TIMER_MGMT)
#undef USED_TIMERS
#endif

//-----------------------------------------------------------验证是否使用DMA驱动Ddhot[未使用则DSHOT_DMAR_OFF]
#ifndef ENABLE_DSHOT_DMAR
#define ENABLE_DSHOT_DMAR DSHOT_DMAR_OFF
#endif

