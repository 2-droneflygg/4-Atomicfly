/*********************************************************************************
 提供运行时配置更改相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "fc/runtime_config.h"
#include "io/beeper.h"

// ---------------------------------------------------------解锁标志
uint8_t armingFlags = 0;

// ---------------------------------------------------------状态标志
uint8_t stateFlags = 0;

// ---------------------------------------------------------飞行模式标志
uint16_t flightModeFlags = 0;

// ---------------------------------------------------------初始化过程中通过检测传感器设置
static uint32_t enabledSensors = 0;	

// ---------------------------------------------------------定义解锁禁用标志列出的顺序的严重程度标志
static armingDisableFlags_e armingDisableFlags = 0;  	
// 解锁禁用标志 - 长度不能超过OSD_WARNINGS_MAX_SIZE(11)，在OSD上可以显示全称
const char *armingDisableFlagNames[]= {
    "NOGYRO",
    "FAILSAFE",
    "RXLOSS",
    "BADRX",
    "BOXFAILSAFE",
    "RUNAWAY",
    "CRASH",
    "THROTTLE",
    "ANGLE",
    "BOOTGRACE",
    "LOAD",
    "CALIB",
    "CMS",
    "GPS",
    "RESCUE_SW",
    "REBOOT_REQD",
    "NO_ACC_CAL",
    "MOTOR_PROTO",
    "MSP"
    "ARMSWITCH",
};

//-------------------------------------------------------------------------------------1.解锁状态部分API

/**********************************************************************
函数名称：setArmingDisabled
函数功能：设置解锁禁用
函数形参：解锁禁用标志列出的顺序的严重程度标志
函数返回值：None  
函数描述：None 
**********************************************************************/
void setArmingDisabled(armingDisableFlags_e flag)
{
    armingDisableFlags = armingDisableFlags | flag;
}

/**********************************************************************
函数名称：unsetArmingDisabled
函数功能：重置解锁失能
函数形参：解锁禁用标志列出的顺序的严重程度标志
函数返回值：None  
函数描述：None 
**********************************************************************/
void unsetArmingDisabled(armingDisableFlags_e flag)
{
    armingDisableFlags = armingDisableFlags & ~flag;
}

/**********************************************************************
函数名称：isArmingDisabled
函数功能：获取解锁失能标志位状态
函数形参：None 
函数返回值：armingDisableFlags
函数描述：None 
**********************************************************************/
bool isArmingDisabled(void)
{
    return armingDisableFlags;
}

/**********************************************************************
函数名称：getArmingDisableFlags
函数功能：获取解锁失能标志位状态
函数形参：None 
函数返回值：armingDisableFlags_e armingDisableFlags
函数描述：None 
**********************************************************************/
armingDisableFlags_e getArmingDisableFlags(void)
{
    return armingDisableFlags;
}

//-------------------------------------------------------------------------------------2.飞行模式状态部分API

/**********************************************************************
函数名称：enableFlightMode
函数功能：使能指定的飞行模式
函数形参：飞行模式
函数返回值：新的'flightModeFlags'值
函数描述：
	当进入飞行模式时，提示蜂鸣声。
**********************************************************************/
uint16_t enableFlightMode(flightModeFlags_e mask)
{
    uint16_t oldVal = flightModeFlags;
	// 设置模式掩码
    flightModeFlags |= (mask);
	// 蜂鸣器提示
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

/**********************************************************************
函数名称：disableFlightMode
函数功能：禁用指定的飞行模式
函数形参：飞行模式
函数返回值：新的'flightModeFlags'值
函数描述：
	当进入飞行模式时，提示蜂鸣声。
**********************************************************************/
uint16_t disableFlightMode(flightModeFlags_e mask)
{
    uint16_t oldVal = flightModeFlags;
	// 清除模式掩码
    flightModeFlags &= ~(mask);
	// 蜂鸣器提示
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

//-------------------------------------------------------------------------------------3.传感器状态部分API

/**********************************************************************
函数名称：sensors
函数功能：获取使能传感器
函数形参：传感器掩码
函数返回值：是否开启
函数描述：None
**********************************************************************/
bool sensors(uint32_t mask)
{
    return enabledSensors & mask;
}

/**********************************************************************
函数名称：sensorsSet
函数功能：设置传感器使能
函数形参：传感器掩码
函数返回值：None
函数描述：None
**********************************************************************/
void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}

/**********************************************************************
函数名称：sensorsClear
函数功能：清除传感器使能
函数形参：传感器掩码
函数返回值：None
函数描述：None
**********************************************************************/
void sensorsClear(uint32_t mask)
{
    enabledSensors &= ~(mask);
}

/**********************************************************************
函数名称：sensorsSet
函数功能：使能传感器掩码
函数形参：None
函数返回值：enabledSensors
函数描述：None
**********************************************************************/
uint32_t sensorsMask(void)
{
    return enabledSensors;
}

