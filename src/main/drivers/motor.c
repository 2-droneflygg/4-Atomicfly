/*********************************************************************************
 提供电机相关操作API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_MOTOR

#include "common/maths.h"

#include "config/feature.h"

#include "drivers/dshot.h"      
#include "drivers/time.h"
#include "drivers/dshot_dpwm.h"

#include "fc/rc_controls.h"    

#include "motor.h"

// ---------------------------------------------------------电机设备信息
static FAST_RAM_ZERO_INIT motorDevice_t *motorDevice;	

// ---------------------------------------------------------PWM输出端口信息
FAST_RAM_ZERO_INIT pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];

// ---------------------------------------------------------电机协议启用状态
static bool motorProtocolEnabled = false;			
// ---------------------------------------------------------电机Dshot协议启用状态 - 现在只有dshot600
static bool motorProtocolDshot 	 = true;			

//-------------------------------------------------------------------------------------获取电机数量相关API

/**********************************************************************
函数名称：motorDeviceCount
函数功能：获取电机设备数量
函数形参：None 
函数返回值：电机设备数量 
函数描述：None 
**********************************************************************/
int motorDeviceCount(void)
{
    return motorDevice->count;
}

//-------------------------------------------------------------------------------------电机控制相关API

/**********************************************************************
函数名称：motorShutdown
函数功能：关闭电机
函数形参：None
函数返回值：None  
函数描述：None 
**********************************************************************/
void motorShutdown(void)
{
	// 停止电机
    motorDevice->vTable.shutdown();
	// 状态失能
    motorDevice->enabled = false;
	// 清空使能时间
    motorDevice->motorEnableTimeMs = 0;
	// 未初始化
    motorDevice->initialized = false;
	// 延时1.5ms
    delayMicroseconds(1500);
}

/**********************************************************************
函数名称：motorWriteAll
函数功能：写入所有电机
函数形参：values
函数返回值：None  
函数描述：None 
**********************************************************************/
void motorWriteAll(float *values)
{
#ifdef USE_PWM_OUTPUT
	// 检索电机是否使能
    if (motorDevice->enabled) {
		// 遍历所有电机
        for (int i = 0; i < motorDevice->count; i++) {
            motorDevice->vTable.write(i, values[i]);
        }
        motorDevice->vTable.updateComplete();
    }
#endif
}

//-------------------------------------------------------------------------------------电机获取协议状态相关API

/**********************************************************************
函数名称：checkMotorProtocolEnabled
函数功能：检查电机协议使能状态
函数形参：checkMotorProtocolEnabled
函数返回值：状态
函数描述：None 
**********************************************************************/
bool checkMotorProtocolEnabled(const motorDevConfig_t *motorDevConfig)
{
    bool enabled = false;
    bool isDshot = false;

	// 判断电机协议
    switch (motorDevConfig->motorPwmProtocol) {
#ifdef USE_DSHOT
	    case PWM_TYPE_DSHOT600:
	        enabled = true;
	        isDshot = true;
	        break;
#endif
	    default:
	        break;
    }
    return enabled;
}

/**********************************************************************
函数名称：checkMotorProtocol
函数功能：检查电机协议
函数形参：motorDevConfig
函数返回值：None 
函数描述：None 
**********************************************************************/
static void checkMotorProtocol(const motorDevConfig_t *motorDevConfig)
{
	// 检查电机协议使能状态
    motorProtocolEnabled = checkMotorProtocolEnabled(motorDevConfig);
}

/**********************************************************************
函数名称：isMotorProtocolEnabled
函数功能：电机协议是否使能
函数形参：None 
函数返回值：motorProtocolEnabled
函数描述：None 
**********************************************************************/
bool isMotorProtocolEnabled(void)
{
    return motorProtocolEnabled;
}

/**********************************************************************
函数名称：isMotorProtocolDshot
函数功能：电机Dshot协议是否使能
函数形参：None 
函数返回值：motorProtocolDshot
函数描述：None 
**********************************************************************/
bool isMotorProtocolDshot(void)
{
    return motorProtocolDshot;
}

//-------------------------------------------------------------------------------------电机结束点初始化相关API

/**********************************************************************
函数名称：motorInitEndpoints
函数功能：初始化电机结束点
函数形参：motorConfig，outputLimit，outputLow，outputHigh，disarm
函数返回值：None 
函数描述：
	在motorDevInit之前从mixerInit调用端点初始化;不能用vtable……
**********************************************************************/
void motorInitEndpoints(const motorConfig_t *motorConfig, float outputLimit, float *outputLow, float *outputHigh, float *disarm)
{
	// 检查电机协议
    checkMotorProtocol(&motorConfig->dev);
	// 检查电机协议是否使能
    if (isMotorProtocolEnabled()) {
#ifdef USE_DSHOT
		// 初始化DSHOT结束点
    	dshotInitEndpoints(motorConfig, outputLimit, outputLow, outputHigh, disarm);
#endif
    }
}

//-------------------------------------------------------------------------------------电机使能相关API

/**********************************************************************
函数名称：motorEnable
函数功能：电机使能
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void motorEnable(void)
{
    if (motorDevice->initialized && motorDevice->vTable.enable()) {
		// 使能
        motorDevice->enabled = true;
		// 记录电机当前使能时间
        motorDevice->motorEnableTimeMs = millis();
    }
}

/**********************************************************************
函数名称：motorIsEnabled
函数功能：获取电机设备使能状态
函数形参：None 
函数返回值：状态
函数描述：None 
**********************************************************************/
bool motorIsEnabled(void)
{
    return motorDevice->enabled;
}

/**********************************************************************
函数名称：motorGetMotorEnableTimeMs
函数功能：获取电机设备使能时间
函数形参：None 
函数返回值：motorGetMotorEnableTimeMs
函数描述：None 
**********************************************************************/
#ifdef USE_DSHOT
timeMs_t motorGetMotorEnableTimeMs(void)
{
    return motorDevice->motorEnableTimeMs;
}
#endif

//-------------------------------------------------------------------------------------电机初始化相关API

// -------------------------------当电机设备无效时使用无效API替代
static bool motorEnableNull(void)	{ return false;}
static void motorDisableNull(void)	{}
static bool motorIsEnabledNull(uint8_t index)	{UNUSED(index);return false;}
bool motorUpdateStartNull(void)	{ return true;}
void motorWriteNull(uint8_t index, float value)	{UNUSED(index);UNUSED(value);}
static void motorWriteIntNull(uint8_t index, uint16_t value)	{UNUSED(index);UNUSED(value);}
void motorUpdateCompleteNull(void)	{}
static void motorShutdownNull(void)	{}
static float motorConvertFromExternalNull(uint16_t value) {UNUSED(value);return 0.0f ;}
static uint16_t motorConvertToExternalNull(float value) {UNUSED(value);return 0;}
// -------------------------------注册电机无效虚函数表（API）
static const motorVTable_t motorNullVTable = {
    .enable = motorEnableNull,
    .disable = motorDisableNull,
    .isMotorEnabled = motorIsEnabledNull,
    .updateStart = motorUpdateStartNull,
    .write = motorWriteNull,
    .writeInt = motorWriteIntNull,
    .updateComplete = motorUpdateCompleteNull,
    .convertExternalToMotor = motorConvertFromExternalNull,
    .convertMotorToExternal = motorConvertToExternalNull,
    .shutdown = motorShutdownNull,
};
// -------------------------------注册电机为无效设备
static motorDevice_t motorNullDevice = {
    .initialized = false,
    .enabled = false,
};

/**********************************************************************
函数名称：motorDevInit
函数功能：电机设备初始化
函数形参：motorDevConfig，idlePulse，motorCount
函数返回值：motorProtocolDshot
函数描述：None 
**********************************************************************/
void motorDevInit(const motorDevConfig_t *motorDevConfig, uint8_t motorCount) {
    memset(motors, 0, sizeof(motors));
	// 检查电机协议是否使能
    if (isMotorProtocolEnabled()) {
		// DSHOT_PWM设备初始化并获取电机设备信息
        motorDevice = dshotPwmDevInit(motorDevConfig, motorCount);
    }
	// 检索电机设备是否可用 - 当电机设备无效时注册无效API
    if (motorDevice) {
        motorDevice->count = motorCount;
        motorDevice->initialized = true;
        motorDevice->motorEnableTimeMs = 0;
        motorDevice->enabled = false;
    } else {
        motorNullDevice.vTable = motorNullVTable;
        motorDevice = &motorNullDevice;
    }
}
#endif // USE_MOTOR

