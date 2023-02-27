/*********************************************************************************
 提供一系列Dshot_PWM相关API（上层）。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "drivers/dshot.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/motor.h"

#include "pg/motor.h"

// 在dshotdmbuffer和dshotburstdmbuffer之间共享一个区域
DSHOT_DMA_BUFFER_ATTRIBUTE DSHOT_DMA_BUFFER_UNIT dshotDmaBuffer[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_ALLOC_SIZE];
#ifdef USE_DSHOT_DMAR
DSHOT_DMA_BUFFER_ATTRIBUTE DSHOT_DMA_BUFFER_UNIT dshotBurstDmaBuffer[MAX_DMA_TIMERS][DSHOT_DMA_BUFFER_SIZE * 4];
#endif
// 是否使用突发DMA驱动Dshot - 在dshotPwmDevInit中进行初始化
#ifdef USE_DSHOT_DMAR
FAST_RAM_ZERO_INIT bool useBurstDshot = false;
#endif

// 加载DMA缓冲区 - loadDmaBufferDshot函数
FAST_RAM_ZERO_INIT loadDmaBufferFn *loadDmaBuffer;

/**********************************************************************
函数名称：loadDmaBufferDshot
函数功能：加载数据包 -> DSHOT_DMA缓存（准备发送数据包）
函数形参：dmaBuffer，步距，数据包
函数返回值：返回DSHOT_DMA缓冲区大小
函数描述：None
**********************************************************************/
uint8_t loadDmaBufferDshot(uint32_t *dmaBuffer, int stride, uint16_t packet)
{
    int i;
	// Dshot600一帧为16bit，遍历16位数据
    for (i = 0; i < 16; i++) {
		// 高位在前 - 1000 0000 0000 0000
        dmaBuffer[i * stride] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;  
        packet <<= 1;
    }
    dmaBuffer[i++ * stride] = 0;
    dmaBuffer[i++ * stride] = 0;
    return DSHOT_DMA_BUFFER_SIZE;
}

/**********************************************************************
函数名称：getDshotHz
函数功能：获取DSHOT频率
函数形参：PWM协议类型
函数返回值：DSHOT协议类型频率
函数描述：None
**********************************************************************/
uint32_t getDshotHz(motorPwmProtocolTypes_e pwmProtocolType)
{
    switch (pwmProtocolType) {
	    case(PWM_TYPE_DSHOT600):
	        return MOTOR_DSHOT600_HZ;
    }
}

/**********************************************************************
函数名称：dshotPwmShutdown
函数功能：DSHOT_PWM关闭
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
static void dshotPwmShutdown(void)
{
	// DShot信号只在写入电机时产生，通过启用写时的检查来阻止
	// 因此这里不需要特殊的处理
    return;
}

/**********************************************************************
函数名称：dshotPwmDisableMotors
函数功能：失能电机DSHOT_PWM
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
static void dshotPwmDisableMotors(void)
{
    // 不需要特殊处理
    return;
}

/**********************************************************************
函数名称：dshotPwmEnableMotors
函数功能：使能电机DSHOT_PWM
函数形参：None
函数返回值：状态
函数描述：None
**********************************************************************/
static bool dshotPwmEnableMotors(void)
{
	// 遍历所有电机设备
    for (int i = 0; i < dshotPwmDevice.count; i++) {
		// 获取电机DMA输出信息
        motorDmaOutput_t *motor = getMotorDmaOutput(i);
		// 获取IO引脚
        const IO_t motorIO = IOGetByTag(motor->timerHardware->tag);
		// 复用IO
        IOConfigGPIOAF(motorIO, motor->iocfg, motor->timerHardware->alternateFunction);
    }
    // 不需要特殊处理
    return true;
}

/**********************************************************************
函数名称：dshotPwmIsMotorEnabled
函数功能：获取电机DSHOT_PWM是否使能
函数形参：index
函数返回值：状态
函数描述：None
**********************************************************************/
static bool dshotPwmIsMotorEnabled(uint8_t index)
{
    return motors[index].enabled;
}

/**********************************************************************
函数名称：dshotWriteInt
函数功能：DSHOT写（Int）
函数形参：index，value
函数返回值：None
函数描述：None
**********************************************************************/
static void dshotWriteInt(uint8_t index, uint16_t value)
{
    pwmWriteDshotInt(index, value);
}

/**********************************************************************
函数名称：dshotWrite
函数功能：DSHOT写（float）
函数形参：index，value
函数返回值：None
函数描述：None
**********************************************************************/
static void dshotWrite(uint8_t index, float value)
{
	// lrintf：四舍五入
    pwmWriteDshotInt(index, lrintf(value));
}

// Dshot_PWM虚函数表注册
static motorVTable_t dshotPwmVTable = {
    .enable = dshotPwmEnableMotors,
    .disable = dshotPwmDisableMotors,
    .isMotorEnabled = dshotPwmIsMotorEnabled,
    .updateStart = motorUpdateStartNull, 
    .write = dshotWrite,
    .writeInt = dshotWriteInt,
    .updateComplete = pwmCompleteDshotMotorUpdate,
    .convertExternalToMotor = dshotConvertFromExternal,
    .convertMotorToExternal = dshotConvertToExternal,
    .shutdown = dshotPwmShutdown,
};

/**********************************************************************
函数名称：dshotPwmDevInit
函数功能：DSHOT_PWM设备初始化
函数形参：电机配置信息，电机数量
函数返回值：电机设备信息
函数描述：None
**********************************************************************/
FAST_RAM_ZERO_INIT motorDevice_t dshotPwmDevice;
motorDevice_t *dshotPwmDevInit(const motorDevConfig_t *motorConfig, uint8_t motorCount)
{
	// 获取虚函数表
    dshotPwmDevice.vTable = dshotPwmVTable;

	// 判断电机PWM协议并加载DMA缓存
    switch (motorConfig->motorPwmProtocol) {
	    case PWM_TYPE_DSHOT600:
	        loadDmaBuffer = loadDmaBufferDshot;
#ifdef USE_DSHOT_DMAR
       		useBurstDshot = motorConfig->useBurstDshot == DSHOT_DMAR_ON || (motorConfig->useBurstDshot == DSHOT_DMAR_AUTO);
#endif
        	break;
    }

	// 遍历所有电机
    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
		// 获取IO引脚标签
        const ioTag_t tag = motorConfig->ioTags[motorIndex];
		// 定时器分配并获取定时器硬件信息
        const timerHardware_t *timerHardware = timerAllocate(tag, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
        if (timerHardware != NULL) {
			// 注册到PWM输出端口信息
            motors[motorIndex].io = IOGetByTag(tag);
			// IO初始化
            IOInit(motors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
			// PWM - Dshot硬件配置
            if (pwmDshotMotorHardwareConfig(timerHardware,motorIndex,motorConfig->motorPwmProtocol,timerHardware->output)) {
				// 状态使能
                motors[motorIndex].enabled = true;
				// 跳过
                continue;
            }
        }
        // 没有足够的电机初始化混控器或电机中断
        dshotPwmDevice.vTable.write = motorWriteNull;
        dshotPwmDevice.vTable.updateComplete = motorUpdateCompleteNull;
        // 待办事项:阻止解锁和添加系统不能解锁原因
        return NULL;
    }
    return &dshotPwmDevice;
}
#endif // USE_DSHOT

