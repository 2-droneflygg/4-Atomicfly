/*********************************************************************************
 提供一系列Dshot命令相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "common/time.h"

#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/time.h"
#include "drivers/timer.h"

#include "drivers/dshot.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/dshot_command.h"

// ---------------------------------------------------------相关参数宏定义
#define DSHOT_PROTOCOL_DETECTION_DELAY_MS 3000						// Dshot协议检测延迟
#define DSHOT_INITIAL_DELAY_US 			  10000						// Dshot初始化延时
#define DSHOT_COMMAND_DELAY_US 			  1000						// Dshot命令延时
#define DSHOT_BEEP_DELAY_US 			  100000					// Dshot蜂鸣器延时
#define DSHOT_MAX_COMMANDS 				  3							// Dshot最大命令
                                         					
/* --------------------------DSHOT命令状态枚举-------------------------- */	
typedef enum {
    DSHOT_COMMAND_STATE_IDLEWAIT,   								// 等待电机空闲
    DSHOT_COMMAND_STATE_STARTDELAY, 								// 命令序列之前的初始延迟时间
    DSHOT_COMMAND_STATE_ACTIVE,     								// 主动发送命令(带有可选的重复输出)
    DSHOT_COMMAND_STATE_POSTDELAY  	 								// 命令发送后的延迟时间
} dshotCommandState_e;

/* -------------------------DSHOT命令控制结构体------------------------- */	
typedef struct dshotCommandControl_s {
    dshotCommandState_e state;										// Dshot命令状态
    uint32_t nextCommandCycleDelay;									// 下一个命令延时周期	
    timeUs_t delayAfterCommandUs;									// 命令延时
    uint8_t repeats;												// 重复
    uint8_t command[MAX_SUPPORTED_MOTORS];							// 命令
} dshotCommandControl_t;

// ---------------------------------------------------------命令队列
// 队列条目相当大，使用不同的机制处理空/满队列可能更好(头或尾的值/显式元素计数)
// 显式的元素计数将使下面的代码更简单，但必须小心避免竞争条件
static dshotCommandControl_t commandQueue[DSHOT_MAX_COMMANDS + 1];	// 命令队列
static uint8_t commandQueueHead;									// 命令队列头
static uint8_t commandQueueTail;									// 命令队列尾

// ---------------------------------------------------------Dshot命令循环时间
// 默认为8KHz (125us)环路，以防止可能的div/0
// 在PID循环初始化时将其设置为实际值
static timeUs_t dshotCommandPidLoopTimeUs = 125; 	

/**********************************************************************
函数名称：allMotorsAreIdle
函数功能：判断所有电机是否有空闲
函数形参：None
函数返回值：状态
函数描述：None
**********************************************************************/
static bool allMotorsAreIdle(void)
{
	// 遍历所有电机
    for (unsigned i = 0; i < dshotPwmDevice.count; i++) {
		// 获取电机DMA输出信息
        const motorDmaOutput_t *motor = getMotorDmaOutput(i);
		// 判断电机是否空闲
        if (motor->protocolControl.value) {
            return false;
        }
    }
    return true;
}

/**********************************************************************
函数名称：dshotSetPidLoopTime
函数功能：设置Dshot命令PID循环时间
函数形参：pidLoopTime
函数返回值：None
函数描述：None
**********************************************************************/
void dshotSetPidLoopTime(uint32_t pidLoopTime)
{
    dshotCommandPidLoopTimeUs = pidLoopTime;
}

/**********************************************************************
函数名称：dshotCommandCyclesFromTime
函数功能：由delayUs获取Dshot命令周期
函数形参：delayUs
函数返回值：Dshot命令周期
函数描述：None
**********************************************************************/
static uint32_t dshotCommandCyclesFromTime(timeUs_t delayUs)
{
	// 找到所需的最小电机输出周期数，至少提供delayUs时间延迟
	// （1000 + 125 - 1） / 125 = 8.992（8个周期）
    return (delayUs + dshotCommandPidLoopTimeUs - 1) / dshotCommandPidLoopTimeUs;
}

/**********************************************************************
函数名称：addCommand
函数功能：添加命令（获取一个新的队列缓冲区来添加命令）
函数形参：None
函数返回值：Dshot命令控制信息
函数描述：None
**********************************************************************/
static dshotCommandControl_t* addCommand()
{
	// 更新缓冲区头
    int newHead = (commandQueueHead + 1) % (DSHOT_MAX_COMMANDS + 1);
	// 判断合法性
    if (newHead == commandQueueTail) {
        return NULL;
    }
	// 获取命令队列
    dshotCommandControl_t* control = &commandQueue[commandQueueHead];
	// 更新缓冲区头
    commandQueueHead = newHead;
    return control;
}

/**********************************************************************
函数名称：dshotCommandQueueFull
函数功能：获取Dshot命令队列是否为满
函数形参：None
函数返回值：状态
函数描述：None
**********************************************************************/
static bool dshotCommandQueueFull()
{
    return (commandQueueHead + 1) % (DSHOT_MAX_COMMANDS + 1) == commandQueueTail;
}

/**********************************************************************
函数名称：dshotCommandQueueEmpty
函数功能：获取Dshot命令队列是否为空
函数形参：None
函数返回值：状态
函数描述：None
**********************************************************************/
bool dshotCommandQueueEmpty(void)
{
    return commandQueueHead == commandQueueTail;
}

/**********************************************************************
函数名称：isLastDshotCommand
函数功能：获取是否为上一个Dshot命令
函数形参：None
函数返回值：状态
函数描述：None
**********************************************************************/
static bool isLastDshotCommand(void)
{
    return ((commandQueueTail + 1) % (DSHOT_MAX_COMMANDS + 1) == commandQueueHead);
}

/**********************************************************************
函数名称：dshotCommandsAreEnabled
函数功能：获取dshot命令是否使能
函数形参：commandType
函数返回值：状态
函数描述：None
**********************************************************************/
bool dshotCommandsAreEnabled(dshotCommandType_e commandType)
{
    bool ret = false;
	// 阻塞发送
    if (commandType == DSHOT_CMD_TYPE_BLOCKING) {
		// 电机禁用时为true
        ret = !motorIsEnabled();
    } 
	// 内联发送
	else if (commandType == DSHOT_CMD_TYPE_INLINE) {
		// 电机启用 
        if (motorIsEnabled() && motorGetMotorEnableTimeMs() && millis() > motorGetMotorEnableTimeMs() + DSHOT_PROTOCOL_DETECTION_DELAY_MS) {
            ret = true;
        }
    }
    return ret;
}

/**********************************************************************
函数名称：dshotCommandGetCurrent
函数功能：获取当前Dshot命令
函数形参：索引
函数返回值：当前Dshot命令
函数描述：None
**********************************************************************/
uint8_t dshotCommandGetCurrent(uint8_t index)
{
    return commandQueue[commandQueueTail].command[index];
}

/**********************************************************************
函数名称：dshotCommandIsProcessing
函数功能：获取Dshot命令是否正在处理
函数形参：None
函数返回值：状态
函数描述：None
**********************************************************************/
bool dshotCommandIsProcessing(void)
{
	// 获取Dshot命令队列是否为空
    if (dshotCommandQueueEmpty()) {
        return false;
    }
	// 获取命令队列尾
    dshotCommandControl_t* command = &commandQueue[commandQueueTail];
    const bool commandIsProcessing = command->state == DSHOT_COMMAND_STATE_STARTDELAY
                                     || command->state == DSHOT_COMMAND_STATE_ACTIVE
                                     || (command->state == DSHOT_COMMAND_STATE_POSTDELAY && !isLastDshotCommand());
    return commandIsProcessing;
}

/**********************************************************************
函数名称：dshotCommandQueueUpdate
函数功能：Dshot命令队列更新
函数形参：None
函数返回值：true表示队列中还有另一个命令，false表示队列为空
函数描述：None
**********************************************************************/
static bool dshotCommandQueueUpdate(void)
{
	// 获取Dshot命令队列是否为空
    if (!dshotCommandQueueEmpty()) {
		// 计算队列尾
        commandQueueTail = (commandQueueTail + 1) % (DSHOT_MAX_COMMANDS + 1);
		// 获取Dshot命令队列是否为空
        if (!dshotCommandQueueEmpty()) {
			// 队列中还有另一个命令，请更新它，以便可以输出序列
			// 可以直接转到DSHOT_COMMAND_STATE_ACTIVE状态并绕过DSHOT_COMMAND_STATE_IDLEWAIT和DSHOT_COMMAND_STATE_STARTDELAY状态
            dshotCommandControl_t* nextCommand = &commandQueue[commandQueueTail];
            nextCommand->state = DSHOT_COMMAND_STATE_ACTIVE;
            nextCommand->nextCommandCycleDelay = 0;
            return true;
        }
    }
    return false;
}

/**********************************************************************
函数名称：dshotCommandWrite
函数功能：写Dshot命令
函数形参：电机索引，电机数量，命令，命令类型
函数返回值：None
函数描述：None
**********************************************************************/
void dshotCommandWrite(uint8_t index, uint8_t motorCount, uint8_t command, dshotCommandType_e commandType)
{
	// 检查合法性 - Dshot协议未使能 || Dshot命令未使能 || 命令不合法 || Dshot命令队列已满
    if (!isMotorProtocolDshot() || !dshotCommandsAreEnabled(commandType) || (command > DSHOT_MAX_COMMAND) || dshotCommandQueueFull()) {
        return;
    }
    uint8_t repeats = 1;									// 重复值
    timeUs_t delayAfterCommandUs = DSHOT_COMMAND_DELAY_US;	// 命令后延时
    
	// ----------------------------------------判断命令
    switch (command) {
	    case DSHOT_CMD_SPIN_DIRECTION_NORMAL:				// 旋转方向 - 正常
	    case DSHOT_CMD_SPIN_DIRECTION_REVERSED:				// 旋转方向 - 反转（反乌龟）
	        repeats = 10;
	        break;
	    case DSHOT_CMD_BEACON1:								// Dshot信标
	    case DSHOT_CMD_BEACON2:
	    case DSHOT_CMD_BEACON3:
	    case DSHOT_CMD_BEACON4:
	    case DSHOT_CMD_BEACON5:
	        delayAfterCommandUs = DSHOT_BEEP_DELAY_US;
	        break;
	    default:
	        break;
    }
	// ----------------------------------------Dshot命令以阻塞方式发送(电机必须禁用)
    if (commandType == DSHOT_CMD_TYPE_BLOCKING) {
		// Dshot命令延时
        delayMicroseconds(DSHOT_INITIAL_DELAY_US - DSHOT_COMMAND_DELAY_US);
        for (; repeats; repeats--) {
			// Dshot命令延时
            delayMicroseconds(DSHOT_COMMAND_DELAY_US);
			// 遍历所有电机
            for (uint8_t i = 0; i < dshotPwmDevice.count; i++) {
				// 判断命令应用到单个电机还是全部电机
                if ((i == index) || (index == ALL_MOTORS)) {
					// 获取电机DMA输出信息
                    motorDmaOutput_t *const motor = getMotorDmaOutput(i);
					// 使能请求遥测位
                    motor->protocolControl.requestTelemetry = true;
					// 写命令
                    dshotPwmDevice.vTable.writeInt(i, command);
                }
            }
			// Dshot完成电机更新
            dshotPwmDevice.vTable.updateComplete();
        }
		// 命令后延时
        delayMicroseconds(delayAfterCommandUs);
    } 
	// ----------------------------------------Dshot命令与电机信号内联发送(电机必须启用)
	else if (commandType == DSHOT_CMD_TYPE_INLINE) {
		// Dshot命令控制信息 - （获取一个新的队列缓冲区来添加命令）
        dshotCommandControl_t *commandControl = addCommand();
        if (commandControl) {
            commandControl->repeats = repeats;
            commandControl->delayAfterCommandUs = delayAfterCommandUs;
			// 遍历所有电机
            for (unsigned i = 0; i < motorCount; i++) {
				// 判断命令应用到单个电机还是全部电机
                if (index == i || index == ALL_MOTORS) {
                    commandControl->command[i] = command;
                } else {
                    commandControl->command[i] = DSHOT_CMD_MOTOR_STOP;
                }
            }
			// 判断所有电机是否有空闲
            if (allMotorsAreIdle()) {
                // 电机空闲 - 命令序列之前的初始延迟时间
                commandControl->state = DSHOT_COMMAND_STATE_STARTDELAY;
				// 由delayUs获取Dshot命令周期
                commandControl->nextCommandCycleDelay = dshotCommandCyclesFromTime(DSHOT_INITIAL_DELAY_US);
            } else {
				// 等待电机空闲
                commandControl->state = DSHOT_COMMAND_STATE_IDLEWAIT;
                commandControl->nextCommandCycleDelay = 0;  
            }
        }
    }
}

/**********************************************************************
函数名称：dshotCommandOutputIsEnabled
函数功能：是否使能Dshot命令输出
函数形参：电机数量
函数返回值：true表示允许发送电机输出，false表示延迟到下一个循环
函数描述：
   将dshot命令的输出时间同步到正常电机输出定时绑定的PID运行频率上
   所以需要以1ms为间隔重复10次的dshot命令示例
   如果有一个8KHz的PID循环，将结束发送每8个dshot命令电机输出
**********************************************************************/
bool dshotCommandOutputIsEnabled(uint8_t motorCount)
{
    UNUSED(motorCount);

    dshotCommandControl_t* command = &commandQueue[commandQueueTail];
    switch (command->state) {
		// -----------------------------------------等待电机空闲
	    case DSHOT_COMMAND_STATE_IDLEWAIT:
			// 判断所有电机是否有空闲
	        if (allMotorsAreIdle()) {
	            command->state = DSHOT_COMMAND_STATE_STARTDELAY;
	            command->nextCommandCycleDelay = dshotCommandCyclesFromTime(DSHOT_INITIAL_DELAY_US);
	        }
	        break;
		// -----------------------------------------命令序列之前的初始延迟时间
	    case DSHOT_COMMAND_STATE_STARTDELAY:
	        if (command->nextCommandCycleDelay) {
	            --command->nextCommandCycleDelay;
				// 延迟电机输出直到开始命令序列
	            return false;  					 
	        }
	        command->state = DSHOT_COMMAND_STATE_ACTIVE;
			// 第一次迭代的重复现在发生
	        command->nextCommandCycleDelay = 0;  
	        FALLTHROUGH;
		// -----------------------------------------主动发送命令(带有可选的重复输出)
	    case DSHOT_COMMAND_STATE_ACTIVE:
	        if (command->nextCommandCycleDelay) {
	            --command->nextCommandCycleDelay;
				// 延迟电机输出，直到下一个命令重复
	            return false;  					 
	        }

	        command->repeats--;
	        if (command->repeats) {
	            command->nextCommandCycleDelay = dshotCommandCyclesFromTime(DSHOT_COMMAND_DELAY_US);
	        } else {
	            command->state = DSHOT_COMMAND_STATE_POSTDELAY;
	            command->nextCommandCycleDelay = dshotCommandCyclesFromTime(command->delayAfterCommandUs);
				// 获取是否为上一个Dshot命令
	            if (!isLastDshotCommand() && command->nextCommandCycleDelay > 0) {
					// 解释命令之间额外的1个电机输出循环
					// 否则命令间的延迟将是DSHOT_COMMAND_DELAY_US + 1循环
	                command->nextCommandCycleDelay--;
	            }
	        }
	        break;
		// -----------------------------------------命令发送后的延迟时间
	    case DSHOT_COMMAND_STATE_POSTDELAY:
	        if (command->nextCommandCycleDelay) {
	            --command->nextCommandCycleDelay;
				// 延迟电机输出直到命令后延迟结束
	            return false;  					 
	        }
	        if (dshotCommandQueueUpdate()) {
				//如果命令队列不是空的，等待下一个命令依次启动
	            return false;
	        }
    }
    return true;
}
#endif // USE_DSHOT

