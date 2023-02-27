#pragma once

/* ----------------------------DSHOT命令枚举---------------------------- */	
typedef enum {
    DSHOT_CMD_MOTOR_STOP = 0,					// 电机停止
    DSHOT_CMD_BEACON1,							// DSHOT信标1
    DSHOT_CMD_BEACON2,							// DSHOT信标2
    DSHOT_CMD_BEACON3,							// DSHOT信标3
    DSHOT_CMD_BEACON4,							// DSHOT信标4
    DSHOT_CMD_BEACON5, 							// DSHOT信标5
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,		// 旋转方向 - 正常
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,		// 旋转方向 - 反转
} dshotCommands_e;
#define DSHOT_MAX_COMMAND  47					// DSHOT最大命令

/* --------------------------DSHOT命令类型枚举-------------------------- */	
typedef enum {
    DSHOT_CMD_TYPE_INLINE = 0,    				// Dshot命令与电机信号内联发送(电机必须启用)
    DSHOT_CMD_TYPE_BLOCKING       				// Dshot命令以阻塞方式发送(电机必须禁用)
} dshotCommandType_e;

void dshotCommandWrite(uint8_t index, uint8_t motorCount, uint8_t command, dshotCommandType_e commandType);
void dshotSetPidLoopTime(uint32_t pidLoopTime);
bool dshotCommandQueueEmpty(void);
bool dshotCommandIsProcessing(void);
uint8_t dshotCommandGetCurrent(uint8_t index);
bool dshotCommandOutputIsEnabled(uint8_t motorCount);
bool dshotCommandsAreEnabled(dshotCommandType_e commandType);

