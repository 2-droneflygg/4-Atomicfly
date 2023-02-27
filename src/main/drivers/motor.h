#pragma once

#include "common/time.h"

#include "pg/motor.h"

#define ALL_MOTORS 255

#define MOTOR_OUTPUT_LIMIT_PERCENT_MIN 1		// 电机输出限制最小百分比
#define MOTOR_OUTPUT_LIMIT_PERCENT_MAX 100	    // 电机输出限制最大百分比

/* --------------------------电机PWM协议类型枚举-------------------------- */	
typedef enum {
    PWM_TYPE_DSHOT600,								// DSHT600协议
    PWM_TYPE_DISABLED,								// PWM失能
} motorPwmProtocolTypes_e;

/* -----------------------------PWM输出端口结构体------------------------------ */	
typedef struct {
    bool enabled;								// 使能状态
    IO_t io;									// IO引脚
} pwmOutputPort_t;
// 声明PWM输出端口结构体
extern FAST_RAM_ZERO_INIT pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];

/* --------------------------电机虚函数表结构体--------------------------- */	
typedef struct motorVTable_s {
    void (*postInit)(void);
    float (*convertExternalToMotor)(uint16_t externalValue);
    uint16_t (*convertMotorToExternal)(float motorValue);
    bool (*enable)(void);
    void (*disable)(void);
    bool (*isMotorEnabled)(uint8_t index);
    bool (*updateStart)(void);
    void (*write)(uint8_t index, float value);
    void (*writeInt)(uint8_t index, uint16_t value);
    void (*updateComplete)(void);
    void (*shutdown)(void);
} motorVTable_t;

/* --------------------------电机设备信息结构体--------------------------- */	
typedef struct motorDevice_s {
    motorVTable_t vTable;							// 虚函数表		- 驱动API		
    uint8_t       count;							// 电机数量
    bool          initialized;						// 初始化状态
    bool          enabled;							// 使能状态
    timeMs_t      motorEnableTimeMs;				// 电机使能时间 - ms
} motorDevice_t;

void motorWriteNull(uint8_t index, float value);
bool motorUpdateStartNull(void);
void motorUpdateCompleteNull(void);
void motorWriteAll(float *values);
void motorInitEndpoints(const motorConfig_t *motorConfig, float outputLimit, float *outputLow, float *outputHigh, float *disarm);
struct motorDevConfig_s; 
void motorDevInit(const motorDevConfig_t *motorDevConfig, uint8_t motorCount);
int motorDeviceCount(void);
bool checkMotorProtocolEnabled(const motorDevConfig_t *motorDevConfig);
bool isMotorProtocolDshot(void);
bool isMotorProtocolEnabled(void);
void motorEnable(void);
bool motorIsEnabled(void);
timeMs_t motorGetMotorEnableTimeMs(void);
void motorShutdown(void); 

