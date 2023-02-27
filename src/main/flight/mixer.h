#pragma once

#include "platform.h"

#include "common/time.h"
#include "pg/pg.h"
#include "drivers/io_types.h"

// ---------------------------------------------------------QUAD电机数量	
#define QUAD_MOTOR_COUNT 4						   

/* --------------------------混控模式枚举-------------------------- */	
typedef enum mixerMode {
    MIXER_QUADX = 3,							   // 默认 - 正X四轴
} mixerMode_e;

/* -------------------------电机混控结构体------------------------- */	
typedef struct motorMixer_s {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;
// 声明电机混控结构体
PG_DECLARE_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer);

/* ---------------------------混控结构体--------------------------- */	
typedef struct mixer_s {
    uint8_t motorCount;							   // 电机数量
    const motorMixer_t *motor;					   // 电机混控结构体
} mixer_t;

/* --------------------------混控配置结构体------------------------ */	
typedef struct mixerConfig_s {
    uint8_t mixerMode;							   // 混控模式
    bool yaw_motors_reversed;					   // 偏航电机反转
    uint8_t crashflip_motor_percent;			   // 反乌龟电机百分比
    uint8_t crashflip_expo;						   // 反乌龟固定曲线值
} mixerConfig_t;
// 声明混控配置结构体
PG_DECLARE(mixerConfig_t, mixerConfig);

extern const mixer_t mixers[];
extern float motor[MAX_SUPPORTED_MOTORS];
extern float motor_disarmed[MAX_SUPPORTED_MOTORS];
extern float motorOutputHigh, motorOutputLow;
struct rxConfig_s;
uint8_t getMotorCount(void);
float getMotorMixRange(void);
bool areMotorsRunning(void);
void initEscEndpoints(void);
void mixerInit(mixerMode_e mixerMode);
void mixerConfigureOutput(void);
void mixerResetDisarmedMotors(void);
void mixTable(timeUs_t currentTimeUs);
void stopMotors(void);
void writeMotors(void);

