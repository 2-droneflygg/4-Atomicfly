#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_MOTOR
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/motor.h"
#include "drivers/motor.h"
#include "drivers/timer.h"

PG_REGISTER_WITH_RESET_FN(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 1);
void pgResetFn_motorConfig(motorConfig_t *motorConfig)
{
	motorConfig->minthrottle = 1070;							// 最小油门
	motorConfig->dev.motorPwmProtocol = PWM_TYPE_DSHOT600;      // 电调协议
    motorConfig->maxthrottle = 2000;							// 最大油门
    motorConfig->digitalIdleOffsetValue = 700;					// DShot协议的空闲值 - 怠速
#ifdef USE_DSHOT_DMAR
    motorConfig->dev.useBurstDshot = ENABLE_DSHOT_DMAR;         // 使用DMA驱动Dshot
#endif
#ifdef USE_TIMER
	// 注册电机IO引脚
    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS; motorIndex++) {
        motorConfig->dev.ioTags[motorIndex] = timerioTagGetByUsage(TIM_USE_MOTOR, motorIndex);
    }
#endif
}
#endif // USE_MOTOR

