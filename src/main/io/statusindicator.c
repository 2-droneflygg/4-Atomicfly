#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/light_led.h"
#include "drivers/time.h"

#include "statusindicator.h"

static uint32_t warningLedTimer = 0;

/* --------------------------警告LED状态枚举-------------------------- */	
typedef enum {
    WARNING_LED_OFF = 0,
    WARNING_LED_ON,
    WARNING_LED_FLASH
} warningLedState_e;
static warningLedState_e warningLedState = WARNING_LED_OFF;

/**********************************************************************
函数名称：warningLedResetTimer
函数功能：警告LED复位定时器
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void warningLedResetTimer(void) {
    uint32_t now = millis();
    warningLedTimer = now + 500000;
}

/**********************************************************************
函数名称：warningLedEnable
函数功能：警告LED使能
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void warningLedEnable(void)
{
    warningLedState = WARNING_LED_ON;
}

/**********************************************************************
函数名称：warningLedDisable
函数功能：警告LED失能
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void warningLedDisable(void)
{
    warningLedState = WARNING_LED_OFF;
}

/**********************************************************************
函数名称：warningLedDisable
函数功能：警告LED Flash
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void warningLedFlash(void)
{
    warningLedState = WARNING_LED_FLASH;
}

/**********************************************************************
函数名称：warningLedRefresh
函数功能：警告LED刷新
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void warningLedRefresh(void)
{
    switch (warningLedState) {
        case WARNING_LED_OFF:
            LED0_OFF;
            break;
        case WARNING_LED_ON:
            LED0_ON;
            break;
        case WARNING_LED_FLASH:
            LED0_TOGGLE;
            break;
    }
	// 获取当前时间节拍
    uint32_t now = micros();
	// 添加下一个警告LED时间
    warningLedTimer = now + 500000;
}

/**********************************************************************
函数名称：warningLedUpdate
函数功能：警告LED更新
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void warningLedUpdate(void)
{
    uint32_t now = micros();

    if ((int32_t)(now - warningLedTimer) < 0) {
        return;
    }
	// 警告LED刷新
    warningLedRefresh();
}

