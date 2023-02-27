/*********************************************************************************
 提供一系列LED配置操作API。
*********************************************************************************/
#include "platform.h"

#include "pg/pg_ids.h"

#include "drivers/io.h"
#include "io_impl.h"

#include "light_led.h"

PG_REGISTER_WITH_RESET_FN(statusLedConfig_t, statusLedConfig, PG_STATUS_LED_CONFIG, 0);
void pgResetFn_statusLedConfig(statusLedConfig_t *statusLedConfig)
{
	// DEFIO_TAG_PB5
    statusLedConfig->ioTags = IO_TAG(LED0_PIN);  
	// 不反转
    statusLedConfig->inversion = 0;
}

static IO_t leds;					  // LED引脚
static uint8_t ledInversion = 0;	  // LED反转

// 是否定义LED0引脚，未定义则置为NONE
#ifndef LED0_PIN
#define LED0_PIN  NONE
#endif

/**********************************************************************
函数名称：ledInit
函数功能：LED初始化
函数形参：statusLedConfig
函数返回值：None
函数描述：None
**********************************************************************/
void ledInit(const statusLedConfig_t *statusLedConfig)
{
    ledInversion = statusLedConfig->inversion;
	// 判断LED引脚是否配置
    if (statusLedConfig->ioTags) { 
        leds = IOGetByTag(statusLedConfig->ioTags);
		// LED引脚初始化
        IOInit(leds, OWNER_LED, RESOURCE_INDEX(0));
        IOConfigGPIO(leds, IOCFG_OUT_PP);
    } else {
        leds = IO_NONE;
    }
    LED0_OFF;
}

/**********************************************************************
函数名称：ledToggle
函数功能：LED反转
函数形参：led
函数返回值：None
函数描述：None
**********************************************************************/
void ledToggle(int led)
{
    IOToggle(leds);
}

/**********************************************************************
函数名称：ledSet
函数功能：LED设置
函数形参：led，on
函数返回值：None
函数描述：None
**********************************************************************/
void ledSet(int led, bool on)
{
    const bool inverted = (1 << (led)) & ledInversion;
    IOWrite(leds, on ? inverted : !inverted);
}

