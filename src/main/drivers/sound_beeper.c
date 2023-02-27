/*********************************************************************************
 提供一系列蜂鸣器配置操作API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/io.h"

#include "pg/beeper_dev.h"

#include "sound_beeper.h"

static IO_t beeperIO = IO_NONE;		        // 蜂鸣器引脚
static bool beeperInverted = false;			// 蜂鸣器引脚开漏输出
static uint16_t beeperFrequency = 0;		// 蜂鸣器频率

/**********************************************************************
函数名称：systemBeep
函数功能：系统蜂鸣器
函数形参：是否开启
函数返回值：None  
函数描述：None 
**********************************************************************/
void systemBeep(bool onoff)
{
    if (beeperFrequency == 0) {
        IOWrite(beeperIO, beeperInverted ? onoff : !onoff);
    }
}

/**********************************************************************
函数名称：beeperInit
函数功能：蜂鸣器初始化
函数形参：是否开启
函数返回值：None  
函数描述：None 
**********************************************************************/
void beeperInit(const beeperDevConfig_t *config)
{
    beeperFrequency = config->frequency;         // 频率
    if (beeperFrequency == 0) {				     // 使用IO驱动
        beeperIO = IOGetByTag(config->ioTag);	 // 获取IO
        beeperInverted = config->isInverted;	 // 获取isInverted
        if (beeperIO) {
			// 初始化蜂鸣器引脚
            IOInit(beeperIO, OWNER_BEEPER, 0);	 
			// 配置GPIO
            IOConfigGPIO(beeperIO, config->isOpenDrain ? IOCFG_OUT_OD : IOCFG_OUT_PP);
        }
		// 系统蜂鸣器关闭
        systemBeep(false);
    }
}

