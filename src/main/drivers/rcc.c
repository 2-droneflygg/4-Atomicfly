/*********************************************************************************
 提供一系列RCC时钟配置API。
*********************************************************************************/
#include "platform.h"
#include "rcc.h"

/**********************************************************************
函数名称：RCC_ClockCmd
函数功能：使能时钟
函数形参：periphTag，使能状态
函数返回值：None
函数描述：None
**********************************************************************/
void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
	// periphTag = RCC_AHB1(GPIOF) -> 0x00000025
	// 转换目标  = RCC_AHB1Periph_GPIOF ((uint32_t)0x00000020)
	// mask = 1 << (0x00000025 & 0x1f);
	//      = 1 << 5 = 0x00000020
    int tag = periphTag >> 5;				  // 右移5位获取端口标签
    uint32_t mask = 1 << (periphTag & 0x1f);  // 获取RCC掩码

    switch (tag) {
	    case RCC_APB2:
	        RCC_APB2PeriphClockCmd(mask, NewState);
	        break;
	    case RCC_APB1:
	        RCC_APB1PeriphClockCmd(mask, NewState);
	        break;
	    case RCC_AHB1:
	        RCC_AHB1PeriphClockCmd(mask, NewState);
	        break;
    }
}

/**********************************************************************
函数名称：RCC_ResetCmd
函数功能：复位时钟
函数形参：periphTag，使能状态
函数返回值：None
函数描述：None
**********************************************************************/
void RCC_ResetCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;				  // 获取标签
    uint32_t mask = 1 << (periphTag & 0x1f);  // 获取RCC掩码
	
    switch (tag) {
	    case RCC_APB2:
	        RCC_APB2PeriphResetCmd(mask, NewState);
	        break;
	    case RCC_APB1:
	        RCC_APB1PeriphResetCmd(mask, NewState);
	        break;
	    case RCC_AHB1:
	        RCC_AHB1PeriphResetCmd(mask, NewState);
	        break;
    }
}

