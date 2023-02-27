/*********************************************************************************
 SysTick定时器:
 	1.SysTick定时器也称为系统滴答定时器或系统定时器。
 	2.SysTick定时器是一个24位（计算范围0～224-1）倒计数（计数方式为减法）定时器，
 	  从预装载值（计数周期）一直计数到0，然后再从重装载寄存器中自动重装载初始值，
 	  只要不把SysTick定时器的使能位清除，那么SysTick定时器就永远不停。
 	3.为系统提供时基单元/时间节拍。
 该文件提供一系列系统时间节拍、延时函数、失败模式蜂鸣器通知、未使用引脚操作API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "platform.h"

#include "build/atomic.h"

#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/nvic.h"
#include "drivers/resource.h"
#include "drivers/sound_beeper.h"

#include "io/serial.h"

#include "system.h"

// ---------------------------------------------------------微秒循环计数
static uint32_t usTicks = 0;
// ---------------------------------------------------------当前运行时间为1kHz systick定时器
static volatile uint32_t sysTickUptime = 0;		// 毫秒计数
static volatile uint32_t sysTickValStamp = 0;	// 系统定时器计数值标志
// ---------------------------------------------------------缓存的RCC值->CSR
uint32_t cachedRccCsrValue;
// ---------------------------------------------------------cpu时钟频率
static uint32_t cpuClockFrequency = 0;
// ---------------------------------------------------------SysTick等待
static volatile int sysTickPending = 0;

/**********************************************************************
函数名称：SysTick_Handler
函数功能：系统定时器中断服务函数 - 1ms更新一次
函数形参：None
函数返回值：None
函数描述：
	提供系统时间节拍。
**********************************************************************/
void SysTick_Handler(void)
{
	// 临界保护 - 不允许被打断
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        sysTickUptime++;   					// 毫秒计数
        sysTickValStamp = SysTick->VAL;		// 读取当前计数值
        sysTickPending = 0;					// 清除挂起
        (void)(SysTick->CTRL);
    }
}

/**********************************************************************
函数名称：cycleCounterInit
函数功能：循环计数器初始化
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
	// 获取芯片上时钟的频率SYSCLK, HCLK, PCLK1 ,PCLK2.
    RCC_GetClocksFreq(&clocks);
	// 配置系统时钟频率 - 168000000
    cpuClockFrequency = clocks.SYSCLK_Frequency;
	// 配置微秒循环计数值 - 168 - 1us
    usTicks = cpuClockFrequency / 1000000;	
}

/**********************************************************************
函数名称：microsISR
函数功能：ISR处理
函数形参：None
函数返回值：当前时间节拍(us)
函数描述：None
**********************************************************************/
uint32_t microsISR(void)
{
    register uint32_t ms, pending, cycle_cnt;
	// 临界保护
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        cycle_cnt = SysTick->VAL;	   // 读取计数值
		// 判断是否有计数至0事件
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) { 	
            sysTickPending = 1;		   // 更新挂起
            cycle_cnt = SysTick->VAL;  // 再次读取计数值以确保在翻转后读取该值
        }
        ms = sysTickUptime;			
        pending = sysTickPending;
    }
    return ((ms + pending) * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

/**********************************************************************
函数名称：micros
函数功能：获取当前时间节拍
函数形参：None
函数返回值：当前时间节拍(us)
函数描述：None
**********************************************************************/
uint32_t micros(void)
{
	// 一般情况下，变量的值是存储在内存中的，MCU 每次使用数据都要从内存中读取。
	// 如果有一些变量使用非常频繁，从内存中读取就会消耗很多时间。
	// 为了解决这个问题，可以将使用频繁的变量放在MCU的通用寄存器中，
	// 这样使用该变量时就不必访问内存，直接从寄存器中读取，大大提高程序的运行效率。
	// register关键字请求“编译器”将变量放到寄存器里。
	// 寄存器的数量是有限的，通常是把使用最频繁的变量定义为register 
	// 信息交换： MCU <-- 寄存器 <-- 内存
    register uint32_t ms, cycle_cnt;

    // 在中断(中断控制状态寄存器)和提升(非零)BASEPRI上下文中调用microsISR()
    if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) || (__get_BASEPRI())) {
        return microsISR();
    }

    do {
        ms = sysTickUptime;			   // 获取当前ms计数
        cycle_cnt = SysTick->VAL;      // 读取当前计数值
    } while (ms != sysTickUptime || cycle_cnt > sysTickValStamp);

	// 						us        +	    (168000 - VAL) / 168 
	// 微秒计数 = （毫秒计数 * 1000） + (系统时钟频率 / 1000 - systick计数值) / (系统时钟频率 / 1000000)
	//				（usTicks = 168000000 / 1000000）（systick计数值：倒数168000下为1ms，自动重装初值COUNTFLAG置位并触发中断）
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

/**********************************************************************
函数名称：millis
函数功能：返回系统正常运行时间
函数形参：None
函数返回值：系统正常运行时间(以毫秒为单位)
函数描述：None
**********************************************************************/
uint32_t millis(void)
{
    return sysTickUptime;
}

/**********************************************************************
函数名称：delayMicroseconds
函数功能：us延时
函数形参：us
函数返回值：None
函数描述：None
**********************************************************************/
void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us);
}

/**********************************************************************
函数名称：delay
函数功能：ms延时
函数形参：ms
函数返回值：None
函数描述：None
**********************************************************************/
void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}

/**********************************************************************
函数名称：indicate
函数功能：蜂鸣器通知
函数形参：count，duration
函数返回值：None
函数描述：None
**********************************************************************/
static void indicate(uint8_t count, uint16_t duration)
{
    if (count) {
        LED0_OFF;
        while (count--) {
            LED0_TOGGLE;
            BEEP_ON;
            delay(duration);

            LED0_TOGGLE;
            BEEP_OFF;
            delay(duration);
        }
    }
}

/**********************************************************************
函数名称：indicateFailure
函数功能：通过蜂鸣器通知失败
函数形参：mode，codeRepeatsRemaining
函数返回值：None
函数描述：None
**********************************************************************/
void indicateFailure(failureMode_e mode, int codeRepeatsRemaining)
{
    while (codeRepeatsRemaining--) {
        indicate(WARNING_FLASH_COUNT, WARNING_FLASH_DURATION_MS);
        delay(WARNING_PAUSE_DURATION_MS);
        indicate(mode + 1, WARNING_CODE_DURATION_LONG_MS);
        delay(1000);
    }
}

/**********************************************************************
函数名称：failureMode
函数功能：失败模式
函数形参：mode
函数返回值：None
函数描述：None
**********************************************************************/
void failureMode(failureMode_e mode)
{
	// 通过蜂鸣器通知失败
    indicateFailure(mode, 10);
	// 重置到系统引导加载程序
    systemResetToBootloader(BOOTLOADER_REQUEST_ROM);
}

/**********************************************************************
函数名称：initialiseMemorySections
函数功能：初始化内存部分
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void initialiseMemorySections(void)
{
#ifdef USE_FAST_RAM
    // 将FAST_RAM变量初始化到DTCM RAM中
    extern uint8_t _sfastram_data;
    extern uint8_t _efastram_data;
    extern uint8_t _sfastram_idata;
    memcpy(&_sfastram_data, &_sfastram_idata, (size_t) (&_efastram_data - &_sfastram_data));
#endif
}

/**********************************************************************
函数名称：unusedPinInit
函数功能：未使用的引脚初始化
函数形参：IO引脚
函数返回值：None
函数描述：None
**********************************************************************/
static void unusedPinInit(IO_t io)
{
	// 如果引脚没有所有者
    if (IOGetOwner(io) == OWNER_FREE) {
		// 配置为上拉输入
        IOConfigGPIO(io, IOCFG_IPU);
    }
}

/**********************************************************************
函数名称：unusedPinsInit
函数功能：未使用的引脚（遍历引脚）初始化
函数形参：None
函数返回值：None
函数描述：
	通过回调unusedPinInit进行遍历未使用的引脚并初始化.
**********************************************************************/
void unusedPinsInit(void)
{
    IOTraversePins(unusedPinInit);
}

