/*********************************************************************************
 提供一系列系统及系统时钟API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/system.h"
#include "drivers/persistent.h"

// AIRCR_VECTKEY掩码
#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)
void SetSysClock(void);

/**********************************************************************
函数名称：systemReset
函数功能：系统复位
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void systemReset(void)
{
	// 关闭所有中断
    __disable_irq();
	// NVIC系统复位
    NVIC_SystemReset();
}

/**********************************************************************
函数名称：systemReset
函数功能：重置到系统引导加载程序
函数形参：引导加载程序请求类型
函数返回值：None
函数描述：None
**********************************************************************/
void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    switch (requestType) {
	    case BOOTLOADER_REQUEST_ROM:
	    default:
	    	// 写入重启原因到RTC备份寄存器实现持久存储 - RESET_BOOTLOADER_REQUEST_ROM
	        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST_ROM);
	        break;
    }
	// 失能所有中断
    __disable_irq();
	// 系统复位
    NVIC_SystemReset();
}

/**********************************************************************
函数名称：checkForBootLoaderRequest
函数功能：检查引导加载程序请求
函数形参：None
函数返回值：None
函数描述：用于F4的启动文件中，跳转到bootloder
**********************************************************************/
// ISR向量表结构体
typedef void resetHandler_t(void);
typedef struct isrVector_s {
    __I uint32_t    stackEnd;
    resetHandler_t *resetHandler;
} isrVector_t;
void checkForBootLoaderRequest(void)
{
	// 获取重启原因
    uint32_t bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);
	// 判断是否请求了引导加载程序调用
    if (bootloaderRequest != RESET_BOOTLOADER_REQUEST_ROM) {
        return;
    }
	// 重置重启原因
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
	// system_isr_vector_table_base由FLASH链接文件引用
    extern isrVector_t system_isr_vector_table_base;
	// 设置主堆栈指针 - stackEnd
    __set_MSP(system_isr_vector_table_base.stackEnd);
	// 跳转到bootloder
    system_isr_vector_table_base.resetHandler();
    while (1);
}

/**********************************************************************
函数名称：enableGPIOPowerUsageAndNoiseReductions
函数功能：使能系统时钟
函数形参：None
函数返回值：成功true，失败false
函数描述：None
**********************************************************************/
void enableGPIOPowerUsageAndNoiseReductions(void)
{
    RCC_AHB1PeriphClockCmd(
        RCC_AHB1Periph_SRAM1 |
        RCC_AHB1Periph_SRAM2 |
        RCC_AHB1Periph_BKPSRAM |
        RCC_AHB1Periph_DMA1 |
        RCC_AHB1Periph_DMA2 |
        0, ENABLE
    );
    RCC_AHB2PeriphClockCmd(0, ENABLE);
#ifdef STM32F40_41xxx
    RCC_AHB3PeriphClockCmd(0, ENABLE);
#endif
    RCC_APB1PeriphClockCmd(
        RCC_APB1Periph_TIM2 |
        RCC_APB1Periph_TIM3 |
        RCC_APB1Periph_TIM4 |
        RCC_APB1Periph_TIM5 |
        RCC_APB1Periph_TIM6 |
        RCC_APB1Periph_TIM7 |
        RCC_APB1Periph_TIM12 |
        RCC_APB1Periph_TIM13 |
        RCC_APB1Periph_TIM14 |
        RCC_APB1Periph_WWDG |
        RCC_APB1Periph_SPI2 |
        RCC_APB1Periph_SPI3 |
        RCC_APB1Periph_USART2 |
        RCC_APB1Periph_USART3 |
        RCC_APB1Periph_UART4 |
        RCC_APB1Periph_UART5 |
        RCC_APB1Periph_I2C1 |
        RCC_APB1Periph_I2C2 |
        RCC_APB1Periph_I2C3 |
        RCC_APB1Periph_CAN1 |
        RCC_APB1Periph_CAN2 |
        RCC_APB1Periph_PWR |
        RCC_APB1Periph_DAC |
        0, ENABLE);
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_TIM1 |
        RCC_APB2Periph_TIM8 |
        RCC_APB2Periph_USART1 |
        RCC_APB2Periph_USART6 |
        RCC_APB2Periph_ADC |
        RCC_APB2Periph_ADC1 |
        RCC_APB2Periph_ADC2 |
        RCC_APB2Periph_ADC3 |
        RCC_APB2Periph_SDIO |
        RCC_APB2Periph_SPI1 |
        RCC_APB2Periph_SYSCFG |
        RCC_APB2Periph_TIM9 |
        RCC_APB2Periph_TIM10 |
        RCC_APB2Periph_TIM11 |
        0, ENABLE);
}

/**********************************************************************
函数名称：systemInit
函数功能：系统时钟初始化
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void systemInit(void)
{
	// 设置系统时钟源
    SetSysClock();
    // 配置NVIC优先级分组 - 组2（抢占优先级2，响应优先级2）
    NVIC_PriorityGroupConfig(NVIC_PRIORITY_GROUPING);
	// VTOR已经在RAM中加载了向量表
    extern uint8_t isr_vector_table_base;
	// 设置向量表的位置和偏移量
    NVIC_SetVectorTable((uint32_t)&isr_vector_table_base, 0x0);
	// 清除RCC重置标志
    RCC_ClearFlag();
	// 使能系统时钟
    enableGPIOPowerUsageAndNoiseReductions();
    // 初始化循环计数器
    cycleCounterInit();
    // 初始化系统计时器及其中断，并启动系统计时计时器 - 周期：1ms
    SysTick_Config(SystemCoreClock / 1000);
}

