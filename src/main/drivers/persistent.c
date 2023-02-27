/**************************************************************************************
*	使用RTC备份数据寄存器实现持久数据存储,保存在软件重置和引导加载程序活动中写入的值
*   备份寄存器共有20个（每个寄存器32位（4个字节））:
*	    X:0~19代表使用具体哪一个备份寄存器.
***************************************************************************************/
#include <stdint.h>
#include "platform.h"

#include "drivers/persistent.h"
#include "drivers/system.h"

// 固定值
#define PERSISTENT_OBJECT_MAGIC_VALUE (('B' << 24)|('e' << 16)|('f' << 8)|('1' << 0))

/**********************************************************************
函数名称：persistentObjectRead
函数功能：从指定的RTC备份数据寄存器读取数据
函数形参：RTC备份数据寄存器号
函数返回值：None
函数描述：None
**********************************************************************/
uint32_t persistentObjectRead(persistentObjectId_e id)
{
    uint32_t value = RTC_ReadBackupRegister(id);
    return value;
}

/**********************************************************************
函数名称：persistentObjectWrite
函数功能：将数据写入指定的RTC备份数据寄存器
函数形参：RTC备份数据寄存器号,写入指定RTC备份数据寄存器的数据
函数返回值：None
函数描述：None
**********************************************************************/
void persistentObjectWrite(persistentObjectId_e id, uint32_t value)
{
    RTC_WriteBackupRegister(id, value);
}

/**********************************************************************
函数名称：persistentObjectRTCEnable
函数功能：配置并启用RTC，使其能够访问备份寄存器
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void persistentObjectRTCEnable(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // 开启对PWR的访问
    PWR_BackupAccessCmd(ENABLE); 					    // 取消备份域保护
    // RTC本身不需要时钟源,跳过它
    RTC_WriteProtectionCmd(ENABLE); 				    // 重置序列
    RTC_WriteProtectionCmd(DISABLE); 				    // 应用序列
}

/**********************************************************************
函数名称：persistentObjectInit
函数功能：初始化RTC备份数据寄存器
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void persistentObjectInit(void)
{
    // 配置并启用RTC，使其能够访问备份寄存器
    persistentObjectRTCEnable();

	// RCC时钟控制和状态寄存器 (RCC_CSR) 
	// SFTRSTF：软件复位标志 - 发生软件复位时，由硬件置 1，通过写入 RMVF 位清零
    uint32_t wasSoftReset;
    wasSoftReset = RCC->CSR & RCC_CSR_SFTRSTF;
	// 未发生软复位 || 固定值不相同
    if (!wasSoftReset || (persistentObjectRead(PERSISTENT_OBJECT_MAGIC) != PERSISTENT_OBJECT_MAGIC_VALUE)) {
		// 遍历清空所有值
        for (int i = 1; i < PERSISTENT_OBJECT_COUNT; i++) {
            persistentObjectWrite(i, 0);
        }
		// 重新写入固定值
        persistentObjectWrite(PERSISTENT_OBJECT_MAGIC, PERSISTENT_OBJECT_MAGIC_VALUE);
    }
}

