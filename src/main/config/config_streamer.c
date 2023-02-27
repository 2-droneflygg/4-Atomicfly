/*********************************************************************************
 提供配置流操作相关API。
*********************************************************************************/
#include <string.h>

#include "platform.h"

#include "drivers/system.h"

#include "config/config_streamer.h"

// 定义F4内部FLASH扇区大小 - 16K扇区
#if !defined(FLASH_PAGE_SIZE)
#define FLASH_PAGE_SIZE                 ((uint32_t)0x4000)   
#endif

/**********************************************************************
函数名称：config_streamer_init
函数功能：配置流初始化
函数形参：config_streamer_t
函数返回值：None
函数描述：
	清空配置流结构体。
**********************************************************************/
void config_streamer_init(config_streamer_t *c)
{
    memset(c, 0, sizeof(*c));
}

/**********************************************************************
函数名称：config_streamer_start
函数功能：配置流开始
函数形参：config_streamer_t，base，size
函数返回值：None
函数描述：None
**********************************************************************/
void config_streamer_start(config_streamer_t *c, uintptr_t base, int size)
{
    // 当使用嵌入式flash时，base必须从FLASH_PAGE_SIZE边界开始
    c->address = base;
    c->size = size;
	// 判断FLASH是否解锁
    if (!c->unlocked) {
		// 解锁FLASH
        FLASH_Unlock();
		// 更新状态
        c->unlocked = true;
    }

#if defined(CONFIG_IN_FLASH)
	// 清除FLASH的挂起标志
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#endif
    c->err = 0;
}

/**********************************************************************
函数名称：getFLASHSectorForEEPROM
函数功能：为EEPROM获取FLASH扇区
函数形参：None
函数返回值：FLASH扇区
函数描述：
	Sector 0    0x08000000 - 0x08003FFF 16 Kbytes
	Sector 1    0x08004000 - 0x08007FFF 16 Kbytes
	Sector 2    0x08008000 - 0x0800BFFF 16 Kbytes
	Sector 3    0x0800C000 - 0x0800FFFF 16 Kbytes
	Sector 4    0x08010000 - 0x0801FFFF 64 Kbytes
	Sector 5    0x08020000 - 0x0803FFFF 128 Kbytes
	Sector 6    0x08040000 - 0x0805FFFF 128 Kbytes
	Sector 7    0x08060000 - 0x0807FFFF 128 Kbytes
	Sector 8    0x08080000 - 0x0809FFFF 128 Kbytes
	Sector 9    0x080A0000 - 0x080BFFFF 128 Kbytes
	Sector 10   0x080C0000 - 0x080DFFFF 128 Kbytes
	Sector 11   0x080E0000 - 0x080FFFFF 128 Kbytes
**********************************************************************/
#if defined(CONFIG_IN_FLASH)
static uint32_t getFLASHSectorForEEPROM(void)
{
    if ((uint32_t)&__config_start <= 0x08003FFF)
        return FLASH_Sector_0;
    if ((uint32_t)&__config_start <= 0x08007FFF)
        return FLASH_Sector_1;
    if ((uint32_t)&__config_start <= 0x0800BFFF)
        return FLASH_Sector_2;
    if ((uint32_t)&__config_start <= 0x0800FFFF)
        return FLASH_Sector_3;
    if ((uint32_t)&__config_start <= 0x0801FFFF)
        return FLASH_Sector_4;
    if ((uint32_t)&__config_start <= 0x0803FFFF)
        return FLASH_Sector_5;
    if ((uint32_t)&__config_start <= 0x0805FFFF)
        return FLASH_Sector_6;
    if ((uint32_t)&__config_start <= 0x0807FFFF)
        return FLASH_Sector_7;
    if ((uint32_t)&__config_start <= 0x0809FFFF)
        return FLASH_Sector_8;
    if ((uint32_t)&__config_start <= 0x080DFFFF)
        return FLASH_Sector_9;
    if ((uint32_t)&__config_start <= 0x080BFFFF)
        return FLASH_Sector_10;
    if ((uint32_t)&__config_start <= 0x080FFFFF)
        return FLASH_Sector_11;

    while (1) {
		// 失败 - 蜂鸣器通知并重置到引导加载程序
        failureMode(FAILURE_CONFIG_STORE_FAILURE);
    }
}
#endif // CONFIG_IN_FLASH

/**********************************************************************
函数名称：write_word
函数功能：字写入
函数形参：c，buffer
函数返回值：0
函数描述：None
**********************************************************************/
static int write_word(config_streamer_t *c, config_streamer_buffer_align_type_t *buffer)
{
    if (c->err != 0) {
        return c->err;
    }
#if defined(CONFIG_IN_FLASH)
    if (c->address % FLASH_PAGE_SIZE == 0) {
		// 擦除指定的FLASH扇区 - 返回擦除状态
        const FLASH_Status status = FLASH_EraseSector(getFLASHSectorForEEPROM(), VoltageRange_3); //0x08080000 to 0x080A0000
        if (status != FLASH_COMPLETE) {
            return -1;
        }
    }
	// 在指定的地址上程序一个字(32位)。
    const FLASH_Status status = FLASH_ProgramWord(c->address, *buffer);
    if (status != FLASH_COMPLETE) {
        return -2;
    }
#endif
	// 地址自增
    c->address += CONFIG_STREAMER_BUFFER_SIZE;
    return 0;
}

/**********************************************************************
函数名称：config_streamer_write
函数功能：配置流写入
函数形参：config_streamer_t，p，size
函数返回值：c->err
函数描述：None
**********************************************************************/
int config_streamer_write(config_streamer_t *c, const uint8_t *p, uint32_t size)
{
    for (const uint8_t *pat = p; pat != (uint8_t*)p + size; pat++) {
        c->buffer.b[c->at++] = *pat;

        if (c->at == sizeof(c->buffer)) {
			// 字写入
            c->err = write_word(c, &c->buffer.w);
            c->at = 0;
        }
    }
    return c->err;
}

/**********************************************************************
函数名称：config_streamer_flush
函数功能：配置流刷新
函数形参：config_streamer_t
函数返回值：c->err
函数描述：None
**********************************************************************/
int config_streamer_flush(config_streamer_t *c)
{
    if (c->at != 0) {
        memset(c->buffer.b + c->at, 0, sizeof(c->buffer) - c->at);
        c->err = write_word(c, &c->buffer.w);
        c->at = 0;
    }
    return c-> err;
}

/**********************************************************************
函数名称：config_streamer_finish
函数功能：配置流完成
函数形参：config_streamer_t
函数返回值：c->err
函数描述：None
**********************************************************************/
int config_streamer_finish(config_streamer_t *c)
{
    if (c->unlocked) {
#if defined(CONFIG_IN_FLASH)
		// FLASH锁定
        FLASH_Lock();
#endif
		// 更新状态
        c->unlocked = false;
    }
    return c->err;
}

