/*********************************************************************************
 提供一系列定时器通用操作API。
*********************************************************************************/
#include "platform.h"

#ifdef USE_TIMER

#include "drivers/io.h"
#include "timer.h"

#ifdef USE_TIMER_MGMT
#include "pg/timerio.h"

// ---------------------------------------------------------空闲所有者信息
const resourceOwner_t freeOwner = { .owner = OWNER_FREE, .resourceIndex = 0 };
// ---------------------------------------------------------定时器所有者信息
static resourceOwner_t timerOwners[MAX_TIMER_PINMAP_COUNT];

/**********************************************************************
函数名称：timerIndexByTag
函数功能：通过IO标签获取定时器索引
函数形参：ioTag
函数返回值：索引
函数描述：None
**********************************************************************/
static uint8_t timerIndexByTag(ioTag_t ioTag)
{
	// 遍历所有定时器引脚
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        if (timerIOConfig(i)->ioTag == ioTag) {
            return timerIOConfig(i)->index;
        }
    }
    return 0;
}

/**********************************************************************
函数名称：timerGetByTagAndIndex
函数功能：通过IO标签和定时器索引获取定时器配置信息
函数形参：ioTag，timerIndex
函数返回值：定时器配置信息
函数描述：None
**********************************************************************/
const timerHardware_t *timerGetByTagAndIndex(ioTag_t ioTag, unsigned timerIndex)
{
	// 验证合法性
    if (!ioTag || !timerIndex) {
        return NULL;
    }
	// 定义索引变量
    uint8_t index = 1;
	// 遍历所有定时器通道
    for (unsigned i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        if (TIMER_HARDWARE[i].tag == ioTag) {
            if (index == timerIndex) {
                return &TIMER_HARDWARE[i];
            }
            ++index;
        }
    }
	// 未找到配置
    return NULL;
}

/**********************************************************************
函数名称：timerGetByTag
函数功能：通过IO标签获取定时器配置信息
函数形参：ioTag
函数返回值：定时器配置信息
函数描述：None
**********************************************************************/
const timerHardware_t *timerGetByTag(ioTag_t ioTag)
{
    uint8_t timerIndex = timerIndexByTag(ioTag);
	// 定时器通过索引获取IO标签
    return timerGetByTagAndIndex(ioTag, timerIndex);
}

/**********************************************************************
函数名称：timerGetOwner
函数功能：获取定时器所有者
函数形参：timerNumber，timerChannel
函数返回值：定时器所有者
函数描述：None
**********************************************************************/
const resourceOwner_t *timerGetOwner(int8_t timerNumber, uint16_t timerChannel)
{
	// 初始化定时器所有者
    const resourceOwner_t *timerOwner = &freeOwner;
	// 遍历所有定时器引脚
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
		// 获取定时器硬件信息
        const timerHardware_t *timer = timerGetByTagAndIndex(timerIOConfig(i)->ioTag, timerIOConfig(i)->index);
        if (timer && timerGetTIMNumber(timer->tim) == timerNumber && timer->channel == timerChannel) {
            timerOwner = &timerOwners[i];
            break;
        }
    }
    return timerOwner;
}

/**********************************************************************
函数名称：timerAllocate
函数功能：定时器分配
函数形参：IO标签，所有者，资源索引
函数返回值：定时器硬件配置信息
函数描述：None
**********************************************************************/
const timerHardware_t *timerAllocate(ioTag_t ioTag, resourceOwner_e owner, uint8_t resourceIndex)
{
	// 判断合法性
    if (!ioTag) {
        return NULL;
    }
	// 遍历所有定时器引脚
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
		// 在定时器硬件配置信息中查找该引脚标签
        if (timerIOConfig(i)->ioTag == ioTag) {
			// 通过IO标签和定时器索引获取定时器配置信息
            const timerHardware_t *timer = timerGetByTagAndIndex(ioTag, timerIOConfig(i)->index);
			// 判断所有者合法性
            if (timerGetOwner(timerGetTIMNumber(timer->tim), timer->channel)->owner) {
                return NULL;
            }
			// 注册定时器所有者
            timerOwners[i].owner = owner;
            timerOwners[i].resourceIndex = resourceIndex;
            return timer;
        }
    }
    return NULL;
}
#endif

/**********************************************************************
函数名称：timerioTagGetByUsage
函数功能：通过使用标志获取定时器IO标签
函数形参：usageFlag，index
函数返回值：定时器IO标签
函数描述：None
**********************************************************************/
ioTag_t timerioTagGetByUsage(timerUsageFlag_e usageFlag, uint8_t index)
{
#if USABLE_TIMER_CHANNEL_COUNT > 0
    uint8_t currentIndex = 0;
	// 遍历所有可用的定时器通道
    for (unsigned i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
		// 如果定时器硬件配置标志与该标志相同则返回该定时器硬件配置引脚标签
        if ((timerHardware[i].usageFlags & usageFlag) == usageFlag) {
            if (currentIndex == index) {
                return timerHardware[i].tag;
            }
            currentIndex++;
        }
    }
#else
    UNUSED(usageFlag);
    UNUSED(index);
#endif
    return IO_TAG_NONE;
}
#endif

