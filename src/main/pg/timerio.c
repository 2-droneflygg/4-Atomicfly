#include <string.h>

#include "platform.h"

#ifdef USE_TIMER_MGMT
#include "drivers/dma_reqmap.h"
#include "drivers/timer.h"

#include "timerio.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(timerIOConfig_t, MAX_TIMER_PINMAP_COUNT, timerIOConfig, PG_TIMER_IO_CONFIG, 0);
void pgResetFn_timerIOConfig(timerIOConfig_t *config)
{
#if defined(USE_TIMER_MGMT) 
    unsigned configIndex = 0;
	// 遍历所有可用的定时器通道
    for (unsigned timerIndex = 0; timerIndex < USABLE_TIMER_CHANNEL_COUNT; timerIndex++) {
		// 定时器硬件配置
        const timerHardware_t *configuredTimer = &timerHardware[timerIndex];
        unsigned positionIndex = 1;
		// 遍历全部定时器通道数量
        for (unsigned fullTimerIndex = 0; fullTimerIndex < FULL_TIMER_CHANNEL_COUNT; fullTimerIndex++) {
			// 完整的定时器硬件配置
            const timerHardware_t *timer = &fullTimerHardware[fullTimerIndex];
			// 遍历定时器硬件配置是否在完整的定时器配置中 - 检验正确性
            if (timer->tag == configuredTimer->tag) {
                if (timer->tim == configuredTimer->tim && timer->channel == configuredTimer->channel) {
                    config[configIndex].ioTag = timer->tag;
                    config[configIndex].index = positionIndex;
                    config[configIndex].dmaopt = dmaGetOptionByTimer(configuredTimer);
                    configIndex++;
                    break;
                } else {
                    positionIndex++;
                }
            }
        }
    }
#else
    UNUSED(config);
#endif
}
#endif

