#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DMA

#include "drivers/nvic.h"
#include "dma.h"
#include "resource.h"

// 1.DMA所有数据流描述块参数设置
static dmaChannelDescriptor_t dmaDescriptors[DMA_LAST_HANDLER] = {
	DEFINE_DMA_CHANNEL(DMA1, 0,  0),
	DEFINE_DMA_CHANNEL(DMA1, 1,  6),
	DEFINE_DMA_CHANNEL(DMA1, 2, 16),
	DEFINE_DMA_CHANNEL(DMA1, 3, 22),
	DEFINE_DMA_CHANNEL(DMA1, 4, 32),
	DEFINE_DMA_CHANNEL(DMA1, 5, 38),
	DEFINE_DMA_CHANNEL(DMA1, 6, 48),
	DEFINE_DMA_CHANNEL(DMA1, 7, 54),

	DEFINE_DMA_CHANNEL(DMA2, 0,  0),
	DEFINE_DMA_CHANNEL(DMA2, 1,  6),
	DEFINE_DMA_CHANNEL(DMA2, 2, 16),
	DEFINE_DMA_CHANNEL(DMA2, 3, 22),
	DEFINE_DMA_CHANNEL(DMA2, 4, 32),
	DEFINE_DMA_CHANNEL(DMA2, 5, 38),
	DEFINE_DMA_CHANNEL(DMA2, 6, 48),
	DEFINE_DMA_CHANNEL(DMA2, 7, 54),
};

// 2.注册中断服务函数（回调） - DMA ## d ## _Stream ## s ## _IRQHandler
DEFINE_DMA_IRQ_HANDLER(1, 0, DMA1_ST0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 1, DMA1_ST1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 2, DMA1_ST2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 3, DMA1_ST3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 4, DMA1_ST4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 5, DMA1_ST5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 6, DMA1_ST6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 7, DMA1_ST7_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 0, DMA2_ST0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 1, DMA2_ST1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 2, DMA2_ST2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 3, DMA2_ST3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 4, DMA2_ST4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 5, DMA2_ST5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 6, DMA2_ST6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 7, DMA2_ST7_HANDLER)


/**********************************************************************
函数名称：dmaFlag_IT_TCIF
函数功能：获取TCIF_FLAG
函数形参：stream
函数返回值：DMA_IT_TCIFx
函数描述：None
**********************************************************************/
#define RETURN_TCIF_FLAG(s, n) if (s == DMA1_Stream ## n || s == DMA2_Stream ## n) return DMA_IT_TCIF ## n
uint32_t dmaFlag_IT_TCIF(const dmaResource_t *stream)
{
	RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 0);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 1);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 2);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 3);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 4);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 5);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 6);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 7);
    return 0;
}

/**********************************************************************
函数名称：dmaInit
函数功能：DMA设备初始化
函数形参：DMA_HANDLER标识符，所有者，资源索引
函数返回值：None
函数描述：None
**********************************************************************/
// DMA时钟选择宏
#define DMA_RCC(x) ((x) == DMA1 ? RCC_AHB1Periph_DMA1 : RCC_AHB1Periph_DMA2)  
void dmaInit(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex)
{
	// 获取索引
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
	// 使能DMA控制器时钟
    RCC_AHB1PeriphClockCmd(DMA_RCC(dmaDescriptors[index].dma), ENABLE);
	// 注册DMA描述块所有者
    dmaDescriptors[index].owner.owner = owner;
	// 注册DMA描述块资源索引
    dmaDescriptors[index].owner.resourceIndex = resourceIndex;
}

/**********************************************************************
函数名称：dmaSetHandler
函数功能：DMA设置中断服务函数
函数形参：DMA_HANDLER标识符，中断服务回调函数，优先级，使用者参数
函数返回值：None
函数描述：None
**********************************************************************/
void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
    NVIC_InitTypeDef NVIC_InitStructure;
	// 获取索引
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
	// 使能DMA控制器时钟
    RCC_AHB1PeriphClockCmd(DMA_RCC(dmaDescriptors[index].dma), ENABLE);
	// 注册DMA描述块中断服务回调函数
    dmaDescriptors[index].irqHandlerCallback = callback;
	// 注册DMA描述块使用者参数
    dmaDescriptors[index].userParam = userParam;
	// 注册DMA描述块传输完成标志位
    dmaDescriptors[index].completeFlag = dmaFlag_IT_TCIF(dmaDescriptors[index].ref);
	// 初始化NVIC控制器
    NVIC_InitStructure.NVIC_IRQChannel = dmaDescriptors[index].irqN;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(priority);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(priority);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**********************************************************************
函数名称：dmaGetIdentifier
函数功能：通过DMA描述块注册的DMA控制器_数据流获取DMA_HANDLER标识符
函数形参：dmaDescriptors[x].ref
函数返回值：DMA标识符
函数描述：None
**********************************************************************/
dmaIdentifier_e dmaGetIdentifier(const dmaResource_t* instance)
{
	// 遍历所有DMA_HANDLER
    for (int i = 0; i < DMA_LAST_HANDLER; i++) {
        if (dmaDescriptors[i].ref == instance) {
            return i + 1;
        }
    }
    return 0;
}
#endif

