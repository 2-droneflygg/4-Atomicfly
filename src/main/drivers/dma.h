/*********************************************************************************
 DMA简介：
	1.两个DMA控制器总共有16个数据流（每个控制器8个）；
	2.每个数据流总共可以有多达8个通道（或称请求）；
	3.每个通道都有一个仲裁器，用于处理 DMA 请求间的优先级；
	4.每一个DMA控制器都用于管理一个或多个外设的存储器访问请求。
 DMA中断：
	对于每个DMA数据流，可在发生以下事件时产生中断：
											事件标志       使能控制位
	1.达到半传输；								  HTIF        HTIE
	2.传输完成；									  TCIF		  TCIE
	3.传输错误；									  TEIF		  TEIE
	4.FIFO错误（上溢、下溢或FIFO级别错误）；           	  FEIF		  FEIE
	5.直接模式错误							      DMEIF		  DMEIE
*********************************************************************************/
#pragma once

#include "drivers/resource.h"

typedef struct dmaResource_s dmaResource_t;          // 不透明的数据类型，表示一个单一的DMA引擎(DMA控制器_数据流)
#define DMA_ARCH_TYPE DMA_Stream_TypeDef			 // DMA数据流类型宏定义

struct dmaChannelDescriptor_s;
// DMA中断服务函数（回调）
typedef void (*dmaCallbackHandlerFuncPtr)(struct dmaChannelDescriptor_s *channelDescriptor);

/* ----------------------------DMA描述块结构体---------------------------- */	
typedef struct dmaChannelDescriptor_s {
    DMA_TypeDef*                dma;				 // DMA控制器
    uint8_t                     stream;				 // 数据流
    dmaResource_t               *ref;				 // DMA控制器_数据流
    dmaCallbackHandlerFuncPtr   irqHandlerCallback;  // 中断服务函数回调函数
    uint8_t                     flagsShift;			 // DMA标志位
    IRQn_Type                   irqN;				 // 中断类型
    uint32_t                    userParam;			 // 使用者参数
    resourceOwner_t             owner;				 // 所有者
    uint8_t                     resourceIndex;		 // 资源索引
    uint32_t                    completeFlag;		 // 传输完成标志位
} dmaChannelDescriptor_t;
// DMA描述块参数设置宏
#define DEFINE_DMA_CHANNEL(d, s, f) { \
    .dma = d, \
    .stream = s, \
    .ref = (dmaResource_t *)d ## _Stream ## s, \
    .irqHandlerCallback = NULL, \
    .flagsShift = f, \
    .irqN = d ## _Stream ## s ## _IRQn, \
    .userParam = 0, \
    .owner.owner = 0, \
    .owner.resourceIndex = 0 \
    }
/* -------------------------DMA_HANDLER标识符枚举------------------------- */	
typedef enum {
    DMA_NONE = 0,
    DMA1_ST0_HANDLER = 1,
    DMA1_ST1_HANDLER,
    DMA1_ST2_HANDLER,
    DMA1_ST3_HANDLER,
    DMA1_ST4_HANDLER,
    DMA1_ST5_HANDLER,
    DMA1_ST6_HANDLER,
    DMA1_ST7_HANDLER,
    DMA2_ST0_HANDLER,
    DMA2_ST1_HANDLER,
    DMA2_ST2_HANDLER,
    DMA2_ST3_HANDLER,
    DMA2_ST4_HANDLER,
    DMA2_ST5_HANDLER,
    DMA2_ST6_HANDLER,
    DMA2_ST7_HANDLER,
    DMA_LAST_HANDLER = DMA2_ST7_HANDLER
} dmaIdentifier_e;
// DMA_HANDLER标识符转索引宏(减一)
#define DMA_IDENTIFIER_TO_INDEX(x) ((x) - 1)

// DMA中断服务函数定义宏 - 注册中断服务函数（回调）
#define DEFINE_DMA_IRQ_HANDLER(d, s, i) void DMA ## d ## _Stream ## s ## _IRQHandler(void) {\
                                                                const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i); \
                                                                dmaCallbackHandlerFuncPtr handler = dmaDescriptors[index].irqHandlerCallback; \
                                                                if (handler) \
                                                                    handler(&dmaDescriptors[index]); \
                                                            }

// DMA清除相关标志宏
#define DMA_CLEAR_FLAG(d, flag)   if (d->flagsShift > 31) d->dma->HIFCR = (flag << (d->flagsShift - 32)); else d->dma->LIFCR = (flag << d->flagsShift)
// DMA获取标志状态宏
#define DMA_GET_FLAG_STATUS(d, flag) (d->flagsShift > 31 ? d->dma->HISR & (flag << (d->flagsShift - 32)): d->dma->LISR & (flag << d->flagsShift))
#define DMA_IT_TCIF         ((uint32_t)0x00000020)			// 传输完成
#define DMA_IT_HTIF         ((uint32_t)0x00000010)			// 半传输
#define DMA_IT_TEIF         ((uint32_t)0x00000008)			// 传输错误
#define DMA_IT_DMEIF        ((uint32_t)0x00000004)			// 直接模式错误
#define DMA_IT_FEIF         ((uint32_t)0x00000001)			// FIFO溢出

// DMA数据流使能/读作低电平时数据流就绪标志（避免直接的寄存器和寄存器位访问）
#define IS_DMA_ENABLED(reg) (((DMA_ARCH_TYPE *)(reg))->CR & DMA_SxCR_EN)

// DMA相关库函数封装宏
#define xDMA_Init(dmaResource, initStruct)   					DMA_Init((DMA_ARCH_TYPE *)(dmaResource), initStruct)                    // DMA初始化
#define xDMA_DeInit(dmaResource)			 					DMA_DeInit((DMA_ARCH_TYPE *)(dmaResource))								// DMA重置默认值
#define xDMA_Cmd(dmaResource, newState)      					DMA_Cmd((DMA_ARCH_TYPE *)(dmaResource), newState)						// DMA使能
#define xDMA_ITConfig(dmaResource, flags, newState) 			DMA_ITConfig((DMA_ARCH_TYPE *)(dmaResource), flags, newState)			// DMA中断配置
#define xDMA_GetCurrDataCounter(dmaResource) 					DMA_GetCurrDataCounter((DMA_ARCH_TYPE *)(dmaResource))					// 获取DMA数据流传输中剩余的数据单元数
#define xDMA_SetCurrDataCounter(dmaResource, count) 			DMA_SetCurrDataCounter((DMA_ARCH_TYPE *)(dmaResource), count)			// 写入DMA数据流要传输的数据单元数
#define xDMA_GetFlagStatus(dmaResource, flags)		   		 	DMA_GetFlagStatus((DMA_ARCH_TYPE *)(dmaResource), flags)				// 获取DMA标志位状态
#define xDMA_ClearFlag(dmaResource, flags) 						DMA_ClearFlag((DMA_ARCH_TYPE *)(dmaResource), flags)					// 清楚DMA标志位
#define xDMA_MemoryTargetConfig(dmaResource, address, target)   DMA_MemoryTargetConfig((DMA_ARCH_TYPE *)(dmaResource), address, target) // 配置下一个缓冲区传输的内存地址

// 函数声明
void dmaInit(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex);
void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam);
dmaIdentifier_e dmaGetIdentifier(const dmaResource_t *stream);

