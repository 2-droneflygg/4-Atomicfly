#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/dma.h"
#include "drivers/io_types.h"
#include "drivers/rcc_types.h"
#include "drivers/resource.h"
#include "drivers/timer_def.h"

#include "pg/timerio.h"


// ---------------------------------------------------------定时器通道_1..4
#define CC_CHANNELS_PER_TIMER    4      

// ---------------------------------------------------------定义捕获比较类型
typedef uint16_t captureCompare_t;           

// ---------------------------------------------------------定义硬件定时器数量
#define HARDWARE_TIMER_DEFINITION_COUNT 14   

// ---------------------------------------------------------MHZ转HZ单位
#define MHZ_TO_HZ(x) ((x) * 1000000)

#if defined(USE_TIMER_MGMT)
// ---------------------------------------------------------定义全部定时器通道数量
#define FULL_TIMER_CHANNEL_COUNT 78	
// ---------------------------------------------------------定义定时器通道数量
#define TIMER_CHANNEL_COUNT  FULL_TIMER_CHANNEL_COUNT
// ---------------------------------------------------------定义完整的定时器硬件配置
#define TIMER_HARDWARE  fullTimerHardware        
// ---------------------------------------------------------定义使用的定时器
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(6) | TIM_N(7) | TIM_N(8) | TIM_N(9) | TIM_N(10) | TIM_N(11) | TIM_N(12) | TIM_N(13) | TIM_N(14) )
#endif // USE_TIMER_MGMT

// ---------------------------------------------------------使用捕获和溢出的不同类型 - 多个溢出处理程序被实现为链表
struct timerCCHandlerRec_s;
struct timerOvrHandlerRec_s;
typedef void timerCCHandlerCallback(struct timerCCHandlerRec_s* self, uint16_t capture);
typedef void timerOvrHandlerCallback(struct timerOvrHandlerRec_s* self, uint16_t capture);

/* -------------------------------定时器使用标志枚举----------------------------- */	
typedef enum {
    TIM_USE_ANY            = 0x0,
    TIM_USE_NONE           = 0x0,
    TIM_USE_MOTOR          = 0x4,				// 电机
} timerUsageFlag_e;

/* ---------------------------------定时器标志枚举------------------------------- */	
typedef enum {
    TIMER_OUTPUT_NONE      = 0,
    TIMER_OUTPUT_INVERTED  = (1 << 0),			// 定时器输出反向
    TIMER_OUTPUT_N_CHANNEL = (1 << 1),			// 定时器输出N通道
} timerFlag_e;

/* ----------------------------------通道类型枚举-------------------------------- */	
typedef enum {
    TYPE_FREE,
    TYPE_PWMOUTPUT_MOTOR,						// 电机 
    TYPE_SOFTSERIAL_RX,							// 软件串口RX
    TYPE_SOFTSERIAL_TX,							// 软件串口TX
    TYPE_SOFTSERIAL_RXTX,        				// 用于软串行的双向引脚
    TYPE_SOFTSERIAL_AUXTIMER,    				// 定时器通道用于软串行。引脚上没有IO功能
    TYPE_TIMER
} channelType_t;

/* ---------------------------定时器通道中断回调信息结构体----------------------- */	
typedef struct timerCCHandlerRec_s {
    timerCCHandlerCallback* fn;					// 回调函数
} timerCCHandlerRec_t;

/* -----------------------------定时器溢出中断信息结构体------------------------- */	
typedef struct timerOvrHandlerRec_s {
    timerOvrHandlerCallback* fn;				// 回调函数
    struct timerOvrHandlerRec_s* next;			// 结构体指针
} timerOvrHandlerRec_t;

/* ----------------------------------定时器定义结构体---------------------------- */	
typedef struct timerDef_s {
    TIM_TypeDef *TIMx;							// 定时器
    rccPeriphTag_t rcc;							// 时钟
    uint8_t inputIrq;							// 输入中断
} timerDef_t;

/* -------------------------------定时器硬件信息结构体--------------------------- */	
typedef struct timerHardware_s {
    TIM_TypeDef *tim;							// TIMx
    ioTag_t tag;								// IO标签
    uint8_t channel;							// 通道
    timerUsageFlag_e usageFlags;				// 使用标志
    uint8_t output;								// 输出
    uint8_t alternateFunction;					// 备用功能
#if defined(USE_TIMER_DMA)
#if defined(USE_DMA_SPEC)
    dmaResource_t *dmaRefConfigured;			// DMA预配置（DMA控制器_数据流）
    uint32_t dmaChannelConfigured;				// DMA通道配置
#endif 
    dmaResource_t *dmaTimUPRef;    			    // 定时器上溢DMA配置（DMA控制器_数据流）
    uint32_t dmaTimUPChannel;					// 定时器上溢DMA通道
    uint8_t dmaTimUPIrqHandler;					// 定时器上溢DMA中断
#endif
} timerHardware_t;

extern const timerHardware_t timerHardware[];
extern const timerHardware_t fullTimerHardware[];
extern const timerDef_t timerDefinitions[];
typedef uint32_t timCCR_t;
void timerConfigure(const timerHardware_t *timHw, uint16_t period, uint32_t hz); 
void timerChConfigIC(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterSamples);
volatile timCCR_t* timerChCCR(const timerHardware_t* timHw);
void timerChCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn);
void timerChOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn);
void timerChConfigCallbacks(const timerHardware_t *channel, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback);
void timerChInit(const timerHardware_t *timHw, channelType_t type, int irqPriority, uint8_t irq);
void timerInit(void);
void timerStart(void);
void timerForceOverflow(TIM_TypeDef *tim);
uint32_t timerClock(TIM_TypeDef *tim);
void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint32_t hz);  
rccPeriphTag_t timerRCC(TIM_TypeDef *tim);
uint8_t timerInputIrq(TIM_TypeDef *tim);
#if defined(USE_TIMER_MGMT)
extern const resourceOwner_t freeOwner;
struct timerIOConfig_s;
struct timerIOConfig_s *timerIoConfigByTag(ioTag_t ioTag);
const resourceOwner_t *timerGetOwner(int8_t timerNumber, uint16_t timerChannel);
#endif
const timerHardware_t *timerGetByTag(ioTag_t ioTag);
const timerHardware_t *timerAllocate(ioTag_t ioTag, resourceOwner_e owner, uint8_t resourceIndex);
const timerHardware_t *timerGetByTagAndIndex(ioTag_t ioTag, unsigned timerIndex);
ioTag_t timerioTagGetByUsage(timerUsageFlag_e usageFlag, uint8_t index);
void timerOCInit(TIM_TypeDef *tim, uint8_t channel, TIM_OCInitTypeDef *init);
void timerOCPreloadConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t preload);
volatile timCCR_t *timerCCR(TIM_TypeDef *tim, uint8_t channel);
uint16_t timerDmaSource(uint8_t channel);
uint16_t timerGetPrescalerByDesiredHertz(TIM_TypeDef *tim, uint32_t hz);
uint16_t timerGetPrescalerByDesiredMhz(TIM_TypeDef *tim, uint16_t mhz);
uint16_t timerGetPeriodByPrescaler(TIM_TypeDef *tim, uint16_t prescaler, uint32_t hz);
int8_t timerGetNumberByIndex(uint8_t index);
int8_t timerGetTIMNumber(const TIM_TypeDef *tim);
uint8_t timerLookupChannelIndex(const uint16_t channel);

