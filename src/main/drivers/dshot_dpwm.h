#pragma once

#include "drivers/timer.h"
#include "drivers/dma.h"
#include "drivers/dshot.h"
#include "drivers/motor.h"


#define MOTOR_DSHOT600_HZ     MHZ_TO_HZ(12)				  // DSHOT600频率

#define MOTOR_BIT_0             7						  // 位0
#define MOTOR_BIT_1             14						  // 位1
#define MOTOR_BITLENGTH         20                        // 位长度

#define MAX_DMA_TIMERS          8						  // 电机DMA相关，从pwm_output.h移动

#define DSHOT_DMA_BUFFER_SIZE   18 						  // 分辨率+帧重置(2us) 
#define PROSHOT_DMA_BUFFER_SIZE 6  				 		  // 分辨率+帧重置(2us) 

#define DSHOT_DMA_BUFFER_ATTRIBUTE 						  // DSHOT_DMA缓冲属性

#define DSHOT_DMA_BUFFER_UNIT uint32_t					  // DSHOT_DMA缓冲单元

#define DSHOT_DMA_BUFFER_ALLOC_SIZE DSHOT_DMA_BUFFER_SIZE // DSHOT_DMA缓冲区大小

/* --------------------------电机定时器DMA结构体-------------------------- */	
typedef struct {
    TIM_TypeDef *timer;									  // TIMx
#if defined(USE_DSHOT)
    uint16_t outputPeriod;								  // 输出周期
#if defined(USE_DSHOT_DMAR)
    dmaResource_t *dmaBurstRef;							  // DMA控制器_数据流
    uint16_t dmaBurstLength;							  // 数据长度
    uint32_t *dmaBurstBuffer;							  // DMA缓冲区
#endif
#endif
    uint16_t timerDmaSources;							  // 定时器DMA源
} motorDmaTimer_t;

/* ---------------------------电机DMA输出结构体--------------------------- */	
typedef struct motorDmaOutput_s {
    dshotProtocolControl_t protocolControl;				  // DSHOT协议
    ioTag_t ioTag;										  // IO引脚
    const timerHardware_t *timerHardware;				  // 定时器硬件
#ifdef USE_DSHOT
    uint16_t timerDmaSource;							  // 定时器DMA源
    uint8_t timerDmaIndex;								  // 定时器DMA索引
    bool configured;									  // 配置状态
    uint8_t output;										  // 输出
    uint8_t index;										  // 索引
    uint32_t iocfg;										  // IO配置
    DMA_InitTypeDef dmaInitStruct;			 		  	  // DMA初始化结构体
    dmaResource_t *dmaRef;								  // DMA控制器_数据流
#endif // USE_DSHOT
    motorDmaTimer_t *timer;								  // 电机定时器DMA信息
    DSHOT_DMA_BUFFER_UNIT *dmaBuffer;					  // DMA缓冲区
} motorDmaOutput_t;

// 函数指针，用于将电机值编码到DMA缓冲区
typedef uint8_t loadDmaBufferFn(uint32_t *dmaBuffer, int stride, uint16_t packet);  

extern bool useBurstDshot;
extern motorDevice_t dshotPwmDevice;
extern FAST_RAM_ZERO_INIT loadDmaBufferFn *loadDmaBuffer;
extern DSHOT_DMA_BUFFER_UNIT dshotDmaBuffer[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_ALLOC_SIZE];
extern DSHOT_DMA_BUFFER_UNIT dshotDmaInputBuffer[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_ALLOC_SIZE];
#ifdef USE_DSHOT_DMAR
extern DSHOT_DMA_BUFFER_UNIT dshotBurstDmaBuffer[MAX_DMA_TIMERS][DSHOT_DMA_BUFFER_SIZE * 4];
#endif
struct motorDevConfig_s;
motorDevice_t *dshotPwmDevInit(const motorDevConfig_t *motorConfig, uint8_t motorCount);
uint8_t loadDmaBufferDshot(uint32_t *dmaBuffer, int stride, uint16_t packet);
uint32_t getDshotHz(motorPwmProtocolTypes_e pwmProtocolType);
motorDmaOutput_t *getMotorDmaOutput(uint8_t index);
void pwmWriteDshotInt(uint8_t index, uint16_t value);
bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, motorPwmProtocolTypes_e pwmProtocolType, uint8_t output);
void pwmCompleteDshotMotorUpdate(void);

