#pragma once

#include "common/time.h"

#include "pg/pg.h"
#include "pg/rx.h"

#include "drivers/io_types.h"

// ---------------------------------------------------------相关宏定义
#define STICK_CHANNEL_COUNT 4							 // 摇杆通道数量

#define PWM_RANGE_MIN 1000								 // PWM范围起始值
#define PWM_RANGE_MAX 2000								 // PWM范围结束值

#define PWM_PULSE_MIN   750       						 // 最小有效脉冲
#define PWM_PULSE_MAX   2250      						 // 最大有效脉冲

#define RXFAIL_STEP_TO_CHANNEL_VALUE(step) (PWM_PULSE_MIN + 25 * step)
#define CHANNEL_VALUE_TO_RXFAIL_STEP(channelValue) ((constrain(channelValue, PWM_PULSE_MIN, PWM_PULSE_MAX) - PWM_PULSE_MIN) / 25)

#define MAX_SUPPORTED_RC_CHANNEL_COUNT 18	 			 // 最大RC通道支持数量
#define NON_AUX_CHANNEL_COUNT 4							 // 非辅助通道数量
#define MAX_AUX_CHANNEL_COUNT (MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT)

#define RSSI_SCALE_MIN 1								 // RSSI最小比例
#define RSSI_SCALE_MAX 255								 // RSSI最大比例
#define RSSI_SCALE_DEFAULT 100							 // RSSI默认比例
#define RSSI_MAX_VALUE 1023								 // RSSI最大值
#define LINK_QUALITY_MAX_VALUE 1023						 // 链接质量最大值

/* ------------------------串行RX类型枚举---------------------- */	
typedef enum {
    SERIALRX_SBUS = 2,
} SerialRXType;

/* ------------------------RX提供者枚举------------------------- */	
typedef enum {
    RX_PROVIDER_NONE = 0,
    RX_PROVIDER_SERIAL,
} rxProvider_t;

/* -----------------------RX通道范围配置------------------------ */	
typedef struct rxChannelRangeConfig_s {
    uint16_t min;
    uint16_t max;
} rxChannelRangeConfig_t;
// 声明RX通道范围配置
PG_DECLARE_ARRAY(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT, rxChannelRangeConfigs);

/* ---------------------RX失控保护通道模式--------------------- */	
typedef enum {
    RX_FAILSAFE_MODE_AUTO = 0,							 // 自动
    RX_FAILSAFE_MODE_HOLD,								 // 保持 
    RX_FAILSAFE_MODE_SET,								 // 设置
    RX_FAILSAFE_MODE_INVALID							 // 无效
} rxFailsafeChannelMode_e;
#define RX_FAILSAFE_MODE_COUNT 3  						 // RX失控保护模式数量

/* ---------------------RX失控保护通道类型--------------------- */	
typedef enum {
    RX_FAILSAFE_TYPE_FLIGHT = 0,
    RX_FAILSAFE_TYPE_AUX
} rxFailsafeChannelType_e;
#define RX_FAILSAFE_TYPE_COUNT 2  						 // RX失控保护类型数量

/* ---------------------RX失控保护通道配置--------------------- */	
typedef struct rxFailsafeChannelConfig_s {
    uint8_t mode; 										 // See rxFailsafeChannelMode_e
    uint8_t step;										 // 步
} rxFailsafeChannelConfig_t;
// 声明RX失控保护通道配置
PG_DECLARE_ARRAY(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, rxFailsafeChannelConfigs);

/* --------------------------RX帧状态-------------------------- */	
typedef enum {
    RX_FRAME_PENDING = 0,								 // RX帧等待
    RX_FRAME_COMPLETE = (1 << 0),						 // RX帧完整
    RX_FRAME_FAILSAFE = (1 << 1),						 // RX帧失控保护
    RX_FRAME_PROCESSING_REQUIRED = (1 << 2),			 // RX帧需要处理
    RX_FRAME_DROPPED = (1 << 3)							 // RX帧丢失
} rxFrameState_e;

/* -------------------------RSSI源枚举-------------------------- */	
typedef enum {
    RSSI_SOURCE_NONE = 0,
    RSSI_SOURCE_RX_CHANNEL,
} rssiSource_e;
				 
/* -----------------------链接质量源枚举------------------------ */	
typedef enum {
    LQ_SOURCE_NONE = 0,
    LQ_SOURCE_RX_PROTOCOL_CRSF,
} linkQualitySource_e;

/* ----------------------RX运行状态结构体----------------------- */	
// 函数指针（回调函数）定义
struct rxRuntimeState_s;
typedef uint16_t (*rcReadRawDataFnPtr)(const struct rxRuntimeState_s *rxRuntimeState, uint8_t chan); 
typedef uint8_t (*rcFrameStatusFnPtr)(struct rxRuntimeState_s *rxRuntimeState);
typedef bool (*rcProcessFrameFnPtr)(const struct rxRuntimeState_s *rxRuntimeState);
typedef timeUs_t rcGetFrameTimeUsFn(void); 																     
typedef struct rxRuntimeState_s {
    rxProvider_t        rxProvider;						 // 接收机模式
    SerialRXType        serialrxProvider;		 	   	 // 接收机协议
    uint8_t             channelCount; 			 	     // 由当前输入通道报告的RC通道数量
    uint16_t            rxRefreshRate;					 // rx刷新率
    rcReadRawDataFnPtr  rcReadRawFn;					 // RC原始数据读取函数
    rcFrameStatusFnPtr  rcFrameStatusFn;				 // RC帧状态函数
    rcProcessFrameFnPtr rcProcessFrameFn;				 // RC帧处理函数
    rcGetFrameTimeUsFn *rcFrameTimeUsFn;				 // 检索最后一个通道数据帧的时间戳(以微秒为单位)函数
    uint16_t            *channelData;					 // 通道数据
    void                *frameData;						 // 帧数据
} rxRuntimeState_t;

extern int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];   
extern rssiSource_e rssiSource;
extern linkQualitySource_e linkQualitySource;		
extern rxRuntimeState_t rxRuntimeState; 		 
extern const char rcChannelLetters[];					 
void rxInit(void);
bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
bool rxIsReceivingSignal(void);
bool calculateRxChannelsAndUpdateFailsafe(timeUs_t currentTimeUs);
struct rxConfig_s;
void parseRcChannels(const char *input, struct rxConfig_s *rxConfig);
void setRssiDirect(uint16_t newRssi, rssiSource_e source);
void setRssi(uint16_t rssiValue, rssiSource_e source);
void updateRSSI(timeUs_t currentTimeUs);
uint16_t getRssi(void);
uint8_t getRssiPercent(void);
uint16_t rxGetLinkQuality(void);
uint16_t rxGetLinkQualityPercent(void);
uint8_t rxGetRfMode(void);
uint16_t rxGetRefreshRate(void);
timeDelta_t rxGetFrameDelta(timeDelta_t *frameAgeUs);

