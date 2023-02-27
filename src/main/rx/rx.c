/**********************************************************************
 RX驱动层相关API函数：
	RSSI：信号强度，这样就可以知道飞机何时超出了飞行范围，或者是否受到了射频干扰。
	信号质量：丢帧率，表示数据链接质量。
**********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include <string.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/maths.h"
#include "common/utils.h"
#include "common/filter.h"

#include "config/config.h"
#include "config/config_reset.h"
#include "config/feature.h"

#include "drivers/adc.h"
#include "drivers/time.h"

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"

#include "flight/failsafe.h"

#include "io/serial.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/sbus.h"

#include "io/serial.h"

// ---------------------------------------------------------RC通道字母
const char rcChannelLetters[] = "AERT12345678abcdefgh";		 

// ---------------------------------------------------------RSSI源
rssiSource_e rssiSource;		
// ---------------------------------------------------------RSSI值 - range: [0;1023]
static uint16_t rssi = 0;                 
// ---------------------------------------------------------RSSI缩放比例偏移
#define RSSI_OFFSET_SCALING (1024 / 100.0f)		
// ---------------------------------------------------------链接质量源
linkQualitySource_e linkQualitySource;				
// ---------------------------------------------------------链接质量
#ifdef USE_RX_LINK_QUALITY_INFO
static uint16_t linkQuality = 0;		   			
static uint8_t rfMode = 0;   			   				
#endif

// ---------------------------------------------------------RX通道数量
static uint8_t rxChannelCount;								 
// ---------------------------------------------------------RX数据处理需求
static bool rxDataProcessingRequired    = false;			
// ---------------------------------------------------------AUX辅助处理需求
static bool auxiliaryProcessingRequired = false;	

// ---------------------------------------------------------RX信号接收
static bool rxSignalReceived      = false;					
// ---------------------------------------------------------RX飞行通道是否有效
static bool rxFlightChannelsValid = false;			    	
// ---------------------------------------------------------RX是否在失控保护模式
static bool rxIsInFailsafeMode    = true;					
// ---------------------------------------------------------下一个RX更新时间
static timeUs_t rxNextUpdateAtUs = 0;
// ---------------------------------------------------------需要之前的RX信号
static uint32_t needRxSignalBefore = 0;				
// ---------------------------------------------------------RX信号最大延时（us）
static uint32_t needRxSignalMaxDelayUs;					
// ---------------------------------------------------------推迟RX信号
static uint32_t suspendRxSignalUntil = 0;					
// ---------------------------------------------------------跳过RX采样
static uint8_t  skipRxSamples = 0;		

// ---------------------------------------------------------原始RC数据，范围:[1000;2000]
static int16_t rcRaw[MAX_SUPPORTED_RC_CHANNEL_COUNT];	    
// ---------------------------------------------------------RC数据，       范围:[1000;2000]
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];     	 	  
// ---------------------------------------------------------RC无效脉冲周期
uint32_t rcInvalidPulsPeriod[MAX_SUPPORTED_RC_CHANNEL_COUNT]; 
// ---------------------------------------------------------最大无效脉冲时间
#define MAX_INVALID_PULS_TIME    300					

// ---------------------------------------------------------错误帧滤波
static pt1Filter_t frameErrFilter;		   				
	
// ---------------------------------------------------------运行频率
#define DELAY_33_HZ (1000000 / 33)
#define DELAY_10_HZ (1000000 / 10)
#define DELAY_5_HZ  (1000000 / 5)
// 1.5秒usec周期(呼叫频率独立)
#define SKIP_RC_ON_SUSPEND_PERIOD 1500000   
// 去掉2个样品掉落错误的测量值(计时独立)
#define SKIP_RC_SAMPLES_ON_RESUME  2                 		 

// ---------------------------------------------------------RX运行状态
rxRuntimeState_t rxRuntimeState;						
// ---------------------------------------------------------RC采样索引
static uint8_t rcSampleIndex = 0;							

// 复位所有RX通道范围配置
PG_REGISTER_ARRAY_WITH_RESET_FN(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT, rxChannelRangeConfigs, PG_RX_CHANNEL_RANGE_CONFIG, 0);
void pgResetFn_rxChannelRangeConfigs(rxChannelRangeConfig_t *rxChannelRangeConfigs)
{
    // 设置默认校准为全范围和1:1映射
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfigs[i].min = PWM_RANGE_MIN;
        rxChannelRangeConfigs[i].max = PWM_RANGE_MAX;
    }
}
// 一阶失控保护预置通道设置
PG_REGISTER_ARRAY_WITH_RESET_FN(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, rxFailsafeChannelConfigs, PG_RX_FAILSAFE_CHANNEL_CONFIG, 0);
void pgResetFn_rxFailsafeChannelConfigs(rxFailsafeChannelConfig_t *rxFailsafeChannelConfigs)
{
	// 俯仰、偏航、横滚设置为 1500
	for(int i = 0;i < 3;i++) {
        rxFailsafeChannelConfigs[i].mode = RX_FAILSAFE_MODE_SET;
        rxFailsafeChannelConfigs[i].step = CHANNEL_VALUE_TO_RXFAIL_STEP(1500);
	}
	// 油门设置为 987
    rxFailsafeChannelConfigs[3].mode = RX_FAILSAFE_MODE_SET;
    rxFailsafeChannelConfigs[3].step = CHANNEL_VALUE_TO_RXFAIL_STEP(987);
	// 切换到失控保护开关
    rxFailsafeChannelConfigs[7].mode = RX_FAILSAFE_MODE_SET;
    rxFailsafeChannelConfigs[7].step = CHANNEL_VALUE_TO_RXFAIL_STEP(2000);
	// 其余通道设置为 HOLD 
    for (int i = 4; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
		if(i != 7) {
	        rxFailsafeChannelConfigs[i].mode = RX_FAILSAFE_MODE_HOLD;
	        rxFailsafeChannelConfigs[i].step = (i == THROTTLE)
	            ? CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MIN_USEC)
	            : CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MID_USEC);
		}
    }
}

//-------------------------------------------------------------------------------------空帧操作相关API

/**********************************************************************
函数名称：nullFrameStatus
函数功能：空帧状态
函数形参：rxRuntimeState
函数返回值：RX_FRAME_PENDING - RX帧等待
函数描述：None
**********************************************************************/
static uint8_t nullFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);
	// RX帧等待
    return RX_FRAME_PENDING;
}

/**********************************************************************
函数名称：nullFrameStatus
函数功能：空进程帧
函数形参：rxRuntimeState
函数返回值：true
函数描述：None
**********************************************************************/
static bool nullProcessFrame(const rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);
    return true;
}

//-------------------------------------------------------------------------------------RX初始化相关API

/**********************************************************************
函数名称：serialRxInit
函数功能：串行RX初始化
函数形参：rxConfig，rxRuntimeState
函数返回值：脉冲是否有效
函数描述：None
**********************************************************************/
#ifdef USE_SERIAL_RX
static bool serialRxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    bool enabled = false;
    switch (rxRuntimeState->serialrxProvider) {
#ifdef USE_SERIALRX_SBUS
	// 串行SBUS协议
    case SERIALRX_SBUS:													
    	// SBUS初始化
        enabled = sbusInit(rxConfig, rxRuntimeState);
        break;
#endif
    default:
        enabled = false;
        break;
    }
    return enabled;
}
#endif

/**********************************************************************
函数名称：rxInit
函数功能：RX初始化
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void rxInit(void)
{
	// 选择接收机模式
    if (featureIsEnabled(FEATURE_RX_SERIAL)) {
		// 串行接收机
        rxRuntimeState.rxProvider = RX_PROVIDER_SERIAL;					
    } else {
    	// 无接收机
        rxRuntimeState.rxProvider = RX_PROVIDER_NONE;					
    }
	// 接收机协议
    rxRuntimeState.serialrxProvider = rxConfig()->serialrx_provider;	
	// RX原始数据读取函数
    rxRuntimeState.rcReadRawFn = 0;					
	// RX状态帧函数
    rxRuntimeState.rcFrameStatusFn = nullFrameStatus;				
	// RX帧处理函数
    rxRuntimeState.rcProcessFrameFn = nullProcessFrame;			
	// RX采样索引
    rcSampleIndex = 0;							
	// 需要RX信号最大延时
    needRxSignalMaxDelayUs = DELAY_10_HZ;								

	// 遍历所有通道
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
		// 中点
        rcData[i] = rxConfig()->midrc;					
		// 无效脉冲周期
        rcInvalidPulsPeriod[i] = millis() + MAX_INVALID_PULS_TIME;		
    }

	// 初始化RC数据为RX最小值
    rcData[THROTTLE] = rxConfig()->rx_min_usec;							

    // 遍历所有激活通道 - 当通过开关解锁时，初始化解锁开关到关闭位置
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *modeActivationCondition = modeActivationConditions(i);
		// 查找解锁通道
        if (modeActivationCondition->modeId == BOXARM && IS_RANGE_USABLE(&modeActivationCondition->range)) {
            // 确定解锁开关关断值
            uint16_t value;
            if (modeActivationCondition->range.startStep > 0) {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.startStep - 1));
            } else {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.endStep + 1));
            }
            // 初始化解锁通道数据为关断值
            rcData[modeActivationCondition->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = value;
        }
    }

	// 初始化接收机
    switch (rxRuntimeState.rxProvider) {
    default:
        break;
#ifdef USE_SERIAL_RX
	// 串行接收机
    case RX_PROVIDER_SERIAL:											
        {
        	// 串行RX初始化
            const bool enabled = serialRxInit(rxConfig(), &rxRuntimeState);
            if (!enabled) {
                rxRuntimeState.rcReadRawFn = 0;
                rxRuntimeState.rcFrameStatusFn = nullFrameStatus;
            }
        }

        break;
#endif
    }

	// 选择RX通道作为RSSI源
    if (rxConfig()->rssi_channel > 0) {
        rssiSource = RSSI_SOURCE_RX_CHANNEL;
    }

    // 设置帧的RSSI低通滤波，以取每个FRAME_ERR_RESAMPLE_US的平均值
    pt1FilterInit(&frameErrFilter, pt1FilterGain(GET_FRAME_ERR_LPF_FREQUENCY(rxConfig()->rssi_src_frame_lpf_period), FRAME_ERR_RESAMPLE_US/1000000.0));

	// 获取RX通道数量
    rxChannelCount = MIN(rxConfig()->max_aux_channel + NON_AUX_CHANNEL_COUNT, rxRuntimeState.channelCount);
}

//-------------------------------------------------------------------------------------通道映射相关API

/**********************************************************************
函数名称：parseRcChannels
函数功能：设置rcmap - 通道映射
函数形参：input，rxConfig
函数返回值：None
函数描述：None
**********************************************************************/
void parseRcChannels(const char *input, rxConfig_t *rxConfig)
{
	// 遍历input字符串
    for (const char *c = input; *c; c++) {
		// 在字符串中搜索第一次出现字符c的位置
        const char *s = strchr(rcChannelLetters, *c);
		// 检查合法性
        if (s && (s < rcChannelLetters + RX_MAPPABLE_CHANNEL_COUNT)) {
			// 通道映射
            rxConfig->rcmap[s - rcChannelLetters] = c - input;
        }
    }
}

//-------------------------------------------------------------------------------------RX通道设置相关API

/**********************************************************************
函数名称：applyRxChannelRangeConfiguraton
函数功能：应用RX通道范围设置
函数形参：sample，range
函数返回值：sample
函数描述：None
**********************************************************************/
STATIC_UNIT_TESTED uint16_t applyRxChannelRangeConfiguraton(int sample, const rxChannelRangeConfig_t *range)
{
    if (sample == 0) {
        return 0;
    }
	// 缩放范围在1000-2000内
    sample = scaleRange(sample, range->min, range->max, PWM_RANGE_MIN, PWM_RANGE_MAX);
	// 约束在有效脉冲范围内
    sample = constrain(sample, PWM_PULSE_MIN, PWM_PULSE_MAX);
    return sample;
}

/**********************************************************************
函数名称：readRxChannelsApplyRanges
函数功能：读取RX通道应用范围
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
static void readRxChannelsApplyRanges(void)
{
	// 遍历所有RC通道
    for (int channel = 0; channel < rxChannelCount; channel++) {
		// 获取原始通道 - 映射无线电通道到内部RPYTA+顺序
        const uint8_t rawChannel = channel < RX_MAPPABLE_CHANNEL_COUNT ? rxConfig()->rcmap[channel] : channel;
        // 通道采样 - sbusChannelsReadRawRC 回调函数
        uint16_t sample = rxRuntimeState.rcReadRawFn(&rxRuntimeState, rawChannel);
        // 应用RX通道范围设置 -    非辅助通道
        if (channel < NON_AUX_CHANNEL_COUNT) {
            sample = applyRxChannelRangeConfiguraton(sample, rxChannelRangeConfigs(channel));
        }
		// 得到RC原始数据
        rcRaw[channel] = sample;
    }
}

//-------------------------------------------------------------------------------------RX刷新率&&帧时差相关API

/**********************************************************************
函数名称：rxGetRefreshRate
函数功能：获取刷新率
函数形参：None
函数返回值：rxRefreshRate
函数描述：
	用于RC插值进程。
**********************************************************************/
uint16_t rxGetRefreshRate(void)
{
    return rxRuntimeState.rxRefreshRate;
}

/**********************************************************************
函数名称：rxGetFrameDelta
函数功能：获取RX帧时差
函数形参：帧年龄
函数返回值：帧时差
函数描述：None
**********************************************************************/
timeDelta_t rxGetFrameDelta(timeDelta_t *frameAgeUs)
{
    static timeUs_t previousFrameTimeUs = 0;
    static timeDelta_t frameTimeDeltaUs = 0;

	// 检索最后一个通道数据帧的时间戳(以微秒为单位)
    if (rxRuntimeState.rcFrameTimeUsFn) {
        const timeUs_t frameTimeUs = rxRuntimeState.rcFrameTimeUsFn();
		// 计算帧年龄
        *frameAgeUs = cmpTimeUs(micros(), frameTimeUs);
		// 计算时差
        const timeDelta_t deltaUs = cmpTimeUs(frameTimeUs, previousFrameTimeUs);
        if (deltaUs) {
            frameTimeDeltaUs = deltaUs;
            previousFrameTimeUs = frameTimeUs;
        }
    }
    return frameTimeDeltaUs;
}

//-------------------------------------------------------------------------------------RX状态相关API

/**********************************************************************
函数名称：rxIsReceivingSignal
函数功能：获取rx是否正在接收信号
函数形参：None
函数返回值：rxSignalReceived
函数描述：None
**********************************************************************/
bool rxIsReceivingSignal(void)
{
    return rxSignalReceived;
}

//-------------------------------------------------------------------------------------RSSI相关API

/**********************************************************************
函数名称：setRssiDirect
函数功能：直接设置RSSI
函数形参：input，rxConfig
函数返回值：None
函数描述：None
**********************************************************************/
void setRssiDirect(uint16_t newRssi, rssiSource_e source)
{
    if (source != rssiSource) {
        return;
    }
    rssi = newRssi;
}

/**********************************************************************
函数名称：updateRSSIPWM
函数功能：更新AUX通道RSSI
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
static void updateRSSIPWM(void)
{
    // 读取AUX通道的值为rssi
    int16_t pwmRssi = rcData[rxConfig()->rssi_channel - 1];

    // 直接设置RSSI - rawPwmRssi的取值范围为[1000;2000]。rssi应该在[0;1023];
    setRssiDirect(scaleRange(constrain(pwmRssi, PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, RSSI_MAX_VALUE), RSSI_SOURCE_RX_CHANNEL);
}

/**********************************************************************
函数名称：updateRSSI
函数功能：更新RSSI
函数形参：currentTimeUs
函数返回值：None
函数描述：None
**********************************************************************/
void updateRSSI(timeUs_t currentTimeUs)
{
    switch (rssiSource) {
	    case RSSI_SOURCE_RX_CHANNEL:  // rssi源：RX通道
	        updateRSSIPWM();
	        break;
	    default:
	        break;
    }
}

/**********************************************************************
函数名称：getRssi
函数功能：获取RSSI
函数形参：currentTimeUs
函数返回值：rssi
函数描述：None
**********************************************************************/
uint16_t getRssi(void)
{
    uint16_t rssiValue = rssi;

    // RSSI_Invert option
    if (rxConfig()->rssi_invert) {
        rssiValue = RSSI_MAX_VALUE - rssiValue;
    }

	// 加入rssi缩放和偏移
    return rxConfig()->rssi_scale / 100.0f * rssiValue + rxConfig()->rssi_offset * RSSI_OFFSET_SCALING;
}

/**********************************************************************
函数名称：getRssiPercent
函数功能：获取百分比RSSI
函数形参：currentTimeUs
函数返回值：rssi百分比
函数描述：None
**********************************************************************/
uint8_t getRssiPercent(void)
{
	// 缩放到0-100
    return scaleRange(getRssi(), 0, RSSI_MAX_VALUE, 0, 100);
}

//-------------------------------------------------------------------------------------LQ相关API

#ifdef USE_RX_LINK_QUALITY_INFO
/**********************************************************************
函数名称：rxGetLinkQuality
函数功能：获取链接质量
函数形参：None
函数返回值：连接质量
函数描述：None
**********************************************************************/
uint16_t rxGetLinkQuality(void)
{
    return linkQuality;
}

/**********************************************************************
函数名称：rxGetRfMode
函数功能：获取RF模式
函数形参：None
函数返回值：rfMode
函数描述：None
**********************************************************************/
uint8_t rxGetRfMode(void)
{
    return rfMode;
}

/**********************************************************************
函数名称：rxGetLinkQualityPercent
函数功能：获取连接质量百分比
函数形参：None
函数返回值：连接质量百分比
函数描述：None
**********************************************************************/
uint16_t rxGetLinkQualityPercent(void)
{
    return (linkQualitySource == LQ_SOURCE_RX_PROTOCOL_CRSF) ?  linkQuality : scaleRange(linkQuality, 0, LINK_QUALITY_MAX_VALUE, 0, 100);
}
#endif

#ifdef USE_RX_LINK_QUALITY_INFO
/**********************************************************************
函数名称：updateLinkQualitySamples
函数功能：更新链接质量采样
函数形参：value
函数返回值：平均值
函数描述：None
**********************************************************************/
// 连接质量样本数量
#define LINK_QUALITY_SAMPLE_COUNT 16									
STATIC_UNIT_TESTED uint16_t updateLinkQualitySamples(uint16_t value)
{
	// 采样缓存数组 - 保存16个数据
    static uint16_t samples[LINK_QUALITY_SAMPLE_COUNT]; 
	// 采样数据索引
    static uint8_t sampleIndex = 0;
	// 均值缓存
    static uint16_t sum = 0;

    sum += value - samples[sampleIndex];
    samples[sampleIndex] = value;
    sampleIndex = (sampleIndex + 1) % LINK_QUALITY_SAMPLE_COUNT;
	// 取平均值
    return sum / LINK_QUALITY_SAMPLE_COUNT;
}
#endif

/**********************************************************************
函数名称：setLinkQuality
函数功能：更新链接质量
函数形参：当前帧是否有效，当前时差
函数返回值：None
函数描述：None
**********************************************************************/
static void setLinkQuality(bool validFrame, timeDelta_t currentDeltaTimeUs)
{
#ifdef USE_RX_LINK_QUALITY_INFO
    if (linkQualitySource != LQ_SOURCE_RX_PROTOCOL_CRSF) {
        // 更新连接质量均值采样 - 输入该帧有效（1023）或无效（0）
        linkQuality = updateLinkQualitySamples(validFrame ? LINK_QUALITY_MAX_VALUE : 0);
    }
#endif
}

//-------------------------------------------------------------------------------------RX失控保护相关API

/**********************************************************************
函数名称：nullFrameStatus
函数功能：获取脉冲是否有效
函数形参：脉冲持续时间
函数返回值：脉冲是否有效
函数描述：
	判断是否在取值范围内。
**********************************************************************/
STATIC_UNIT_TESTED bool isPulseValid(uint16_t pulseDuration)
{
    return  pulseDuration >= rxConfig()->rx_min_usec &&
            pulseDuration <= rxConfig()->rx_max_usec;
}

/**********************************************************************
函数名称：getRxfailValue
函数功能：获取RC失败帧
函数形参：通道
函数返回值：RXFAIL_STEP_TO_CHANNEL_VALUE
函数描述：None
**********************************************************************/
static uint16_t getRxfailValue(uint8_t channel)
{
    const rxFailsafeChannelConfig_t *channelFailsafeConfig = rxFailsafeChannelConfigs(channel);

    switch (channelFailsafeConfig->mode) {
	    case RX_FAILSAFE_MODE_AUTO:						// 自动
	        switch (channel) {
		        case ROLL:
		        case PITCH:
		        case YAW:								// RPY应用通道中值1500
		            return rxConfig()->midrc;
		        case THROTTLE:							// T应用通道最小值885
		            return rxConfig()->rx_min_usec;
	        }
	    FALLTHROUGH;
	    default:
	    case RX_FAILSAFE_MODE_INVALID:					// 无效
	    case RX_FAILSAFE_MODE_HOLD:						// 保持 - 将会应用RC数据保持
	        return rcData[channel];
	    case RX_FAILSAFE_MODE_SET:						// 设置 - 应用通道预先设置的失控保护值
	        return RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfig->step);
    }
}

/**********************************************************************
函数名称：detectAndApplySignalLossBehaviour
函数功能：检测并应用信号丢失行为
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
static void detectAndApplySignalLossBehaviour(void)
{
    const uint32_t currentTimeMs = millis();

	// 使用来自RC的值 - RX信号接收 && 未在失控保护模式（来自接收机RX_FRAME_FAILSAFE位）
    const bool useValueFromRx = rxSignalReceived && !rxIsInFailsafeMode;

	// RX飞行通道有效
    rxFlightChannelsValid = true;
	
	// 遍历所有RX通道 
    for (int channel = 0; channel < rxChannelCount; channel++) {
		// 采样应用原始RC数据
        uint16_t sample = rcRaw[channel];

		// 获取脉冲是否有效 - 失控保护模式状态 && 通道数据是否在有效范围(885-2115)
        const bool validPulse = useValueFromRx && isPulseValid(sample);

		// 信号有效
        if (validPulse) {
			// 更新RC无效脉冲周期 - 当前时间节拍+允许的最大无效脉冲时间
            rcInvalidPulsPeriod[channel] = currentTimeMs + MAX_INVALID_PULS_TIME;
        } 
		// 信号丢失
		else {
			// 如果在允许的最大无效脉冲时间内则跳过
            if (cmp32(currentTimeMs, rcInvalidPulsPeriod[channel]) < 0) {
				// 跳到下一个通道以保持通道值MAX_INVALID_PULS_TIME
                continue;         				    
            } 
			// 超过允许的最大无效脉冲时间 - 触发失控保护
			else {
				// 获取RC失败帧 - 之后应用rxfail值
                sample = getRxfailValue(channel);  
                if (channel < NON_AUX_CHANNEL_COUNT) {
					// 当前通道无效
                    rxFlightChannelsValid = false;
                }
            }
        }

		// RC数据 = RC原始数据
        rcData[channel] = sample;
    }

	// 当前通道有效       && 未激活失控保护
    if (rxFlightChannelsValid && !IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
		// 对收到的有效数据进行失控保护
        failsafeOnValidDataReceived();
    } 
	// 失控保护
	else {
		// 置位失控保护模式
        rxIsInFailsafeMode = true;
		// 失效数据的失控保护
        failsafeOnValidDataFailed();
		// 遍历所有通道
        for (int channel = 0; channel < rxChannelCount; channel++) {
			// 应用RC失控保护数据
            rcData[channel] = getRxfailValue(channel);
        }
    }
}

/**********************************************************************
函数名称：calculateRxChannelsAndUpdateFailsafe
函数功能：计算Rx通道和更新失控保护
函数形参：currentTimeUs
函数返回值：状态
函数描述：None
**********************************************************************/
bool calculateRxChannelsAndUpdateFailsafe(timeUs_t currentTimeUs)
{
	// 辅助处理要求
    if (auxiliaryProcessingRequired) {
        auxiliaryProcessingRequired = !rxRuntimeState.rcProcessFrameFn(&rxRuntimeState);
    }
	// 如果不需要处理rx数据 - 直接返回false
    if (!rxDataProcessingRequired) {
        return false;
    }
	
	// 复位RX数据处理需求，为下一次做准备
    rxDataProcessingRequired = false;
	
	// 更新下一个更新RX时间（33HZ运行频率）
    rxNextUpdateAtUs = currentTimeUs + DELAY_33_HZ;

    // 只有当没有更多的采样要跳过和暂停时间结束时才继续
    if (skipRxSamples || currentTimeUs <= suspendRxSignalUntil) {
        if (currentTimeUs > suspendRxSignalUntil) {
            skipRxSamples--;
        }
        return true;
    }

	// 读取RX通道应用范围 - 应用RC范围并得到RC原始数据（rcRaw）
    readRxChannelsApplyRanges();
	// 检测并应用信号丢失行为 - 未失控则更新rcData
    detectAndApplySignalLossBehaviour();

	// RC采样累积
    rcSampleIndex++;

    return true;
}

//-------------------------------------------------------------------------------------RC更新检查函数API

/**********************************************************************
函数名称：rxUpdateCheck
函数功能：RC更新检查函数 
函数形参：当前时间节拍，当前时差
函数返回值：rxDataProcessingRequired || auxiliaryProcessingRequired
函数描述：
	由调度程序检查事件。
**********************************************************************/
bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
	// 信号接收
    bool signalReceived = false;				
	// 使用数据驱动处理
    bool useDataDrivenProcessing = true;								

	// --------------------------------------------选择接收机模式
    switch (rxRuntimeState.rxProvider) {
	    default:
	        break;
		// RX由串口提供
	    case RX_PROVIDER_SERIAL:										
	        {
	        	// 获取RX帧状态 - sbusFrameStatus ISR回调函数
	            const uint8_t frameStatus = rxRuntimeState.rcFrameStatusFn(&rxRuntimeState);
				// 检验RX帧完整性
	            if (frameStatus & RX_FRAME_COMPLETE) {
					// 检验是否在失控保护模式
	                rxIsInFailsafeMode = (frameStatus & RX_FRAME_FAILSAFE) != 0;  
					// 检验是否有RX帧丢失
	                bool rxFrameDropped = (frameStatus & RX_FRAME_DROPPED) != 0;
					// 判断帧有效性 - 无失控保护和丢帧
	                signalReceived = !(rxIsInFailsafeMode || rxFrameDropped);
	                if (signalReceived) {
						// 在设置时间之前需要RX信号 = 当前时间节拍 + RX信号最大延时（us）
	                    needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
	                }
					// 设置连接质量 - 更新连接质量均值采样
	                setLinkQuality(signalReceived, currentDeltaTimeUs);
	            }
				// AUX辅助处理需求
	            if (frameStatus & RX_FRAME_PROCESSING_REQUIRED) {
	                auxiliaryProcessingRequired = true;
	            }
	        }
        break;
    }

	// --------------------------------------------判断RX帧是否接收成功
    if (signalReceived) {
        rxSignalReceived = true;
    } else if (currentTimeUs >= needRxSignalBefore) {
    	// 超时
        rxSignalReceived = false;
    }

	// --------------------------------------------判断RX数据处理需求 - RX帧有效 || 到了下一个需要更新的时间
    if ((signalReceived && useDataDrivenProcessing) || cmpTimeUs(currentTimeUs, rxNextUpdateAtUs) > 0) {
        rxDataProcessingRequired = true;
    }

	// 需要处理RX数据 || AUX辅助处理需求
    return rxDataProcessingRequired || auxiliaryProcessingRequired; 	
}

