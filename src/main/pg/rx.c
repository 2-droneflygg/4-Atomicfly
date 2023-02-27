#include "platform.h"

#if defined(USE_SERIAL_RX) 
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "rx.h"

#include "config/config_reset.h"

#include "drivers/io.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "rx/rx.h"

PG_REGISTER_WITH_RESET_FN(rxConfig_t, rxConfig, PG_RX_CONFIG, 2);
void pgResetFn_rxConfig(rxConfig_t *rxConfig)
{
    RESET_CONFIG_2(rxConfig_t, rxConfig,
        .halfDuplex = 0,											  // 半双工
        .serialrx_provider = SERIALRX_PROVIDER,						  // 串行接收机			  
        .serialrx_inverted = 0,										  // 不反相
        .midrc = RX_MID_USEC,										  // 中位
        .mincheck = 1050,											  // 摇杆低位阈值
        .maxcheck = 1900,											  // 摇杆高位阈值
        .rx_min_usec = RX_MIN_USEC,          						  // 低于这个值的前4个通道中的任何一个都将触发rx丢失检测
        .rx_max_usec = RX_MAX_USEC,          						  // 高于这个值的前4个通道中的任何一个都将触发rx丢失检测
        .rssi_src_frame_errors = false,								  // 不使用RX帧错误作为RSSI源
        .rssi_channel = 16,											  // RSSI通道
        .rssi_scale = RSSI_SCALE_DEFAULT,							  // RSSI缩放
        .rssi_offset = 0,											  // RSSI偏移
        .rssi_invert = 0,											  // RSSI反转
        .rssi_src_frame_lpf_period = 30,							  // RSSI动态帧周期
        .rcInterpolation = RC_SMOOTHING_AUTO,						  // RC插值类型		
        .rcInterpolationChannels = INTERPOLATION_CHANNELS_RPYT,       // 插值通道：RPYT
        .rcInterpolationInterval = 19,								  // RC插值区间
        .airModeActivateThreshold = 25,								  // Airmode激活阈值
        .max_aux_channel = DEFAULT_AUX_CHANNEL_COUNT,				  // 最大通道数
        .rc_smoothing_type = RC_SMOOTHING_TYPE_INTERPOLATION,         // RC平滑：插值法
        .rc_smoothing_input_cutoff = 0,      						  // 自动计算截止默认值
        .rc_smoothing_derivative_cutoff = 0, 						  // 自动计算截止默认值
        .rc_smoothing_input_type = RC_SMOOTHING_INPUT_BIQUAD,		  // 输入滤波器类型
        .rc_smoothing_derivative_type = RC_SMOOTHING_DERIVATIVE_AUTO, // 根据前馈方法自动选择类型
        .rc_smoothing_auto_factor = 10,								  // RC平滑自动因素
        .sbus_baud_fast = false,									  // 快速SBUS
    );
	// 通道映射
    parseRcChannels("AETR1234", rxConfig);
}
#endif

