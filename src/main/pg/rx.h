#pragma once

#include "drivers/io_types.h"

#include "pg/pg.h"

#define GET_FRAME_ERR_LPF_FREQUENCY(period) (1 / (period / 10.0f))
#define FRAME_ERR_RESAMPLE_US 100000

/* ------------------------------RX配置结构体---------------------------- */	
typedef struct rxConfig_s {
    uint8_t rcmap[RX_MAPPABLE_CHANNEL_COUNT];   // 映射无线电通道到内部RPYTA+顺序
    uint8_t serialrx_provider;                  // 基于uart的接收器类型(0 = spek 10, 1 = spek 11, 2 = sbus)。必须先由FEATURE_RX_SERIAL启用
    uint8_t serialrx_inverted;              	// 将串行RX协议与它的默认设置进行对比
    uint8_t halfDuplex;                     	// 允许rx在F4上以半双工模式运行，忽略F1和F3。
    uint8_t rssi_channel;
    uint8_t rssi_scale;
    uint8_t rssi_invert;
    uint16_t midrc;                         	// 有些无线电在1500点没有中立点，可以在这里更改
    uint16_t mincheck;                      	// minimum rc end
    uint16_t maxcheck;                      	// maximum rc end
    uint8_t rcInterpolation;
    uint8_t rcInterpolationChannels;
    uint8_t rcInterpolationInterval;
    uint8_t airModeActivateThreshold;       	// Throttle setpoint percent where airmode gets activated
    uint16_t rx_min_usec;
    uint16_t rx_max_usec;
    uint8_t max_aux_channel;
    uint8_t rssi_src_frame_errors;          	// true to use frame drop flags in the rx protocol
    int8_t rssi_offset;                     	// offset applied to the RSSI value before it is returned
    uint8_t rc_smoothing_type;              	// Determines the smoothing algorithm to use: INTERPOLATION or FILTER
    uint8_t rc_smoothing_input_cutoff;      	// Filter cutoff frequency for the input filter (0 = auto)
    uint8_t rc_smoothing_derivative_cutoff; 	// Filter cutoff frequency for the setpoint weight derivative filter (0 = auto)
    uint8_t rc_smoothing_input_type;        	// Input filter type (0 = PT1, 1 = BIQUAD)
    uint8_t rc_smoothing_derivative_type;   	// Derivative filter type (0 = OFF, 1 = PT1, 2 = BIQUAD)
    uint8_t rc_smoothing_auto_factor;       	// Used to adjust the "smoothness" determined by the auto cutoff calculations
    uint8_t rssi_src_frame_lpf_period;      	// Period of the cutoff frequency for the source frame RSSI filter (in 0.1 s)
    uint8_t sbus_baud_fast; 					// Select SBus fast baud rate
} rxConfig_t;
// 声明RX配置结构体
PG_DECLARE(rxConfig_t, rxConfig);

