/*********************************************************************************
 提供OSD字库中的字符地址。
 	字符存储器说明：上电时，显示存储器指向字符存储器的第一个字符，
 	器件正确初始化后，将叠加显示正确的字符，
 	要保证字库的第一个字符（SYM_NONE）都为透明的，否则第一个字符将覆盖整个屏幕。
*********************************************************************************/
#pragma once

// 相关符号
#define SYM_NONE                    0x00		    // 0
#define SYM_END_OF_FONT             0xFF            // 255
#define SYM_HYPHEN                  0x2D		    // 斜杠符号

// GPS相关图标
#define SYM_HOMEFLAG                0x11			// 家的位置图标
#define SYM_LAT                     0x89			// 经度图标
#define SYM_LON                     0x98			// 纬度图标 
#define SYM_ALTITUDE                0x7F			// 高度图标
#define SYM_TOTAL_DISTANCE          0x71			// 总航程图标
#define SYM_OVER_HOME               0x05			// 家图标 - 起飞点

// RSSI
#define SYM_RSSI                    0x01			// RSSI图标
#define SYM_LINK_QUALITY            0x7B			// 链接质量图标

// 油门位置 (%)
#define SYM_THR                     0x04			// 油门百分比图标

// 单位图标 (公制)
#define SYM_M                       0x0C			// m图标
#define SYM_KM                      0x7D			// km图标

// 单位图标 (英制)
#define SYM_FT                      0x0F			// FT图标
#define SYM_MILES                   0x7E

// 磁力计标尺
#define SYM_HEADING_N               0x18
#define SYM_HEADING_S               0x19
#define SYM_HEADING_E               0x1A
#define SYM_HEADING_W               0x1B
#define SYM_HEADING_DIVIDED_LINE    0x1C
#define SYM_HEADING_LINE            0x1D

// AH Center screen Graphics
#define SYM_AH_CENTER_LINE          0x72
#define SYM_AH_CENTER               0x73
#define SYM_AH_CENTER_LINE_RIGHT    0x74
#define SYM_AH_RIGHT                0x02
#define SYM_AH_LEFT                 0x03
#define SYM_AH_DECORATION           0x13

// 卫星图标
#define SYM_SAT_L                   0x1E
#define SYM_SAT_R                   0x1F

// 方向箭头
#define SYM_ARROW_SOUTH             0x60
#define SYM_ARROW_2                 0x61
#define SYM_ARROW_3                 0x62
#define SYM_ARROW_4                 0x63
#define SYM_ARROW_EAST              0x64
#define SYM_ARROW_6                 0x65
#define SYM_ARROW_7                 0x66
#define SYM_ARROW_8                 0x67
#define SYM_ARROW_NORTH             0x68
#define SYM_ARROW_10                0x69
#define SYM_ARROW_11                0x6A
#define SYM_ARROW_12                0x6B
#define SYM_ARROW_WEST              0x6C
#define SYM_ARROW_14                0x6D
#define SYM_ARROW_15                0x6E
#define SYM_ARROW_16                0x6F

#define SYM_ARROW_SMALL_UP          0x75
#define SYM_ARROW_SMALL_DOWN        0x76

// AH Bars
#define SYM_AH_BAR9_0               0x80
#define SYM_AH_BAR9_1               0x81
#define SYM_AH_BAR9_2               0x82
#define SYM_AH_BAR9_3               0x83
#define SYM_AH_BAR9_4               0x84
#define SYM_AH_BAR9_5               0x85
#define SYM_AH_BAR9_6               0x86
#define SYM_AH_BAR9_7               0x87
#define SYM_AH_BAR9_8               0x88

// 电压图标
#define SYM_BATT_FULL               0x90
#define SYM_BATT_5                  0x91
#define SYM_BATT_4                  0x92
#define SYM_BATT_3                  0x93
#define SYM_BATT_2                  0x94
#define SYM_BATT_1                  0x95
#define SYM_BATT_EMPTY              0x96

// Batt Icons
#define SYM_MAIN_BATT               0x97

// 电压和电流（安培）
#define SYM_VOLT                    0x06
#define SYM_AMP                     0x9A

// 定时器
#define SYM_ON_M                    0x9B
#define SYM_FLY_M                   0x9C

// 速度
#define SYM_SPEED                   0x70
#define SYM_KPH                     0x9E
#define SYM_MPH                     0x9D
#define SYM_MPS                     0x9F
#define SYM_FTPS                    0x99

// 菜单光标
#define SYM_CURSOR                  SYM_AH_LEFT

