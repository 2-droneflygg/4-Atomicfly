#pragma once

#include "common/time.h"

#include "drivers/display.h"

#include "pg/pg.h"

// OSD定时器源名称
#define OSD_NUM_TIMER_TYPES 4
extern const char * const osdTimerSourceNames[OSD_NUM_TIMER_TYPES];

// OSD元素缓存长度
#define OSD_ELEMENT_BUFFER_LENGTH 32			

// OSD配置文件名称长度
#define OSD_PROFILE_NAME_LENGTH 16

// OSD配置文件数量
#define OSD_PROFILE_COUNT 1

// OSD RC通道数量
#define OSD_RCCHANNELS_COUNT 4

// OSD配置文件操作宏
#define OSD_PROFILE_BITS_POS 11
#define OSD_PROFILE_MASK    (((1 << OSD_PROFILE_COUNT) - 1) << OSD_PROFILE_BITS_POS)
#define OSD_POS_MAX   0x3FF
#define OSD_POSCFG_MAX   (OSD_PROFILE_MASK | 0x3FF) 
#define OSD_PROFILE_FLAG(x)  (1 << ((x) - 1 + OSD_PROFILE_BITS_POS))
#define OSD_PROFILE_1_FLAG  OSD_PROFILE_FLAG(1)
#define VISIBLE(x) ((x) & OSD_PROFILE_MASK)
#define VISIBLE_IN_OSD_PROFILE(item, profile) VISIBLE(item)

// OSD GPS救援失效警报持续时间
#define OSD_GPS_RESCUE_DISABLED_WARNING_DURATION_US 3000000 			// 3 seconds

// 特性协调
#define OSD_POSITION_BITS 5 					               		    // 5位给出0-31的范围
#define OSD_POSITION_XY_MASK ((1 << OSD_POSITION_BITS) - 1)             // 10 0000   
#define OSD_POS(x,y)  ((x & OSD_POSITION_XY_MASK) | ((y & OSD_POSITION_XY_MASK) << OSD_POSITION_BITS))  // 高位y。低位x
#define OSD_X(x)      (x & OSD_POSITION_XY_MASK)						// 转换x
#define OSD_Y(x)      ((x >> OSD_POSITION_BITS) & OSD_POSITION_XY_MASK) // 转换y

// 定时器配置
// 存储为15[alarm:8][precision:4][source:4]0
#define OSD_TIMER(src, prec, alarm) ((src & 0x0F) | ((prec & 0x0F) << 4) | ((alarm & 0xFF ) << 8))
#define OSD_TIMER_SRC(timer)        (timer & 0x0F)
#define OSD_TIMER_PRECISION(timer)  ((timer >> 4) & 0x0F)
#define OSD_TIMER_ALARM(timer)      ((timer >> 8) & 0xFF)

/* ------------------------OSD显示端口设备枚举-------------------- */	
typedef enum {
    OSD_DISPLAYPORT_DEVICE_NONE = 0,
    OSD_DISPLAYPORT_DEVICE_AUTO,
    OSD_DISPLAYPORT_DEVICE_MAX7456
} osdDisplayPortDevice_e;

/* --------------------------OSD元素枚举-------------------------- */	
// 为确保向后兼容，新enum的值必须被添加在OSD_XXXX_COUNT表项的末尾
// 请参见osd/osd_elements.c的顶部信息如何添加OSD元素
typedef enum {
    OSD_RSSI_VALUE,          		// RSSI值
    OSD_MAIN_BATT_VOLTAGE,   		// 电池电压
    OSD_CROSSHAIRS,          		// 十字准星
    OSD_ARTIFICIAL_HORIZON,  		// 模拟地平线
    OSD_HORIZON_SIDEBARS,    		// 模拟地平线侧标
    OSD_ITEM_TIMER_1,	     		// 定时器1
    OSD_ITEM_TIMER_2,	     		// 定时器2
    OSD_FLYMODE,             		// 飞行模式
    OSD_THROTTLE_POS,        		// 油门位置
    OSD_VTX_CHANNEL,		 		// 图传频道
    OSD_CURRENT_DRAW,        		// 实时电流计
    OSD_GPS_SPEED,           		// GPS速度
    OSD_GPS_SATS,            		// GPS星数
    OSD_ALTITUDE,            		// 海报高度
    OSD_PIDRATE_PROFILE,			// PID和RATE文件名称
    OSD_WARNINGS,					// 警告
    OSD_GPS_LON,					// GPS经度
    OSD_GPS_LAT,					// GPS纬度
    OSD_DISARMED,					// 已上锁
    OSD_HOME_DIR,					// 到起飞点方向
    OSD_HOME_DIST,					// 到起飞点距离
    OSD_NUMERICAL_VARIO,			// 数字式垂直速度表
    OSD_COMPASS_BAR,				// 罗盘标尺
    OSD_FLIP_ARROW,					// 反乌龟箭头
    OSD_LINK_QUALITY,				// 连接质量
    OSD_FLIGHT_DIST,				// 飞行总距离
    OSD_ITEM_COUNT 					// MUST BE LAST
} osd_items_e;

/* --------------------------OSD统计枚举-------------------------- */	
// 当新元素被添加到'osd_items_e'时，确保是incrementosdc中osdConfig的参数组版本
// 不重新排序STATS枚举。这里的顺序与启用标志位的位置一致
// 存储和更改顺序将破坏用户设置。任何新的统计必须添加到最后
// 在OSD_STAT_COUNT项之前。还必须将新的统计添加到osd.c中的osdStatsDisplayOrder数组
// 如果你想重新排序STATS显示，然后调整osdStatsDisplayOrder数组的排序
typedef enum {
    OSD_STAT_TIMER_1,				// 计时器1
    OSD_STAT_TIMER_2,				// 计时器2（统计解锁时间）
    OSD_STAT_MAX_SPEED,				// 最大飞行速度
    OSD_STAT_MAX_DISTANCE,			// 最大飞行距离
    OSD_STAT_MIN_BATTERY,			// 电池电压最小值
    OSD_STAT_END_BATTERY,			// 电池电压截止值
    OSD_STAT_BATTERY,				// 电池电压
    OSD_STAT_MIN_RSSI,				// RSSI最小值
    OSD_STAT_MAX_CURRENT,			// 电池电流最大值
    OSD_STAT_MAX_ALTITUDE,			// 最大高度
    OSD_STAT_MIN_LINK_QUALITY,		// 连接质量最小值
    OSD_STAT_FLIGHT_DISTANCE,		// 飞行距离
    OSD_STAT_TOTAL_TIME,			// 总飞行时间
    OSD_STAT_TOTAL_DIST,			// 总飞行距离
    OSD_STAT_COUNT 					// MUST BE LAST
} osd_stats_e;
// 确保统计数据的数量不超过可用的32位存储
STATIC_ASSERT(OSD_STAT_COUNT <= 32, osdstats_overflow);

/* ------------------------OSD警告标志位枚举---------------------- */	
typedef enum {
    OSD_WARNING_ARMING_DISABLE,
    OSD_WARNING_BATTERY_NOT_FULL,
    OSD_WARNING_BATTERY_WARNING,
    OSD_WARNING_BATTERY_CRITICAL,
    OSD_WARNING_VISUAL_BEEPER,
    OSD_WARNING_CRASH_FLIP,
    OSD_WARNING_ESC_FAIL,
    OSD_WARNING_CORE_TEMPERATURE,
    OSD_WARNING_RC_SMOOTHING,
    OSD_WARNING_FAIL_SAFE,
    OSD_WARNING_LAUNCH_CONTROL,
    OSD_WARNING_GPS_RESCUE_UNAVAILABLE,
    OSD_WARNING_GPS_RESCUE_DISABLED,
    OSD_WARNING_RSSI,
    OSD_WARNING_LINK_QUALITY,
    OSD_WARNING_RSSI_DBM,
    OSD_WARNING_COUNT    // MUST BE LAST
} osdWarningsFlags_e;
// 确保警告的数量不超过可用的32位存储空间
STATIC_ASSERT(OSD_WARNING_COUNT <= 32, osdwarnings_overflow);

/* --------------------------OSD单位枚举-------------------------- */	
typedef enum {
    OSD_UNIT_IMPERIAL,
    OSD_UNIT_METRIC
} osd_unit_e;

/* -------------------------OSD定时器枚举------------------------- */	
typedef enum {
    OSD_TIMER_1,
    OSD_TIMER_2,
    OSD_TIMER_COUNT
} osd_timer_e;

/* ------------------------OSD定时器源枚举------------------------ */	
typedef enum {
    OSD_TIMER_SRC_ON,
    OSD_TIMER_SRC_TOTAL_ARMED,
    OSD_TIMER_SRC_LAST_ARMED,
    OSD_TIMER_SRC_ON_OR_ARMED,
    OSD_TIMER_SRC_COUNT
} osd_timer_source_e;

/* ------------------------OSD定时器精度枚举---------------------- */	
typedef enum {
    OSD_TIMER_PREC_SECOND,
    OSD_TIMER_PREC_HUNDREDTHS,
    OSD_TIMER_PREC_TENTHS,
    OSD_TIMER_PREC_COUNT
} osd_timer_precision_e;

/* --------------------------OSD配置结构体------------------------ */	
typedef struct osdConfig_s {
    // 警报
    uint16_t alt_alarm;
    uint8_t rssi_alarm;
    osd_unit_e units;
    uint16_t timers[OSD_TIMER_COUNT];
    uint32_t enabledWarnings;
	uint8_t ahMaxPitch;
    uint8_t ahMaxRoll;
    uint32_t enabled_stats;
    uint8_t core_temp_alarm;
    uint8_t ahInvert;                         // 反转模拟地平线
    uint8_t osdProfileIndex;				  // OSD 配置文件索引
    uint8_t overlay_radio_mode;	
    char profile[OSD_PROFILE_COUNT][OSD_PROFILE_NAME_LENGTH + 1];
    uint16_t link_quality_alarm;
    uint8_t gps_sats_show_hdop;				 
    int8_t rcChannels[OSD_RCCHANNELS_COUNT];  // 显示RC通道值，无则为-1
    uint8_t displayPortDevice;                // 显示端口设备
    uint16_t distance_alarm;				  // 距离警报
    uint8_t logo_on_arming;                   // 显示标志在解锁
    uint8_t logo_on_arming_duration;          // 显示时间以0.1s为单位
} osdConfig_t;
// 声明OSD配置结构体
PG_DECLARE(osdConfig_t, osdConfig);

/* ------------------------OSD元素配置结构体---------------------- */	
typedef struct osdElementConfig_s {
    uint16_t item_pos[OSD_ITEM_COUNT];
} osdElementConfig_t;
// 声明OSD元素配置结构体
PG_DECLARE(osdElementConfig_t, osdElementConfig);

/* --------------------------OSD统计结构体------------------------ */	
typedef struct statistic_s {
    timeUs_t armed_time;					  // 解锁时间
    int16_t max_speed;						  // 最大速度
    int16_t min_voltage;   					  // 最小电压/100
    uint16_t end_voltage;					  // 截止电压
    int16_t max_current;   					  // 最大电流/10
    uint8_t min_rssi;						  // 最小RSSI
    int32_t max_altitude;					  // 最大高度
    int16_t max_distance;					  // 最大距离
    uint16_t min_link_quality;				  // 最小链接质量
    int16_t min_rssi_dbm;					  // 最小RSSI DBM
} statistic_t;


extern const uint16_t osdTimerDefault[OSD_TIMER_COUNT];
extern const osd_stats_e osdStatsDisplayOrder[OSD_STAT_COUNT];
extern timeUs_t resumeRefreshAt;
extern timeUs_t osdFlyTime;
#if defined(USE_ACC)
extern float osdGForce;
#endif
void osdInit(displayPort_t *osdDisplayPort, osdDisplayPortDevice_e displayPortDevice);
bool osdInitialized(void);
void osdUpdate(timeUs_t currentTimeUs);
void osdStatSetState(uint8_t statIndex, bool enabled);
bool osdStatGetState(uint8_t statIndex);
void osdWarnSetState(uint8_t warningIndex, bool enabled);
bool osdWarnGetState(uint8_t warningIndex);
void osdSuppressStats(bool flag);
void osdAnalyzeActiveElements(void);
uint8_t getCurrentOsdProfileIndex(void);
void changeOsdProfileIndex(uint8_t profileIndex);
bool osdElementVisible(uint16_t value);
bool osdGetVisualBeeperState(void);
statistic_t *osdGetStats(void);
bool osdNeedsAccelerometer(void);
displayPort_t *osdGetDisplayPort(osdDisplayPortDevice_e *displayPortDeviceType);

