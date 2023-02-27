/**********************************************************************
 提供一系列OSD操作相关API。
**********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "platform.h"

#ifdef USE_OSD
#include "build/build_config.h"
#include "build/version.h"

#include "cms/cms.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/typeconversion.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/display.h"
#include "drivers/dshot.h"
#include "drivers/osd_symbols.h"
#include "drivers/time.h"

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/position.h"

#include "io/beeper.h"
#include "io/gps.h"

#include "osd/osd.h"
#include "osd/osd_elements.h"

#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"

/* --------------------------OSD_LOGO_ARMING-------------------------- */	
typedef enum {
    OSD_LOGO_ARMING_OFF,
    OSD_LOGO_ARMING_ON,
    OSD_LOGO_ARMING_FIRST
} osd_logo_on_arming_e;

/* --------------------------OSD源定时器名称-------------------------- */
const char * const osdTimerSourceNames[] = {
    "ON TIME  ",
    "TOTAL ARM",
    "LAST ARM ",
    "ON/ARM   "
};

// ---------------------------------------------------------RC数据判断宏 - OSD和CMS中都有
#define IS_HI(X)  (rcData[X] > 1750)                            // 高
#define IS_LO(X)  (rcData[X] < 1250)							// 低
#define IS_MID(X) (rcData[X] > 1250 && rcData[X] < 1750)

// ---------------------------------------------------------飞行时间
timeUs_t osdFlyTime = 0;		

// ---------------------------------------------------------Force
#if defined(USE_ACC)
float osdGForce = 0;
#endif

// ---------------------------------------------------------视觉蜂鸣器
static bool showVisualBeeper = false;

// ---------------------------------------------------------统计
static statistic_t stats;

// ---------------------------------------------------------重新刷新时间
timeUs_t resumeRefreshAt = 0;									
#define REFRESH_1S    1000 * 1000

// ---------------------------------------------------------解锁状态
static uint8_t armState;

// ---------------------------------------------------------OSD显示端口
static displayPort_t *osdDisplayPort;
static osdDisplayPortDevice_e osdDisplayPortDeviceType;

// ---------------------------------------------------------OSD就绪状态
static bool osdIsReady;

// ---------------------------------------------------------显示是否支持统计
static bool suppressStatsDisplay = false;
static uint8_t osdStatsRowCount = 0;

// ---------------------------------------------------------背景层支持
static bool backgroundLayerSupported = false;

// ---------------------------------------------------------控制OSD飞行后统计信息的显示顺序
// osd_stats_e中的每个条目都应该被表示，任何丢失都不会显示在飞行后统计页面
// 如果重新排序数据，可能需要做同样的更新单元测试
// 如果添加新的统计数据，如果统计使用加速度计请添加到osdStatsNeedAccelerometer()函数
const osd_stats_e osdStatsDisplayOrder[OSD_STAT_COUNT] = {
    OSD_STAT_TIMER_1,
    OSD_STAT_TIMER_2,
    OSD_STAT_MAX_ALTITUDE,
    OSD_STAT_MAX_SPEED,
    OSD_STAT_MAX_DISTANCE,
    OSD_STAT_FLIGHT_DISTANCE,
    OSD_STAT_MIN_BATTERY,
    OSD_STAT_END_BATTERY,
    OSD_STAT_BATTERY,
    OSD_STAT_MIN_RSSI,
    OSD_STAT_MAX_CURRENT,
    OSD_STAT_MIN_LINK_QUALITY,
};

// ---------------------------------------------------------OSD 默认时间
const uint16_t osdTimerDefault[OSD_TIMER_COUNT] = {
        OSD_TIMER(OSD_TIMER_SRC_ON, OSD_TIMER_PREC_SECOND, 10),
        OSD_TIMER(OSD_TIMER_SRC_TOTAL_ARMED, OSD_TIMER_PREC_SECOND, 10)
};

STATIC_ASSERT(OSD_POS_MAX == OSD_POS(31,31), OSD_POS_MAX_incorrect);
PG_REGISTER_WITH_RESET_FN(osdConfig_t, osdConfig, PG_OSD_CONFIG, 8);
void pgResetFn_osdConfig(osdConfig_t *osdConfig)
{
    // 启用默认数值统计
    osdConfig->enabled_stats = 0; 				     	  			// 将所有设置为“关闭”，初始时只启用一部分
	osdStatSetState(OSD_STAT_MAX_CURRENT, true);	 	  			// 电池电流最大值
	osdStatSetState(OSD_STAT_BATTERY, true);	     	  			// 电池电压
	osdStatSetState(OSD_STAT_END_BATTERY, true);     	  			// 电池电压截止值
	osdStatSetState(OSD_STAT_MIN_BATTERY, true);     	  			// 电池电压最小值
	osdStatSetState(OSD_STAT_FLIGHT_DISTANCE, true); 	  			// 飞行距离
	osdStatSetState(OSD_STAT_TIMER_2, true);         	  			// 计时器2（统计解锁时间）
	osdStatSetState(OSD_STAT_MAX_DISTANCE, true);	 	  			// 最大飞行距离
	osdStatSetState(OSD_STAT_MIN_LINK_QUALITY, true); 	  			// 连接质量最小值
	osdStatSetState(OSD_STAT_MAX_SPEED, true);	     	  			// 最大飞行速度
    osdStatSetState(OSD_STAT_TOTAL_DIST, true);	     	  			// 总飞行距离
    osdStatSetState(OSD_STAT_TOTAL_TIME, true);	     	  			// 总飞行时间
    osdStatSetState(OSD_STAT_MAX_ALTITUDE, true);	 	  			// 最大高度
    osdStatSetState(OSD_STAT_MIN_RSSI, true);		 	  			// RSSI最小值
	// OSD单位：公制
    osdConfig->units = OSD_UNIT_METRIC;

    // 默认情况下启用所有警告
    for (int i=0; i < OSD_WARNING_COUNT; i++) {
        osdWarnSetState(i, true);
    }
    // 默认关闭RSSI_DBM和链路质量警告
    osdWarnSetState(OSD_WARNING_LINK_QUALITY, false);
    osdWarnSetState(OSD_WARNING_RSSI_DBM, false);

    osdConfig->timers[OSD_TIMER_1] = osdTimerDefault[OSD_TIMER_1];
    osdConfig->timers[OSD_TIMER_2] = osdTimerDefault[OSD_TIMER_2];

    osdConfig->overlay_radio_mode = 2;

    osdConfig->rssi_alarm = 40;							  			// 警告RSSI
    osdConfig->link_quality_alarm = 70;					  			// 警告链接质量
    osdConfig->alt_alarm  = 200; 						  			// 警告高度
    osdConfig->core_temp_alarm = 70; 					  			// 温度超过70℃应产生警告，在80℃以上已报告锁存

    osdConfig->ahMaxPitch = 20; 						  			// 20 degrees
    osdConfig->ahMaxRoll = 40; 				              			// 40 degrees

    osdConfig->osdProfileIndex = 1;						  			// 默认OSD配置文件
    osdConfig->ahInvert = false;
    for (int i=0; i < OSD_PROFILE_COUNT; i++) {
        osdConfig->profile[i][0] = '\0';
    }
    osdConfig->gps_sats_show_hdop = false;

    for (int i = 0; i < OSD_RCCHANNELS_COUNT; i++) {
        osdConfig->rcChannels[i] = -1;
    }

    osdConfig->displayPortDevice = OSD_DISPLAYPORT_DEVICE_MAX7456;  // 显示端口设备

    osdConfig->distance_alarm = 0;
    osdConfig->logo_on_arming = OSD_LOGO_ARMING_OFF;
    osdConfig->logo_on_arming_duration = 5;  			  			// 0.5 seconds
}

PG_REGISTER_WITH_RESET_FN(osdElementConfig_t, osdElementConfig, PG_OSD_ELEMENT_CONFIG, 0);
void pgResetFn_osdElementConfig(osdElementConfig_t *osdElementConfig)
{
    // 将元素放置在屏幕中心附近，默认情况下禁用
    for (int i = 0; i < OSD_ITEM_COUNT; i++) {
        osdElementConfig->item_pos[i] = OSD_POS(10, 7);
    }

    // 默认启用的元素 - 所有OSD配置文件
    uint16_t profileFlags = 0;
    for (unsigned i = 1; i <= OSD_PROFILE_COUNT; i++) {
        profileFlags |= OSD_PROFILE_FLAG(i);
    }
	
	osdElementConfig->item_pos[OSD_WARNINGS]  = OSD_POS(9, 10) | profileFlags;          // 警告

	// 默认启用的元素 - OSD配置文件1
	uint16_t profile1Flags = 0;    // OSD配置文件1
	profile1Flags |= OSD_PROFILE_FLAG(1);
	
 	osdElementConfig->item_pos[OSD_RSSI_VALUE] = OSD_POS(4, 5)| profile1Flags;          // RSSI值
	osdElementConfig->item_pos[OSD_LINK_QUALITY] = OSD_POS(4, 6)| profile1Flags;        // 连接质量
	osdElementConfig->item_pos[OSD_THROTTLE_POS] = OSD_POS(3, 7)| profile1Flags;        // 油门位置

	osdElementConfig->item_pos[OSD_DISARMED]  = OSD_POS(10, 5) | profile1Flags;         // 已上锁
    osdElementConfig->item_pos[OSD_CROSSHAIRS]         = OSD_POS(13, 6)| profile1Flags; // 十字准星
    osdElementConfig->item_pos[OSD_ARTIFICIAL_HORIZON] = OSD_POS(14, 2)| profile1Flags; // 模拟地平线
    osdElementConfig->item_pos[OSD_FLIP_ARROW]  = OSD_POS(10, 7) | profile1Flags;       // 反乌龟箭头
	
	osdElementConfig->item_pos[OSD_GPS_SPEED] = OSD_POS(22, 5) | profile1Flags;         // GPS速度
	osdElementConfig->item_pos[OSD_ALTITUDE]  = OSD_POS(22, 6) | profile1Flags;         // 海报高度
	osdElementConfig->item_pos[OSD_NUMERICAL_VARIO]  = OSD_POS(22, 7) | profile1Flags;  // 数字式垂直速度表

	osdElementConfig->item_pos[OSD_VTX_CHANNEL]  = OSD_POS(21, 1) | profile1Flags;      // 图传频道
	osdElementConfig->item_pos[OSD_GPS_LON]  = OSD_POS(2, 12) | profile1Flags;          // GPS经度
	osdElementConfig->item_pos[OSD_GPS_LAT]  = OSD_POS(16, 12) | profile1Flags;         // GPS纬度

	osdElementConfig->item_pos[OSD_ITEM_TIMER_2]  = OSD_POS(2, 11) | profile1Flags;     // 定时器2
	osdElementConfig->item_pos[OSD_MAIN_BATT_VOLTAGE]  = OSD_POS(2, 10) | profile1Flags;// 电池总电压
	osdElementConfig->item_pos[OSD_CURRENT_DRAW]  = OSD_POS(1, 9) | profile1Flags;      // 实时电流计

	osdElementConfig->item_pos[OSD_GPS_SATS]  = OSD_POS(2, 1) | profile1Flags;          // GPS卫星数
	osdElementConfig->item_pos[OSD_FLIGHT_DIST]  = OSD_POS(2, 2) | profile1Flags;       // 飞行总距离

	osdElementConfig->item_pos[OSD_COMPASS_BAR]  = OSD_POS(10, 1) | profile1Flags;      // 罗盘标尺

	osdElementConfig->item_pos[OSD_FLYMODE]  = OSD_POS(21, 11) | profile1Flags;         // 飞行模式

	osdElementConfig->item_pos[OSD_HOME_DIR]  = OSD_POS(18, 3) | profile1Flags;         // 到起飞点方向
	osdElementConfig->item_pos[OSD_HOME_DIST]  = OSD_POS(20, 3) | profile1Flags;        // 到起飞点距离

	osdElementConfig->item_pos[OSD_PIDRATE_PROFILE]  = OSD_POS(26, 11) | profile1Flags;  // PID和RATE文件名称
	
	// 这些元素默认为旧的固定位置
    osdElementConfig->item_pos[OSD_HORIZON_SIDEBARS]   = OSD_POS(14, 6);                // 模拟地平线侧标尺
}


//-------------------------------------------------------------------------------------OSD初始化相关API

/**********************************************************************
函数名称：osdDrawLogo
函数功能：绘制开机LOGO
函数形参：坐标
函数返回值：None
函数描述：
	大小必须是 288 x72 像素.(每个字符12*18 - 24*4个字符)
**********************************************************************/
static void osdDrawLogo(int x, int y)
{
    int fontOffset = 160;									// 从第160行开始
    for (int row = 0; row < 4; row++) {						// 行		
        for (int column = 0; column < 24; column++) {		// 列 
            if (fontOffset <= SYM_END_OF_FONT)
                displayWriteChar(osdDisplayPort, x + column, y + row, DISPLAYPORT_ATTR_NONE, fontOffset++);
        }
    }
}

/**********************************************************************
函数名称：osdCompleteInitialization
函数功能：osd完成初始化
函数形参：None
函数返回值：None
函数描述：None 
**********************************************************************/
static void osdCompleteInitialization(void)
{
	// 获取解锁状态
    armState = ARMING_FLAG(ARMED);
	// 复位警报
    osdResetAlarms();
	// 获取背景层是否支持
    backgroundLayerSupported = displayLayerSupported(osdDisplayPort, DISPLAYPORT_LAYER_BACKGROUND);
	// 选择显示层 - 前景
    displayLayerSelect(osdDisplayPort, DISPLAYPORT_LAYER_FOREGROUND);
	// 清除屏幕
    displayClearScreen(osdDisplayPort);

	// 绘制开机LOGO
    osdDrawLogo(3, 2);

    char string_buffer[30];
	// 显示固件版本
    tfp_sprintf(string_buffer, "V%s", FC_VERSION_STRING);
    displayWrite(osdDisplayPort, 20, 6, DISPLAYPORT_ATTR_NONE, string_buffer);
#ifdef USE_CMS
	// 显示进入OSD菜单帮助
    displayWrite(osdDisplayPort, 7, 8,  DISPLAYPORT_ATTR_NONE, CMS_STARTUP_HELP_TEXT1);
    displayWrite(osdDisplayPort, 11, 9, DISPLAYPORT_ATTR_NONE, CMS_STARTUP_HELP_TEXT2);
    displayWrite(osdDisplayPort, 11, 10, DISPLAYPORT_ATTR_NONE, CMS_STARTUP_HELP_TEXT3);
#endif

    resumeRefreshAt = micros() + (4 * REFRESH_1S);
	// OSD元素初始化
    osdElementsInit(backgroundLayerSupported);
	// 分析OSD激活元素
    osdAnalyzeActiveElements();

    osdIsReady = true;
}

/**********************************************************************
函数名称：osdInit
函数功能：osd初始化
函数形参：显示端口结构体，显示设备
函数返回值：None
函数描述：
	初始化过程配置OSD时注册显示端口设备。
**********************************************************************/
void osdInit(displayPort_t *osdDisplayPortToUse, osdDisplayPortDevice_e displayPortDeviceType)
{
	// 获取显示端口设备类型
    osdDisplayPortDeviceType = displayPortDeviceType;
    if (!osdDisplayPortToUse) {
        return;
    }
	// 获取显示端口
    osdDisplayPort = osdDisplayPortToUse;
	// CMS显示端口注册
#ifdef USE_CMS
    cmsDisplayPortRegister(osdDisplayPort);
#endif
	// OSD 完成初始化
    if (displayIsReady(osdDisplayPort)) {
        osdCompleteInitialization();
    }
}

//-------------------------------------------------------------------------------------OSD显示端口相关API

/**********************************************************************
函数名称：osdInitialized
函数功能：获取OSD显示端口
函数形参：None
函数返回值：None
函数描述：None 
**********************************************************************/
bool osdInitialized(void)
{
    return osdDisplayPort;
}

/**********************************************************************
函数名称：osdGetDisplayPort
函数功能：获取OSD显示端口
函数形参：displayPortDeviceType
函数返回值：osdDisplayPort
函数描述：None
**********************************************************************/
displayPort_t *osdGetDisplayPort(osdDisplayPortDevice_e *displayPortDeviceType)
{
    if (displayPortDeviceType) {
        *displayPortDeviceType = osdDisplayPortDeviceType;
    }
    return osdDisplayPort;
}

//-------------------------------------------------------------------------------------OSD元素相关API

/**********************************************************************
函数名称：osdAnalyzeActiveElements
函数功能：分析OSD激活元素
函数形参：None
函数返回值：None
函数描述：None 
**********************************************************************/
void osdAnalyzeActiveElements(void)
{
	// 检查元素并构建一个只包含激活元素的列表加速渲染
    osdAddActiveElements();
	// 绘制激活元素背景 
    osdDrawActiveElementsBackground(osdDisplayPort);
}

/**********************************************************************
函数名称：osdDrawElements
函数功能：绘制OSD元素
函数形参：None
函数返回值：None
函数描述：None 
**********************************************************************/
static void osdDrawElements(timeUs_t currentTimeUs)
{
    // OSDSW模式（隐藏OSD画面）为开启时，隐藏OSD
    if (IS_RC_MODE_ACTIVE(BOXOSD)) {
		// 清除屏幕
        displayClearScreen(osdDisplayPort);
        return;
    }

	// 背景图层覆盖到前景层
    if (backgroundLayerSupported) {
		// 复制显示层 - BACKGROUND->FOREGROUND
        displayLayerCopy(osdDisplayPort, DISPLAYPORT_LAYER_FOREGROUND, DISPLAYPORT_LAYER_BACKGROUND);
    } 
	// 背景层不支持，只清除前景，用于绘制元素及其背景
	else {
		// 清除屏幕
        displayClearScreen(osdDisplayPort);
    }

	// OSD绘制激活元素
    osdDrawActiveElements(osdDisplayPort, currentTimeUs);
}

//-------------------------------------------------------------------------------------OSD警告相关API

/**********************************************************************
函数名称：osdWarnSetState
函数功能：设置OSD警告状态
函数形参：警告索引，使能状态
函数返回值：None  
函数描述：None 
**********************************************************************/
void osdWarnSetState(uint8_t warningIndex, bool enabled)
{
    if (enabled) {
        osdConfigMutable()->enabledWarnings |= (1 << warningIndex);
    } else {
        osdConfigMutable()->enabledWarnings &= ~(1 << warningIndex);
    }
}

/**********************************************************************
函数名称：osdWarnGetState
函数功能：获取OSD警告状态
函数形参：警告索引
函数返回值：警告索引
函数描述：None 
**********************************************************************/
bool osdWarnGetState(uint8_t warningIndex)
{
    return osdConfig()->enabledWarnings & (1 << warningIndex);
}

//-------------------------------------------------------------------------------------OSD统计相关API

/**********************************************************************
函数名称：osdStatSetState
函数功能：设置OSD统计状态
函数形参：统计索引，使能状态
函数返回值：None  
函数描述：None
**********************************************************************/
void osdStatSetState(uint8_t statIndex, bool enabled)
{
    if (enabled) {
        osdConfigMutable()->enabled_stats |= (1 << statIndex);
    } else {
        osdConfigMutable()->enabled_stats &= ~(1 << statIndex);
    }
}

/**********************************************************************
函数名称：osdStatGetState
函数功能：获取OSD统计状态
函数形参：统计索引
函数返回值：统计状态
函数描述：None 
**********************************************************************/
bool osdStatGetState(uint8_t statIndex)
{
    return osdConfig()->enabled_stats & (1 << statIndex);
}

/**********************************************************************
函数名称：osdResetStats
函数功能：OSD统计复位
函数形参：None
函数返回值：None
函数描述：None 
**********************************************************************/
static void osdResetStats(void)
{
    stats.max_current  = 0;
    stats.max_speed    = 0;
    stats.min_voltage  = 5000;
    stats.end_voltage  = 0;
    stats.min_rssi     = 99; // percent
    stats.max_altitude = 0;
    stats.max_distance = 0;
    stats.armed_time   = 0;
    stats.min_link_quality =  (linkQualitySource == LQ_SOURCE_RX_PROTOCOL_CRSF) ? 100 : 99; // percent
    stats.min_rssi_dbm = 20;
}

/**********************************************************************
函数名称：osdUpdateStats
函数功能：osd更新统计数据
函数形参：None
函数返回值：None
函数描述：None 
**********************************************************************/
static void osdUpdateStats(void)
{
    int16_t value = 0;

#ifdef USE_GPS
    if (gpsConfig()->gps_use_3d_speed) {
        value = gpsSol.speed3d;
    } else {
        value = gpsSol.groundSpeed;
    }
    if (stats.max_speed < value) {
        stats.max_speed = value;
    }
#endif

    value = getBatteryVoltage();
    if (stats.min_voltage > value) {
        stats.min_voltage = value;
    }

    value = getAmperage() / 100;
    if (stats.max_current < value) {
        stats.max_current = value;
    }

    value = getRssiPercent();
    if (stats.min_rssi > value) {
        stats.min_rssi = value;
    }

    int32_t altitudeCm = getEstimatedAltitudeCm();
    if (stats.max_altitude < altitudeCm) {
        stats.max_altitude = altitudeCm;
    }

#ifdef USE_RX_LINK_QUALITY_INFO
    value = rxGetLinkQualityPercent();
    if (stats.min_link_quality > value) {
        stats.min_link_quality = value;
    }
#endif

#ifdef USE_GPS
    if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
        if (stats.max_distance < GPS_distanceToHome) {
            stats.max_distance = GPS_distanceToHome;
        }
    }
#endif
}

/**********************************************************************
函数名称：osdDisplayStatisticLabel
函数功能：显示统计标签
函数形参：y，text，value
函数返回值：None
函数描述：None 
**********************************************************************/
static void osdDisplayStatisticLabel(uint8_t y, const char * text, const char * value)
{
    displayWrite(osdDisplayPort, 2, y, DISPLAYPORT_ATTR_NONE, text);
    displayWrite(osdDisplayPort, 20, y, DISPLAYPORT_ATTR_NONE, ":");
    displayWrite(osdDisplayPort, 22, y, DISPLAYPORT_ATTR_NONE, value);
}

/**********************************************************************
函数名称：isSomeStatEnabled
函数功能：获取是否有一些统计启用
函数形参：None
函数返回值：是否有一些状态启用
函数描述：None 
**********************************************************************/
static bool isSomeStatEnabled(void)
{
    return (osdConfig()->enabled_stats != 0);
}

/**********************************************************************
函数名称：osdDisplayStat
函数功能：显示统计数据
函数形参：None
函数返回值：状态
函数描述：
	 之前需要统计显示顺序来匹配枚举定义，以便它匹配配置器中显示的顺序。
	 但是，为了允许重新排序这个屏幕而不破坏兼容性，
	 这个要求已经放宽到最好的努力的方法。
	 重新排序的元素在统计屏幕上将会比不完全匹配的麻烦更有益配置列表。
**********************************************************************/
static bool osdDisplayStat(int statistic, uint8_t displayRow)
{
    char buff[OSD_ELEMENT_BUFFER_LENGTH];

    switch (statistic) {
	    case OSD_STAT_TIMER_1:
	        osdFormatTimer(buff, false, (OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_1]) == OSD_TIMER_SRC_ON ? false : true), OSD_TIMER_1);
	        osdDisplayStatisticLabel(displayRow, osdTimerSourceNames[OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_1])], buff);
	        return true;

	    case OSD_STAT_TIMER_2:
	        osdFormatTimer(buff, false, (OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_2]) == OSD_TIMER_SRC_ON ? false : true), OSD_TIMER_2);
	        osdDisplayStatisticLabel(displayRow, osdTimerSourceNames[OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_2])], buff);
	        return true;

	    case OSD_STAT_MAX_ALTITUDE: {
	        const int alt = osdGetMetersToSelectedUnit(stats.max_altitude) / 10;
	        tfp_sprintf(buff, "%d.%d%c", alt / 10, alt % 10, osdGetMetersToSelectedUnitSymbol());
	        osdDisplayStatisticLabel(displayRow, "MAX ALTITUDE", buff);
	        return true;
	    }

#ifdef USE_GPS
	    case OSD_STAT_MAX_SPEED:
	        if (featureIsEnabled(FEATURE_GPS)) {
	            tfp_sprintf(buff, "%d%c", osdGetSpeedToSelectedUnit(stats.max_speed), osdGetSpeedToSelectedUnitSymbol());
	            osdDisplayStatisticLabel(displayRow, "MAX SPEED", buff);
	            return true;
	        }
	        break;

	    case OSD_STAT_MAX_DISTANCE:
	        if (featureIsEnabled(FEATURE_GPS)) {
	            osdFormatDistanceString(buff, stats.max_distance, SYM_NONE);
	            osdDisplayStatisticLabel(displayRow, "MAX DISTANCE", buff);
	            return true;
	        }
	        break;

	    case OSD_STAT_FLIGHT_DISTANCE:
	        if (featureIsEnabled(FEATURE_GPS)) {
	            const int distanceFlown = GPS_distanceFlownInCm / 100;
	            osdFormatDistanceString(buff, distanceFlown, SYM_NONE);
	            osdDisplayStatisticLabel(displayRow, "FLIGHT DISTANCE", buff);
	            return true;
	        }
	        break;
#endif

	    case OSD_STAT_MIN_BATTERY:
	        tfp_sprintf(buff, "%d.%02d%c", stats.min_voltage / 100, stats.min_voltage % 100, SYM_VOLT);
	        osdDisplayStatisticLabel(displayRow, "MIN BATTERY", buff);
	        return true;

	    case OSD_STAT_END_BATTERY:
	        tfp_sprintf(buff, "%d.%02d%c", stats.end_voltage / 100, stats.end_voltage % 100, SYM_VOLT);
	        osdDisplayStatisticLabel(displayRow, "END BATTERY", buff);
	        return true;

	    case OSD_STAT_BATTERY:
	        tfp_sprintf(buff, "%d.%02d%c", getBatteryVoltage() / 100, getBatteryVoltage() % 100, SYM_VOLT);
	        osdDisplayStatisticLabel(displayRow, "BATTERY", buff);
	        return true;

	    case OSD_STAT_MIN_RSSI:
	        itoa(stats.min_rssi, buff, 10);
	        strcat(buff, "%");
	        osdDisplayStatisticLabel(displayRow, "MIN RSSI", buff);
	        return true;

	    case OSD_STAT_MAX_CURRENT:
	        if (batteryConfig()->currentMeterSource != CURRENT_METER_NONE) {
	            tfp_sprintf(buff, "%d%c", stats.max_current, SYM_AMP);
	            osdDisplayStatisticLabel(displayRow, "MAX CURRENT", buff);
	            return true;
	        }
	        break;

#ifdef USE_RX_LINK_QUALITY_INFO
	    case OSD_STAT_MIN_LINK_QUALITY:
	        tfp_sprintf(buff, "%d", stats.min_link_quality);
	        strcat(buff, "%");
	        osdDisplayStatisticLabel(displayRow, "MIN LINK", buff);
	        return true;
#endif
    }
    return false;
}

/**********************************************************************
函数名称：osdShowStats
函数功能：OSD显示统计
函数形参：statsRowCount
函数返回值：top
函数描述：None 
**********************************************************************/
static uint8_t osdShowStats(int statsRowCount)
{
    uint8_t top = 0;
    bool displayLabel = false;

    // 如果statsRowCount为0，那么我们正在对活动的stats项进行初始分析
    if (statsRowCount > 0) {
        const int availableRows = osdDisplayPort->rows;
        int displayRows = MIN(statsRowCount, availableRows);
        if (statsRowCount < availableRows) {
            displayLabel = true;
            displayRows++;
        }
        top = (availableRows - displayRows) / 2;  // center the stats vertically
    }

    if (displayLabel) {
        displayWrite(osdDisplayPort, 2, top++, DISPLAYPORT_ATTR_NONE, "  --- STATS ---");
    }

    for (int i = 0; i < OSD_STAT_COUNT; i++) {
        if (osdStatGetState(osdStatsDisplayOrder[i])) {
            if (osdDisplayStat(osdStatsDisplayOrder[i], top)) {
                top++;
            }
        }
    }
    return top;
}

/**********************************************************************
函数名称：osdRefreshStats
函数功能：osd刷新统计数据
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdRefreshStats(void)
{
    displayClearScreen(osdDisplayPort);
    if (osdStatsRowCount == 0) {
		// 没有设置统计行数，遍历逻辑一次，确定实际显示了多少统计信息
        osdStatsRowCount = osdShowStats(0);
		// 然后清除屏幕并开始正常的统计显示，确定标题是否应该显示，内容垂直居中
        displayClearScreen(osdDisplayPort);
    }
    osdShowStats(osdStatsRowCount);
}

/**********************************************************************
函数名称：osdGetStats
函数功能：osd获取统计
函数形参：None
函数返回值：stats
函数描述：None
**********************************************************************/
statistic_t *osdGetStats(void)
{
    return &stats;
}

/**********************************************************************
函数名称：osdSuppressStats
函数功能：osd禁止统计数据
函数形参：flag
函数返回值：None  
函数描述：None
**********************************************************************/
void osdSuppressStats(bool flag)
{
    suppressStatsDisplay = flag;
}

//-------------------------------------------------------------------------------------OSD锁定状态显示相关API

/**********************************************************************
函数名称：osdShowArmed
函数功能：OSD显示锁定状态
函数形参：None 
函数返回值：ret
函数描述：None 
**********************************************************************/
static timeDelta_t osdShowArmed(void)
{
    timeDelta_t ret;

    displayClearScreen(osdDisplayPort);

    if ((osdConfig()->logo_on_arming == OSD_LOGO_ARMING_ON) || ((osdConfig()->logo_on_arming == OSD_LOGO_ARMING_FIRST) && !ARMING_FLAG(WAS_EVER_ARMED))) {
        osdDrawLogo(3, 1);
        ret = osdConfig()->logo_on_arming_duration * 1e5;
    } else {
        ret = (REFRESH_1S / 2);
    }
    displayWrite(osdDisplayPort, 12, 7, DISPLAYPORT_ATTR_NONE, "ARMED");

    return ret;
}

//-------------------------------------------------------------------------------------OSD视觉蜂鸣器相关API

/**********************************************************************
函数名称：osdGetVisualBeeperState
函数功能：osd获取视觉蜂鸣器状态
函数形参：None
函数返回值：showVisualBeeper
函数描述：None
**********************************************************************/
bool osdGetVisualBeeperState(void)
{
    return showVisualBeeper;
}

//-------------------------------------------------------------------------------------检查ACC需求相关API

#ifdef USE_ACC
/**********************************************************************
函数名称：osdNeedsAccelerometer
函数功能：检查是否有任何启用的元素或状态需要ACC
函数形参：None
函数返回值：None
函数描述：
	用于获取加速度计是否有校准需求。
**********************************************************************/
bool osdNeedsAccelerometer(void)
{
    return osdElementsNeedAccelerometer();
}
#endif // USE_ACC

//-------------------------------------------------------------------------------------OSD刷新相关API

/**********************************************************************
函数名称：osdUpdate
函数功能：OSD刷新
函数形参：当前节拍时间
函数返回值：None  
函数描述：None
**********************************************************************/
STATIC_UNIT_TESTED void osdRefresh(timeUs_t currentTimeUs)
{
    static timeUs_t lastTimeUs = 0;
    static bool osdStatsEnabled = false;
    static bool osdStatsVisible = false;
    static timeUs_t osdStatsRefreshTimeUs;

	// ----------------------------------------------判断OSD是否就绪 - 是否在初始化阶段
    if (!osdIsReady) {
		// 显示设备是否就绪 - 初始化阶段设备检测结果
        if (!displayIsReady(osdDisplayPort)) {
			// 如果显示设备未就绪 - 重新同步并return
            displayResync(osdDisplayPort);
            return;
        }
		// osd完成初始化 - 开机界面&&相关初始化
        osdCompleteInitialization();
    }

    // ----------------------------------------------解锁状态有变动 - 数据统计相关
    if (armState != ARMING_FLAG(ARMED)) {	
		// 已解锁
        if (ARMING_FLAG(ARMED)) {
			// 关闭统计状态显示
            osdStatsEnabled = false;
            osdStatsVisible = false;
			// OSD统计复位
            osdResetStats();
			// OSD显示锁定状态
            resumeRefreshAt = osdShowArmed() + currentTimeUs;
        } 
		// 获取是否有统计启用          - 如果起飞失控触发锁定和警告元素可见，禁止统计
		else if (isSomeStatEnabled() && !suppressStatsDisplay 
			      && (!(getArmingDisableFlags() & (ARMING_DISABLED_RUNAWAY_TAKEOFF | ARMING_DISABLED_CRASH_DETECTED))
                  || !VISIBLE(osdElementConfig()->item_pos[OSD_WARNINGS]))) { 
			// 显示OSD统计信息
			osdStatsEnabled = true;
            resumeRefreshAt = currentTimeUs + (60 * REFRESH_1S);
            stats.end_voltage = getBatteryVoltage();
			// 重置为0，所以它将在下一次统计刷新时重新计算
            osdStatsRowCount = 0;    
        }

		// 更新解锁状态
        armState = ARMING_FLAG(ARMED);
    }

	// 已解锁
    if (ARMING_FLAG(ARMED)) {
		// 更新飞行统计数据
        osdUpdateStats();
		// 更新时差
        timeUs_t deltaT = currentTimeUs - lastTimeUs;
		// 更新飞行时间
        osdFlyTime += deltaT;
		// 更新解锁时间
        stats.armed_time += deltaT;
    } 
	// 判断统计信息是否显示
	else if (osdStatsEnabled) {  
        if (displayIsGrabbed(osdDisplayPort)) {
            osdStatsEnabled = false;
            resumeRefreshAt = 0;
            stats.armed_time = 0;
        } else {
			// 判断是否通过SW禁用OSD显示
            if (IS_RC_MODE_ACTIVE(BOXOSD) && osdStatsVisible) {
                osdStatsVisible = false;
				// 清除屏幕
                displayClearScreen(osdDisplayPort);
            } else if (!IS_RC_MODE_ACTIVE(BOXOSD)) {
                if (!osdStatsVisible) {
                    osdStatsVisible = true;
                    osdStatsRefreshTimeUs = 0;
                }
                if (currentTimeUs >= osdStatsRefreshTimeUs) {
                    osdStatsRefreshTimeUs = currentTimeUs + REFRESH_1S;
					// 刷新统计数据
                    osdRefreshStats();
                }
            }
        }
    }
    lastTimeUs = currentTimeUs;

    if (resumeRefreshAt) {
		// 未超时
        if (cmp32(currentTimeUs, resumeRefreshAt) < 0) {
            // 检查激活恢复显示的摇杆操作（油门高 || 俯仰高）
            if (IS_HI(THROTTLE) || IS_HI(PITCH)) {
                resumeRefreshAt = currentTimeUs;
            }
            return;
        } 
		// 超时
		else {
			// 清除屏幕
            displayClearScreen(osdDisplayPort);
            resumeRefreshAt = 0;
			// 关闭统计信息显示
            osdStatsEnabled = false;
			// 解锁时间重置
            stats.armed_time = 0;
        }
    }

	// ----------------------------------------------元素绘制
#ifdef USE_CMS
    if (!displayIsGrabbed(osdDisplayPort))
#endif
    {
    	// 更新OSD警报
        osdUpdateAlarms();
		// 绘制OSD元素
        osdDrawElements(currentTimeUs);
    }
}

/**********************************************************************
函数名称：osdUpdate
函数功能：OSD更新
函数形参：当前节拍时间
函数返回值：None  
函数描述：
	由调度器以任务形式调用.
**********************************************************************/
void osdUpdate(timeUs_t currentTimeUs)
{
    static uint32_t counter = 0;

	// -------------------------------视觉蜂鸣器
    if (isBeeperOn()) {
        showVisualBeeper = true;
    }

    // -------------------------------绘制激活层缓冲区
    // 刷新频率分母
	#define DRAW_FREQ_DENOM 5      
    if (counter % DRAW_FREQ_DENOM == 0) {
		// OSD刷新 - 写入显存
        osdRefresh(currentTimeUs);
        showVisualBeeper = false;
    } else {
        bool doDrawScreen = true;
        if (doDrawScreen) {
			// 获取激活层显存 - 和影子寄存器缓存做对比来判断缓存是否有变化
			// 绘制被更改部分的显存 - MAX7456
			// 分散负载和SPI总线的利用率
            displayDrawScreen(osdDisplayPort);
        }
    }
    ++counter;
}
#endif // USE_OSD

