/**********************************************************************
 提供一系列OSD元素相关API。
	绘制应该只渲染的动态部分元素。
	静态(不变的)元素，在背景函数中呈现。
	例外：警告闪烁。
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

#include "common/axis.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/typeconversion.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/display.h"
#include "drivers/dshot.h"
#include "drivers/osd_symbols.h"
#include "drivers/time.h"
#include "drivers/vtx_common.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/gps_rescue.h"
#include "flight/failsafe.h"
#include "flight/position.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/beeper.h"
#include "io/gps.h"
#include "io/vtx.h"

#include "osd/osd.h"
#include "osd/osd_elements.h"

#include "pg/motor.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"


// ---------------------------------------------------------AH
#define AH_SYMBOL_COUNT 9
#define AH_SIDEBAR_WIDTH_POS 7
#define AH_SIDEBAR_HEIGHT_POS 3

// ---------------------------------------------------------满循环
#define FULL_CIRCLE 360

// ---------------------------------------------------------罗盘标尺
static const char compassBar[] = {
  SYM_HEADING_W,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE,
  SYM_HEADING_N,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE,
  SYM_HEADING_E,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE,
  SYM_HEADING_S,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE,
  SYM_HEADING_W,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE,
  SYM_HEADING_N,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE
};

// ---------------------------------------------------------激活元素
// ------激活元素数量
static unsigned activeOsdElementCount = 0;
// ------缓存
static uint8_t activeOsdElementArray[OSD_ITEM_COUNT];
// ------是否支持背景层
static bool backgroundLayerSupported = false;

// ---------------------------------------------------------闪烁控制
// 闪烁状态 - 在osdDrawActiveElements函数中进行计算
static bool blinkState = true;
// 闪烁位缓存
static uint32_t blinkBits[(OSD_ITEM_COUNT + 31) / 32];
// ---------------------------------------------------------设置闪烁
#define SET_BLINK(item) (blinkBits[(item) / 32] |= (1 << ((item) % 32)))     
// ---------------------------------------------------------清除闪烁
#define CLR_BLINK(item) (blinkBits[(item) / 32] &= ~(1 << ((item) % 32)))    
// ---------------------------------------------------------获取闪烁状态
#define IS_BLINK(item) (blinkBits[(item) / 32] & (1 << ((item) % 32)))       
// ---------------------------------------------------------获取闪烁状态 && 闪烁使能
#define BLINK(item) (IS_BLINK(item) && blinkState)							    


//-------------------------------------------------------------------------------------字符操作相关API

/**********************************************************************
函数名称：osdDisplayWrite
函数功能：OSD显示端口写操作 - 写到显存
函数形参：元素，X坐标，Y坐标，属性，内容
函数返回值：displayWrite
函数描述：None 
**********************************************************************/
static int osdDisplayWrite(osdElementParms_t *element, uint8_t x, uint8_t y, uint8_t attr, const char *s)
{		
	// 获取闪烁状态
    if (IS_BLINK(element->item)) {
        attr |= DISPLAYPORT_ATTR_BLINK;
    }
    return displayWrite(element->osdDisplayPort, x, y, attr, s);
}

/**********************************************************************
函数名称：osdDisplayWriteChar
函数功能：OSD显示端口写字符
函数形参：元素，X坐标，Y坐标，属性，内容
函数返回值：displayWrite
函数描述：None 
**********************************************************************/
static int osdDisplayWriteChar(osdElementParms_t *element, uint8_t x, uint8_t y, uint8_t attr, char c)
{
    char buf[2];
    buf[0] = c;
    buf[1] = 0;
    return osdDisplayWrite(element, x, y, attr, buf);
}

/**********************************************************************
函数名称：osdFormatAltitudeString
函数功能：高度字符串的osd形式
函数形参：buff，altitudeCm
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdFormatAltitudeString(char * buff, int32_t altitudeCm)
{
    const int alt = osdGetMetersToSelectedUnit(altitudeCm) / 10;
    int pos = 0;
    buff[pos++] = SYM_ALTITUDE;
    if (alt < 0) {
        buff[pos++] = '-';
    }
    tfp_sprintf(buff + pos, "%01d.%01d%c", abs(alt) / 10 , abs(alt) % 10, osdGetMetersToSelectedUnitSymbol());
}

/**********************************************************************
函数名称：osdFormatCoordinate
函数功能：osd高度字符串格式
函数形参：buff，altitudeCm
函数返回值：None 
函数描述：None 
**********************************************************************/
#ifdef USE_GPS
static void osdFormatCoordinate(char *buff, char sym, int32_t val)
{
	// 纬度最大整数宽度为3(-90)。
	// 最大经度整数宽度为4(-180)。
	// 显示7个小数，因此需要使用12个字符:
	// s=符号，z=零终结符，0和1之间的十进制分隔符
    int pos = 0;
    buff[pos++] = sym;
    if (val < 0) {
        buff[pos++] = '-';
        val = -val;
    }
    tfp_sprintf(buff + pos, "%d.%07d", val / GPS_DEGREES_DIVIDER, val % GPS_DEGREES_DIVIDER);
}
#endif // USE_GPS

/**********************************************************************
函数名称：osdFormatDistanceString
函数功能：osd距离字符串格式
函数形参：ptr，distance，leadingSymbol
函数返回值：None 
函数描述：None 
**********************************************************************/
void osdFormatDistanceString(char *ptr, int distance, char leadingSymbol)
{
    const int convertedDistance = osdGetMetersToSelectedUnit(distance);
    char unitSymbol;
    char unitSymbolExtended;
    int unitTransition;

    if (leadingSymbol != SYM_NONE) {
        *ptr++ = leadingSymbol;
    }
    switch (osdConfig()->units) {
    case OSD_UNIT_IMPERIAL:
        unitTransition = 5280;
        unitSymbol = SYM_FT;
        unitSymbolExtended = SYM_MILES;
        break;
    default:
        unitTransition = 1000;
        unitSymbol = SYM_M;
        unitSymbolExtended = SYM_KM;
        break;
    }
    if (convertedDistance < unitTransition) {
        tfp_sprintf(ptr, "%d%c", convertedDistance, unitSymbol);
    } else {
        const int displayDistance = convertedDistance * 100 / unitTransition;
        if (displayDistance >= 1000) { // >= 10 miles or km - 1 decimal place
            tfp_sprintf(ptr, "%d.%d%c", displayDistance / 100, (displayDistance / 10) % 10, unitSymbolExtended);
        } else {                       // < 10 miles or km - 2 decimal places
            tfp_sprintf(ptr, "%d.%02d%c", displayDistance / 100, displayDistance % 100, unitSymbolExtended);
        }
    }
}

//-------------------------------------------------------------------------------------定时器相关API

/**********************************************************************
函数名称：osdFormatTime
函数功能：osd时间格式
函数形参：buff，精度，时间
函数返回值：None 
函数描述：None 
**********************************************************************/
void osdFormatTime(char * buff, osd_timer_precision_e precision, timeUs_t time)
{
    int seconds = time / 1000000;
    const int minutes = seconds / 60;
    seconds = seconds % 60;

    switch (precision) {
	    case OSD_TIMER_PREC_SECOND:
	    default:
	        tfp_sprintf(buff, "%02d:%02d", minutes, seconds);
	        break;
	    case OSD_TIMER_PREC_HUNDREDTHS:
	        {
	            const int hundredths = (time / 10000) % 100;
	            tfp_sprintf(buff, "%02d:%02d.%02d", minutes, seconds, hundredths);
	            break;
	        }
	    case OSD_TIMER_PREC_TENTHS:
	        {
	            const int tenths = (time / 100000) % 10;
	            tfp_sprintf(buff, "%02d:%02d.%01d", minutes, seconds, tenths);
	            break;
	        }
    }
}

/**********************************************************************
函数名称：osdGetTimerSymbol
函数功能：osd获取定时器符号
函数形参：src
函数返回值：定时器符号
函数描述：None 
**********************************************************************/
static char osdGetTimerSymbol(osd_timer_source_e src)
{
    switch (src) {
	    case OSD_TIMER_SRC_ON:
	        return SYM_ON_M;
	    case OSD_TIMER_SRC_TOTAL_ARMED:
	    case OSD_TIMER_SRC_LAST_ARMED:
	        return SYM_FLY_M;
	    case OSD_TIMER_SRC_ON_OR_ARMED:
	        return ARMING_FLAG(ARMED) ? SYM_FLY_M : SYM_ON_M;
	    default:
	        return ' ';
    }
}

/**********************************************************************
函数名称：osdGetTimerValue
函数功能：osd获取定时器值
函数形参：src
函数返回值：定时器值
函数描述：None 
**********************************************************************/
static timeUs_t osdGetTimerValue(osd_timer_source_e src)
{
    switch (src) {
	    case OSD_TIMER_SRC_ON:
	        return micros();
	    case OSD_TIMER_SRC_TOTAL_ARMED:
	        return osdFlyTime;
	    case OSD_TIMER_SRC_LAST_ARMED: {
	        statistic_t *stats = osdGetStats();
	        return stats->armed_time;
	    }
	    case OSD_TIMER_SRC_ON_OR_ARMED:
	        return ARMING_FLAG(ARMED) ? osdFlyTime : micros();
	    default:
	        return 0;
    }
}

/**********************************************************************
函数名称：osdFormatTimer
函数功能：osd定时器格式
函数形参：buff，showSymbol，usePrecision，timerIndex
函数返回值：None
函数描述：None 
**********************************************************************/
void osdFormatTimer(char *buff, bool showSymbol, bool usePrecision, int timerIndex)
{
    const uint16_t timer = osdConfig()->timers[timerIndex];
    const uint8_t src = OSD_TIMER_SRC(timer);
    if (showSymbol) {
        *(buff++) = osdGetTimerSymbol(src);
    }
    osdFormatTime(buff, (usePrecision ? OSD_TIMER_PRECISION(timer) : OSD_TIMER_PREC_SECOND), osdGetTimerValue(src));
}

//-------------------------------------------------------------------------------------获取符号相关API

/**********************************************************************
函数名称：osdGetBatterySymbol
函数功能：osd获取电池符号
函数形参：cellVoltage
函数返回值：SYM_BATT_EMPTY - constrain(symOffset, 0, 6)
函数描述：None 
**********************************************************************/
static char osdGetBatterySymbol(int cellVoltage)
{
	// 目前的BAT符号，理想情况下替换为一个带有感叹号的电池
    if (getBatteryState() == BATTERY_CRITICAL) {
        return SYM_MAIN_BATT; 
    } else {
        // 使用电池电压在全电池电压范围内计算符号偏移量
        const int symOffset = scaleRange(cellVoltage, batteryConfig()->vbatmincellvoltage, batteryConfig()->vbatmaxcellvoltage, 0, 8);
        return SYM_BATT_EMPTY - constrain(symOffset, 0, 6);
    }
}

/**********************************************************************
函数名称：osdGetHeadingIntoDiscreteDirections
函数功能：osd获取回家的方向
函数形参：heading，directions
函数返回值：段号
函数描述：None 
**********************************************************************/
static uint8_t osdGetHeadingIntoDiscreteDirections(int heading, unsigned directions)
{
    heading += FULL_CIRCLE;  // Ensure positive value
	// 分割输入标题0..359分成扇区0..(方向-1)，但偏移
	// 半个扇区，使扇区0以0为中心。
	// 我们将标题与方向相乘，以保证划分时精度不松散
	// 通过这种方式，每个段将是一个FULL_CIRCLE的长度
    int direction = (heading * directions + FULL_CIRCLE / 2) /  FULL_CIRCLE; // scale with rounding
    direction %= directions; // normalize
	// 返回段数
    return direction;   
}

/**********************************************************************
函数名称：osdGetDirectionSymbolFromHeading
函数功能：osd获取航向符号
函数形参：heading
函数返回值：SYM_ARROW_SOUTH + heading
函数描述：None 
**********************************************************************/
static uint8_t osdGetDirectionSymbolFromHeading(int heading)
{
    heading = osdGetHeadingIntoDiscreteDirections(heading, 16);
	// 现在的箭头有 向上=0，右=4，下=8和左=12
	// 符号是Down=0, Right=4, Up=8, Left=12
	// 有16个箭头符号改变
    heading = 16 - heading;
    heading = (heading + 8) % 16;
    return SYM_ARROW_SOUTH + heading;
}

/**********************************************************************
函数名称：osdGetMetersToSelectedUnit
函数功能：转换高度基于当前单位系统
函数形参：meters
函数返回值：转换高度
函数描述：None 
**********************************************************************/
int32_t osdGetMetersToSelectedUnit(int32_t meters)
{
    switch (osdConfig()->units) {
	    case OSD_UNIT_IMPERIAL:
	        return (meters * 328) / 100; // Convert to feet / 100
	    default:
	        return meters;               // Already in metre / 100
    }
}

/**********************************************************************
函数名称：osdGetMetersToSelectedUnitSymbol
函数功能：获取当前单元系统的正确高度符号
函数形参：None 
函数返回值：正确高度符号
函数描述：None 
**********************************************************************/
char osdGetMetersToSelectedUnitSymbol(void)
{
    switch (osdConfig()->units) {
	    case OSD_UNIT_IMPERIAL:
	        return SYM_FT;
	    default:
	        return SYM_M;
    }
}

/**********************************************************************
函数名称：osdGetSpeedToSelectedUnit
函数功能：转换速度基于当前单位系统
函数形参：要转换的cm/s值
函数返回值：转换速度
函数描述：None 
**********************************************************************/
int32_t osdGetSpeedToSelectedUnit(int32_t value)
{
    switch (osdConfig()->units) {
	    case OSD_UNIT_IMPERIAL:
	        return CM_S_TO_MPH(value);
	    default:
	        return CM_S_TO_KM_H(value);
    }
}

/**********************************************************************
函数名称：osdGetSpeedToSelectedUnitSymbol
函数功能：获取当前单元系统的正确速度符号
函数形参：None 
函数返回值：正确速度符号
函数描述：None 
**********************************************************************/
char osdGetSpeedToSelectedUnitSymbol(void)
{
    switch (osdConfig()->units) {
	    case OSD_UNIT_IMPERIAL:
	        return SYM_MPH;
	    default:
	        return SYM_KPH;
    }
}

/**********************************************************************
函数名称：osdGetVarioToSelectedUnitSymbol
函数功能：获取当前单元系统的正确垂直速度符号
函数形参：None 
函数返回值：正确垂直速度符号
函数描述：None 
**********************************************************************/
char osdGetVarioToSelectedUnitSymbol(void)
{
    switch (osdConfig()->units) {
	    case OSD_UNIT_IMPERIAL:
	        return SYM_FTPS;
	    default:
	        return SYM_MPS;
    }
}

//-------------------------------------------------------------------------------------元素绘制函数
// 第一种情况 - 将要显示的字符存放到对应的元素缓存中。
// 第二种情况 - 直接写到显存中。

/**********************************************************************
函数名称：osdElementAltitude
函数功能：OSD高度元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdElementAltitude(osdElementParms_t *element)
{
    bool haveBaro = false;
    bool haveGps = false;
#ifdef USE_BARO
    haveBaro = sensors(SENSOR_BARO);
#endif // USE_BARO
#ifdef USE_GPS
    haveGps = sensors(SENSOR_GPS) && STATE(GPS_FIX);
#endif // USE_GPS
    if (haveBaro || haveGps) {
		// 高度字符串的osd形式
        osdFormatAltitudeString(element->buff, getEstimatedAltitudeCm());
    } else {
        element->buff[0] = SYM_ALTITUDE;
        element->buff[1] = SYM_HYPHEN;     
		// 当没有有效的值时，使用这个符号
        element->buff[2] = '\0';
    }
}

/**********************************************************************
函数名称：osdElementArtificialHorizon
函数功能：OSD模拟地平线元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
#ifdef USE_ACC
static void osdElementArtificialHorizon(osdElementParms_t *element)
{
    // 俯仰和滚转限制以十分之一度为单位
    const int maxPitch = osdConfig()->ahMaxPitch * 10;
    const int maxRoll = osdConfig()->ahMaxRoll * 10;
    const int ahSign = osdConfig()->ahInvert ? -1 : 1;
	// 角度约束
    const int rollAngle = constrain(attitude.values.roll * ahSign, -maxRoll, maxRoll);
    int pitchAngle = constrain(attitude.values.pitch * ahSign, -maxPitch, maxPitch);
	
	// 将pitchAngle转换为y补偿值
	// (maxPitch / 25)除数匹配之前的固定除数为8和固定最大AHI俯仰角20.0度的设置
    if (maxPitch > 0) {
        pitchAngle = ((pitchAngle * 25) / maxPitch);
    }
	// 41 = 4 * AH_SYMBOL_COUNT + 5
    pitchAngle -= 41; 		       

	// 绘制9个字符
	// x坐标为固定9个字符 - 计算对应x坐标上y坐标的值
    for (int x = -4; x <= 4; x++) {
		// 计算y坐标
        const int y = ((-rollAngle * x) / 64) - pitchAngle;
        if (y >= 0 && y <= 81) {
            osdDisplayWriteChar(element, element->elemPosX + x, element->elemPosY + (y / AH_SYMBOL_COUNT), DISPLAYPORT_ATTR_NONE, (SYM_AH_BAR9_0 + (y % AH_SYMBOL_COUNT)));
        }
    }
	// 元素已绘制
    element->drawElement = false;  
}
#endif // USE_ACC

/**********************************************************************
函数名称：osdElementCompassBar
函数功能：OSD磁力计标尺元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdElementCompassBar(osdElementParms_t *element)
{
    memcpy(element->buff, compassBar + osdGetHeadingIntoDiscreteDirections(DECIDEGREES_TO_DEGREES(attitude.values.yaw), 16), 9);
    element->buff[9] = 0;
}

/**********************************************************************
函数名称：osdElementCrashFlipArrow
函数功能：OSD反乌龟箭头元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
#ifdef USE_ACC
static void osdElementCrashFlipArrow(osdElementParms_t *element)
{
    int rollAngle = attitude.values.roll / 10;
    const int pitchAngle = attitude.values.pitch / 10;
    if (abs(rollAngle) > 90) {
        rollAngle = (rollAngle < 0 ? -180 : 180) - rollAngle;
    }

    if ((isFlipOverAfterCrashActive() || (!ARMING_FLAG(ARMED) && !isUpright())) && !((imuConfig()->small_angle < 180 && isUpright()) || (rollAngle == 0 && pitchAngle == 0))) {
        if (abs(pitchAngle) < 2 * abs(rollAngle) && abs(rollAngle) < 2 * abs(pitchAngle)) {
            if (pitchAngle > 0) {
                if (rollAngle > 0) {
                    element->buff[0] = SYM_ARROW_WEST + 2;
                } else {
                    element->buff[0] = SYM_ARROW_EAST - 2;
                }
            } else {
                if (rollAngle > 0) {
                    element->buff[0] = SYM_ARROW_WEST - 2;
                } else {
                    element->buff[0] = SYM_ARROW_EAST + 2;
                }
            }
        } else {
            if (abs(pitchAngle) > abs(rollAngle)) {
                if (pitchAngle > 0) {
                    element->buff[0] = SYM_ARROW_SOUTH;
                } else {
                    element->buff[0] = SYM_ARROW_NORTH;
                }
            } else {
                if (rollAngle > 0) {
                    element->buff[0] = SYM_ARROW_WEST;
                } else {
                    element->buff[0] = SYM_ARROW_EAST;
                }
            }
        }
        element->buff[1] = '\0';
    }
}
#endif // USE_ACC

/**********************************************************************
函数名称：osdBackgroundCrosshairs
函数功能：OSD十字准星背景
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdBackgroundCrosshairs(osdElementParms_t *element)
{
    element->buff[0] = SYM_AH_CENTER_LINE;
    element->buff[1] = SYM_AH_CENTER;
    element->buff[2] = SYM_AH_CENTER_LINE_RIGHT;
    element->buff[3] = 0;
}

/**********************************************************************
函数名称：osdElementCurrentDraw
函数功能：OSD电流计元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdElementCurrentDraw(osdElementParms_t *element)
{
    const int32_t amperage = getAmperage();
    tfp_sprintf(element->buff, "%3d.%02d%c", abs(amperage) / 100, abs(amperage) % 100, SYM_AMP);
}

/**********************************************************************
函数名称：osdElementDisarmed
函数功能：OSD解锁元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdElementDisarmed(osdElementParms_t *element)
{
    if (!ARMING_FLAG(ARMED)) {
        tfp_sprintf(element->buff, "DISARMED");
    }
}

/**********************************************************************
函数名称：osdElementFlymode
函数功能：OSD飞行模式元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdElementFlymode(osdElementParms_t *element)
{
    // 注意：飞行模式的显示优先于其他要显示的内容
    if (FLIGHT_MODE(FAILSAFE_MODE)) {
        strcpy(element->buff, "!FS!");
    } else if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        strcpy(element->buff, "RESC");
    } else if (FLIGHT_MODE(ANGLE_MODE)) {
        strcpy(element->buff, "ANGL");
    } else if (airmodeIsEnabled()) {
        strcpy(element->buff, "AIR ");
    } else {
        strcpy(element->buff, "ACRO");
    }
}

/**********************************************************************
函数名称：osdElementGpsFlightDistance
函数功能：OSD GPS飞行总距离元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
#ifdef USE_GPS
static void osdElementGpsFlightDistance(osdElementParms_t *element)
{
    if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
        osdFormatDistanceString(element->buff, GPS_distanceFlownInCm / 100, SYM_TOTAL_DISTANCE);
    } else {
        // We use this symbol when we don't have a FIX
        tfp_sprintf(element->buff, "%c%c", SYM_TOTAL_DISTANCE, SYM_HYPHEN);
    }
}

/**********************************************************************
函数名称：osdElementGpsHomeDirection
函数功能：OSD GPS距离家的方向元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdElementGpsHomeDirection(osdElementParms_t *element)
{
    if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
        if (GPS_distanceToHome > 0) {
            const int h = GPS_directionToHome - DECIDEGREES_TO_DEGREES(attitude.values.yaw);
            element->buff[0] = osdGetDirectionSymbolFromHeading(h);
        } else {
            element->buff[0] = SYM_OVER_HOME;
        }
    } else {
        // We use this symbol when we don't have a FIX
        element->buff[0] = SYM_HYPHEN;
    }

    element->buff[1] = 0;
}

/**********************************************************************
函数名称：osdElementGpsHomeDistance
函数功能：OSD GPS距离家的距离元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdElementGpsHomeDistance(osdElementParms_t *element)
{
    if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
        osdFormatDistanceString(element->buff, GPS_distanceToHome, SYM_HOMEFLAG);
    } else {
        element->buff[0] = SYM_HOMEFLAG;
        // We use this symbol when we don't have a FIX
        element->buff[1] = SYM_HYPHEN;
        element->buff[2] = '\0';
    }
}

/**********************************************************************
函数名称：osdElementGpsLatitude
函数功能：OSD GPS经度元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdElementGpsLatitude(osdElementParms_t *element)
{
    osdFormatCoordinate(element->buff, SYM_LAT, gpsSol.llh.lat);
}

/**********************************************************************
函数名称：osdElementGpsLongitude
函数功能：OSD GPS纬度元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdElementGpsLongitude(osdElementParms_t *element)
{
    osdFormatCoordinate(element->buff, SYM_LON, gpsSol.llh.lon);
}

/**********************************************************************
函数名称：osdElementGpsSats
函数功能：OSD GPS星数元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdElementGpsSats(osdElementParms_t *element)
{
    if (osdConfig()->gps_sats_show_hdop) {
        tfp_sprintf(element->buff, "%c%c%2d %d.%d", SYM_SAT_L, SYM_SAT_R, gpsSol.numSat, gpsSol.hdop / 100, (gpsSol.hdop / 10) % 10);
    } else {
        tfp_sprintf(element->buff, "%c%c%2d", SYM_SAT_L, SYM_SAT_R, gpsSol.numSat);
    }
}

/**********************************************************************
函数名称：osdElementGpsSpeed
函数功能：OSD GPS速度元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdElementGpsSpeed(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "%c%3d%c", SYM_SPEED, osdGetSpeedToSelectedUnit(gpsConfig()->gps_use_3d_speed ? gpsSol.speed3d : gpsSol.groundSpeed), osdGetSpeedToSelectedUnitSymbol());
}
#endif // USE_GPS

/**********************************************************************
函数名称：osdBackgroundHorizonSidebars
函数功能：OSD地平线侧边栏背景
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdBackgroundHorizonSidebars(osdElementParms_t *element)
{
    // Draw AH sides
    const int8_t hudwidth = AH_SIDEBAR_WIDTH_POS;
    const int8_t hudheight = AH_SIDEBAR_HEIGHT_POS;
    for (int y = -hudheight; y <= hudheight; y++) {
        osdDisplayWriteChar(element, element->elemPosX - hudwidth, element->elemPosY + y, DISPLAYPORT_ATTR_NONE, SYM_AH_DECORATION);
        osdDisplayWriteChar(element, element->elemPosX + hudwidth, element->elemPosY + y, DISPLAYPORT_ATTR_NONE, SYM_AH_DECORATION);
    }

    // AH level indicators
    osdDisplayWriteChar(element, element->elemPosX - hudwidth + 1, element->elemPosY, DISPLAYPORT_ATTR_NONE, SYM_AH_LEFT);
    osdDisplayWriteChar(element, element->elemPosX + hudwidth - 1, element->elemPosY, DISPLAYPORT_ATTR_NONE, SYM_AH_RIGHT);

    element->drawElement = false;  // element already drawn
}

/**********************************************************************
函数名称：osdElementLinkQuality
函数功能：OSD连接质量元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
#ifdef USE_RX_LINK_QUALITY_INFO
static void osdElementLinkQuality(osdElementParms_t *element)
{
    uint16_t osdLinkQuality = 0;
	// 0-99
    if (linkQualitySource == LQ_SOURCE_RX_PROTOCOL_CRSF) { 
        osdLinkQuality = rxGetLinkQuality();
        const uint8_t osdRfMode = rxGetRfMode();
        tfp_sprintf(element->buff, "%c%1d:%2d", SYM_LINK_QUALITY, osdRfMode, osdLinkQuality);
    } else { // 0-9
        osdLinkQuality = rxGetLinkQuality() * 10 / LINK_QUALITY_MAX_VALUE;
        if (osdLinkQuality >= 10) {
            osdLinkQuality = 9;
        }
        tfp_sprintf(element->buff, "%c%1d", SYM_LINK_QUALITY, osdLinkQuality);
    }
}
#endif // USE_RX_LINK_QUALITY_INFO

/**********************************************************************
函数名称：osdElementMainBatteryVoltage
函数功能：OSD电池电压元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdElementMainBatteryVoltage(osdElementParms_t *element)
{
    int batteryVoltage = getBatteryVoltage();

    element->buff[0] = osdGetBatterySymbol(getBatteryAverageCellVoltage());
    if (batteryVoltage >= 1000) {
        batteryVoltage = (batteryVoltage + 5) / 10;
        tfp_sprintf(element->buff + 1, "%d.%d%c", batteryVoltage / 10, batteryVoltage % 10, SYM_VOLT);
    } else {
        tfp_sprintf(element->buff + 1, "%d.%02d%c", batteryVoltage / 100, batteryVoltage % 100, SYM_VOLT);
    }
}

/**********************************************************************
函数名称：osdElementNumericalVario
函数功能：OSD数字式垂直速度表元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
#ifdef USE_VARIO
static void osdElementNumericalVario(osdElementParms_t *element)
{
    bool haveBaro = false;
    bool haveGps = false;
#ifdef USE_BARO
    haveBaro = sensors(SENSOR_BARO);
#endif // USE_BARO
#ifdef USE_GPS
    haveGps = sensors(SENSOR_GPS) && STATE(GPS_FIX);
#endif // USE_GPS
    if (haveBaro || haveGps) {
        const int verticalSpeed = osdGetMetersToSelectedUnit(getEstimatedVario());
        const char directionSymbol = verticalSpeed < 0 ? SYM_ARROW_SMALL_DOWN : SYM_ARROW_SMALL_UP;
        tfp_sprintf(element->buff, "%c%01d.%01d%c", directionSymbol, abs(verticalSpeed / 100), abs((verticalSpeed % 100) / 10), osdGetVarioToSelectedUnitSymbol());
    } else {
        // We use this symbol when we don't have a valid measure
        element->buff[0] = SYM_HYPHEN;
        element->buff[1] = '\0';
    }
}
#endif // USE_VARIO

/**********************************************************************
函数名称：osdElementPidRateProfile
函数功能：OSD 速率、PID文件元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdElementPidRateProfile(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "%d-%d", getCurrentPidProfileIndex() + 1, getCurrentControlRateProfileIndex() + 1);
}

/**********************************************************************
函数名称：osdElementRssi
函数功能：OSD RSSI元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdElementRssi(osdElementParms_t *element)
{
	// change range
    uint16_t osdRssi = getRssi() * 100 / 1024; 
    if (osdRssi >= 100) {
        osdRssi = 99;
    }

    tfp_sprintf(element->buff, "%c%2d", SYM_RSSI, osdRssi);
}

/**********************************************************************
函数名称：osdElementThrottlePosition
函数功能：OSD油门位置元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdElementThrottlePosition(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "%c%3d", SYM_THR, calculateThrottlePercent());
}

/**********************************************************************
函数名称：osdElementTimer
函数功能：OSD定时器元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdElementTimer(osdElementParms_t *element)
{
    osdFormatTimer(element->buff, true, true, element->item - OSD_ITEM_TIMER_1);
}

/**********************************************************************
函数名称：osdElementVtxChannel
函数功能：OSD图传通道元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
#ifdef USE_VTX_COMMON
static void osdElementVtxChannel(osdElementParms_t *element)
{
    const vtxDevice_t *vtxDevice = vtxCommonDevice();
    const char vtxBandLetter = vtxCommonLookupBandLetter(vtxDevice, vtxSettingsConfig()->band);
    const char *vtxChannelName = vtxCommonLookupChannelName(vtxDevice, vtxSettingsConfig()->channel);
    unsigned vtxStatus = 0;
    uint8_t vtxPower = vtxSettingsConfig()->power;
    if (vtxDevice) {
        vtxCommonGetStatus(vtxDevice, &vtxStatus);

        if (vtxSettingsConfig()->lowPowerDisarm) {
            vtxCommonGetPowerIndex(vtxDevice, &vtxPower);
        }
    }
    const char *vtxPowerLabel = vtxCommonLookupPowerName(vtxDevice, vtxPower);

    char vtxStatusIndicator = '\0';
    if (vtxStatus & VTX_STATUS_PIT_MODE) {
        vtxStatusIndicator = 'P';
    }

    if (vtxStatus & VTX_STATUS_LOCKED) {
        tfp_sprintf(element->buff, "-:-:-:L");
    } else if (vtxStatusIndicator) {
        tfp_sprintf(element->buff, "%c:%s:%s:%c", vtxBandLetter, vtxChannelName, vtxPowerLabel, vtxStatusIndicator);
    } else {
        tfp_sprintf(element->buff, "%c:%s:%s", vtxBandLetter, vtxChannelName, vtxPowerLabel);
    }
}
#endif // USE_VTX_COMMON

/**********************************************************************
函数名称：osdElementWarnings
函数功能：OSD警告元素
函数形参：element
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdElementWarnings(osdElementParms_t *element)
{
	#define OSD_WARNINGS_MAX_SIZE 12
	#define OSD_FORMAT_MESSAGE_BUFFER_SIZE (OSD_WARNINGS_MAX_SIZE + 1)

    STATIC_ASSERT(OSD_FORMAT_MESSAGE_BUFFER_SIZE <= OSD_ELEMENT_BUFFER_LENGTH, osd_warnings_size_exceeds_buffer_size);

    const batteryState_e batteryState = getBatteryState();
    const timeUs_t currentTimeUs = micros();

    static timeUs_t armingDisabledUpdateTimeUs;
    static unsigned armingDisabledDisplayIndex;

    CLR_BLINK(OSD_WARNINGS);

    // 循环通过解除解锁的原因
    if (osdWarnGetState(OSD_WARNING_ARMING_DISABLE)) {
        if (IS_RC_MODE_ACTIVE(BOXARM) && isArmingDisabled()) {
            const armingDisableFlags_e armSwitchOnlyFlag = 1 << (ARMING_DISABLE_FLAGS_COUNT - 1);
            armingDisableFlags_e flags = getArmingDisableFlags();

            // 除去解锁开关的标志，除非它是唯一的
            if ((flags & armSwitchOnlyFlag) && (flags != armSwitchOnlyFlag)) {
                flags -= armSwitchOnlyFlag;
            }

			// 延迟0.5秒后旋转到下一个解除解锁的原因
			// 如果当前标志不再被设置
            if ((currentTimeUs - armingDisabledUpdateTimeUs > 5e5) || !(flags & (1 << armingDisabledDisplayIndex))) {
                if (armingDisabledUpdateTimeUs == 0) {
                    armingDisabledDisplayIndex = ARMING_DISABLE_FLAGS_COUNT - 1;
                }
                armingDisabledUpdateTimeUs = currentTimeUs;

                do {
                    if (++armingDisabledDisplayIndex >= ARMING_DISABLE_FLAGS_COUNT) {
                        armingDisabledDisplayIndex = 0;
                    }
                } while (!(flags & (1 << armingDisabledDisplayIndex)));
            }

            tfp_sprintf(element->buff, "%s", armingDisableFlagNames[armingDisabledDisplayIndex]);
            element->attr = DISPLAYPORT_ATTR_WARNING;
            return;
        } else {
            armingDisabledUpdateTimeUs = 0;
        }
    }

#ifdef USE_DSHOT
    if (isTryingToArm() && !ARMING_FLAG(ARMED)) {
        int armingDelayTime = (getLastDshotBeaconCommandTimeUs() + DSHOT_BEACON_GUARD_DELAY_US - currentTimeUs) / 1e5;
        if (armingDelayTime < 0) {
            armingDelayTime = 0;
        }
        if (armingDelayTime >= (DSHOT_BEACON_GUARD_DELAY_US / 1e5 - 5)) {
            tfp_sprintf(element->buff, " BEACON ON"); // 在前0.5秒显示此消息
        } else {
            tfp_sprintf(element->buff, "ARM IN %d.%d", armingDelayTime / 10, armingDelayTime % 10);
        }
        element->attr = DISPLAYPORT_ATTR_INFO;
        return;
    }
#endif // USE_DSHOT
    if (osdWarnGetState(OSD_WARNING_FAIL_SAFE) && failsafeIsActive()) {
        tfp_sprintf(element->buff, "FAIL SAFE");
        element->attr = DISPLAYPORT_ATTR_CRITICAL;
        SET_BLINK(OSD_WARNINGS);
        return;
    }

    // 当在崩溃模式后翻转时发出警告
    if (osdWarnGetState(OSD_WARNING_CRASH_FLIP) && isFlipOverAfterCrashActive()) {
        tfp_sprintf(element->buff, "CRASH FLIP");
        element->attr = DISPLAYPORT_ATTR_INFO;
        return;
    }

    // RSSI
    if (osdWarnGetState(OSD_WARNING_RSSI) && (getRssiPercent() < osdConfig()->rssi_alarm)) {
        tfp_sprintf(element->buff, "RSSI LOW");
        element->attr = DISPLAYPORT_ATTR_WARNING;
        SET_BLINK(OSD_WARNINGS);
        return;
    }

#ifdef USE_RX_LINK_QUALITY_INFO
    // Link Quality
    if (osdWarnGetState(OSD_WARNING_LINK_QUALITY) && (rxGetLinkQualityPercent() < osdConfig()->link_quality_alarm)) {
        tfp_sprintf(element->buff, "LINK QUALITY");
        element->attr = DISPLAYPORT_ATTR_WARNING;
        SET_BLINK(OSD_WARNINGS);
        return;
    }
#endif // USE_RX_LINK_QUALITY_INFO

    if (osdWarnGetState(OSD_WARNING_BATTERY_CRITICAL) && batteryState == BATTERY_CRITICAL) {
        tfp_sprintf(element->buff, " LAND NOW");
        element->attr = DISPLAYPORT_ATTR_CRITICAL;
        SET_BLINK(OSD_WARNINGS);
        return;
    }

#ifdef USE_GPS_RESCUE
	// GPS救援不可用
    if (osdWarnGetState(OSD_WARNING_GPS_RESCUE_UNAVAILABLE) &&
       ARMING_FLAG(ARMED) &&
       gpsRescueIsConfigured() &&
       !gpsRescueIsDisabled() &&
       !gpsRescueIsAvailable()) {
        tfp_sprintf(element->buff, "RESCUE N/A");
        element->attr = DISPLAYPORT_ATTR_WARNING;
        SET_BLINK(OSD_WARNINGS);
        return;
    }

	// GPS救援失效
    if (osdWarnGetState(OSD_WARNING_GPS_RESCUE_DISABLED) &&
       ARMING_FLAG(ARMED) &&
       gpsRescueIsConfigured() &&
       gpsRescueIsDisabled()) {

        statistic_t *stats = osdGetStats();
        if (cmpTimeUs(stats->armed_time, OSD_GPS_RESCUE_DISABLED_WARNING_DURATION_US) < 0) {
            tfp_sprintf(element->buff, "RESCUE OFF");
            element->attr = DISPLAYPORT_ATTR_WARNING;
            SET_BLINK(OSD_WARNINGS);
            return;
        }
    }
#endif // USE_GPS_RESCUE

	// 电池电压低 
    if (osdWarnGetState(OSD_WARNING_BATTERY_WARNING) && batteryState == BATTERY_WARNING) {
        tfp_sprintf(element->buff, "LOW BATTERY");
        element->attr = DISPLAYPORT_ATTR_WARNING;
        SET_BLINK(OSD_WARNINGS);
        return;
    }

    // 电池不满
    if (osdWarnGetState(OSD_WARNING_BATTERY_NOT_FULL) && !(ARMING_FLAG(ARMED) || ARMING_FLAG(WAS_EVER_ARMED)) && (getBatteryState() == BATTERY_OK)
          && getBatteryAverageCellVoltage() < batteryConfig()->vbatfullcellvoltage) {
        tfp_sprintf(element->buff, "BATT < FULL");
        element->attr = DISPLAYPORT_ATTR_INFO;
        return;
    }

    // 视觉蜂鸣器
    if (osdWarnGetState(OSD_WARNING_VISUAL_BEEPER) && osdGetVisualBeeperState()) {
        tfp_sprintf(element->buff, "  * * * *");
        element->attr = DISPLAYPORT_ATTR_INFO;
        return;
    }

}

//-------------------------------------------------------------------------------------定义元素绘制的顺序
// 在列表中位置较晚的元素将覆盖较早的元素，如果字符位置重叠，则为1
// 添加需要特殊运行时条件处理的元素 osdAddActiveElements()
static const uint8_t osdElementDisplayOrder[] = {
    OSD_MAIN_BATT_VOLTAGE,			// 电压
    OSD_RSSI_VALUE,					// RSSI
    OSD_CROSSHAIRS,					// 十字准星
    OSD_HORIZON_SIDEBARS,			// 地平线侧尺
    OSD_ITEM_TIMER_1,				// 定时器1
    OSD_ITEM_TIMER_2,				// 定时器2
    OSD_FLYMODE,					// 飞行模式
    OSD_THROTTLE_POS,				// 油门位置
    OSD_VTX_CHANNEL,				// 图传通道
    OSD_CURRENT_DRAW,				// 电流
    OSD_ALTITUDE,					// 高度
    OSD_PIDRATE_PROFILE,			// PID文件
    OSD_WARNINGS,					// 警告
    OSD_DISARMED,					// 锁定
#ifdef USE_VARIO
    OSD_NUMERICAL_VARIO,			// 垂直速度
#endif
    OSD_COMPASS_BAR,				// 罗盘
#ifdef USE_ACC
    OSD_FLIP_ARROW,					// 反乌龟箭头
#endif
#ifdef USE_RX_LINK_QUALITY_INFO
    OSD_LINK_QUALITY,				// 链接质量
#endif
};
//-------------------------------------------------------------------------------------OSD元素id和它的绘制函数之间的映射（动态元素）
const osdElementDrawFn osdElementDrawFunction[OSD_ITEM_COUNT] = {
    [OSD_RSSI_VALUE]              = osdElementRssi,
    [OSD_MAIN_BATT_VOLTAGE]       = osdElementMainBatteryVoltage,
    [OSD_CROSSHAIRS]              = NULL,  // only has background
#ifdef USE_ACC
    [OSD_ARTIFICIAL_HORIZON]      = osdElementArtificialHorizon,
#endif
    [OSD_HORIZON_SIDEBARS]        = NULL,  // only has background
    [OSD_ITEM_TIMER_1]            = osdElementTimer,
    [OSD_ITEM_TIMER_2]            = osdElementTimer,
    [OSD_FLYMODE]                 = osdElementFlymode,
    [OSD_THROTTLE_POS]            = osdElementThrottlePosition,
#ifdef USE_VTX_COMMON
    [OSD_VTX_CHANNEL]             = osdElementVtxChannel,
#endif
    [OSD_CURRENT_DRAW]            = osdElementCurrentDraw,
#ifdef USE_GPS
    [OSD_GPS_SPEED]               = osdElementGpsSpeed,
    [OSD_GPS_SATS]                = osdElementGpsSats,
#endif
    [OSD_ALTITUDE]                = osdElementAltitude,
    [OSD_PIDRATE_PROFILE]         = osdElementPidRateProfile,
    [OSD_WARNINGS]                = osdElementWarnings,
#ifdef USE_GPS
    [OSD_GPS_LON]                 = osdElementGpsLongitude,
    [OSD_GPS_LAT]                 = osdElementGpsLatitude,
#endif
    [OSD_DISARMED]                = osdElementDisarmed,
#ifdef USE_GPS
    [OSD_HOME_DIR]                = osdElementGpsHomeDirection,
    [OSD_HOME_DIST]               = osdElementGpsHomeDistance,
#endif
#ifdef USE_VARIO
    [OSD_NUMERICAL_VARIO]         = osdElementNumericalVario,
#endif
    [OSD_COMPASS_BAR]             = osdElementCompassBar,
#ifdef USE_ACC
    [OSD_FLIP_ARROW]              = osdElementCrashFlipArrow,
#endif
#ifdef USE_RX_LINK_QUALITY_INFO
    [OSD_LINK_QUALITY]            = osdElementLinkQuality,
#endif
#ifdef USE_GPS
    [OSD_FLIGHT_DIST]             = osdElementGpsFlightDistance,
#endif
#ifdef USE_PROFILE_NAMES
#endif
};
//-------------------------------------------------------------------------------------OSD元素id和绘制其背景函数的映射(静态元素)
// 只需要定义实际具有后台函数的条目
const osdElementDrawFn osdElementBackgroundFunction[OSD_ITEM_COUNT] = {
    [OSD_CROSSHAIRS]              = osdBackgroundCrosshairs,
    [OSD_HORIZON_SIDEBARS]        = osdBackgroundHorizonSidebars,
};

//-------------------------------------------------------------------------------------元素操作相关API

/**********************************************************************
函数名称：osdAddActiveElement
函数功能：添加激活元素
函数形参：element
函数返回值：None 
函数描述：
	激活元素缓存列表。
**********************************************************************/
static void osdAddActiveElement(osd_items_e element)
{
    if (VISIBLE(osdElementConfig()->item_pos[element])) {
        activeOsdElementArray[activeOsdElementCount++] = element;
    }
}

/**********************************************************************
函数名称：osdAddActiveElements
函数功能：检查元素并构建一个只包含激活元素的列表加速渲染
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void osdAddActiveElements(void)
{
    activeOsdElementCount = 0;
#ifdef USE_ACC
    if (sensors(SENSOR_ACC)) {
		// 模拟地平线
        osdAddActiveElement(OSD_ARTIFICIAL_HORIZON);		
    }
#endif
	// 其他OSD元素 
	// 遍历元素列表
    for (unsigned i = 0; i < sizeof(osdElementDisplayOrder); i++) {
        osdAddActiveElement(osdElementDisplayOrder[i]);
    }
#ifdef USE_GPS
    if (sensors(SENSOR_GPS)) {
		// GPS相关元素
        osdAddActiveElement(OSD_GPS_SATS);
        osdAddActiveElement(OSD_GPS_SPEED);
        osdAddActiveElement(OSD_GPS_LAT);
        osdAddActiveElement(OSD_GPS_LON);
        osdAddActiveElement(OSD_HOME_DIST);
        osdAddActiveElement(OSD_HOME_DIR);
        osdAddActiveElement(OSD_FLIGHT_DIST);
    }
#endif // GPS
}

//-------------------------------------------------------------------------------------元素绘制[写入显存]相关API

/**********************************************************************
函数名称：osdDrawSingleElement
函数功能：OSD绘制单一元素
函数形参：osdDisplayPort，item
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdDrawSingleElement(displayPort_t *osdDisplayPort, uint8_t item)
{
    if (!osdElementDrawFunction[item]) {
		// 没有绘制函数
        return;
    }
    if (!osdDisplayPort->useDeviceBlink && BLINK(item)) {
		// 闪烁
        return;
    }
	// --------------------------------------获取元素坐标 - 默认配置 
    uint8_t elemPosX = OSD_X(osdElementConfig()->item_pos[item]);
    uint8_t elemPosY = OSD_Y(osdElementConfig()->item_pos[item]);
	// 初始化buff
    char buff[OSD_ELEMENT_BUFFER_LENGTH] = "";
	// 创建元素参数缓冲区
    osdElementParms_t element;
    element.item = item;
    element.elemPosX = elemPosX;
    element.elemPosY = elemPosY;
    element.buff = (char *)&buff;
    element.osdDisplayPort = osdDisplayPort;
    element.drawElement = true;
    element.attr = DISPLAYPORT_ATTR_NONE;
    // --------------------------------------调用元素绘制函数（动态元素） - 填充元素参数缓冲区
    osdElementDrawFunction[item](&element);
    if (element.drawElement) {
		// OSD显示端口写操作 - 元素显示内容将写入显存对应坐标
        osdDisplayWrite(&element, elemPosX, elemPosY, element.attr, buff);
    }
}

/**********************************************************************
函数名称：osdDrawSingleElementBackground
函数功能：OSD绘制单一元素背景
函数形参：osdDisplayPort，item
函数返回值：None 
函数描述：None 
**********************************************************************/
static void osdDrawSingleElementBackground(displayPort_t *osdDisplayPort, uint8_t item)
{
    if (!osdElementBackgroundFunction[item]) {
		// 没有绘制函数
        return;
    }
	// --------------------------------------获取元素坐标
    uint8_t elemPosX = OSD_X(osdElementConfig()->item_pos[item]);
    uint8_t elemPosY = OSD_Y(osdElementConfig()->item_pos[item]);
	// 初始化buff
    char buff[OSD_ELEMENT_BUFFER_LENGTH] = "";
	// 创建元素参数缓冲区
    osdElementParms_t element;
    element.item = item;
    element.elemPosX = elemPosX;
    element.elemPosY = elemPosY;
    element.buff = (char *)&buff;
    element.osdDisplayPort = osdDisplayPort;
    element.drawElement = true;
    // --------------------------------------调用元素背景绘制函数（静态元素）
    osdElementBackgroundFunction[item](&element);
    if (element.drawElement) {
		// OSD显示端口写操作 - 写入显存
        osdDisplayWrite(&element, elemPosX, elemPosY, DISPLAYPORT_ATTR_NONE, buff);
    }
}

/**********************************************************************
函数名称：osdDrawActiveElements
函数功能：绘制OSD激活元素
函数形参：osdDisplayPort，currentTimeUs
函数返回值：None 
函数描述：None 
**********************************************************************/
void osdDrawActiveElements(displayPort_t *osdDisplayPort, timeUs_t currentTimeUs)
{
#ifdef USE_GPS
	// 处理GPS_SENSOR在激活时可能被延迟的情况 - 如果与模块失去通信，则取消激活
	static bool lastGpsSensorState;
    const bool currentGpsSensorState = sensors(SENSOR_GPS);
    if (lastGpsSensorState != currentGpsSensorState) {
        lastGpsSensorState = currentGpsSensorState;
		// 分析OSD激活元素
        osdAnalyzeActiveElements();
    }
#endif 
	// 计算闪烁状态
    blinkState = (currentTimeUs / 200000) % 2;
	// 遍历所有激活元素
    for (unsigned i = 0; i < activeOsdElementCount; i++) {
		// --------------绘制静态元素
        if (!backgroundLayerSupported) {
			// 绘制单一元素背景 - 如果背景层不被支持，那么也必须绘制元素的静态层
            osdDrawSingleElementBackground(osdDisplayPort, activeOsdElementArray[i]);
        }
		// --------------绘制动态元素
        osdDrawSingleElement(osdDisplayPort, activeOsdElementArray[i]);
    }
}

/**********************************************************************
函数名称：osdDrawActiveElementsBackground
函数功能：OSD绘制激活元素背景 
函数形参：osdDisplayPort
函数返回值：None 
函数描述：None 
**********************************************************************/
void osdDrawActiveElementsBackground(displayPort_t *osdDisplayPort)
{
    if (backgroundLayerSupported) {
		// 选择背景层
        displayLayerSelect(osdDisplayPort, DISPLAYPORT_LAYER_BACKGROUND);
		// 清除屏幕
        displayClearScreen(osdDisplayPort);
		// 遍历所有激活元素
        for (unsigned i = 0; i < activeOsdElementCount; i++) {
			// 绘制单一元素背景 - 如果背景层不被支持，那么也必须绘制元素的静态层
            osdDrawSingleElementBackground(osdDisplayPort, activeOsdElementArray[i]);
        }
		// 选择前景层
        displayLayerSelect(osdDisplayPort, DISPLAYPORT_LAYER_FOREGROUND);
    }
}

//-------------------------------------------------------------------------------------元素初始化相关API

/**********************************************************************
函数名称：osdElementsInit
函数功能：OSD元素初始化
函数形参：backgroundLayerFlag
函数返回值：None 
函数描述：None 
**********************************************************************/
void osdElementsInit(bool backgroundLayerFlag)
{
    backgroundLayerSupported = backgroundLayerFlag;
    activeOsdElementCount = 0;
}

/**********************************************************************
函数名称：osdResetAlarms
函数功能：OSD复位警报
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void osdResetAlarms(void)
{
    memset(blinkBits, 0, sizeof(blinkBits));
}

//-------------------------------------------------------------------------------------OSD警报元素更新相关API
// 闪烁警告

/**********************************************************************
函数名称：osdUpdateAlarms
函数功能：更新OSD警报
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void osdUpdateAlarms(void)
{
    int32_t alt = osdGetMetersToSelectedUnit(getEstimatedAltitudeCm()) / 100;

	// RSSI百分比
    if (getRssiPercent() < osdConfig()->rssi_alarm) {
        SET_BLINK(OSD_RSSI_VALUE);
    } else {
        CLR_BLINK(OSD_RSSI_VALUE);
    }

	// 链接质量
#ifdef USE_RX_LINK_QUALITY_INFO
    if (rxGetLinkQualityPercent() < osdConfig()->link_quality_alarm) {
        SET_BLINK(OSD_LINK_QUALITY);
    } else {
        CLR_BLINK(OSD_LINK_QUALITY);
    }
#endif // USE_RX_LINK_QUALITY_INFO

	// 电池电压电流
    if (getBatteryState() == BATTERY_OK) {
        CLR_BLINK(OSD_MAIN_BATT_VOLTAGE);
    } else {
        SET_BLINK(OSD_MAIN_BATT_VOLTAGE);
    }

	// GPS星数
#ifdef USE_GPS
    if ((STATE(GPS_FIX) == 0) || (gpsSol.numSat < 5)
#ifdef USE_GPS_RESCUE
            || ((gpsSol.numSat < gpsRescueConfig()->minSats) && gpsRescueIsConfigured())
#endif
            ) {
        SET_BLINK(OSD_GPS_SATS);
    } else {
        CLR_BLINK(OSD_GPS_SATS);
    }
#endif //USE_GPS

	// 定时器
    for (int i = 0; i < OSD_TIMER_COUNT; i++) {
        const uint16_t timer = osdConfig()->timers[i];
        const timeUs_t time = osdGetTimerValue(OSD_TIMER_SRC(timer));
		// convert from minutes to us
        const timeUs_t alarmTime = OSD_TIMER_ALARM(timer) * 60000000; 
        if (alarmTime != 0 && time >= alarmTime) {
            SET_BLINK(OSD_ITEM_TIMER_1 + i);
        } else {
            CLR_BLINK(OSD_ITEM_TIMER_1 + i);
        }
    }

	// 高度
    if ((alt >= osdConfig()->alt_alarm) && ARMING_FLAG(ARMED)) {
        SET_BLINK(OSD_ALTITUDE);
    } else {
        CLR_BLINK(OSD_ALTITUDE);
    }

	// 距离家的距离
#ifdef USE_GPS
    if (sensors(SENSOR_GPS) && ARMING_FLAG(ARMED) && STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
        if (osdConfig()->distance_alarm && GPS_distanceToHome >= osdConfig()->distance_alarm) {
            SET_BLINK(OSD_HOME_DIST);
        } else {
            CLR_BLINK(OSD_HOME_DIST);
        }
    } else {
        CLR_BLINK(OSD_HOME_DIST);;
    }
#endif
}

//-------------------------------------------------------------------------------------动态元素相关API

/**********************************************************************
函数名称：osdElementIsActive
函数功能：检测是否为OSD动态元素
函数形参：element
函数返回值：状态
函数描述：None 
**********************************************************************/
#ifdef USE_ACC
static bool osdElementIsActive(osd_items_e element)
{
    for (unsigned i = 0; i < activeOsdElementCount; i++) {
        if (activeOsdElementArray[i] == element) {
            return true;
        }
    }
    return false;
}

/**********************************************************************
函数名称：osdElementsNeedAccelerometer
函数功能：确定是否有任何动态元素需要ACC
函数形参：None
函数返回值：None
函数描述：None 
**********************************************************************/
bool osdElementsNeedAccelerometer(void)
{
    return osdElementIsActive(OSD_ARTIFICIAL_HORIZON) ||
           osdElementIsActive(OSD_FLIP_ARROW);
}
#endif // USE_ACC
#endif // USE_OSD

