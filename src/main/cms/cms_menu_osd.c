#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#if defined(USE_OSD) && defined(USE_CMS)
#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_osd.h"

#include "common/utils.h"

#include "drivers/motor.h"
#include "drivers/max7456.h"
#include "drivers/osd_font.h"

#include "config/feature.h"
#include "config/config.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "io/displayport_max7456.h"
#include "io/serial.h"

#include "osd/osd.h"
#include "osd/osd_elements.h"
#include "sensors/battery.h"

#include "flight/mixer.h"
#include "drivers/system.h"
#include "drivers/time.h"


#ifdef USE_EXTENDED_CMS_MENUS
static uint16_t osdConfig_item_pos[OSD_ITEM_COUNT];

/**********************************************************************
函数名称：cmsx_Blackbox_GetDeviceStatus
函数功能：进入OSD激活元素菜单
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *menuOsdActiveElemsOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);
    memcpy(&osdConfig_item_pos[0], &osdElementConfig()->item_pos[0], sizeof(uint16_t) * OSD_ITEM_COUNT);
    return NULL;
}

/**********************************************************************
函数名称：menuOsdActiveElemsOnExit
函数功能：退出OSD激活元素菜单
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *menuOsdActiveElemsOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);
    memcpy(&osdElementConfigMutable()->item_pos[0], &osdConfig_item_pos[0], sizeof(uint16_t) * OSD_ITEM_COUNT);
    osdAnalyzeActiveElements();
    return NULL;
}

// osd激活元素菜单管理内容
const OSD_Entry menuOsdActiveElemsEntries[] =
{
    {"--- ACTIV ELEM ---", OME_Label,   NULL, NULL, 0},
    {"RSSI",               OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_RSSI_VALUE], DYNAMIC},
    {"BATTERY VOLTAGE",    OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_MAIN_BATT_VOLTAGE], DYNAMIC},
    {"CROSSHAIRS",         OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_CROSSHAIRS], DYNAMIC},
    {"HORIZON",            OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_ARTIFICIAL_HORIZON], DYNAMIC},
    {"HORIZON SIDEBARS",   OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_HORIZON_SIDEBARS], DYNAMIC},
    {"TIMER 1",            OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_ITEM_TIMER_1], DYNAMIC},
    {"TIMER 2",            OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_ITEM_TIMER_2], DYNAMIC},
    {"FLY MODE",           OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_FLYMODE], DYNAMIC},
    {"THROTTLE",           OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_THROTTLE_POS], DYNAMIC},
#ifdef USE_VTX_CONTROL
    {"VTX CHAN",           OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_VTX_CHANNEL], DYNAMIC},
#endif // VTX
    {"CURRENT (A)",        OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_CURRENT_DRAW], DYNAMIC},
#ifdef USE_GPS
    {"GPS SPEED",          OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_GPS_SPEED], DYNAMIC},
    {"GPS SATS",           OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_GPS_SATS], DYNAMIC},
    {"GPS LAT",            OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_GPS_LAT], DYNAMIC},
    {"GPS LON",            OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_GPS_LON], DYNAMIC},
    {"HOME DIR",           OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_HOME_DIR], DYNAMIC},
    {"HOME DIST",          OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_HOME_DIST], DYNAMIC},
    {"FLIGHT DIST",        OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_FLIGHT_DIST], DYNAMIC},
#endif // GPS
    {"COMPASS BAR",        OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_COMPASS_BAR], DYNAMIC},
    {"ALTITUDE",           OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_ALTITUDE], DYNAMIC},
    {"PROFILES",           OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_PIDRATE_PROFILE], DYNAMIC},
    {"WARNINGS",           OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_WARNINGS], DYNAMIC},
    {"DISARMED",           OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_DISARMED], DYNAMIC},
#ifdef USE_VARIO
    {"VARIO",              OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_NUMERICAL_VARIO], DYNAMIC},
#endif
    {"FLIP ARROW",         OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_FLIP_ARROW], DYNAMIC},
#ifdef USE_RX_LINK_QUALITY_INFO
    {"LINK QUALITY",       OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_LINK_QUALITY], DYNAMIC},
#endif
    {"BACK",               OME_Back,    NULL, NULL, 0},
    {NULL,                 OME_END,     NULL, NULL, 0}
};

// osd激活元素菜单配置
static CMS_Menu menuOsdActiveElems = {
    .onEnter = menuOsdActiveElemsOnEnter,
    .onExit = menuOsdActiveElemsOnExit,
    .onDisplayUpdate = NULL,
    .entries = menuOsdActiveElemsEntries
};

static uint8_t osdConfig_rssi_alarm;
static uint16_t osdConfig_link_quality_alarm;
static uint8_t osdConfig_rssi_dbm_alarm;
static uint16_t osdConfig_alt_alarm;
static uint16_t osdConfig_distance_alarm;
static uint8_t batteryConfig_vbatDurationForWarning;
static uint8_t batteryConfig_vbatDurationForCritical;

/**********************************************************************
函数名称：menuAlarmsOnEnter
函数功能：进入警告菜单
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *menuAlarmsOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    osdConfig_rssi_alarm = osdConfig()->rssi_alarm;
    osdConfig_link_quality_alarm = osdConfig()->link_quality_alarm;
    osdConfig_alt_alarm = osdConfig()->alt_alarm;
    osdConfig_distance_alarm = osdConfig()->distance_alarm;
    batteryConfig_vbatDurationForWarning = batteryConfig()->vbatDurationForWarning;
    batteryConfig_vbatDurationForCritical = batteryConfig()->vbatDurationForCritical;

    return NULL;
}

/**********************************************************************
函数名称：menuAlarmsOnExit
函数功能：退出警告菜单
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *menuAlarmsOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    osdConfigMutable()->rssi_alarm = osdConfig_rssi_alarm;
    osdConfigMutable()->link_quality_alarm = osdConfig_link_quality_alarm;
    osdConfigMutable()->alt_alarm = osdConfig_alt_alarm;
    osdConfigMutable()->distance_alarm = osdConfig_distance_alarm;
    batteryConfigMutable()->vbatDurationForWarning = batteryConfig_vbatDurationForWarning;
    batteryConfigMutable()->vbatDurationForCritical = batteryConfig_vbatDurationForCritical;

    return NULL;
}

// 警告菜单管理内容
const OSD_Entry menuAlarmsEntries[] =
{
    {"--- ALARMS ---", OME_Label, NULL, NULL, 0},
    {"RSSI",     OME_UINT8,  NULL, &(OSD_UINT8_t){&osdConfig_rssi_alarm, 5, 90, 5}, 0},
    {"LINK QUALITY", OME_UINT16,  NULL, &(OSD_UINT16_t){&osdConfig_link_quality_alarm, 5, 300, 5}, 0},
    {"RSSI DBM", OME_UINT8,  NULL, &(OSD_UINT8_t){&osdConfig_rssi_dbm_alarm, 5, 130, 5}, 0},
    {"MAX ALT",  OME_UINT16, NULL, &(OSD_UINT16_t){&osdConfig_alt_alarm, 1, 200, 1}, 0},
    {"MAX DISTANCE", OME_UINT16, NULL, &(OSD_UINT16_t){&osdConfig_distance_alarm, 0, UINT16_MAX, 10}, 0},
    {"VBAT WARN DUR", OME_UINT8, NULL, &(OSD_UINT8_t){ &batteryConfig_vbatDurationForWarning, 0, 200, 1 }, 0 },
    {"VBAT CRIT DUR", OME_UINT8, NULL, &(OSD_UINT8_t){ &batteryConfig_vbatDurationForCritical, 0, 200, 1 }, 0 },
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

// 警告菜单配置
static CMS_Menu menuAlarms = {
    .onEnter = menuAlarmsOnEnter,
    .onExit = menuAlarmsOnExit,
    .onDisplayUpdate = NULL,
    .entries = menuAlarmsEntries,
};

osd_timer_source_e timerSource[OSD_TIMER_COUNT];
osd_timer_precision_e timerPrecision[OSD_TIMER_COUNT];
uint8_t timerAlarm[OSD_TIMER_COUNT];

/**********************************************************************
函数名称：menuTimersOnEnter
函数功能：进入定时器菜单
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *menuTimersOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    for (int i = 0; i < OSD_TIMER_COUNT; i++) {
        const uint16_t timer = osdConfig()->timers[i];
        timerSource[i] = OSD_TIMER_SRC(timer);
        timerPrecision[i] = OSD_TIMER_PRECISION(timer);
        timerAlarm[i] = OSD_TIMER_ALARM(timer);
    }

    return NULL;
}

/**********************************************************************
函数名称：menuTimersOnExit
函数功能：退出定时器菜单
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *menuTimersOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    for (int i = 0; i < OSD_TIMER_COUNT; i++) {
        osdConfigMutable()->timers[i] = OSD_TIMER(timerSource[i], timerPrecision[i], timerAlarm[i]);
    }

    return NULL;
}

// 定时器名称
static const char * osdTimerPrecisionNames[] = {"SCND", "HDTH"};

// 定时器菜单管理内容
const OSD_Entry menuTimersEntries[] =
{
	{"--- FUNCTION ---", OME_Label, NULL, NULL, 0},
    {"--- TIMERS ---", OME_Label, NULL, NULL, 0},
    {"1 SRC",          OME_TAB,   NULL, &(OSD_TAB_t){&timerSource[OSD_TIMER_1], OSD_TIMER_SRC_COUNT - 1, osdTimerSourceNames}, 0 },
    {"1 PREC",         OME_TAB,   NULL, &(OSD_TAB_t){&timerPrecision[OSD_TIMER_1], OSD_TIMER_PREC_COUNT - 1, osdTimerPrecisionNames}, 0},
    {"1 ALARM",        OME_UINT8, NULL, &(OSD_UINT8_t){&timerAlarm[OSD_TIMER_1], 0, 0xFF, 1}, 0},
    {"2 SRC",          OME_TAB,   NULL, &(OSD_TAB_t){&timerSource[OSD_TIMER_2], OSD_TIMER_SRC_COUNT - 1, osdTimerSourceNames}, 0 },
    {"2 PREC",         OME_TAB,   NULL, &(OSD_TAB_t){&timerPrecision[OSD_TIMER_2], OSD_TIMER_PREC_COUNT - 1, osdTimerPrecisionNames}, 0},
    {"2 ALARM",        OME_UINT8, NULL, &(OSD_UINT8_t){&timerAlarm[OSD_TIMER_2], 0, 0xFF, 1}, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};
	
// 定时器菜单配置
static CMS_Menu menuTimers = {
    .onEnter = menuTimersOnEnter,
    .onExit = menuTimersOnExit,
    .onDisplayUpdate = NULL,
    .entries = menuTimersEntries,
};
#endif /* USE_EXTENDED_CMS_MENUS */

#ifdef USE_MAX7456
static bool displayPortProfileMax7456_invert;
static uint8_t displayPortProfileMax7456_blackBrightness;
static uint8_t displayPortProfileMax7456_whiteBrightness;
#endif

/**********************************************************************
函数名称：cmsx_menuOsdOnEnter
函数功能：进入osd菜单
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_menuOsdOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

#ifdef USE_MAX7456
    displayPortProfileMax7456_invert = displayPortProfileMax7456()->invert;
    displayPortProfileMax7456_blackBrightness = displayPortProfileMax7456()->blackBrightness;
    displayPortProfileMax7456_whiteBrightness = displayPortProfileMax7456()->whiteBrightness;
#endif

    return NULL;
}

/**********************************************************************
函数名称：cmsx_menuOsdOnExit
函数功能：退出osd菜单
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_menuOsdOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    return NULL;
}

/**********************************************************************
函数名称：cmsx_max7456Update
函数功能：更新max7456
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
#ifdef USE_MAX7456
static const void *cmsx_max7456Update(displayPort_t *pDisp, const void *self)
{
    UNUSED(self);

    displayPortProfileMax7456Mutable()->invert = displayPortProfileMax7456_invert;
    displayPortProfileMax7456Mutable()->blackBrightness = displayPortProfileMax7456_blackBrightness;
    displayPortProfileMax7456Mutable()->whiteBrightness = displayPortProfileMax7456_whiteBrightness;

    displayClearScreen(pDisp);

    return NULL;
}
#endif // USE_MAX7456

/**********************************************************************
函数名称：cmsFontupdate
函数功能：OSD字库更新功能函数（烧录256个字符到字库并重启系统）
函数形参：pDisp，selected
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsFontupdate(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);
	
	write_max7456_font(0,(unsigned char *)zi_ku0);
	write_max7456_font(1,(unsigned char *)zi_ku1);
	write_max7456_font(2,(unsigned char *)zi_ku2);
	write_max7456_font(3,(unsigned char *)zi_ku3);
	write_max7456_font(4,(unsigned char *)zi_ku4);
	write_max7456_font(5,(unsigned char *)zi_ku5);
	write_max7456_font(6,(unsigned char *)zi_ku6);
	write_max7456_font(7,(unsigned char *)zi_ku7);
	write_max7456_font(8,(unsigned char *)zi_ku8);
	write_max7456_font(9,(unsigned char *)zi_ku9);
	write_max7456_font(10,(unsigned char *)zi_ku10);
	write_max7456_font(11,(unsigned char *)zi_ku11);
	write_max7456_font(12,(unsigned char *)zi_ku12);
	write_max7456_font(13,(unsigned char *)zi_ku13);
	write_max7456_font(14,(unsigned char *)zi_ku14);
	write_max7456_font(15,(unsigned char *)zi_ku15);
	write_max7456_font(16,(unsigned char *)zi_ku16);
	write_max7456_font(17,(unsigned char *)zi_ku17);
	write_max7456_font(18,(unsigned char *)zi_ku18);
	write_max7456_font(19,(unsigned char *)zi_ku19);
	write_max7456_font(20,(unsigned char *)zi_ku20);
	write_max7456_font(21,(unsigned char *)zi_ku21);
	write_max7456_font(22,(unsigned char *)zi_ku22);
	write_max7456_font(23,(unsigned char *)zi_ku23);
	write_max7456_font(24,(unsigned char *)zi_ku24);
	write_max7456_font(25,(unsigned char *)zi_ku25);
	write_max7456_font(26,(unsigned char *)zi_ku26);
	write_max7456_font(27,(unsigned char *)zi_ku27);
	write_max7456_font(28,(unsigned char *)zi_ku28);
	write_max7456_font(29,(unsigned char *)zi_ku29);
	write_max7456_font(30,(unsigned char *)zi_ku30);
	write_max7456_font(31,(unsigned char *)zi_ku31);
	write_max7456_font(32,(unsigned char *)zi_ku32);
	write_max7456_font(33,(unsigned char *)zi_ku33);
	write_max7456_font(34,(unsigned char *)zi_ku34);
	write_max7456_font(35,(unsigned char *)zi_ku35);
	write_max7456_font(36,(unsigned char *)zi_ku36);
	write_max7456_font(37,(unsigned char *)zi_ku37);
	write_max7456_font(38,(unsigned char *)zi_ku38);
	write_max7456_font(39,(unsigned char *)zi_ku39);
	write_max7456_font(40,(unsigned char *)zi_ku40);
	write_max7456_font(41,(unsigned char *)zi_ku41);
	write_max7456_font(42,(unsigned char *)zi_ku42);
	write_max7456_font(43,(unsigned char *)zi_ku43);
	write_max7456_font(44,(unsigned char *)zi_ku44);
	write_max7456_font(45,(unsigned char *)zi_ku45);
	write_max7456_font(46,(unsigned char *)zi_ku46);
	write_max7456_font(47,(unsigned char *)zi_ku47);
	write_max7456_font(48,(unsigned char *)zi_ku48);
	write_max7456_font(49,(unsigned char *)zi_ku49);
	write_max7456_font(50,(unsigned char *)zi_ku50);
	write_max7456_font(51,(unsigned char *)zi_ku51);
	write_max7456_font(52,(unsigned char *)zi_ku52);
	write_max7456_font(53,(unsigned char *)zi_ku53);
	write_max7456_font(54,(unsigned char *)zi_ku54);
	write_max7456_font(55,(unsigned char *)zi_ku55);
	write_max7456_font(56,(unsigned char *)zi_ku56);
	write_max7456_font(57,(unsigned char *)zi_ku57);
	write_max7456_font(58,(unsigned char *)zi_ku58);
	write_max7456_font(59,(unsigned char *)zi_ku59);
	write_max7456_font(60,(unsigned char *)zi_ku60);
	write_max7456_font(61,(unsigned char *)zi_ku61);
	write_max7456_font(62,(unsigned char *)zi_ku62);
	write_max7456_font(63,(unsigned char *)zi_ku63);
	write_max7456_font(64,(unsigned char *)zi_ku64);
	write_max7456_font(65,(unsigned char *)zi_ku65);
	write_max7456_font(66,(unsigned char *)zi_ku66);
	write_max7456_font(67,(unsigned char *)zi_ku67);
	write_max7456_font(68,(unsigned char *)zi_ku68);
	write_max7456_font(69,(unsigned char *)zi_ku69);
	write_max7456_font(70,(unsigned char *)zi_ku70);
	write_max7456_font(71,(unsigned char *)zi_ku71);
	write_max7456_font(72,(unsigned char *)zi_ku72);
	write_max7456_font(73,(unsigned char *)zi_ku73);
	write_max7456_font(74,(unsigned char *)zi_ku74);
	write_max7456_font(75,(unsigned char *)zi_ku75);
	write_max7456_font(76,(unsigned char *)zi_ku76);
	write_max7456_font(77,(unsigned char *)zi_ku77);
	write_max7456_font(78,(unsigned char *)zi_ku78);
	write_max7456_font(79,(unsigned char *)zi_ku79);
	write_max7456_font(80,(unsigned char *)zi_ku80);
	write_max7456_font(81,(unsigned char *)zi_ku81);
	write_max7456_font(82,(unsigned char *)zi_ku82);
	write_max7456_font(83,(unsigned char *)zi_ku83);
	write_max7456_font(84,(unsigned char *)zi_ku84);
	write_max7456_font(85,(unsigned char *)zi_ku85);
	write_max7456_font(86,(unsigned char *)zi_ku86);
	write_max7456_font(87,(unsigned char *)zi_ku87);
	write_max7456_font(88,(unsigned char *)zi_ku88);
	write_max7456_font(89,(unsigned char *)zi_ku89);
	write_max7456_font(90,(unsigned char *)zi_ku90);
	write_max7456_font(91,(unsigned char *)zi_ku91);
	write_max7456_font(92,(unsigned char *)zi_ku92);
	write_max7456_font(93,(unsigned char *)zi_ku93);
	write_max7456_font(94,(unsigned char *)zi_ku94);
	write_max7456_font(95,(unsigned char *)zi_ku95);
	write_max7456_font(96,(unsigned char *)zi_ku96);
	write_max7456_font(97,(unsigned char *)zi_ku97);
	write_max7456_font(98,(unsigned char *)zi_ku98);
	write_max7456_font(99,(unsigned char *)zi_ku99);
	write_max7456_font(100,(unsigned char *)zi_ku100);
	write_max7456_font(101,(unsigned char *)zi_ku101);
	write_max7456_font(102,(unsigned char *)zi_ku102);
	write_max7456_font(103,(unsigned char *)zi_ku103);
	write_max7456_font(104,(unsigned char *)zi_ku104);
	write_max7456_font(105,(unsigned char *)zi_ku105);
	write_max7456_font(106,(unsigned char *)zi_ku106);
	write_max7456_font(107,(unsigned char *)zi_ku107);
	write_max7456_font(108,(unsigned char *)zi_ku108);
	write_max7456_font(109,(unsigned char *)zi_ku109);
	write_max7456_font(110,(unsigned char *)zi_ku110);
	write_max7456_font(111,(unsigned char *)zi_ku111);
	write_max7456_font(112,(unsigned char *)zi_ku112);
	write_max7456_font(113,(unsigned char *)zi_ku113);
	write_max7456_font(114,(unsigned char *)zi_ku114);
	write_max7456_font(115,(unsigned char *)zi_ku115);
	write_max7456_font(116,(unsigned char *)zi_ku116);
	write_max7456_font(117,(unsigned char *)zi_ku117);
	write_max7456_font(118,(unsigned char *)zi_ku118);
	write_max7456_font(119,(unsigned char *)zi_ku119);
	write_max7456_font(120,(unsigned char *)zi_ku120);
	write_max7456_font(121,(unsigned char *)zi_ku121);
	write_max7456_font(122,(unsigned char *)zi_ku122);
	write_max7456_font(123,(unsigned char *)zi_ku123);
	write_max7456_font(124,(unsigned char *)zi_ku124);
	write_max7456_font(125,(unsigned char *)zi_ku125);
	write_max7456_font(126,(unsigned char *)zi_ku126);
	write_max7456_font(127,(unsigned char *)zi_ku127);
	write_max7456_font(128,(unsigned char *)zi_ku128);
	write_max7456_font(129,(unsigned char *)zi_ku129);
	write_max7456_font(130,(unsigned char *)zi_ku130);
	write_max7456_font(131,(unsigned char *)zi_ku131);
	write_max7456_font(132,(unsigned char *)zi_ku132);
	write_max7456_font(133,(unsigned char *)zi_ku133);
	write_max7456_font(134,(unsigned char *)zi_ku134);
	write_max7456_font(135,(unsigned char *)zi_ku135);
	write_max7456_font(136,(unsigned char *)zi_ku136);
	write_max7456_font(137,(unsigned char *)zi_ku137);
	write_max7456_font(138,(unsigned char *)zi_ku138);
	write_max7456_font(139,(unsigned char *)zi_ku139);
	write_max7456_font(140,(unsigned char *)zi_ku140);
	write_max7456_font(141,(unsigned char *)zi_ku141);
	write_max7456_font(142,(unsigned char *)zi_ku142);
	write_max7456_font(143,(unsigned char *)zi_ku143);
	write_max7456_font(144,(unsigned char *)zi_ku144);
	write_max7456_font(145,(unsigned char *)zi_ku145);
	write_max7456_font(146,(unsigned char *)zi_ku146);
	write_max7456_font(147,(unsigned char *)zi_ku147);
	write_max7456_font(148,(unsigned char *)zi_ku148);
	write_max7456_font(149,(unsigned char *)zi_ku149);
	write_max7456_font(150,(unsigned char *)zi_ku150);
	write_max7456_font(151,(unsigned char *)zi_ku151);
	write_max7456_font(152,(unsigned char *)zi_ku152);
	write_max7456_font(153,(unsigned char *)zi_ku153);
	write_max7456_font(154,(unsigned char *)zi_ku154);
	write_max7456_font(155,(unsigned char *)zi_ku155);
	write_max7456_font(156,(unsigned char *)zi_ku156);
	write_max7456_font(157,(unsigned char *)zi_ku157);
	write_max7456_font(158,(unsigned char *)zi_ku158);
	write_max7456_font(159,(unsigned char *)zi_ku159);
	write_max7456_font(160,(unsigned char *)zi_ku160);
	write_max7456_font(161,(unsigned char *)zi_ku161);
	write_max7456_font(162,(unsigned char *)zi_ku162);
	write_max7456_font(163,(unsigned char *)zi_ku163);
	write_max7456_font(164,(unsigned char *)zi_ku164);
	write_max7456_font(165,(unsigned char *)zi_ku165);
	write_max7456_font(166,(unsigned char *)zi_ku166);
	write_max7456_font(167,(unsigned char *)zi_ku167);
	write_max7456_font(168,(unsigned char *)zi_ku168);
	write_max7456_font(169,(unsigned char *)zi_ku169);
	write_max7456_font(170,(unsigned char *)zi_ku170);
	write_max7456_font(171,(unsigned char *)zi_ku171);
	write_max7456_font(172,(unsigned char *)zi_ku172);
	write_max7456_font(173,(unsigned char *)zi_ku173);
	write_max7456_font(174,(unsigned char *)zi_ku174);
	write_max7456_font(175,(unsigned char *)zi_ku175);
	write_max7456_font(176,(unsigned char *)zi_ku176);
	write_max7456_font(177,(unsigned char *)zi_ku177);
	write_max7456_font(178,(unsigned char *)zi_ku178);
	write_max7456_font(179,(unsigned char *)zi_ku179);
	write_max7456_font(180,(unsigned char *)zi_ku180);
	write_max7456_font(181,(unsigned char *)zi_ku181);
	write_max7456_font(182,(unsigned char *)zi_ku182);
	write_max7456_font(183,(unsigned char *)zi_ku183);
	write_max7456_font(184,(unsigned char *)zi_ku184);
	write_max7456_font(185,(unsigned char *)zi_ku185);
	write_max7456_font(186,(unsigned char *)zi_ku186);
	write_max7456_font(187,(unsigned char *)zi_ku187);
	write_max7456_font(188,(unsigned char *)zi_ku188);
	write_max7456_font(189,(unsigned char *)zi_ku189);
	write_max7456_font(190,(unsigned char *)zi_ku190);
	write_max7456_font(191,(unsigned char *)zi_ku191);
	write_max7456_font(192,(unsigned char *)zi_ku192);
	write_max7456_font(193,(unsigned char *)zi_ku193);
	write_max7456_font(194,(unsigned char *)zi_ku194);
	write_max7456_font(195,(unsigned char *)zi_ku195);
	write_max7456_font(196,(unsigned char *)zi_ku196);
	write_max7456_font(197,(unsigned char *)zi_ku197);
	write_max7456_font(198,(unsigned char *)zi_ku198);
	write_max7456_font(199,(unsigned char *)zi_ku199);
	write_max7456_font(200,(unsigned char *)zi_ku200);
	write_max7456_font(201,(unsigned char *)zi_ku201);
	write_max7456_font(202,(unsigned char *)zi_ku202);
	write_max7456_font(203,(unsigned char *)zi_ku203);
	write_max7456_font(204,(unsigned char *)zi_ku204);
	write_max7456_font(205,(unsigned char *)zi_ku205);
	write_max7456_font(206,(unsigned char *)zi_ku206);
	write_max7456_font(207,(unsigned char *)zi_ku207);
	write_max7456_font(208,(unsigned char *)zi_ku208);
	write_max7456_font(209,(unsigned char *)zi_ku209);
	write_max7456_font(210,(unsigned char *)zi_ku210);
	write_max7456_font(211,(unsigned char *)zi_ku211);
	write_max7456_font(212,(unsigned char *)zi_ku212);
	write_max7456_font(213,(unsigned char *)zi_ku213);
	write_max7456_font(214,(unsigned char *)zi_ku214);
	write_max7456_font(215,(unsigned char *)zi_ku215);
	write_max7456_font(216,(unsigned char *)zi_ku216);
	write_max7456_font(217,(unsigned char *)zi_ku217);
	write_max7456_font(218,(unsigned char *)zi_ku218);
	write_max7456_font(219,(unsigned char *)zi_ku219);
	write_max7456_font(220,(unsigned char *)zi_ku220);
	write_max7456_font(221,(unsigned char *)zi_ku221);
	write_max7456_font(222,(unsigned char *)zi_ku222);
	write_max7456_font(223,(unsigned char *)zi_ku223);
	write_max7456_font(224,(unsigned char *)zi_ku224);
	write_max7456_font(225,(unsigned char *)zi_ku225);
	write_max7456_font(226,(unsigned char *)zi_ku226);
	write_max7456_font(227,(unsigned char *)zi_ku227);
	write_max7456_font(228,(unsigned char *)zi_ku228);
	write_max7456_font(229,(unsigned char *)zi_ku229);
	write_max7456_font(230,(unsigned char *)zi_ku230);
	write_max7456_font(231,(unsigned char *)zi_ku231);
	write_max7456_font(232,(unsigned char *)zi_ku232);
	write_max7456_font(233,(unsigned char *)zi_ku233);
	write_max7456_font(234,(unsigned char *)zi_ku234);
	write_max7456_font(235,(unsigned char *)zi_ku235);
	write_max7456_font(236,(unsigned char *)zi_ku236);
	write_max7456_font(237,(unsigned char *)zi_ku237);
	write_max7456_font(238,(unsigned char *)zi_ku238);
	write_max7456_font(239,(unsigned char *)zi_ku239);
	write_max7456_font(240,(unsigned char *)zi_ku240);
	write_max7456_font(241,(unsigned char *)zi_ku241);
	write_max7456_font(242,(unsigned char *)zi_ku242);
	write_max7456_font(243,(unsigned char *)zi_ku243);
	write_max7456_font(244,(unsigned char *)zi_ku244);
	write_max7456_font(245,(unsigned char *)zi_ku245);
	write_max7456_font(246,(unsigned char *)zi_ku246);
	write_max7456_font(247,(unsigned char *)zi_ku247);
	write_max7456_font(248,(unsigned char *)zi_ku248);
	write_max7456_font(249,(unsigned char *)zi_ku249);
	write_max7456_font(250,(unsigned char *)zi_ku250);
	write_max7456_font(251,(unsigned char *)zi_ku251);
	write_max7456_font(252,(unsigned char *)zi_ku252);
	write_max7456_font(253,(unsigned char *)zi_ku253);
	write_max7456_font(254,(unsigned char *)zi_ku254);
	write_max7456_font(255,(unsigned char *)zi_ku255);

	// 关闭所有电机并重启
    stopMotors();
    motorShutdown();
    delay(200);
    systemReset();
	
    return NULL;
}

// osd菜单管理内容
const OSD_Entry cmsx_menuOsdEntries[] =
{
    {"---OSD---",   OME_Label,   NULL,          NULL,                0},
#ifdef USE_EXTENDED_CMS_MENUS
    {"ACTIVE ELEM", OME_Submenu, cmsMenuChange, &menuOsdActiveElems, 0},
    {"TIMERS",      OME_Submenu, cmsMenuChange, &menuTimers,         0},
    {"ALARMS",      OME_Submenu, cmsMenuChange, &menuAlarms,         0},
#endif
#ifdef USE_MAX7456
    {"INVERT",    OME_Bool,  cmsx_max7456Update, &displayPortProfileMax7456_invert,                                   0},
    {"BRT BLACK", OME_UINT8, cmsx_max7456Update, &(OSD_UINT8_t){&displayPortProfileMax7456_blackBrightness, 0, 3, 1}, 0},
    {"BRT WHITE", OME_UINT8, cmsx_max7456Update, &(OSD_UINT8_t){&displayPortProfileMax7456_whiteBrightness, 0, 3, 1}, 0},
#endif
	{"FONT UPDATE", OME_Funcall, cmsFontupdate, NULL, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL,   OME_END,  NULL, NULL, 0}
};

// osd菜单配置
CMS_Menu cmsx_menuOsd = {
    .onEnter = cmsx_menuOsdOnEnter,
    .onExit = cmsx_menuOsdOnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuOsdEntries
};
#endif // CMS

