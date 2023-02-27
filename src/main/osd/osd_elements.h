#pragma once

#include "drivers/display.h"

#include "osd/osd.h"

/* --------------------------OSD元素参数枚举-------------------------- */	
typedef struct osdElementParms_s {
    uint8_t item;						// OSD元素
    uint8_t elemPosX;					// X坐标
    uint8_t elemPosY;					// Y坐标
    char *buff;							// 内容
    displayPort_t *osdDisplayPort;	    // OSD显示端口
    bool drawElement;					// 绘制元素
    uint8_t attr;						// 属性
} osdElementParms_t;

// OSD元素绘制函数指针
typedef void (*osdElementDrawFn)(osdElementParms_t *element);

int osdConvertTemperatureToSelectedUnit(int tempInDegreesCelcius);
void osdFormatDistanceString(char *result, int distance, char leadingSymbol);
bool osdFormatRtcDateTime(char *buffer);
void osdFormatTime(char * buff, osd_timer_precision_e precision, timeUs_t time);
void osdFormatTimer(char *buff, bool showSymbol, bool usePrecision, int timerIndex);
int32_t osdGetMetersToSelectedUnit(int32_t meters);
char osdGetMetersToSelectedUnitSymbol(void);
int32_t osdGetSpeedToSelectedUnit(int32_t value);
char osdGetSpeedToSelectedUnitSymbol(void);
char osdGetTemperatureSymbolForSelectedUnit(void);
void osdAddActiveElements(void);
void osdDrawActiveElements(displayPort_t *osdDisplayPort, timeUs_t currentTimeUs);
void osdDrawActiveElementsBackground(displayPort_t *osdDisplayPort);
void osdElementsInit(bool backgroundLayerFlag);
void osdResetAlarms(void);
void osdUpdateAlarms(void);
bool osdElementsNeedAccelerometer(void);

