#pragma once

#include "drivers/osd.h"

/* ---------------------------显示端口属性枚举------------------------------- */	
typedef enum {
    DISPLAYPORT_ATTR_NONE = 0,
    DISPLAYPORT_ATTR_INFO,
    DISPLAYPORT_ATTR_WARNING,
    DISPLAYPORT_ATTR_CRITICAL,
} displayPortAttr_e;
#define DISPLAYPORT_ATTR_BLINK  0x80  	    	// 闪烁位或进入displayPortAttr_e

/* ----------------------------显示端口层枚举-------------------------------- */	
typedef enum {
    DISPLAYPORT_LAYER_FOREGROUND,				// 前景
    DISPLAYPORT_LAYER_BACKGROUND,				// 背景
    DISPLAYPORT_LAYER_COUNT,					// 数量
} displayPortLayer_e;

/* -----------------------------显示端口结构体------------------------------- */	
struct displayPortVTable_s;
typedef struct displayPort_s {
    const struct displayPortVTable_s *vTable;   // 虚函数表
    void *device;								// 设备
    uint8_t rows;								// 行
    uint8_t cols;								// 列
    uint8_t posX;								// X
    uint8_t posY;								// Y
    bool useFullscreen;							// 使用全屏
    bool cleared;								// 清除
    int8_t cursorRow;							// 行光标
    int8_t grabCount;							// 获取数量
    bool useDeviceBlink;			
} displayPort_t;

/* --------------------------显示端口虚函数表结构体-------------------------- */	
struct osdCharacter_s;
struct displayCanvas_s;
typedef struct displayPortVTable_s {
    int (*grab)(displayPort_t *displayPort);
    int (*release)(displayPort_t *displayPort);
    int (*clearScreen)(displayPort_t *displayPort);
    int (*drawScreen)(displayPort_t *displayPort);
    int (*screenSize)(const displayPort_t *displayPort);
    int (*writeString)(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, const char *text);
    int (*writeChar)(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, uint8_t c);
    bool (*isTransferInProgress)(const displayPort_t *displayPort);
    void (*resync)(displayPort_t *displayPort);
    bool (*isSynced)(const displayPort_t *displayPort);
    uint32_t (*txBytesFree)(const displayPort_t *displayPort);
    bool (*layerSupported)(displayPort_t *displayPort, displayPortLayer_e layer);
    bool (*layerSelect)(displayPort_t *displayPort, displayPortLayer_e layer);
    bool (*layerCopy)(displayPort_t *displayPort, displayPortLayer_e destLayer, displayPortLayer_e sourceLayer);
    bool (*isReady)(displayPort_t *displayPort);
    bool (*getCanvas)(struct displayCanvas_s *canvas, const displayPort_t *displayPort);
	bool (*writeFontCharacter)(displayPort_t *instance, uint16_t addr, const struct osdCharacter_s *chr);
} displayPortVTable_t;

void displayGrab(displayPort_t *instance);
void displayRelease(displayPort_t *instance);
bool displayIsGrabbed(const displayPort_t *instance);
void displayClearScreen(displayPort_t *instance);
void displayDrawScreen(displayPort_t *instance);
int displayWrite(displayPort_t *instance, uint8_t x, uint8_t y, uint8_t attr, const char *s);
int displayWriteChar(displayPort_t *instance, uint8_t x, uint8_t y, uint8_t attr, uint8_t c);
void displayResync(displayPort_t *instance);
uint16_t displayTxBytesFree(const displayPort_t *instance);
bool displayIsReady(displayPort_t *instance);
void displayInit(displayPort_t *instance, const displayPortVTable_t *vTable);
bool displayLayerSupported(displayPort_t *instance, displayPortLayer_e layer);
bool displayLayerSelect(displayPort_t *instance, displayPortLayer_e layer);
bool displayLayerCopy(displayPort_t *instance, displayPortLayer_e destLayer, displayPortLayer_e sourceLayer);
bool displayWriteFontCharacter(displayPort_t *instance, uint16_t addr, const osdCharacter_t *chr);

