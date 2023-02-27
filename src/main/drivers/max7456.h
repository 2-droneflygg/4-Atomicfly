#pragma once

#include "drivers/display.h"

/* PAL或NTSC，字符数&&行数 */
#define VIDEO_BUFFER_CHARS_NTSC   390
#define VIDEO_BUFFER_CHARS_PAL    480
#define VIDEO_LINES_NTSC          13
#define VIDEO_LINES_PAL           16

extern uint16_t maxScreenSize;
struct vcdProfile_s;
struct max7456Config_s;
void max7456ReInitIfRequired(bool forceStallCheck);
void    max7456PreInit(const struct max7456Config_s *max7456Config);
bool    max7456Init(const struct max7456Config_s *max7456Config, const struct vcdProfile_s *vcdProfile, bool cpuOverclock);
void    max7456Invert(bool invert);
void    max7456Brightness(uint8_t black, uint8_t white);
void    max7456DrawScreen(void);
bool 	write_max7456_font(uint16_t char_address, unsigned char *font_data);
uint8_t max7456GetRowsCount(void);
void    max7456Write(uint8_t x, uint8_t y, const char *buff);
void    max7456WriteChar(uint8_t x, uint8_t y, uint8_t c);
void    max7456ClearScreen(void);
void    max7456RefreshAll(void);
bool    max7456DmaInProgress(void);
bool    max7456BuffersSynced(void);
bool    max7456LayerSupported(displayPortLayer_e layer);
bool    max7456LayerSelect(displayPortLayer_e layer);
bool    max7456LayerCopy(displayPortLayer_e destLayer, displayPortLayer_e sourceLayer);
bool    max7456IsDeviceDetected(void);

