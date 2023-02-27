/*********************************************************************************
 系统功能函数声明。
*********************************************************************************/
#pragma once

#include <stdint.h>
#include "common/time.h"

void delayMicroseconds(timeUs_t us);
void delay(timeMs_t ms);
timeUs_t micros(void);
timeUs_t microsISR(void);
timeMs_t millis(void);

