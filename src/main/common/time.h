#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "pg/pg.h"

typedef int32_t timeDelta_t;		      // 时差
typedef uint32_t timeMs_t ;				  // 毫秒
typedef uint32_t timeUs_t;				  // 微秒
#define TIMEUS_MAX UINT32_MAX			  // us最大值

/**********************************************************************
函数名称：cmpTimeUs
函数功能：计算时差
函数形参：时间1，时间2
函数返回值：时差
函数描述：None
**********************************************************************/
static inline timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) 
{ 
	return (timeDelta_t)(a - b); 
}

