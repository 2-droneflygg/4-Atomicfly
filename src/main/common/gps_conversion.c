/*********************************************************************************
 提供GPS坐标转换相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>

#include "platform.h"

#ifdef USE_GPS

#define DIGIT_TO_VAL(_x)    (_x - '0')

/**********************************************************************
函数名称：GPS_coord_to_degrees
函数功能：GPS坐标转度
函数形参：coordinateString
函数返回值：度
函数描述：None 
**********************************************************************/
uint32_t GPS_coord_to_degrees(const char* coordinateString)
{
    const char *fieldSeparator, *remainingString;
    uint8_t degress = 0, minutes = 0;
    uint16_t fractionalMinutes = 0;
    uint8_t digitIndex;

    // 扫描小数点或字段结束
    for (fieldSeparator = coordinateString; isdigit((unsigned char)*fieldSeparator); fieldSeparator++) {
        if (fieldSeparator >= coordinateString + 15)
            return 0; // stop potential fail
    }
    remainingString = coordinateString;

    // 转换度
    while ((fieldSeparator - remainingString) > 2) {
        if (degress)
            degress *= 10;
        degress += DIGIT_TO_VAL(*remainingString++);
    }
    // 转换为分钟
    while (fieldSeparator > remainingString) {
        if (minutes)
            minutes *= 10;
        minutes += DIGIT_TO_VAL(*remainingString++);
    }
	// 转换分数分钟
	// 最多4位，结果是in
	// 千分之一分钟
    if (*fieldSeparator == '.') {
        remainingString = fieldSeparator + 1;
        for (digitIndex = 0; digitIndex < 4; digitIndex++) {
            fractionalMinutes *= 10;
            if (isdigit((unsigned char)*remainingString))
                fractionalMinutes += *remainingString++ - '0';
        }
    }
    return degress * 10000000UL + (minutes * 1000000UL + fractionalMinutes * 100UL) / 6;
}
#endif

