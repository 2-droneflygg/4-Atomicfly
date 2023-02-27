/*********************************************************************************
 提供CRC校验相关API：
	生成多项式的最高幂次项系数是固定的1，故在简记式中，将最高位的1去掉
	如04C11DB7实际上是104C11DB7
	简称CRC-CCITT，ITU的前身是CCITT。
*********************************************************************************/
#include <stdint.h>

#include "platform.h"

//-------------------------------------------------------------------------------------crc16相关API

/**********************************************************************
函数名称：crc16_ccitt
函数功能：crc16_ccitt算法
函数形参：crc，a
函数返回值：crc
函数描述：None
**********************************************************************/
uint16_t crc16_ccitt(uint16_t crc, unsigned char a)
{
    crc ^= (uint16_t)a << 8;
	// 循环计算8次
    for (int ii = 0; ii < 8; ++ii) {
		// 判断最高位是否为1
        if (crc & 0x8000) {
			// 去掉最高位的1并^ 0x1021
            crc = (crc << 1) ^ 0x1021;
        } else {
        	// 去掉最高位的1
            crc = crc << 1;
        }
    }
    return crc;
}

/**********************************************************************
函数名称：crc16_ccitt_update
函数功能：crc16_ccitt更新
函数形参：crc，data，length
函数返回值：crc计算结果
函数描述：None
**********************************************************************/
uint16_t crc16_ccitt_update(uint16_t crc, const void *data, uint32_t length)
{
	// 获取数据
    const uint8_t *p = (const uint8_t *)data;
	// 结束地址
    const uint8_t *pend = p + length;
	// 迭代运算
    for (; p != pend; p++) {
        crc = crc16_ccitt(crc, *p);
    }
    return crc;
}

//-------------------------------------------------------------------------------------crc8相关API

/**********************************************************************
函数名称：crc8_dvb_s2
函数功能：crc8_dvb_s2算法
函数形参：crc，a
函数返回值：crc
函数描述：None
**********************************************************************/
uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

/**********************************************************************
函数名称：crc8_dvb_s2_update
函数功能：crc8_dvb_s2_update更新
函数形参：crc，data，length
函数返回值：crc
函数描述：None
**********************************************************************/
uint8_t crc8_dvb_s2_update(uint8_t crc, const void *data, uint32_t length)
{
    const uint8_t *p = (const uint8_t *)data;
    const uint8_t *pend = p + length;

    for (; p != pend; p++) {
        crc = crc8_dvb_s2(crc, *p);
    }
    return crc;
}

