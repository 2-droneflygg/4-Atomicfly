/*********************************************************************************
 提供传感器板对齐相关API - 传感器数据板对齐。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#include "common/utils.h"
#include "common/maths.h"
#include "common/axis.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/sensor.h"

#include "boardalignment.h"
 
// 默认值为0，不需要模板 - CW0_DEG 
PG_REGISTER(boardAlignment_t, boardAlignment, PG_BOARD_ALIGNMENT, 0);

/**********************************************************************
函数名称：alignSensorViaRotation
函数功能：传感器数据板对齐
函数形参：向量，旋转方向
函数返回值：None
函数描述：
	传感器更新数据时调用，进行数据板对齐。
**********************************************************************/
void alignSensorViaRotation(float *dest, uint8_t rotation)
{
    const float x = dest[X];
    const float y = dest[Y];
    const float z = dest[Z];

	// 选择板对齐方向
    switch (rotation) {
	    default:
	    case CW0_DEG:
	        dest[X] = x;
	        dest[Y] = y;
	        dest[Z] = z;
	        break;
	    case CW90_DEG:
	        dest[X] = y;
	        dest[Y] = -x;
	        dest[Z] = z;
	        break;
	    case CW180_DEG:
	        dest[X] = -x;
	        dest[Y] = -y;
	        dest[Z] = z;
	        break;
	    case CW270_DEG:
	        dest[X] = -y;
	        dest[Y] = x;
	        dest[Z] = z;
	        break;
	    case CW0_DEG_FLIP:
	        dest[X] = -x;
	        dest[Y] = y;
	        dest[Z] = -z;
	        break;
	    case CW90_DEG_FLIP:
	        dest[X] = y;
	        dest[Y] = x;
	        dest[Z] = -z;
	        break;
	    case CW180_DEG_FLIP:
	        dest[X] = x;
	        dest[Y] = -y;
	        dest[Z] = -z;
	        break;
	    case CW270_DEG_FLIP:
	        dest[X] = -y;
	        dest[Y] = -x;
	        dest[Z] = -z;
	        break;
    }
}

