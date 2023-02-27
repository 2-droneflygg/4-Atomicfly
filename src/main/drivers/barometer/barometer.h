/*********************************************************************************
 提供气压计驱动相关信息。
*********************************************************************************/
#pragma once

#include "drivers/bus.h" 
#include "drivers/exti.h"

struct baroDev_s;
typedef void (*baroOpFuncPtr)(struct baroDev_s *baro);                                 // 气压开始操作
typedef bool (*baroGetFuncPtr)(struct baroDev_s *baro);                                // 气压读取/get操作
typedef void (*baroCalculateFuncPtr)(int32_t *pressure, int32_t *temperature);  // 气压计算(填充参数为压力和温度)

/* -----------------------------气压计设备结构体------------------------------ */	
// 变量名中的“u”表示“未补偿”，“t”表示温度，“p”表示压力。
typedef struct baroDev_s {
    busDevice_t busdev;					   // 设备总线
#ifdef USE_EXTI
    extiCallbackRec_t exti;                // 外部中断回调函数
#endif
    bool           combined_read;
    uint16_t       ut_delay;
    uint16_t       up_delay;
    baroOpFuncPtr  start_ut;
    baroGetFuncPtr read_ut;
    baroGetFuncPtr get_ut;
    baroOpFuncPtr  start_up;
    baroGetFuncPtr read_up;
    baroGetFuncPtr get_up;
    baroCalculateFuncPtr calculate;
} baroDev_t;

