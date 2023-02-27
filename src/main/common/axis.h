/*********************************************************************************
 提供相关轴系描述。
*********************************************************************************/
#pragma once

/* ------------------------------X、Y、Z轴枚举------------------------------ */	
typedef enum {
    X = 0,
    Y,
    Z
} axis_e;
// 3轴
#define XYZ_AXIS_COUNT 3

/* -----------------------------飞行动态索引枚举---------------------------- */	
// 欧拉角描述
typedef enum {
    FD_ROLL = 0,			// 横滚
    FD_PITCH,				// 俯仰
    FD_YAW					// 偏航
} flight_dynamics_index_t;
// 3轴
#define FLIGHT_DYNAMICS_INDEX_COUNT 3

/* -------------------------------角度索引枚举------------------------------ */	
// 救援模式使用
typedef enum {
    AI_ROLL = 0,			// 横滚
    AI_PITCH				// 俯仰
} angle_index_t;
// 角度索引数据
#define ANGLE_INDEX_COUNT 2				
// 获取方向（是否取反）
#define GET_DIRECTION(isReversed) ((isReversed) ? -1 : 1)

