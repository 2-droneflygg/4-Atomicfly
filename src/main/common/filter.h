#pragma once
#include <stdbool.h>

struct filter_s;
typedef struct filter_s filter_t;

/* ----------------------一阶巴特沃斯滤波结构体--------------------- */	
typedef struct pt1Filter_s {
    float state;								// 滤波输出	
    float k;									// 滤波系数
} pt1Filter_t;

/* -----------------------二阶巴特沃斯滤波结构体-------------------- */	
// 保存通过滤波器更新样本所需的数据 
typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquadFilter_t;

/* --------------------------滞后平均移动结构体--------------------- */	
typedef struct laggedMovingAverage_s {
    uint16_t movingWindowIndex;					// 移动窗口索引	
    uint16_t windowSize;						// 窗口大小
    float movingSum;							// 移动和
    float *buf;									// 缓存
    bool primed;								// 就绪状态
} laggedMovingAverage_t;

/* --------------------------低通滤波器类型枚举--------------------- */	
typedef enum {
    FILTER_PT1 = 0,								// 一阶
} lowpassFilterType_e;

// 滤波应用函数指针
typedef float (*filterApplyFnPtr)(filter_t *filter, float input);

float nullFilterApply(filter_t *filter, float input);
void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate);
float biquadFilterApply(biquadFilter_t *filter, float input);
void laggedMovingAverageInit(laggedMovingAverage_t *filter, uint16_t windowSize, float *buf);
float laggedMovingAverageUpdate(laggedMovingAverage_t *filter, float input);
float pt1FilterGain(float f_cut, float dT);
void pt1FilterInit(pt1Filter_t *filter, float k);
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k);
float pt1FilterApply(pt1Filter_t *filter, float input);
int32_t quickMedianFilter3(int32_t * v);

