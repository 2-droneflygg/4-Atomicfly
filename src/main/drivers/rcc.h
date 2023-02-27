#pragma once

#include "rcc_types.h"

/* ------------------------------RCC标签枚举------------------------------ */	
enum rcc_reg {
    RCC_EMPTY = 0,   // 确保默认值(0)不启用任何东西
    RCC_AHB,
    RCC_APB2,
    RCC_APB1,
    RCC_AHB1,
};

// RCC配置宏 - stm32f4xx.h(9211行)
//                             （RCC标签 << 5）    | LOG2_32BIT（RCC_AHB1ENR_GPIOxEN）
// LOG2_32BIT（RCC_AHB1ENR_GPIOxEN） -  缩小数据的绝对数值，方便计算。
#define RCC_ENCODE(reg, mask)   (((reg) << 5) | LOG2_32BIT(mask))              // RCC编码
// RCC_AHB1外设时钟使能寄存器 (RCC_AHB1ENR)       		RCC_AHB1ENR_GPIOxEN
#define RCC_AHB1(periph) RCC_ENCODE(RCC_AHB1, RCC_AHB1ENR_ ## periph ## EN)    // AHB1编码
// RCC_APB1外设时钟使能寄存器 (RCC_APB1ENR) 				RCC_APB1ENR_periphEN
#define RCC_APB1(periph) RCC_ENCODE(RCC_APB1, RCC_APB1ENR_ ## periph ## EN)    // APB1编码
// RCC_APB2外设时钟使能寄存器 (RCC_APB2ENR) 				RCC_APB2ENR_periphEN
#define RCC_APB2(periph) RCC_ENCODE(RCC_APB2, RCC_APB2ENR_ ## periph ## EN)    // APB2编码

void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState);
void RCC_ResetCmd(rccPeriphTag_t periphTag, FunctionalState NewState);

