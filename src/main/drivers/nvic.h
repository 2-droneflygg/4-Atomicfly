/*********************************************************************************
 嵌套向量中断控制器 (NVIC)：
	接收中断请求，判断优先级高低，指挥CPU按先后顺序处理中断源。
	提供一系列预先设定好的外设NVIC中断优先级。
*********************************************************************************/
#pragma once

// NVIC优先级分组配置 - 选择组2（抢占优先级2，响应优先级2）
#define NVIC_PRIORITY_GROUPING  	   NVIC_PriorityGroup_2

// 构建NVIC中断优先级 - 抢占优先级，响应优先级
#define NVIC_BUILD_PRIORITY(base,sub) (((((base)<<(4-(7-(NVIC_PRIORITY_GROUPING>>8))))|((sub)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8)))))<<4)&0xf0)
// 抢占优先级 - 数值为NVIC_BUILD_PRIORITY设置的base数值
#define NVIC_PRIORITY_BASE(prio)      (((prio)>>(4-(7-(NVIC_PRIORITY_GROUPING>>8))))>>4)
// 响应优先级 - 数值为NVIC_BUILD_PRIORITY设置的sub数值
#define NVIC_PRIORITY_SUB(prio)       (((prio)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8))))>>4)

// 建立外设NVIC中断优先级（不能用 0）
#define NVIC_PRIO_MAX                      NVIC_BUILD_PRIORITY(0, 1)
#define NVIC_PRIO_TIMER                    NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_DSHOT_DMA                NVIC_BUILD_PRIORITY(2, 1)
#define NVIC_PRIO_MPU_INT_EXTI             NVIC_BUILD_PRIORITY(0x0f, 0x0f)
#define NVIC_PRIO_SERIALUART_TXDMA         NVIC_BUILD_PRIORITY(1, 1)  // 所有SERIALUARTx TXDMA中最高的
#define NVIC_PRIO_SERIALUART1_TXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART1_RXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART2_TXDMA        NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SERIALUART2_RXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART2              NVIC_BUILD_PRIORITY(1, 2)
#define NVIC_PRIO_SERIALUART1              NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART3_TXDMA        NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SERIALUART3_RXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART3              NVIC_BUILD_PRIORITY(1, 2)
#define NVIC_PRIO_SERIALUART6_TXDMA        NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SERIALUART6_RXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART6              NVIC_BUILD_PRIORITY(1, 2)
#define NVIC_PRIO_I2C_ER                   NVIC_BUILD_PRIORITY(0, 0)
#define NVIC_PRIO_I2C_EV                   NVIC_BUILD_PRIORITY(0, 0)
#define NVIC_PRIO_USB                      NVIC_BUILD_PRIORITY(2, 0)
