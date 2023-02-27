/*********************************************************************************
 提供IO描述信息块结构体定义。
*********************************************************************************/
#pragma once

#include "platform.h"
#include "drivers/io.h"

/* ------------------------------IO描述结构体------------------------------ */	
typedef struct ioRec_s {
    GPIO_TypeDef *gpio;		 	 // GPIO端口
    uint16_t pin;			  	 // 引脚
    resourceOwner_e owner;	  	 // 资源映射
    uint8_t index;			  	 // 资源索引
} ioRec_t;
extern ioRec_t ioRecs[];

int IO_GPIOPortIdx(IO_t io);
int IO_GPIOPinIdx(IO_t io);
int IO_GPIO_PinSource(IO_t io);
int IO_GPIO_PortSource(IO_t io);
int IO_EXTI_PortSourceGPIO(IO_t io);
int IO_EXTI_PinSource(IO_t io);
GPIO_TypeDef* IO_GPIO(IO_t io);
uint16_t IO_Pin(IO_t io);
uint32_t IO_EXTI_Line(IO_t io);
ioRec_t *IO_Rec(IO_t io);

