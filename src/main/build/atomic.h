/**************************************************************************************
原子操作：
	STM32中断屏蔽寄存器BASEPRI,基于优先级屏蔽异常或中断，保护临界段。
	注意：屏蔽的是主优先级。
	如果某些代码段不允许被中断打断，那么这段代码就必须用关中断的方式给保护起来。
***************************************************************************************/
#pragma once

#include <stdint.h>

/**********************************************************************
函数名称：__basepriRestoreMem
函数功能：ARM BASEPRI操作
函数形参：val
函数返回值：None
函数描述：
	1.内联函数;
	2.使用全局内存屏蔽恢复BASEPRI(称为cleanup函数)。
**********************************************************************/
static inline void __basepriRestoreMem(uint8_t *val)
{
	// 设置基本优先级 - 将给定的值赋给基本优先级寄存器
	// 设置为val后，屏蔽所有优先级数值大于等于val的中断和异常
    __set_BASEPRI(*val);
}

/**********************************************************************
函数名称：__basepriSetMemRetVal
函数功能：使用全局内存屏障设置BASEPRI_MAX
函数形参：prio
函数返回值：返回true
函数描述：内联函数
**********************************************************************/
static inline uint8_t __basepriSetMemRetVal(uint8_t prio)
{
	// 使用条件设置基本优先级
	//	- 只有当BASEPRI屏蔽被禁用时，才会将给定的值分配给基本优先级寄存器，
	//    或者新值增加BASEPRI优先级级别。

	// 和BASEPRI类似，但有个限制，即后写入的优先级数值要比当前的BASEPRI值小才会起作用，
	// 否则不起作用。影响范围最广，影响CPU内的所有中断源。
	// 事实上BASEPRI_MAX和BASSEPRI是操作同一个寄存器，不过BASEPRI_MAX是一个条件写指令。
    __set_BASEPRI_MAX(prio);
    return 1;
}

/**********************************************************************
	运行带有较高抢占优先级的块(使用BASEPRI_MAX)。
	在退出时恢复抢占优先级处理所有的退出路径，实现为for循环，
	拦截中断和继续Full memory barrier被放置在block的开始处和出口处。
	__unused__属性用于抑制CLang警告。
**********************************************************************/
#define ATOMIC_BLOCK(prio) for ( uint8_t __basepri_save __attribute__ ((__cleanup__ (__basepriRestoreMem), __unused__)) = __get_BASEPRI(), \
                                     __ToDo = __basepriSetMemRetVal(prio); __ToDo ; __ToDo = 0 )
                                     
