/******************************************************************************
**  File        : stm32_flash_f405.ld
**  Abstract    : 该链接文件用于STM32F405RG设备
**                1024KByte FLASH, 128KByte RAM 64KByte CCM (F4内核专用的全速64KB RAM)
**  1.设置堆空间大小、栈空间大小、然后根据应用的请求设置栈的位置；
**  2.如果使用了外部内存，则设置内存块的区域和大小；
**  3.当GCC最后链接STM32程序时候，为应用程序划分好对应的内存地址与大小，
**    并将对应的数据、函数入口等信息存放到对应段中。
******************************************************************************/

/* -----------------------指定内存区域----------------------- */
/* 
	NAME1 [(ATTR)] : ORIGIN = ORIGIN1, LENGTH = LEN2
	NAME：存储区域名称
	ATTR：定义该存储区域属性
		R 只读section
		W 读/写section
		X 可执行section
		A 可分配的section
		I 初始化了的section
		L 同I
		! 不满足该字符之后的任何一个属性的section
	ORIGIN：关键字，内存区域开始地址，可简写成org或o
	LENGTH：关键字，内存区域大小，可简写成len或l
*/
MEMORY {
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 10K		   /* 中断向量表, 启动代码 */
    FLASH_CUSTOM_DEFAULTS (r) : ORIGIN = 0x08002800, LENGTH = 6K   /* 配置存储部分地址 */
    FLASH_CONFIG (r)  : ORIGIN = 0x08004000, LENGTH = 16K		   /* PG参数组存储地址 */	
    FLASH1 (rx)       : ORIGIN = 0x08008000, LENGTH = 992K         /* 固件地址 */
    SYSTEM_MEMORY (rx): ORIGIN = 0x1FFF0000, LENGTH = 29K		   /* BootLoader地址 */
    RAM (rwx)         : ORIGIN = 0x20000000, LENGTH = 128K         /* RAM地址 */
    CCM (rwx)         : ORIGIN = 0x10000000, LENGTH = 64K		   /* CCM地址 */
}

/* ---------------------内存region的别名--------------------- */ 
/* REGION_ALIAS(alias,region) */
/* 为region创建一个alias的别名，可以让输出section到region有更灵活的映射 */
REGION_ALIAS("STACKRAM", CCM)
REGION_ALIAS("FASTRAM", CCM)
REGION_ALIAS("VECTAB", RAM)

/* ----------------包含STM32通用FLASH链接脚本---------------- */
INCLUDE "stm32_flash_split.ld"

