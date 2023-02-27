/******************************************************************************
**  File        : stm32_flash_split.ld
**  Abstract    : 用于STM32设备的通用FLASH链接脚本
******************************************************************************/

/* --------------------------入口点-------------------------- */
/* 
	使用ENTRY命令来指定输出的可执行文件的入口函数;
	就是程序加载完成后要执行的第一条指令的位置;
	该命令的参数是一个符号名，有可能是入口函数的函数名，也有可能是节区的名字.
*/
/* 入口点为启动文件复位处理 */
ENTRY(Reset_Handler)

/* --------------------用户模式栈的最高地址------------------ */
_Hot_Reboot_Flags_Size = 16;
/* end of RAM */
_estack = ORIGIN(STACKRAM) + LENGTH(STACKRAM) - _Hot_Reboot_Flags_Size;   

/* -------------如果堆和栈不适合RAM，生成一个链接错误-------- */
/* 所需堆量  */
_Min_Heap_Size = 0;      			
/* 所需堆量 */
_Min_Stack_Size = 0x800; 	

/* -------------FLASH用户配置文件（PG）存储的基地地址-------- */
/* 起始地址 */
__config_start = ORIGIN(FLASH_CONFIG);            
/* 结束地址 */
__config_end = ORIGIN(FLASH_CONFIG) + LENGTH(FLASH_CONFIG);										

/* -----------------------定义输出部分----------------------- */
/*
	1.链接脚本用来指定目标文件生成输出文件时的规则：
		命令：SECTIONS
		精确控制输入段在输出文件中的顺序、位置和所在段.
		
	2.相关语法：
		(1)定位符号.表示当前地址，可以被赋值也可以赋值给某个变量：
			链接器链接是按照SECTIONS里的段顺序排列的，前面的排列完之后就能计算出当前地址.
		(2)PROVIDE关键字：
			定义一个(目标文件内被引用但没定义)符号;
			相当于定义一个全局变量的符号表，其他C文件可以通过该符号来操作对应的存储内存;
			注意：在链接脚本中声明符号，没有存储内存，并不是一个变量，该符对应（映射）的是一个地址；
			C文件中引用用法如下：
				// 引用该变量
				extern char _etext;
				char *p = &_etext;
				(若在链接脚本中"_etext = 0x100;"，即表示符号_etext对应的地址为0X100， 
				此时&_etext的值为0x100，char a=*p；表示为从0X100地址取值存储的值赋值给变量a.)
		(3)KEEP关键字：
			在链接命令行内使用了选项–gc-sections后，链接器可能将某些它认为没用的section过滤掉;
			此时就有必要强制链接器保留一些特定的section.
			可用KEEP()关键字达此目的：如KEEP(*(.text))，防止被优化.
		(4)ALIGN关键字：
			表示字节对齐，如. = ALIGN(4);表示从该地址开始后面的存储进行4字节对齐.
*/
SECTIONS {
	/* ------------------------------------------ISR向量表在FLASH开始加载，但它被链接(保留空间)在VECTAB区域的开始 */
	/* 可以别名为FLASH或RAM，表可以从FLASH复制到RAM，用于表重定位 */
	_isr_vector_table_flash_base = LOADADDR(.isr_vector);
	PROVIDE (isr_vector_table_flash_base = _isr_vector_table_flash_base);

	/* ------------------------------------------启动代码首先进入FLASH,中断向量表放在.isr_vector段中 */
	.isr_vector : {
		. = ALIGN(4);
		PROVIDE (isr_vector_table_base = .);
		KEEP(*(.isr_vector)) 
		. = ALIGN(4);
		PROVIDE (isr_vector_table_end = .);
	} >VECTAB AT> FLASH

	/* ------------------------------------------程序代码和其他部分数据存放在.text段中 */
	.text : {
		. = ALIGN(4);
		*(.text)           		   /* .text sections (code) */
		*(.text*)          		   /* .text* sections (code) */
		*(.rodata)         		   /* .rodata sections (constants, strings, etc.) */
		*(.rodata*)        		   /* .rodata* sections (constants, strings, etc.) */
		*(.glue_7)         		   /* glue arm to thumb code */
		*(.glue_7t)        		   /* glue thumb to arm code */
		*(.eh_frame)
		KEEP (*(.init))
		KEEP (*(.fini))
		. = ALIGN(4);
		_etext = .;        		   /* 在代码的末尾定义一个全局符号 */
	} >FLASH1

	/* ------------------------------------------外部扩展的空间地址（如FSMC外部的地址空间的一个段） */
	.ARM.extab : { 
		*(.ARM.extab* .gnu.linkonce.armextab.*) 
	} >FLASH
	.ARM : {
		__exidx_start = .;
		  *(.ARM.exidx*)
		  __exidx_end = .;
	} >FLASH

	/* ------------------------------------------.preinit_array 和 .init_array 保存程序或共享对象加载时的初始化函数指针 */
	.preinit_array : {
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array*))
		PROVIDE_HIDDEN (__preinit_array_end = .);
	} >FLASH
	.init_array : {
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array*))
		PROVIDE_HIDDEN (__init_array_end = .);
	} >FLASH

	/* ------------------------------------------.fini_array 保存程序或共享对象退出时的退出函数地址 */
	.fini_array : {
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP (*(.fini_array*))
		KEEP (*(SORT(.fini_array.*)))
		PROVIDE_HIDDEN (__fini_array_end = .);
	} >FLASH

	/* ------------------------------------------存储配置部分的地址，这样就可以从十六进制文件中获取它 */
	.custom_defaults : {
		. = ALIGN(4);
		KEEP (*(.custom_defaults_start_address))
		. = ALIGN(4);
		KEEP (*(.custom_defaults_end_address))
		. = ALIGN(4);
		__custom_defaults_internal_start = .;
		*(.custom_defaults);
	} >FLASH_CUSTOM_DEFAULTS

	/* ------------------------------------------用于启动时初始化数据 */
	_sidata = LOADADDR(.data);

	/* ------------------------------------------.data 存放经过初始化的全局变量和静态变量 */
	.data : {
		. = ALIGN(4);
		_sdata = .;        		   /* 在数据开始时创建一个全局符号 */
		*(.data)           		   /* .data sections */
		*(.data*)          		   /* .data* sections */
		. = ALIGN(4);
		_edata = .;        		   /* 在代码的末尾定义一个全局符号 */
	} >RAM AT> FLASH

	/* ------------------------------------------未初始化数据段 */
	. = ALIGN(4);
	.bss (NOLOAD) : {
		/* 启动时用来初始化.bss区段 */
		_sbss = .;         		   /* 在bss开始处定义一个全局符号 */
		__bss_start__ = _sbss;
		*(.bss)
		*(SORT_BY_ALIGNMENT(.bss*))
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;         		   /* 在bss端定义一个全局符号 */
		__bss_end__ = _ebss;
	} >RAM

	/* ------------------------------------------在启动时用于初始化CCM_data */
	_sfastram_idata = LOADADDR(.fastram_data);

	/* ------------------------------------------初始化CCM_RAM - 数据段 */
	.fastram_data : {
		. = ALIGN(4);
		_sfastram_data = .;        /* 在数据开始处创建一个全局符号 */
		*(.fastram_data)           /* .data sections */
		*(.fastram_data*)          /* .data* sections */
		. = ALIGN(4);
		_efastram_data = .;        /* 在数据端定义一个全局符号 */
	} >FASTRAM AT> FLASH1

	/* ------------------------------------------初始化CCM_RAM - BSS段 */
	. = ALIGN(4);
	.fastram_bss (NOLOAD) : {
		__fastram_bss_start__ = .;
		*(.fastram_bss)
		*(SORT_BY_ALIGNMENT(.fastram_bss*))
		. = ALIGN(4);
		__fastram_bss_end__ = .;
	} >FASTRAM

	/* ------------------------------------------用户堆栈部分，用于检查是否有足够的剩余RAM */
	_heap_stack_end = ORIGIN(STACKRAM) + LENGTH(STACKRAM) - _Hot_Reboot_Flags_Size;
	_heap_stack_begin = _heap_stack_end - _Min_Stack_Size  - _Min_Heap_Size;
	. = _heap_stack_begin;
	._user_heap_stack : {
		. = ALIGN(4);
		PROVIDE ( end = . );
		PROVIDE ( _end = . );
		. = . + _Min_Heap_Size;
		. = . + _Min_Stack_Size;
		. = ALIGN(4);
	} >STACKRAM = 0xa5

	/* ------------------------------------------系统内存(只读引导加载程序)中断向量 - BootLoader */
	.system_isr_vector (NOLOAD) : {
		. = ALIGN(4);
		PROVIDE (system_isr_vector_table_base = .);
		KEEP(*(.system_isr_vector))       /* Bootloader code */
		. = ALIGN(4);
	} >SYSTEM_MEMORY

	/* ------------------------------------------用户PG寄存器（固件配置） */
	.pg_registry : {
		PROVIDE_HIDDEN (__pg_registry_start = .);
		KEEP (*(.pg_registry))
		KEEP (*(SORT(.pg_registry.*)))
		PROVIDE_HIDDEN (__pg_registry_end = .);
	} >FLASH

	/* ------------------------------------------用户PG寄存器数据复位 */
	.pg_resetdata : {
		PROVIDE_HIDDEN (__pg_resetdata_start = .);
		KEEP (*(.pg_resetdata))
		PROVIDE_HIDDEN (__pg_resetdata_end = .);
	} >FLASH

	/* ------------------------------------------从标准库中删除信息 */
	/DISCARD/ : {
		libc.a ( * )
		libm.a ( * )
		libgcc.a ( * )
	}
	.ARM.attributes 0 : { *(.ARM.attributes) }
}
