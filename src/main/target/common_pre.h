/**********************************************************************
 飞控功能预配置：
**********************************************************************/
#pragma once

// ---------------------------------------------------------------------------------类型转换警告
// #pragma GCC诊断警告“-Wconversion”
// - wfilling可以打开来检查struct的填充
#pragma GCC diagnostic ignored "-Wsign-conversion"

// ---------------------------------------------------------------------------------根据FPU启用状态控制RC通道数量
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
#define DEFAULT_AUX_CHANNEL_COUNT  MAX_AUX_CHANNEL_COUNT
#else
#define DEFAULT_AUX_CHANNEL_COUNT  6
#endif

// ---------------------------------------------------------------------------------MCU超频控制宏
// 默认MCU超频 - 不超频
#define DEFAULT_CPU_OVERCLOCK 0				 

// ---------------------------------------------------------------------------------指定段
// 启用FAST_RAM - 64KByte CCM (F4内核专用的全速64KB RAM)
#define USE_FAST_RAM	
// GUN C - __attribute__机制
// __attribute__((section("section_name"))) - 将作用的函数或数据放入指定名为"section_name"的段
#ifdef USE_FAST_RAM			
// 数据段 - BSS段：存放未初始化的外部变量、静态局部变量和常量，存放在BSS段中的变量均默认初始化为0（预先清空）
#define FAST_RAM_ZERO_INIT          __attribute__ ((section(".fastram_bss"), aligned(4)))
// 数据段 - 静态数据区：存放已初始化的外部变量、静态局部变量和常量
#define FAST_RAM                    __attribute__ ((section(".fastram_data"), aligned(4)))
#else
#define FAST_RAM_ZERO_INIT
#define FAST_RAM
#endif // USE_FAST_RAM

// ---------------------------------------------------------------------------------飞控功能控制宏
// ---------------------------------------------超频功能
//	启用超频	
#define USE_OVERCLOCK	
//	启用IIC超频	- 硬件IIC
#define I2C1_OVERCLOCK 		true					 
#define I2C2_OVERCLOCK 		true
#define I2C3_OVERCLOCK	 	true

// ---------------------------------------------任务调度功能
// 陀螺仪线程执行频率 - 8kHz[125us]
#define TASK_GYROPID_DESIRED_PERIOD     125  
// 任务调度器延时限制 - (10khz)以防止调度程序阻塞
#define SCHEDULER_DELAY_LIMIT           10   

// ---------------------------------------------ADC功能
// 启用ADC
#define USE_ADC		

// ---------------------------------------------DMA功能
// 启用DMA 
#define USE_DMA		
// 启用DMA信息块管理
#define USE_DMA_SPEC   

// ---------------------------------------------定时器功能
// 启用定时器
#define USE_TIMER				
// 启用定时器管理
#define USE_TIMER_MGMT						 
    					 
// ---------------------------------------------陀螺仪滤波功能
// 启用动态低通滤波         		（第一级为动态）
#define USE_DYN_LPF							 
// 启用第二级陀螺仪低通滤波 		（第二级为静态）
#define USE_GYRO_LPF2						 
	// 低通滤波器有两个类型：静态和动态。
	// 同一时刻只能启用一个类型（静态或动态） - 如果开启动态低通滤波，那么静态低通滤波器1将变为动态低通滤波器1
	// 静态滤波器需要预先定义一个截止频率
	// 动态滤波器则通过设置最低和最高值，来定义一个截止频率的移动范围，截止频率跟随油门摇杆在最低和最高范围内移动。

// ---------------------------------------------PID辅助功能
// 启用前馈功能
#define USE_INTERPOLATED_SP 			     
// 启用油门线性化功能
#define USE_THRUST_LINEARIZATION			 
// 启用TPA模式，允许衰减PD或单独衰减D
#define USE_TPA_MODE						 
// 启用油门增压功能
#define USE_THROTTLE_BOOST					 
	// 通过根据摇杆快速移动量来短暂提高油门变化值以增加电机的加速扭矩，使得油门响应更为迅速。
// 启用I项释放功能 - 动态调节I
#define USE_ITERM_RELAX						 
	// 在快速移动时限制 I 值的积累。这特别有助于降低翻滚等快速移动结尾时的反弹。可以选择该功能生效的轴向，以及快速移动的判断依据是陀螺仪还是设定点（摇杆）。
	// I值释放只在四轴进行大动作快速机动时抑制I值的积累，从而可减少翻滚动作结束时的过冲，另一方面使得可以提高默认的I值，从而使四轴的飞行手感更为跟手。
// 启用D_MIN   - 动态调节D
#define USE_D_MIN							 
	// 控制正常前进飞行中的阻尼(D-term)。
	// D Min 提供了一个方法，使飞机在正常飞行中可以维持一个较低的 D 值，又可以在如翻转等快速动作时提高到一个较高的值来抑制过冲。
	// 它也会在发生洗桨时提高 D 值。调整增益(Gain) 可以调整 D 到达最高值的速度，同时它基于陀螺仪数据来确定快速移动与洗桨事件。
	// 超前(Advance) 通过使用设定点而不是陀螺仪来判定快速移动的方法来更早地提高 D 值。

// ---------------------------------------------配置文件功能
#define USE_PROFILE_NAMES					 // 启用配置文件名称
#define PID_PROFILE_COUNT 			3		 // PID配置文件数量
#define CONTROL_RATE_PROFILE_COUNT  6		 // 速率控制配置文件数量

// ---------------------------------------------失控保护功能
// 启用反自旋功能（判断Z轴角速度是否超过阈值） - 防止桨叶安装错误或者电机转向错误导致死亡螺旋起飞
#define USE_YAW_SPIN_RECOVERY			
// 启用防止失控起飞（判断三轴pidSum和三轴角速度是否有超过失控起飞阈值） - 仅在上电首次起飞时在失控事件期间进行干预，不会对正常飞行产生任何影响
#define USE_RUNAWAY_TAKEOFF     			 

// ---------------------------------------------GPS功能
// 启用GPS
#define USE_GPS						
// 启用GPS NMEA协议
#define USE_GPS_NMEA			
// 启用GPS救援功能
#define USE_GPS_RESCUE				
// 启用垂直速度表功能
#define USE_VARIO							 
	  
// ---------------------------------------------接收机功能
// 启用串行接收机 - 串口
#define USE_SERIAL_RX						 
// 启用SBUS协议 - 遥控数据
#define USE_SERIALRX_SBUS           	
// 启用链接质量功能
#define USE_RX_LINK_QUALITY_INFO			 

// ---------------------------------------------OSD功能
// 启用OSD
#define USE_OSD						
// 启用OSD_CMS菜单 
#define USE_CMS								
// 启用OSD CMS扩展菜单
#define USE_EXTENDED_CMS_MENUS				
// 当Failsafe触发时显示FAILSAFE菜单
#define USE_CMS_FAILSAFE_MENU		
// 当GPS_RESCUE触发时显示GPS_RESCUE菜单
#define USE_CMS_GPS_RESCUE_MENU				 

// ---------------------------------------------图传功能
// 启用图传常用功能
#define USE_VTX_COMMON						 
// 启用图传控制功能
#define USE_VTX_CONTROL						
// 启用黑羊图传协议 
#define USE_VTX_SMARTAUDIO		

// ---------------------------------------------电机功能
// 启用电机控制
#define USE_MOTOR					
// 启用PWM输出
#define USE_PWM_OUTPUT						 
// 启用DSHOT
#define USE_DSHOT					
// 使用DMAR来驱动Dshot
#define USE_DSHOT_DMAR	

