/**********************************************************************
> Project_Name：FPV四轴飞行器
> Author：丁相府
> Mail：1971738522@qq.com
> Compiling_environment：LINUX - GUNC Makefile
> Crossing_compile_evironment: arm-none-eabi-gcc 5.4.1
> Code_IDE：sourceinsight
> Program_download: SWD J-FLASH
> Hardware(core):
    GYRO/ACC: MPU6000 - SPI
	BARO:     BMP280 - SPI
	MAG：      HMC5883L - IIC
	GPS：      BEITIAN_BN880 - NMEA0183协议
	OSD:      AT7456E - SPI
	ESC:      PWM - DSHOT600协议
	RX:       SBUS - USART
	VTX:	  SMARTAUDIO - USART
**********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "platform.h"             
#include "fc/init.h"               
#include "scheduler/scheduler.h"   

/**********************************************************************
函数名称：main
函数功能：主函数
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
int main(void)
{
	/* ---系统及硬件初始化--- */
    init();
	/* ------- 主循环 ------- */
    while (true) {
		// 任务调度 - 基于时间片
        scheduler();
    }
    return 0;
}

