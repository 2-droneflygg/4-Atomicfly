/*********************************************************************************
Dshot600数字电调通讯协议：每秒传输600k比特：
	0：低电平大概占据625ns， 
	1：高电平大概占据1250ns， 
	一个bit周期大概为1.67us

	Dshot600一帧为16bit ： 16位 = 11位油门信号 + 1位电调信息回传 + 4位循环冗余校验：
		0-10bit为油门数据(高位在前).
		11bit请求数据回传标志(Tlm request)：
			0时表示不请求，为1表示请求，
			若发送请求，支持数据回传功能的ESC会回传ESC的各种状态信息给控制端.										
		12-15bit为CRC校验位 ：	这4个校验位是对前面11个油门值+1个回传标志这12个位按每组4位分3组，
			也就是三组4位的数据进行异或计算(C语言运算符就是^)，
			计算结果取低4位作为这串数据的校验码插入								
	（16位数据发送时高位在前，逐位发送）
	16-17bit为两个周期的低电平，表示帧间隔（通常有至少两毫秒的时间来表示复位信号，即一条指令结束）
	所以说一帧数据大约为26.7us.

	油门值(Throttle) 11位，0~2047共2048个油门值，可以表示非常精细的油门信号
   （两帧数据（指令）之间，通常有至少两毫秒的时间来表示复位信号，即一条指令结束）
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_DSHOT
#include "build/atomic.h"

#include "common/maths.h"
#include "common/time.h"

#include "config/feature.h"

#include "drivers/dshot.h"
#include "drivers/motor.h"
#include "drivers/timer.h"

#include "drivers/dshot_dpwm.h" 
#include "drivers/dshot_command.h"
#include "drivers/nvic.h"

#include "fc/rc_controls.h"     

#include "rx/rx.h"

/**********************************************************************
函数名称：dshotInitEndpoints
函数功能：初始化DSHOT结束点
函数形参：电机配置信息，输出限制（0-1.0），最低输出，最高输出，电机停止
函数返回值：None
函数描述：None
**********************************************************************/
void dshotInitEndpoints(const motorConfig_t *motorConfig, 
								float outputLimit, float *outputLow, float *outputHigh, 
								float *disarm) {
	// 输出限制偏移 = （Dshot油门差） * （1 - outputLimit）
	// 比如百分之90 - 1999 * 0.1 = 199.9
    float outputLimitOffset = (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE) * (1 - outputLimit);
	// 电机停止命令
    *disarm = DSHOT_CMD_MOTOR_STOP;
	// 最低输出 = 48 + （19.99） * 5.5 = 157.945
    *outputLow = DSHOT_MIN_THROTTLE + ((DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE) / 100.0f) 
					* CONVERT_PARAMETER_TO_PERCENT(motorConfig->digitalIdleOffsetValue);
	// 最高输出 = Dshot最大油门 - 输出限制偏移
	// 百分之90 - 2047 - 199.9 = 1,847.1
    *outputHigh = DSHOT_MAX_THROTTLE - outputLimitOffset;
}
								
/**********************************************************************
函数名称：dshotConvertFromExternal
函数功能：外部值转换电机值
函数形参：externalValue
函数返回值：motorValue
函数描述：None
**********************************************************************/
float dshotConvertFromExternal(uint16_t externalValue)
{
    float motorValue;
	// 限制范围 - 1000~2000
    externalValue = constrain(externalValue, PWM_RANGE_MIN, PWM_RANGE_MAX);
	// 计算电机值 ：
	// 		-- 如果当前值为最小值 - 电机停止命令
	//      -- 否则将PWM范围缩放到Dshot范围 -（PWM_RANGE_MIN + 1, PWM_RANGE_MAX）->（DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE）
	motorValue = (externalValue == PWM_RANGE_MIN) ? DSHOT_CMD_MOTOR_STOP : scaleRangef(externalValue, PWM_RANGE_MIN + 1, PWM_RANGE_MAX, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
    return motorValue;
}

/**********************************************************************
函数名称：dshotConvertToExternal
函数功能：电机值转外部值
函数形参：motorValue
函数返回值：externalValue
函数描述：None
**********************************************************************/
uint16_t dshotConvertToExternal(float motorValue)
{
    uint16_t externalValue;
	// 计算外部值 - 同理将Dshot范围缩放到PWM范围
    externalValue = (motorValue < DSHOT_MIN_THROTTLE) ? PWM_RANGE_MIN : scaleRangef(motorValue, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE, PWM_RANGE_MIN + 1, PWM_RANGE_MAX);
    return externalValue;
}

/**********************************************************************
函数名称：prepareDshotPacket
函数功能：准备Dshot数据包
函数形参：Dshot协议结构体
函数返回值：packet
函数描述：
	后4位校验数据：
		对前面11个油门值+1个回传标志这12个位按4位一组分3组
		三组4位的数据进行异或计算
		计算结果取低4位作为这串数据的校验码插入
		(低四位分别与另两组高四位异或)
**********************************************************************/
uint16_t prepareDshotPacket(dshotProtocolControl_t *pcb)
{
    uint16_t packet;

    ATOMIC_BLOCK(NVIC_PRIO_DSHOT_DMA) {
		// 完整前11位数据
        packet = (pcb->value << 1) | (pcb->requestTelemetry ? 1 : 0);
 	    // 重置遥测请求，确保连续只触发一次
		pcb->requestTelemetry = false;   
    }

    // 计算后4位校验和
    unsigned csum = 0;
    unsigned csum_data = packet;
    for (int i = 0; i < 3; i++) {			// 4位一组分3组次计算
        csum ^=  csum_data;  				// 进行异或计算     1100 0101 1010
        csum_data >>= 4;					// 进行下一组计算   0000 1100 0101
    }	
    // 追加校验和
    csum &= 0xf;
    packet = (packet << 4) | csum;			// 异或后的12位数据 & 1111（目的是取低4位）
	// 返回16位数据（原始12位数据或上CRC低四位数据）
    return packet;							
}
#endif 

