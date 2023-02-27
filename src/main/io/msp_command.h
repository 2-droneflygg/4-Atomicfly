#pragma once

#include "msp.h"

// -----------------------命令宏定义
#define MSP_STATUS                 0       // MSP相关状态
#define MSP_ATTITUDE               1       // 欧拉角
#define MSP_ACC_CALIBRATION        2       // 加速度计校准
#define MSP_ACC_CALIBRATION_STATUS 3       // 加速度计校准状态
#define MSP_MAG_CALIBRATION        4       // 磁力计校准
#define MSP_MAG_CALIBRATION_STATUS 5       // 磁力计校准状态
#define MSP_SYSTEM_CONFIG          6       // 系统配置
#define MSP_SET_SYSTEM_CONFIG1     7       // 设置系统配置1
#define MSP_SET_SYSTEM_CONFIG2     8       // 设置系统配置2
#define MSP_SET_SYSTEM_CONFIG3     9       // 设置系统配置3
#define MSP_SYSTEM_REBOOT          10      // 系统重启
#define MSP_BATTERY_STATUS         11      // 电池状态
#define MSP_BATTERY_CONFIG         12      // 电池配置
#define MSP_VOLTAGE_CONFIG         13      // 电池电压传感器配置
#define MSP_CURRENT_CONFIG         14      // 电池电流传感器配置
#define MSP_SET_BATTERY_CONFIG     15      // 设置电池配置
#define MSP_HEARTBEAT              16      // 心跳包
#define MSP_LINK                   17      // MSP链接
#define MSP_FAILSAFE_CONFIG        18      // 失控保护配置
#define MSP_RXFAIL_CONFIG          19      // 一阶失控保护通道预设
#define MSP_GPS_RESCUE_CONFIG1     20      // 二阶失控保护通道预设
#define MSP_GPS_RESCUE_CONFIG2     21      // 二阶失控保护通道预设
#define MSP_SET_FAILSAFE_CONFIG    22      // 失控保护设置
#define MSP_SET_RXFAIL_CONFIG      23      // 一阶失控保护通道预设
#define MSP_SET_GPS_RESCUE_CONFIG1 24      // 二阶失控保护通道预设
#define MSP_SET_GPS_RESCUE_CONFIG2 25      // 二阶失控保护通道预设
#define MSP_PID_RATE_PROFILE       26      // PID_RATE配置文件
#define MSP_PID_CONFIG             27      // PID配置
#define MSP_RATE_CONFIG            28      // RATE配置
#define MSP_THR_CONFIG             29      // THR配置
#define MSP_FILTER_CONFIG          30      // 滤波器配置
#define MSP_SET_PID_RATE_PROFILE   31      // 设置PID_RATE配置文件
#define MSP_SET_PID_CONFIG         32      // 设置PID配置
#define MSP_SET_RATE_CONFIG        33      // 设置RATE配置
#define MSP_SET_THR_CONFIG         34      // 设置THR配置
#define MSP_SET_FILTER_CONFIG      35      // 设置滤波器配置
#define MSP_COPY_PID_PROFILE       36      // 复制PID配置文件
#define MSP_COPY_RATE_PROFILE      37      // 复制RATE配置文件
#define MSP_RC1                    38      // RC数据
#define MSP_RC2                    39      // RC数据
#define MSP_RX_CONFIG              40      // RX配置
#define MSP_SET_RX_CONFIG          41      // 设置RX配置
#define MSP_MODE_RANGES            42      // 模式范围
#define MSP_SET_MODE_RANGES        43      // 设置模式范围
#define MSP_LINK_STATUS            44      // MSP链接状态
#define MSP_MOTOR_INFO             45      // 电机控制辅助信息
#define MSP_MOTOR_CONTROL_START    46      // 电机控制开始
#define MSP_MOTOR_CONTROL          47      // 电机控制进行
#define MSP_MOTOR_CONTROL_END      48      // 电机控制结束
#define MSP_OSD_CONFIG             49      // OSD配置
#define MSP_SET_OSD_CONFIG         50      // 设置OSD配置
#define MSP_OSD_CHARACTER          51      // OSD字库烧录
#define MSP_CURVE_GYRO             52      // 陀螺仪曲线数据
#define MSP_CURVE_ACC              53      // 加速度计曲线数据
#define MSP_CURVE_MAG              54      // 磁力计曲线数据
#define MSP_CURVE_BARO             55      // 气压计曲线数据
#define MSP_VTX_CONFIG             56      // VTX配置
#define MSP_SET_VTX_CONFIG         57      // 设置VTX配置

bool mspProcessOutCommand(int16_t cmdMSP, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn);
mspResult_e mspProcessInCommand(int16_t cmdMSP, sbuf_t *src, mspPostProcessFnPtr *mspPostProcessFn);

