#pragma once

#include "common/time.h"
#include "config/config.h"

/* --------------------------任务执行周期计算宏定义-------------------------- */	
#define TASK_PERIOD_HZ(hz) (1000000 / (hz))	   // hz
#define TASK_PERIOD_MS(ms) ((ms) * 1000)	   // ms
#define TASK_PERIOD_US(us) (us)                // us
// 如果陀螺任务即将运行，则不要运行任何其他任务
#define GYRO_TASK_GUARD_INTERVAL_US 10   
// 加载一个百分比
#define LOAD_PERCENTAGE_ONE 100

/* ------------------------------任务优先级枚举------------------------------ */	
typedef enum {
    TASK_PRIORITY_REALTIME = -1, 			   // 任务将在调度逻辑之外运行
    TASK_PRIORITY_IDLE = 0,      			   // 禁用动态调度，只有在没有其他任务处于活动状态时才执行任务
    TASK_PRIORITY_LOW = 1,					   // 低
    TASK_PRIORITY_MEDIUM = 3,				   // 中
    TASK_PRIORITY_MEDIUM_HIGH = 4,			   // 中高
    TASK_PRIORITY_HIGH = 5,					   // 高
    TASK_PRIORITY_MAX = 255      
} taskPriority_e;

/* ------------------------------任务信息结构体------------------------------ */
typedef struct {
    bool         isEnabled;                    // 任务使能状态
    int8_t       staticPriority;               // 静态优先级
    timeDelta_t  desiredPeriodUs;              // 执行周期
} taskInfo_t;

/* --------------------------------任务结构体-------------------------------- */
typedef struct {
    bool (*checkFunc)(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs); // 检查函数指针(事件驱动)
    void (*taskFunc) (timeUs_t currentTimeUs);                                 // 任务函数指针
    timeDelta_t desiredPeriodUs;        	   // 执行周期（多久运行一次）
    const int8_t staticPriority;        	   // 静态优先级

    // 线程调度
    uint16_t dynamicPriority;           	   // 动态优先级
    uint16_t taskAgeCycles;             	   // 任务周期年龄（判断是否错过任务需要运行的时间）
    timeDelta_t taskLatestDeltaTimeUs;  	   // 任务时差
    timeUs_t lastExecutedAtUs;          	   // 任务最后一次调用时间
    timeUs_t lastSignaledAtUs;          	   // 检查任务最后一次调用时间
    timeUs_t lastDesiredAt;             	   // 任务期望执行的时间
} task_t;

/* --------------------------------任务ID枚举-------------------------------- */
typedef enum {
    /* 核心任务 -     维持飞行器基本飞行          */
    TASK_SYSTEM = 0,      				  	   // 系统任务 - 系统负载
    TASK_SERIAL,                               // USB任务
    TASK_GYRO,            				  	   // 陀螺仪任务
    TASK_FILTER,          				  	   // 滤波任务
    TASK_PID,             				  	   // PID任务 
    TASK_ACCEL,           				  	   // 加速度计任务
    TASK_ATTITUDE,        				  	   // 姿态控制器任务
    TASK_RX,              				  	   // 接收机任务
    TASK_BATTERY_VOLTAGE, 				  	   // 电池电压检测任务
    TASK_BATTERY_CURRENT, 				  	   // 电池电流计任务
    TASK_BATTERY_ALERTS,  				  	   // 电池警报任务
    
    /* 拓展任务 */
#ifdef USE_BEEPER         				  	   // 蜂鸣器任务
    TASK_BEEPER,
#endif
#ifdef USE_GPS 			  				  	   // GPS任务
    TASK_GPS,
#endif 
#ifdef USE_MAG            				  	   // 磁力计任务
    TASK_COMPASS, 
#endif
#ifdef USE_BARO			                  	   // 气压计任务
    TASK_BARO,
#endif
#if defined(USE_BARO) || defined(USE_GPS) 	   // 高度计算任务
    TASK_ALTITUDE,
#endif

#ifdef USE_OSD                            	   // OSD任务
    TASK_OSD,
#endif
#ifdef USE_CMS							  	   // CMS任务
    TASK_CMS,
#endif
#ifdef USE_VTX_CONTROL				      	   // 图传控制任务
    TASK_VTXCTRL,
#endif
    /* 实际任务的计数 */
    TASK_COUNT,
    /* 服务任务id */
    TASK_NONE = TASK_COUNT,
    TASK_SELF
} taskId_e;

void rescheduleTask(taskId_e taskId, timeDelta_t newPeriodUs);
void setTaskEnabled(taskId_e taskId, bool newEnabledState);
timeDelta_t getTaskDeltaTimeUs(taskId_e taskId);
void schedulerInit(void);
void scheduler(void);
timeUs_t schedulerExecuteTask(task_t *selectedTask, timeUs_t currentTimeUs);
void taskSystemLoad(timeUs_t currentTimeUs);
void schedulerOptimizeRate(bool optimizeRate);
void schedulerEnableGyro(void);
uint16_t getAverageSystemLoadPercent(void);

