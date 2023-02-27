/**********************************************************************
任务线程逻辑调度器：
	时间片查询的方式。每个任务没有自己的堆栈，不会进行强行跳转。
	任何一个任务时间过长都会导致其它任务时间延迟。
	任务可以是事件驱动型，或者是周期型，或者两者都是。先执行事件驱动，然后是周期运行。
	系统时间，微秒计时。
	执行任务，使用函数指针指向当前任务函数。

 任务调度逻辑：
	一、运行调度器执行逻辑之外的任务 - 最高优先级
		实时陀螺仪采样/滤波/PID任务得到最高的优先级 - 8KHz
	二、运行调度器执行逻辑之内的任务 - 陀螺仪任务未即将运行
		1.更新任务动态优先级 - 遍历全部任务队列
			更新静态优先级不在调度程序逻辑之外任务的动态优先级
				(1)该任务有检查函数(由事件驱动)
					增加任务动态优先级。
				(2)该任务无检查函数（无事件驱动）
					计算任务周期年龄 = （当前时间节拍 - 任务最后调用的时间节拍）/ 任务执行周期
					如果任务周期年龄大于0说明任务的执行周期发生了延迟 -> 增加任务动态优先级
				(3)选择动态优先级高的任务
		2.执行动态优先级最高的任务
			添加到目前为止用于检查函数和调度器逻辑的时间
			如果用于检查函数和调度器逻辑的时间 < 到下一次执行陀螺仪线程的剩余节拍数，则执行该任务，否则不执行
**********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "io/serial.h"

#include "fc/core.h"
#include "fc/tasks.h"

#include "scheduler.h"

#include "flight/imu.h"


// ---------------------------------------------------------减少任务平均执行时间
#define TASK_AVERAGE_EXECUTE_FALLBACK_US 30  				  							
	
// ---------------------------------------------------------记录当前任务线程
static FAST_RAM_ZERO_INIT task_t *currentTask = NULL;         							

// ---------------------------------------------------------任务等待计数
static FAST_RAM_ZERO_INIT uint32_t totalWaitingTasks;			
// ---------------------------------------------------------任务总等待计数
static FAST_RAM_ZERO_INIT uint32_t totalWaitingTasksSamples;							

// ---------------------------------------------------------平均系统负载百分比
FAST_RAM_ZERO_INIT uint16_t averageSystemLoadPercent = 0;     							

// ---------------------------------------------------------队列任务项
static FAST_RAM_ZERO_INIT int taskQueuePos = 0;         
// ---------------------------------------------------------初始化任务队列任务数量为0
STATIC_UNIT_TESTED FAST_RAM_ZERO_INIT int taskQueueSize = 0;  							

// ---------------------------------------------------------周期计算基准偏移量 - 返回lastExecutedAtUs在task_t中的字节偏移量
// offsetof(type, member-designator)
// type -- 这是一个 class 类型，其中，member-designator 是一个有效的成员指示器。
// member-designator -- 这是一个 class 类型的成员指示器
// 返回值：该宏返回类型为 size_t 的值，表示 type 中成员的偏移量
static FAST_RAM int periodCalculationBasisOffset = offsetof(task_t, lastExecutedAtUs);  

// ---------------------------------------------------------陀螺仪使能状态
static FAST_RAM_ZERO_INIT bool gyroEnabled;                                             

// ---------------------------------------------------------创建任务队列
// 队列不需要链表，因为项目只在启动时插入队列末尾空指针的额外项
STATIC_UNIT_TESTED FAST_RAM_ZERO_INIT task_t* taskQueueArray[TASK_COUNT + 1];

/**********************************************************************
函数名称：queueClear  
函数功能：清除任务队列  
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
void queueClear(void)
{
    memset(taskQueueArray, 0, sizeof(taskQueueArray)); 
    taskQueuePos = 0; 
    taskQueueSize = 0; 
}

/**********************************************************************
函数名称：queueContains
函数功能：查询队列是否已经包含该任务
函数形参：任务队列项
函数返回值：true:队列中已经包含该任务，false:队列中未包含该任务
函数描述：None
**********************************************************************/
bool queueContains(task_t *task)
{
    for (int ii = 0; ii < taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == task) {
            return true;
        }
    }
    return false;
}

/**********************************************************************
函数名称：queueAdd
函数功能：在队列中添加任务
函数形参：任务队列项
函数返回值：true:添加成功，false:添加失败
函数描述：None
**********************************************************************/
bool queueAdd(task_t *task)
{
    if ((taskQueueSize >= TASK_COUNT) || queueContains(task)) {
        return false;
    }
    for (int ii = 0; ii <= taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == NULL || taskQueueArray[ii]->staticPriority < task->staticPriority) {
            memmove(&taskQueueArray[ii+1], &taskQueueArray[ii], sizeof(task) * (taskQueueSize - ii));
            taskQueueArray[ii] = task;
            ++taskQueueSize;
            return true;
        }
    }
    return false;
}

/**********************************************************************
函数名称：queueRemove
函数功能：在队列中删除任务
函数形参：任务队列项
函数返回值：true:删除成功，false:删除失败
函数描述：None
**********************************************************************/
bool queueRemove(task_t *task)
{
    for (int ii = 0; ii < taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == task) {
            memmove(&taskQueueArray[ii], &taskQueueArray[ii+1], sizeof(task) * (taskQueueSize - ii));
            --taskQueueSize;
            return true;
        }
    }
    return false;
}

/**********************************************************************
函数名称：queueFirst
函数功能：返回队列中第一个任务信息 - 用于轮询任务队列
函数形参：None
函数返回值：如果为空则返回NULL
函数描述：None
**********************************************************************/
task_t *queueFirst(void)
{
    taskQueuePos = 0;
    return taskQueueArray[0]; 
}

/**********************************************************************
函数名称：queueNext
函数功能：返回队列中的下一个任务信息 - 用于轮询任务队列
函数形参：None
函数返回值：如果在队列末尾则返回NULL
函数描述：保证在队列结束时为空
**********************************************************************/
task_t *queueNext(void)
{
    return taskQueueArray[++taskQueuePos]; 
}

/**********************************************************************
函数名称：taskSystemLoad
函数功能：计算任务系统负载&&DEBUG
函数形参：当前时间节拍
函数返回值：None
函数描述：
	由调度器以任务形式调用.
**********************************************************************/
void taskSystemLoad(timeUs_t currentTimeUs)
{
	// 未使用该变量
    UNUSED(currentTimeUs);

    // 计算系统负载
    if (totalWaitingTasksSamples > 0) {
		// 平均系统负载百分比 = 100 * 任务等待 / 总任务等待
        averageSystemLoadPercent = 100 * totalWaitingTasks / totalWaitingTasksSamples;
        totalWaitingTasksSamples = 0;
        totalWaitingTasks = 0;
    }
}

/**********************************************************************
函数名称：getAverageSystemLoadPercent
函数功能：获得系统负载百分比
函数形参：None
函数返回值：平均系统负载百分比
函数描述：None
**********************************************************************/
uint16_t getAverageSystemLoadPercent(void)
{
    return averageSystemLoadPercent;
}

/**********************************************************************
函数名称：rescheduleTask
函数功能：重新设置任务周期
函数形参：任务信息，新的周期
函数返回值：None
函数描述：None
**********************************************************************/
void rescheduleTask(taskId_e taskId, timeDelta_t newPeriodUs)
{
    if (taskId == TASK_SELF) {
        task_t *task = currentTask;
        task->desiredPeriodUs = MAX(SCHEDULER_DELAY_LIMIT, newPeriodUs);  // 将延迟限制在100us (10khz)以防止调度程序阻塞
    } else if (taskId < TASK_COUNT) {
        task_t *task = getTask(taskId);
        task->desiredPeriodUs = MAX(SCHEDULER_DELAY_LIMIT, newPeriodUs);  // 将延迟限制在100us (10khz)以防止调度程序阻塞
    }
}

/**********************************************************************
函数名称：setTaskEnabled
函数功能：设置任务使能状态
函数形参：任务ID，使能状态
函数返回值：None
函数描述：使能：在队列中添加该任务，失能：在队列中删除该任务
**********************************************************************/
void setTaskEnabled(taskId_e taskId, bool enabled)
{
    if (taskId == TASK_SELF || taskId < TASK_COUNT) {
        task_t *task = taskId == TASK_SELF ? currentTask : getTask(taskId);
        if (enabled && task->taskFunc) {
            queueAdd(task);
        } else {
            queueRemove(task);
        }
    }
}

/**********************************************************************
函数名称：getTaskDeltaTimeUs
函数功能：获取任务时差
函数形参：任务ID
函数返回值：成功：返回任务时差，失败：返回0
函数描述：None
**********************************************************************/
timeDelta_t getTaskDeltaTimeUs(taskId_e taskId)
{
    if (taskId == TASK_SELF) {
        return currentTask->taskLatestDeltaTimeUs;
    } else if (taskId < TASK_COUNT) {
        return getTask(taskId)->taskLatestDeltaTimeUs;
    } else {
        return 0;
    }
}

/**********************************************************************
函数名称：schedulerInit
函数功能：任务调度器初始化
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void schedulerInit(void)
{
	// 清除任务队列信息
    queueClear();              
	// 添加系统任务 - 系统负载计算任务
    queueAdd(getTask(TASK_SYSTEM));         
}

/**********************************************************************
函数名称：schedulerOptimizeRate
函数功能：选择调度程序优化率
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void schedulerOptimizeRate(bool optimizeRate)
{
	// 判断选择是任务期望执行的时间还是任务最后一次调用时间
    periodCalculationBasisOffset = optimizeRate ? offsetof(task_t, lastDesiredAt) : offsetof(task_t, lastExecutedAtUs);
}

/**********************************************************************
函数名称：getPeriodCalculationBasis
函数功能：获取任务基础周期
函数形参：任务队列项
函数返回值：返回预期时间
函数描述：None
**********************************************************************/
inline static timeUs_t getPeriodCalculationBasis(const task_t* task)
{
	// 任务将在调度程序逻辑之外运行
    if (task->staticPriority == TASK_PRIORITY_REALTIME) {        
		// 返回该任务最后一次调用的时间节拍
        return *(timeUs_t*)((uint8_t*)task + periodCalculationBasisOffset);
    } 
	// 任务在调度程序逻辑之内运行
	else {                                                                 
    	// 返回该任务最后一次调用的时间节拍
        return task->lastExecutedAtUs;
    }
}

/**********************************************************************
函数名称：schedulerEnableGyro
函数功能：使能陀螺仪任务调度
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void schedulerEnableGyro(void)
{
    gyroEnabled = true;
}

/**********************************************************************
函数名称：schedulerExecuteTask
函数功能：执行任务
函数形参：任务队列项，当前时间节拍
函数返回值：返回任务执行时间
函数描述：None
**********************************************************************/
timeUs_t schedulerExecuteTask(task_t *selectedTask, timeUs_t currentTimeUs)
{
    timeUs_t taskExecutionTimeUs = 0;

    if (selectedTask) {
		// 记录当前任务线程
        currentTask = selectedTask;
		// 获取任务增量时间 = 当前时间节拍 - 最后一次调用时间节拍
        selectedTask->taskLatestDeltaTimeUs = cmpTimeUs(currentTimeUs, selectedTask->lastExecutedAtUs);
		// 将当前的时间节拍作为该任务下一次最后一次调用的时间 
        selectedTask->lastExecutedAtUs = currentTimeUs;
		// 最后期望执行的时间 += （当前时间节拍 - 最后期望执行的时间） /  执行周期
        selectedTask->lastDesiredAt += (cmpTimeUs(currentTimeUs, selectedTask->lastDesiredAt) / selectedTask->desiredPeriodUs) * selectedTask->desiredPeriodUs;
		// 动态优先级 = 0 
        selectedTask->dynamicPriority = 0;
		
        // ---任务执行阶段
        selectedTask->taskFunc(currentTimeUs);
    }

    return taskExecutionTimeUs;
}

/**********************************************************************
函数名称：scheduler
函数功能：任务调度
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void scheduler(void)
{
    const timeUs_t schedulerStartTimeUs = micros();    			// 获取当前时钟节拍
    timeUs_t currentTimeUs = schedulerStartTimeUs;     			// 记录当前时钟节拍
    timeUs_t taskExecutionTimeUs = 0;                  			// 任务执行时间缓存变量
    task_t *selectedTask = NULL;                       			// 任务选择（任务结构体信息）
    uint16_t selectedTaskDynamicPriority = 0;          			// 选择的任务动态优先级
    uint16_t waitingTasks = 0;                         			// 任务等待数
    bool realtimeTaskRan = false;                      			// 实时任务运行状态
    timeDelta_t gyroTaskDelayUs = 0;                   			// 到下一次执行陀螺仪线程的剩余节拍数

	/* ----------------------------------------运行调度器执行逻辑之外的任务-------------------------------------- */
	// -----实时陀螺仪采样/滤波/PID任务得到最高的优先级 - 核心任务
	// -----运行频率：8KHz
    if (gyroEnabled) {
        // 获取陀螺仪任务
        task_t *gyroTask = getTask(TASK_GYRO);
		
		// 陀螺仪应该执行的时间节拍 = 最后一次调用陀螺仪任务的时间节拍 + 陀螺仪任务期望执行的时间节拍
        const timeUs_t gyroExecuteTimeUs = getPeriodCalculationBasis(gyroTask) + gyroTask->desiredPeriodUs;
		// 到下一次执行陀螺仪线程的剩余节拍数 = 陀螺仪应该的执行时间节拍 - 当前时间节拍  [用于判断其他任务是否可以在此时间节拍数内执行]
        gyroTaskDelayUs = cmpTimeUs(gyroExecuteTimeUs, currentTimeUs); 
		
		// 如果当前时间节拍 >= 陀螺仪线程应该执行的时间节拍 - 则执行陀螺仪线程
        if (cmpTimeUs(currentTimeUs, gyroExecuteTimeUs) >= 0) {
			// 执行陀螺仪采样任务
            taskExecutionTimeUs = schedulerExecuteTask(gyroTask, currentTimeUs);
			// 执行陀螺仪滤波任务
            if (gyroFilterReady()) {
                taskExecutionTimeUs += schedulerExecuteTask(getTask(TASK_FILTER), currentTimeUs);
            }
			// 执行PID任务 - RC命令处理->PID控制器->混控输出
            if (pidLoopReady()) {
                taskExecutionTimeUs += schedulerExecuteTask(getTask(TASK_PID), currentTimeUs);
            }
			// 重新获取当前时间节拍
            currentTimeUs = micros();
			// 开启实时任务运行
            realtimeTaskRan = true;
        }
    }

	/* ----------------------------------------运行调度器执行逻辑之内的任务-------------------------------------- */
	// -----陀螺仪禁能状态 || 实时任务运行 || 陀螺仪任务未即将运行
    if (!gyroEnabled || realtimeTaskRan || (gyroTaskDelayUs > GYRO_TASK_GUARD_INTERVAL_US)) {
        // ------------------------------------------------------------------------------------------------------------1.更新任务动态优先级 - 遍历全部任务队列
        for (task_t *task = queueFirst(); task != NULL; task = queueNext()) {
			// 更新静态优先级不在调度程序逻辑之外任务的动态优先级
            if (task->staticPriority != TASK_PRIORITY_REALTIME) {      
                // --------------------------------------------------------------------------------(1)该任务有检查函数(由事件驱动)
                if (task->checkFunc) {                                           
					// 记录调用检查函数之前的时间节拍
                    const timeUs_t currentTimeBeforeCheckFuncCallUs = currentTimeUs;  

                    // 如果该任务的动态优先级大于0，则增加事件驱动任务的动态优先级
                    if (task->dynamicPriority > 0) {
						// 更新任务周期年龄 = （（当前时间节拍 - 任务最后一次调用时间）/ 任务执行周期 ）+ 1
                        task->taskAgeCycles = 1 + ((currentTimeUs - task->lastSignaledAtUs) / task->desiredPeriodUs);
						// 增加该任务动态优先级 =（静态优先级 * 任务周期年龄）+ 1
                        task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
						// 任务等待计数
                        waitingTasks++;
                    } 
					// 执行检查函数 - 如果发生了事件则再次增加该任务动态优先级
					else if (task->checkFunc(currentTimeBeforeCheckFuncCallUs, cmpTimeUs(currentTimeBeforeCheckFuncCallUs, task->lastExecutedAtUs))) {
						// 该任务最后一次调用时间 = 当前时间节拍
                        task->lastSignaledAtUs = currentTimeBeforeCheckFuncCallUs;
						// 该任务迫切需要得到运行 - 任务周期年龄置为 1  
                        task->taskAgeCycles = 1;
						// 增加该任务动态优先级 = 任务动态优先级 + 1 
                        task->dynamicPriority = 1 + task->staticPriority;
						// 任务等待计数
                        waitingTasks++;
                    } else {
                        task->taskAgeCycles = 0;
                    }
                } 
				// --------------------------------------------------------------------------------(2)该任务无检查函数（无事件驱动）
				else {
					// 计算任务周期年龄 = （当前时间节拍 - 任务最后调用的时间节拍）/ 任务执行周期
                    task->taskAgeCycles = ((currentTimeUs - getPeriodCalculationBasis(task)) / task->desiredPeriodUs);
					// 如果任务周期年龄大于0说明任务的执行周期发生了延迟
                    if (task->taskAgeCycles > 0) {
						// 增加该任务动态优先级 = （静态优先级 * 任务周期年龄）+ 1
                        task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
						// 任务等待计数
                        waitingTasks++;
                    }
                }
				// --------------------------------------------------------------------------------(3)选择动态优先级高的任务
                if (task->dynamicPriority > selectedTaskDynamicPriority) {
					// 选定任务动态优先级
                    selectedTaskDynamicPriority = task->dynamicPriority;
					// 调度该任务
                    selectedTask = task;
                }
            }
        }

        totalWaitingTasksSamples++;
        totalWaitingTasks += waitingTasks;

		// ------------------------------------------------------------------------------------------------------------2.执行动态优先级最高的任务任务
        if (selectedTask) {
			// 任务信息统计不可用
            timeDelta_t taskRequiredTimeUs = TASK_AVERAGE_EXECUTE_FALLBACK_US;  
            // 添加到目前为止用于检查函数和调度器逻辑的时间
            taskRequiredTimeUs += cmpTimeUs(micros(), currentTimeUs);
			// 如果用于检查函数和调度器逻辑的时间                  < 到下一次执行陀螺仪线程的剩余节拍数，则执行该任务，否则不执行
            if (!gyroEnabled || realtimeTaskRan || (taskRequiredTimeUs < gyroTaskDelayUs)) {
                taskExecutionTimeUs += schedulerExecuteTask(selectedTask, currentTimeUs);
            } else {
                selectedTask = NULL;
            }
        }
    }

	/**** 清空任务执行时间缓存变量 ****/
    UNUSED(taskExecutionTimeUs);
}

