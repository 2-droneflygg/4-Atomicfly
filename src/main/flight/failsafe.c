/**********************************************************************
失控保护：
	失控保护有两个阶段；
	若飞控在任一接收通道收到无效的脉冲长度，或者接收机汇报已失控，或者彻底没有收到接收机信号，飞控将进入一阶失控保护状态。
	飞控将会在所有通道使用预置通道设置，如果短时间内信号恢复将可退出失控保护状态。
	若飞机处于解锁状态，并且一阶失控保护模式持续时间超过了预设的时间，飞控将进入二阶失控保护状态，所有通道将会保持当前输出。
	注意： 在飞控进入一阶失控保护之前，如果AUX 通道出现无效的脉冲，该通道一样会使用预置通道设置。

	失控保护开关：选项确定通过辅助开关激活失控保护时发生的情况:
					阶段 1 :活阶段1失控保护，果要模拟信号丢失时的失控保护行为，此功能非常有用。
					阶段 2 :跳过阶段1并立即激活阶段2流程
					终止 :立即锁定 （你的飞行器将会坠毁）
					
	一阶失控保护：预置通道设置有三种情况 - 自动、保持、设置
	二阶失控保护：
		触发二阶失控保护的信号丢失保护时间: 一阶失控保护等待信号恢复时间
		低油门失控保护延迟: 在油门保持低位超过该时间，不再执行失控保护措施转而直接锁定电机。
		二阶失控保护措施: 坠落、降落、GPS救援
**********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/axis.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "drivers/time.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"

#include "io/beeper.h"

#include "rx/rx.h"

#include "flight/pid.h"

/**************************************************************
* 使用方法:
* 在使用其他方法之前必须调用failsafeInit()和failsafeReset()。
* failsafeInit()和failsafeReset()可以按任意顺序调用。
* failsafeInit()应该只调用一次，enable()应该在系统初始化后调用。
**************************************************************/
static failsafeState_t failsafeState;

PG_REGISTER_WITH_RESET_TEMPLATE(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 2);
PG_RESET_TEMPLATE(failsafeConfig_t, failsafeConfig,
    .failsafe_throttle = 1000,                       		// 默认关闭油门量
    .failsafe_throttle_low_delay = 100,              		// 低油门失控保护延迟 [1 = 0.1 秒]
    .failsafe_delay = 6,                             		// 触发二阶失控保护的信号丢失保护时间[1 = 0.1 秒]
    .failsafe_off_delay = 10,                       		// 1sec
    .failsafe_switch_mode = FAILSAFE_SWITCH_MODE_STAGE2,	// 失控保护开关行为
    .failsafe_procedure = FAILSAFE_PROCEDURE_GPS_RESCUE,	// 默认二阶失控保护措施：GPS救援模式
    .failsafe_recovery_delay = 20,                   		// 2秒有效的rx数据(外加200毫秒)需要从故障安全程序中恢复
    .failsafe_stick_threshold = 30                   		// 操纵杆偏斜30%，退出GPS救援程序
);

// 失效保护程序名称
const char * const failsafeProcedureNames[FAILSAFE_PROCEDURE_COUNT] = {
    "AUTO-LAND",
    "DROP",
#ifdef USE_GPS_RESCUE
    "GPS-RESCUE",
#endif
};

/**********************************************************************
函数名称：failsafeReset
函数功能：失控保护复位
函数形参：None  
函数返回值：None  
函数描述：当需要更改故障安全配置时
			——例如，选择了一个不同的配置文件时，应该调用
**********************************************************************/
void failsafeReset(void)
{
    failsafeState.rxDataFailurePeriod = PERIOD_RXDATA_FAILURE + failsafeConfig()->failsafe_delay * MILLIS_PER_TENTH_SECOND;
    failsafeState.rxDataRecoveryPeriod = PERIOD_RXDATA_RECOVERY + failsafeConfig()->failsafe_recovery_delay * MILLIS_PER_TENTH_SECOND;
    failsafeState.validRxDataReceivedAt = 0;
    failsafeState.validRxDataFailedAt = 0;
    failsafeState.throttleLowPeriod = 0;
    failsafeState.landingShouldBeFinishedAt = 0;
    failsafeState.receivingRxDataPeriod = 0;
    failsafeState.receivingRxDataPeriodPreset = 0;
    failsafeState.phase = FAILSAFE_IDLE;
    failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
}

/**********************************************************************
函数名称：failsafeInit
函数功能：失控保护初始化
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
void failsafeInit(void)
{
    failsafeState.events = 0;
    failsafeState.monitoring = false;
    return;
}

/**********************************************************************
函数名称：failsafeIsMonitoring
函数功能：失控保护监控
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
bool failsafeIsMonitoring(void)
{
    return failsafeState.monitoring;
}

/**********************************************************************
函数名称：failsafeIsActive
函数功能：失控保护是否激活
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
bool failsafeIsActive(void)
{
    return failsafeState.active;
}

/**********************************************************************
函数名称：failsafeStartMonitoring
函数功能：失控保护开始监控
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
void failsafeStartMonitoring(void)
{
    failsafeState.monitoring = true;
}

/**********************************************************************
函数名称：failsafeShouldHaveCausedLandingByNow
函数功能：失控保护开始着陆
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
static bool failsafeShouldHaveCausedLandingByNow(void)
{
    return (millis() > failsafeState.landingShouldBeFinishedAt);
}

/**********************************************************************
函数名称：failsafeActivate
函数功能：失控保护激活
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
static void failsafeActivate(void)
{
    failsafeState.active = true;
    failsafeState.phase = FAILSAFE_LANDING;			// 着陆中阶段
    ENABLE_FLIGHT_MODE(FAILSAFE_MODE);				// 使能失控保护模式
    failsafeState.landingShouldBeFinishedAt = millis() + failsafeConfig()->failsafe_off_delay * MILLIS_PER_TENTH_SECOND;
    failsafeState.events++;
}

/**********************************************************************
函数名称：failsafeApplyControlInput
函数功能：应用失控保护控制输入
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
static void failsafeApplyControlInput(void)
{
#ifdef USE_GPS_RESCUE
	// 预设失控保护措施为GPS救援
    if (failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE) {
		// 使能GPS救援模式
        ENABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
        return;
    }
#endif
	// RPY应用通道中值
    for (int i = 0; i < 3; i++) {
        rcData[i] = rxConfig()->midrc;
    }
	// 油门应用失控保护油门
    rcData[THROTTLE] = failsafeConfig()->failsafe_throttle;
}

/**********************************************************************
函数名称：failsafeIsReceivingRxData
函数功能：获取RX链接是否在线
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
bool failsafeIsReceivingRxData(void)
{
    return (failsafeState.rxLinkState == FAILSAFE_RXLINK_UP);
}
 
/**********************************************************************
函数名称：failsafeOnValidDataReceived
函数功能：对收到的有效数据进行失控保护
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
void failsafeOnValidDataReceived(void)
{
	// 记录有效RX数据接收时间
    failsafeState.validRxDataReceivedAt = millis();
	// 如果超出RX数据恢复周期
    if ((failsafeState.validRxDataReceivedAt - failsafeState.validRxDataFailedAt) > failsafeState.rxDataRecoveryPeriod) {
        failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;					// 更新链接状态为在线
        unsetArmingDisabled(ARMING_DISABLED_RX_FAILSAFE);				// 取消设置解锁失能 - 失控保护
    }
}

/**********************************************************************
函数名称：failsafeOnValidDataFailed
函数功能：失效数据的失控保护
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
void failsafeOnValidDataFailed(void)
{
	// 设置解锁禁用 - 防止没有RX链接的解锁
    setArmingDisabled(ARMING_DISABLED_RX_FAILSAFE); 
	// 记录RX数据失败时间
    failsafeState.validRxDataFailedAt = millis();
	// 如果超出数据失败周期
    if ((failsafeState.validRxDataFailedAt - failsafeState.validRxDataReceivedAt) > failsafeState.rxDataFailurePeriod) {
        failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;				// 更新链接状态为掉线
    }
}

/**********************************************************************
函数名称：failsafeUpdateState
函数功能：更新失控保护状态
函数形参：None  
函数返回值：None  
函数描述：None 
**********************************************************************/
void failsafeUpdateState(void)
{
	// 如果失控保护未进行监控则直接return 
    if (!failsafeIsMonitoring()) {
        return;
    }

	// 获取RX链接是否在线
    bool receivingRxData = failsafeIsReceivingRxData();
	// 获取解锁状态
    bool armed = ARMING_FLAG(ARMED);
	// 获取解锁失控保护模式状态
    bool failsafeSwitchIsOn = IS_RC_MODE_ACTIVE(BOXFAILSAFE);
	// 蜂鸣器模式初始化为沉默
    beeperMode_e beeperMode = BEEPER_SILENCE;

	// 如果开启失控保护 && 失控保护模式为阶段2
    if (failsafeSwitchIsOn && failsafeConfig()->failsafe_switch_mode == FAILSAFE_SWITCH_MODE_STAGE2) {
		// RX链接掉线
        receivingRxData = false; 
    }

    // 只有RX链接掉线 && 已解锁过 才会进行蜂鸣器通知RX_LOST
    if (!receivingRxData && (armed || ARMING_FLAG(WAS_EVER_ARMED))) {
        beeperMode = BEEPER_RX_LOST;
    }

	// 失控保护进程状态
    bool reprocessState;
	// 直到失控保护阶段执行完毕 
    do {
        reprocessState = false;
		// 失控保护阶段进程执行
        switch (failsafeState.phase) {
            case FAILSAFE_IDLE:										// --------------------------------失控保护空闲
            	// -----------------------已解锁
                if (armed) {
                    // 在高油门期间更新低油门低油门失控保护延迟 - 在油门保持低位超过该时间，不再执行失控保护措施转而直接锁定电机
                    if (THROTTLE_HIGH == calculateThrottleStatus()) {
                        failsafeState.throttleLowPeriod = millis() + failsafeConfig()->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
                    }
                    // 失控保护开启 &&　预设的失控保护模式为终止
                    if (failsafeSwitchIsOn && failsafeConfig()->failsafe_switch_mode == FAILSAFE_SWITCH_MODE_KILL) {
                        // 失控保护激活
                        failsafeActivate();
                        failsafeState.phase = FAILSAFE_LANDED;     							 // 跳过自动降落过程 - 坠落
                        failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_1_SECONDS;     // 需要1秒的有效rxData
                        reprocessState = true;												 // 执行完毕
                    } 
					// RX链接掉线
					else if (!receivingRxData) {
						// 低油门持续时间超过了保护延迟 && 预设失控保护措施非GPS救援模式
                        if (millis() > failsafeState.throttleLowPeriod
#ifdef USE_GPS_RESCUE
                            && failsafeConfig()->failsafe_procedure != FAILSAFE_PROCEDURE_GPS_RESCUE
#endif
                            ) {
                            // 失控保护激活
                            failsafeActivate();
                            failsafeState.phase = FAILSAFE_LANDED;  					     // 跳过自动降落过程 - 坠落
                            failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_3_SECONDS; // 需要3秒的有效rxData
                        } 
						// 未触发低油门保护
						else {
                            failsafeState.phase = FAILSAFE_RX_LOSS_DETECTED;				 // RX丢失检测阶段
                        }
                        reprocessState = true;												 // 执行完毕
                    }
                } 
				// -----------------------未解锁
				else {
                    // 未解锁时，OSD显示故障安全开关的rxLinkState(故障安全模式)
                    if (failsafeSwitchIsOn) {
                        ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
                    } else {
                        DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
                    }
                    // 低油门周保护延迟复位
                    failsafeState.throttleLowPeriod = 0;
                }
                break;

            case FAILSAFE_RX_LOSS_DETECTED:						    // --------------------------------失控保护RX丢失检测
            	// RX链接在线
                if (receivingRxData) {
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;						 // RX丢失恢复阶段
                } 
				// RX链接掉线
				else {
                    switch (failsafeConfig()->failsafe_procedure) {
                        case FAILSAFE_PROCEDURE_AUTO_LANDING:	
                            // 失控保护激活
                            failsafeActivate();
                            break;

                        case FAILSAFE_PROCEDURE_DROP_IT:
   							// 失控保护激活
                            failsafeActivate();
                            failsafeState.phase = FAILSAFE_LANDED;      					 // 坠落
                            break;
#ifdef USE_GPS_RESCUE
                        case FAILSAFE_PROCEDURE_GPS_RESCUE:
							// 失控保护激活
                            failsafeActivate();
                            failsafeState.phase = FAILSAFE_GPS_RESCUE;						 // GPS救援
                            break;
#endif
                    }
                }
                reprocessState = true;												 		 // 执行完毕
                break;

            case FAILSAFE_LANDING:									// --------------------------------失控保护着陆中
            	// RX链接重新在线
                if (receivingRxData) {	
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;						// RX丢失恢复阶段
                    reprocessState = true;												    // 执行完毕
                }
				// 已解锁
                if (armed) {
					// 应用失控保护控制输入 - GPS救援模式和RPYT通道设置
                    failsafeApplyControlInput();
					// 蜂鸣器通知
                    beeperMode = BEEPER_RX_LOST_LANDING;
                }
				// 失控保护开始着陆 || 崩溃恢复模式激活 || 未解锁
                if (failsafeShouldHaveCausedLandingByNow() || crashRecoveryModeActive() || !armed) {
                    failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_30_SECONDS; 		 // 需要30秒的有效rxData
                    failsafeState.phase = FAILSAFE_LANDED;									 // 失控保护安全着陆阶段
                    reprocessState = true;													 // 执行完毕
                }
                break;
#ifdef USE_GPS_RESCUE
            case FAILSAFE_GPS_RESCUE:								// --------------------------------失控保护GPS救援
            	// RX链接重新在线
                if (receivingRxData) {
					// 操纵杆偏斜30%，退出GPS救援程序
                    if (areSticksActive(failsafeConfig()->failsafe_stick_threshold)) {
                        failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;					 // RX丢失恢复阶段
                        reprocessState = true;												 // 执行完毕
                    }
                }
				// 已解锁
                if (armed) {
					// 应用失控保护控制输入 - GPS救援模式和RPYT通道设置
                    failsafeApplyControlInput();
					// 蜂鸣器通知
                    beeperMode = BEEPER_RX_LOST_LANDING;
                } 
				// 未解锁
				else {
                    failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_30_SECONDS; 		 // 需要30秒的有效rxData
                    failsafeState.phase = FAILSAFE_LANDED;									 // 失控保护安全着陆阶段
                    reprocessState = true;													 // 执行完毕
                }
                break;
#endif
            case FAILSAFE_LANDED:									// --------------------------------失控保护安全着陆
            	// 设置解锁禁用 - 以防止由间歇性的rx链接意外重新解锁
                setArmingDisabled(ARMING_DISABLED_FAILSAFE); 
				// 禁止解锁
                disarm(DISARM_REASON_FAILSAFE);
				// 设置有效rxData的所需时间
                failsafeState.receivingRxDataPeriod = millis() + failsafeState.receivingRxDataPeriodPreset; 
				// 切换阶段为RX丢失监测阶段
                failsafeState.phase = FAILSAFE_RX_LOSS_MONITORING;
                reprocessState = true;													     // 执行完毕
                break;

            case FAILSAFE_RX_LOSS_MONITORING:						// --------------------------------失控保护RX丢失监测
                // RX重新链接
                if (receivingRxData) {
                    if (millis() > failsafeState.receivingRxDataPeriod) {
                        // rx链接现在是好的，当通过开关解锁，它必须先关闭
                        if (!(IS_RC_MODE_ACTIVE(BOXARM))) {
                            unsetArmingDisabled(ARMING_DISABLED_FAILSAFE);					 // 复位失控保护解锁失能 
                            failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;				 // RX丢失恢复阶段
                            reprocessState = true;											 // 执行完毕
                        }
                    }
                } 
				// RX链接掉线状态
				else {
					// 更新信号丢失保护时间
                    failsafeState.receivingRxDataPeriod = millis() + failsafeState.receivingRxDataPeriodPreset;
                }
                break;

            case FAILSAFE_RX_LOSS_RECOVERED:						// --------------------------------失控保护RX丢失恢复
				// 进入怠速时，要求油门优先必须为failsafe_throttle_low_delay period的min_check
				// 这是为了防止JustDisarm在下一次迭代时被激活，因为这将有关闭间歇连接失控保护的效果
                failsafeState.throttleLowPeriod = millis() + failsafeConfig()->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
                failsafeState.phase = FAILSAFE_IDLE;										 // 失控保护空闲阶段
                failsafeState.active = false;							
                DISABLE_FLIGHT_MODE(FAILSAFE_MODE);											 // 失能失控保护模式
                reprocessState = true;													     // 执行完毕
                break;

            default:
                break;
        }
    } while (reprocessState);												  												 // 执行完毕

	// 执行蜂鸣器通知
    if (beeperMode != BEEPER_SILENCE) {
        beeper(beeperMode);
    }
}

