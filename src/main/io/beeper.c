/**********************************************************************
 蜂鸣器相关API函数：
**********************************************************************/
#include "stdbool.h"
#include "stdint.h"

#include "platform.h"

#include "common/utils.h"

#include "config/feature.h"

#include "drivers/dshot_command.h"
#include "drivers/io.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/motor.h"

#include "flight/mixer.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/runtime_config.h"

#include "io/statusindicator.h"
#include "io/vtx_control.h"

#ifdef USE_GPS
#include "io/gps.h"
#endif

#include "pg/beeper.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"

#include "beeper.h"

// ---------------------------------------------------------判断引脚需要什么输出状态
#ifdef BEEPER_INVERTED
#define IS_OPEN_DRAIN   false
#define IS_INVERTED     true
#else
#define IS_OPEN_DRAIN   true
#define IS_INVERTED     false
#endif

// ---------------------------------------------------------持续时间
#define BEEPER_CONFIRMATION_BEEP_DURATION 2
// ---------------------------------------------------------间隔持续时间
#define BEEPER_CONFIRMATION_BEEP_GAP_DURATION 20
// ---------------------------------------------------------警告长哔声乘数
#define BEEPER_WARNING_LONG_BEEP_MULTIPLIER 5

// ---------------------------------------------------------蜂鸣器启用状态[是否正在使用]
static bool beeperIsOn = false;

// ---------------------------------------------------------蜂鸣器频率
#define BEEPER_PWM_HZ   0	

// ---------------------------------------------------------'beep multiBeeps[]'的大小限制
#define MAX_MULTI_BEEPS 64    

// ---------------------------------------------------------蜂鸣器命令重复
#define BEEPER_COMMAND_REPEAT 0xFE	
// ---------------------------------------------------------蜂鸣器命令停止
#define BEEPER_COMMAND_STOP   0xFF	

// ---------------------------------------------------------上一个Dshot信标命令时间
static timeUs_t lastDshotBeaconCommandTimeUs;

// ---------------------------------------------------------用于可变哔哔声(报告GPS卫星数量等)的序列
static uint8_t beep_multiBeeps[MAX_MULTI_BEEPS + 1];

// ---------------------------------------------------------蜂鸣器条目频率（sequence）序列下标
static uint16_t beeperPos = 0;
// ---------------------------------------------------------蜂鸣器下一次触发时间
static uint32_t beeperNextToggleTime = 0;
// ---------------------------------------------------------最后一次启动时间(以微秒计)
static uint32_t armingBeepTimeMicros = 0;

/***************************************************
蜂鸣器声音序列:(方波生成)
	序列必须以0xFF或0xFE结束。
	0xFE重复序列，开始时0xFF停止声音
	延迟的单位是毫秒/10(即，5 => 50ms)
***************************************************/
// short fast beep - RX_SET
static const uint8_t beep_shortBeep[] = {
    10, 10, BEEPER_COMMAND_STOP
};
// arming beep
static const uint8_t beep_armingBeep[] = {
    30, 5, 5, 5,BEEPER_COMMAND_STOP
};
// Arming when GPS is fixed
static const uint8_t beep_armingGpsFix[] = {
    5, 5, 15, 5, 5, 5, 15, 30, BEEPER_COMMAND_STOP
};
// Arming when GPS is not fixed
static const uint8_t beep_armingGpsNoFix[] = {
    30, 5, 30, 5, 30, 5, BEEPER_COMMAND_STOP
};
// armed beep (first pause, then short beep)
static const uint8_t beep_armedBeep[] = {
    0, 245, 10, 5, BEEPER_COMMAND_STOP
};
// disarming beeps
static const uint8_t beep_disarmBeep[] = {
    15, 15, 5, 15, BEEPER_COMMAND_STOP
};
// beeps while stick held in disarm position (after pause)
static const uint8_t beep_disarmRepeatBeep[] = {
    0, 100, 10, BEEPER_COMMAND_STOP
};
// Long beep and pause after that
static const uint8_t beep_lowBatteryBeep[] = {
    25, 50, BEEPER_COMMAND_STOP
};
// critical battery beep
static const uint8_t beep_critBatteryBeep[] = {
    50, 2, BEEPER_COMMAND_STOP
};
// transmitter-signal-lost tone
static const uint8_t beep_txLostBeep[] = {
    50, 50, BEEPER_COMMAND_STOP
};
// SOS morse code:
static const uint8_t beep_sos[] = {
    10, 10, 10, 10, 10, 40, 40, 10, 40, 10, 40, 40, 10, 10, 10, 10, 10, 70, BEEPER_COMMAND_STOP
};
// Ready beeps. When gps has fix and copter is ready to fly.
static const uint8_t beep_readyBeep[] = {
    4, 5, 4, 5, 8, 5, 15, 5, 8, 5, 4, 5, 4, 5, BEEPER_COMMAND_STOP
};
// 2 fast short beeps
static const uint8_t beep_2shortBeeps[] = {
    5, 5, 5, 5, BEEPER_COMMAND_STOP
};
// 2 longer beeps
static const uint8_t beep_2longerBeeps[] = {
    20, 15, 35, 5, BEEPER_COMMAND_STOP
};
// 3 beeps
static const uint8_t beep_gyroCalibrated[] = {
    20, 10, 20, 10, 20, 10, BEEPER_COMMAND_STOP
};
// RC Smoothing filter not initialized - 3 short + 1 long
static const uint8_t beep_rcSmoothingInitFail[] = {
    10, 10, 10, 10, 10, 10, 50, 25, BEEPER_COMMAND_STOP
};

/* --------------------------蜂鸣器列表条目结构体-------------------------- */	
typedef struct beeperTableEntry_s {
    uint8_t mode;                		// 模式
    uint8_t priority;	 		 		// 优先级 - 0 = Highest
    const uint8_t *sequence;     		// 频率
    const char *name;			 		// 名称
} beeperTableEntry_t;

// ------------------------------------------------蜂鸣器表 - 按优先级顺序排列 - 0 = 最高
// 蜂鸣器条目宏 - 高优先级可以直接打断低优先级进行切换。
// 优先级要按照重要程度排列。
#define BEEPER_ENTRY(a,b,c,d) a,b,c,d
static const beeperTableEntry_t beeperTable[] = {
    { BEEPER_ENTRY(BEEPER_GYRO_CALIBRATED,       0, beep_gyroCalibrated,  	   "GYRO_CALIBRATED") },
    { BEEPER_ENTRY(BEEPER_RX_LOST,               1, beep_txLostBeep,       	   "RX_LOST") },
    { BEEPER_ENTRY(BEEPER_RX_LOST_LANDING,       2, beep_sos,              	   "RX_LOST_LANDING") },
    { BEEPER_ENTRY(BEEPER_DISARMING,             3, beep_disarmBeep,      	   "DISARMING") },
    { BEEPER_ENTRY(BEEPER_ARMING,                4, beep_armingBeep,       	   "ARMING")  },
    { BEEPER_ENTRY(BEEPER_ARMING_GPS_FIX,        5, beep_armingGpsFix,    	   "ARMING_GPS_FIX") },
    { BEEPER_ENTRY(BEEPER_ARMING_GPS_NO_FIX,     6, beep_armingGpsNoFix,   	   "ARMING_GPS_NO_FIX") },
    { BEEPER_ENTRY(BEEPER_BAT_CRIT_LOW,          7, beep_critBatteryBeep, 	   "BAT_CRIT_LOW") },
    { BEEPER_ENTRY(BEEPER_BAT_LOW,               8, beep_lowBatteryBeep,       "BAT_LOW") },
    { BEEPER_ENTRY(BEEPER_GPS_STATUS,            9, beep_multiBeeps,       	   "GPS_STATUS") },
    { BEEPER_ENTRY(BEEPER_RX_SET,                10, beep_shortBeep,       	   "RX_SET") },
    { BEEPER_ENTRY(BEEPER_ACC_CALIBRATION,       11, beep_2shortBeeps,         "ACC_CALIBRATION") },
    { BEEPER_ENTRY(BEEPER_ACC_CALIBRATION_FAIL,  12, beep_2longerBeeps,    	   "ACC_CALIBRATION_FAIL") },
    { BEEPER_ENTRY(BEEPER_READY_BEEP,            13, beep_readyBeep,      	   "READY_BEEP") },
    { BEEPER_ENTRY(BEEPER_MULTI_BEEPS,           14, beep_multiBeeps,      	   "MULTI_BEEPS") }, 
    { BEEPER_ENTRY(BEEPER_DISARM_REPEAT,         15, beep_disarmRepeatBeep,    "DISARM_REPEAT") },
    { BEEPER_ENTRY(BEEPER_ARMED,                 16, beep_armedBeep,      	   "ARMED") },
    { BEEPER_ENTRY(BEEPER_SYSTEM_INIT,           17, NULL,                	   "SYSTEM_INIT") },
    { BEEPER_ENTRY(BEEPER_USB,                   18, NULL,                	   "ON_USB") },
    { BEEPER_ENTRY(BEEPER_BLACKBOX_ERASE,        19, beep_2shortBeeps,    	   "BLACKBOX_ERASE") },
    { BEEPER_ENTRY(BEEPER_CRASH_FLIP_MODE,       20, beep_2longerBeeps,   	   "CRASH_FLIP") },
    { BEEPER_ENTRY(BEEPER_RC_SMOOTHING_INIT_FAIL,23, beep_rcSmoothingInitFail, "RC_SMOOTHING_INIT_FAIL") },
    { BEEPER_ENTRY(BEEPER_ALL,                   24, NULL,                     "ALL") },
};

// ---------------------------------------------------------当前蜂鸣器条目
static const beeperTableEntry_t *currentBeeperEntry = NULL;
// ---------------------------------------------------------蜂鸣器表条目数量
#define BEEPER_TABLE_ENTRY_COUNT (sizeof(beeperTable) / sizeof(beeperTableEntry_t))


/**********************************************************************
函数名称：beeperSilence
函数功能：蜂鸣器沉默（静音）
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void beeperSilence(void)
{
    BEEP_OFF;
    beeperIsOn = false;
	// 警告LED失能
    warningLedDisable();
	// 警告LED刷新
    warningLedRefresh();
	// 复位触发时间
    beeperNextToggleTime = 0;
	// 复位序列下标
    beeperPos = 0;
	// 复位当前条目
    currentBeeperEntry = NULL;
}


/**********************************************************************
函数名称：beeper
函数功能：调用激活/取消鸣叫
函数形参：模式
函数返回值：None
函数描述：
	选择蜂鸣器条目 - 该函数立即返回(不阻塞)。
	如果当前已经存在蜂鸣器命令进程，高优先级可以直接打断低优先级进行切换命令进程。
**********************************************************************/
void beeper(beeperMode_e mode)
{
	// ------------------------------------------------------------------判断蜂鸣器静音
    if (mode == BEEPER_SILENCE || beeperConfig()->beeper_off_flags) {
		// 沉默
        beeperSilence();
        return;
    }

	// ------------------------------------------------------------------遍历所有蜂鸣器条目
    const beeperTableEntry_t *selectedCandidate = NULL;
    for (uint32_t i = 0; i < BEEPER_TABLE_ENTRY_COUNT; i++) {
		// 获取条目
        const beeperTableEntry_t *candidate = &beeperTable[i];
		// 模式匹配
        if (candidate->mode != mode) {
            continue;
        }
		// -----------------------------1.如果当前条目为空 - 则选择该条目
        if (!currentBeeperEntry) {
            selectedCandidate = candidate;
            break;
        }
		// -----------------------------2.当前条目不为空 - 需要判断优先级（高优先级可以直接打断低优先级进行切换）
        if (candidate->priority < currentBeeperEntry->priority) {
			// 选择优先级高的条目
            selectedCandidate = candidate;
        }
        break;
    }

	// 检查选择条目合法性 - 确保不为空 
    if (!selectedCandidate) {
        return;
    }
	// 更新当前条目
    currentBeeperEntry = selectedCandidate;
	// 复位序列下标 - 重新用于推进该序列
    beeperPos = 0;
	// 复位触发时间 - 重新用于该序列触发时间
    beeperNextToggleTime = 0;
}

/**********************************************************************
函数名称：beeperConfirmationBeeps
函数功能：发出给定数量的20ms提示音(200ms间隔)
函数形参：鸣叫数量
函数返回值：None
函数描述：
	执行不定长蜂鸣器条目 - 该函数立即返回(不阻塞)。
**********************************************************************/
void beeperConfirmationBeeps(uint8_t beepCount)
{
    uint32_t i = 0;
    uint32_t cLimit = beepCount * 2;
	// 检查鸣叫数量合法性
    if (cLimit > MAX_MULTI_BEEPS) {
        cLimit = MAX_MULTI_BEEPS;
    }
	// 添加序列
    do {
		// 持续时间
        beep_multiBeeps[i++] = BEEPER_CONFIRMATION_BEEP_DURATION;
		// 间隔
        beep_multiBeeps[i++] = BEEPER_CONFIRMATION_BEEP_GAP_DURATION;
    } while (i < cLimit);
	// 添加停止命令
    beep_multiBeeps[i] = BEEPER_COMMAND_STOP;
	// 调用beeper添加该条目
    beeper(BEEPER_MULTI_BEEPS);
}

/**********************************************************************
函数名称：beeperWarningBeeps
函数功能：蜂鸣器警告提示音
函数形参：警告数量
函数返回值：None
函数描述：
	在尝试解锁时如果禁止解锁则警告提示。
	beepCount为禁止解锁理由（armingDisableFlags_e）。
**********************************************************************/
void beeperWarningBeeps(uint8_t beepCount)
{
	// 商[整数]
    uint8_t longBeepCount = beepCount / BEEPER_WARNING_LONG_BEEP_MULTIPLIER;
	// 余数
    uint8_t shortBeepCount = beepCount % BEEPER_WARNING_LONG_BEEP_MULTIPLIER;

    unsigned i = 0;

	// FLASH警告 - 添加数量：WARNING_FLASH_COUNT
    unsigned count = 0;
    while (i < MAX_MULTI_BEEPS - 1 && count < WARNING_FLASH_COUNT) {
		// 持续时间
        beep_multiBeeps[i++] = WARNING_FLASH_DURATION_MS / 10;
        if (++count < WARNING_FLASH_COUNT) {
			// 持续时间
            beep_multiBeeps[i++] = WARNING_FLASH_DURATION_MS / 10;
        } else {
        	// 暂停间隔
            beep_multiBeeps[i++] = WARNING_PAUSE_DURATION_MS / 10;
        }
    }
	// 警告长持续时间 - 添加数量：longBeepCount
    while (i < MAX_MULTI_BEEPS - 1 && longBeepCount > 0) {
		// 长持续时间
        beep_multiBeeps[i++] = WARNING_CODE_DURATION_LONG_MS / 10;
        if (--longBeepCount > 0) {
			// 长持续时间
            beep_multiBeeps[i++] = WARNING_CODE_DURATION_LONG_MS / 10;
        } else {
        	// 暂停间隔
            beep_multiBeeps[i++] = WARNING_PAUSE_DURATION_MS / 10;
        }
    }
	// 警告短持续时间 - 添加数量：shortBeepCount
    while (i < MAX_MULTI_BEEPS - 1 && shortBeepCount > 0) {
		// 短持续时间
        beep_multiBeeps[i++] = WARNING_CODE_DURATION_SHORT_MS / 10;
        if (--shortBeepCount > 0) {
			// 长持续时间
            beep_multiBeeps[i++] = WARNING_CODE_DURATION_LONG_MS / 10;
        }
    }
	// 添加停止命令
    beep_multiBeeps[i] = BEEPER_COMMAND_STOP;
	// 调用beeper添加该条目
    beeper(BEEPER_MULTI_BEEPS);
}

/**********************************************************************
函数名称：beeperGpsStatus
函数功能：GPS定位状态提示音
函数形参：beepCount
函数返回值：None
函数描述：None
**********************************************************************/
#ifdef USE_GPS
static void beeperGpsStatus(void)
{
    if (!(beeperConfig()->beeper_off_flags & BEEPER_GET_FLAG(BEEPER_GPS_STATUS))) {
        // 如果GPS定位，就会发出卫星数量的滴滴声
        if (STATE(GPS_FIX) && gpsSol.numSat >= 5) {
            uint8_t i = 0;
            do {
                beep_multiBeeps[i++] = 5;
                beep_multiBeeps[i++] = 10;
            } while (i < MAX_MULTI_BEEPS && gpsSol.numSat > i / 2);
			// 上一次间隔延长
            beep_multiBeeps[i - 1] = 50; 	
			// 添加停止命令
            beep_multiBeeps[i] = BEEPER_COMMAND_STOP;
			// 调用beeper添加该条目
            beeper(BEEPER_MULTI_BEEPS);  
        }
    }
}
#endif

/**********************************************************************
函数名称：beeperProcessCommand
函数功能：蜂鸣器命令进程
函数形参：currentTimeUs
函数返回值：None
函数描述：
	由beeperUpdate()调用。
**********************************************************************/
static void beeperProcessCommand(timeUs_t currentTimeUs)
{
    if (currentBeeperEntry->sequence[beeperPos] == BEEPER_COMMAND_REPEAT) {
		// 命令重复
        beeperPos = 0;
    } else if (currentBeeperEntry->sequence[beeperPos] == BEEPER_COMMAND_STOP) {
    	// 命令停止 - 沉默
        beeperSilence();
    } else {
        // 计算下一次触发时间 = 当前时间节拍 + 注册频率转微秒（[注册单位是毫秒/10(即，5 => 50ms)]*10*1000） 
        beeperNextToggleTime = currentTimeUs + 1000 * 10 * currentBeeperEntry->sequence[beeperPos];
		// 推进序列
        beeperPos++;
    }
}

/**********************************************************************
函数名称：beeperUpdate
函数功能：蜂鸣器更新
函数形参：currentTimeUs
函数返回值：None
函数描述：
	由调度器以任务形式调用。
	100Hz - 10ms足够。
**********************************************************************/
void beeperUpdate(timeUs_t currentTimeUs)
{
    // ------------------------------------------------------------------功能：通过开关激活蜂鸣器
    if (IS_RC_MODE_ACTIVE(BOXBEEPERON)) {
        beeper(BEEPER_RX_SET);
#ifdef USE_GPS
    } 
	// ------------------------------------------------------------------功能：通过蜂鸣器提示GPS星数功能
	else if (featureIsEnabled(FEATURE_GPS) && IS_RC_MODE_ACTIVE(BOXBEEPGPSCOUNT)) {
        beeperGpsStatus();
#endif
    }

    // ------------------------------------------------------------------1.如果没有任何正在进行的声音，蜂鸣器程序不需要更新
    if (currentBeeperEntry == NULL) {
        return;
    }
	// ------------------------------------------------------------------2.判断蜂鸣器序列触发时间是否到达
    if (beeperNextToggleTime > currentTimeUs) {
        return;
    }
	// ------------------------------------------------------------------3.触发时间到达 - 蜂鸣器命令序列执行[控制蜂鸣器]
    if (!beeperIsOn) {
		// --------------------------------------蜂鸣器未在运行 - 控制BEEP_ON
		// ---------------------(1)Dshot信标序列
#ifdef USE_DSHOT
		// 电机未在运行 && 控制电调警报 || RX丢失电调警报
		// BEEPER_RX_SET控制： 开启：0000 0000 0000 0000 0000 0000 0000 0000，关闭：0000 0000 0000 0000 0000 0010 0000 0000
		// BEEPER_RX_LOST控制：开启：0000 0000 0000 0000 0000 0000 0000 0000，关闭：0000 0000 0000 0000 0000 0000 0000 0100
        if (!areMotorsRunning()
            && ((currentBeeperEntry->mode == BEEPER_RX_SET && !(beeperConfig()->dshotBeaconOffFlags & BEEPER_GET_FLAG(BEEPER_RX_SET)))
            || (currentBeeperEntry->mode == BEEPER_RX_LOST && !(beeperConfig()->dshotBeaconOffFlags & BEEPER_GET_FLAG(BEEPER_RX_LOST))))) {
			// 写dshot命令 - Dshot信标
            if ((currentTimeUs - getLastDisarmTimeUs() > DSHOT_BEACON_GUARD_DELAY_US) && !isTryingToArm()) {
                lastDshotBeaconCommandTimeUs = currentTimeUs;
                dshotCommandWrite(ALL_MOTORS, getMotorCount(), beeperConfig()->dshotBeaconTone, DSHOT_CMD_TYPE_INLINE);
            }
        }
#endif
		// ---------------------(2)执行蜂鸣器序列 
        if (currentBeeperEntry->sequence[beeperPos] != 0) { 
            if (!(beeperConfig()->beeper_off_flags & BEEPER_GET_FLAG(currentBeeperEntry->mode))) {
				// 刷新状态
                BEEP_ON;
                beeperIsOn = true;
            }
			// 警告LED使能
            warningLedEnable();
			// 警告LED刷新
            warningLedRefresh();
			// 如果为启动哔哔 - 更新启动时间
            if (beeperPos == 0
                && (currentBeeperEntry->mode == BEEPER_ARMING || currentBeeperEntry->mode == BEEPER_ARMING_GPS_FIX
                || currentBeeperEntry->mode == BEEPER_ARMING_GPS_NO_FIX)) {
                armingBeepTimeMicros = micros();
            }
        }
    }else {
		// --------------------------------------蜂鸣器正在运行 - 控制BEEP_OFF
        if (currentBeeperEntry->sequence[beeperPos] != 0) {
			// 刷新状态
            BEEP_OFF;
            beeperIsOn = false;
			// 警告LED使能
            warningLedDisable();
			// 警告LED刷新
            warningLedRefresh();
        }
    }

	// ------------------------------------------------------------------4.蜂鸣器命令序列更新[推进序列]
    beeperProcessCommand(currentTimeUs);
}

/**********************************************************************
函数名称：isBeeperOn
函数功能：获取蜂鸣器是否开启
函数形参：idx
函数返回值：如果蜂鸣器开启则返回true，否则返回false
函数描述：
	用于视觉蜂鸣器[OSD提示]。
**********************************************************************/
bool isBeeperOn(void)
{
    return beeperIsOn;
}

/**********************************************************************
函数名称：getLastDshotBeaconCommandTimeUs
函数功能：获得最后一次Dshot信标命令时间
函数形参：None
函数返回值：lastDshotBeaconCommandTimeUs
函数描述：None
**********************************************************************/
timeUs_t getLastDshotBeaconCommandTimeUs(void)
{
    return lastDshotBeaconCommandTimeUs;
}

