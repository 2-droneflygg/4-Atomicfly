/*********************************************************************************
 提供配置操作相关API。
	配置文件操作、验证和修复配置、配置相关操作、重启需求。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "config/config_eeprom.h"
#include "config/feature.h"

#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "drivers/system.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/rc_modes.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/beeper.h"
#include "io/gps.h"
#include "io/serial.h"
#include "io/vtx.h"

#include "osd/osd.h"

#include "pg/adc.h"
#include "pg/beeper.h"
#include "pg/beeper_dev.h"
#include "pg/gyrodev.h"
#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "config.h"

// ---------------------------------------------------------指定的配置文件被修改，但还没有保存 
static bool configIsDirty;  		 		
// ---------------------------------------------------------如果配置更改需要重新启动才能生效，则设置
static bool rebootRequired = false;  	
// ---------------------------------------------------------记录当前PID配置文件
pidProfile_t *currentPidProfile;     			
#define DFLIGHT_MAX_SRATE  100

PG_REGISTER_WITH_RESET_TEMPLATE(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 2);
PG_RESET_TEMPLATE(systemConfig_t, systemConfig,
    .pidProfileIndex = 0,        
    .activeRateProfile = 0,
    .cpu_overclock = DEFAULT_CPU_OVERCLOCK,
    .powerOnArmingGraceTime = 5,
    .boardIdentifier = TARGET_BOARD_IDENTIFIER,
    .hseMhz = SYSTEM_HSE_VALUE,      
    .configurationState = CONFIGURATION_STATE_DEFAULTS_BARE,
    .schedulerOptimizeRate = SCHEDULER_OPTIMIZE_RATE_AUTO,
);

//-------------------------------------------------------------------------------------配置文件操作相关API

/**********************************************************************
函数名称：getCurrentPidProfileIndex
函数功能：加载Pid配置文件
函数形参：None
函数返回值：当前Pid配置文件索引
函数描述：None
**********************************************************************/
static void loadPidProfile(void)
{
    currentPidProfile = pidProfilesMutable(systemConfig()->pidProfileIndex);
}

/**********************************************************************
函数名称：getCurrentPidProfileIndex
函数功能：获取当前Pid配置文件索引
函数形参：None
函数返回值：当前Pid配置文件索引
函数描述：None
**********************************************************************/
uint8_t getCurrentPidProfileIndex(void)
{
    return systemConfig()->pidProfileIndex;
}

/**********************************************************************
函数名称：changePidProfile
函数功能：更改PID配置文件
函数形参：PID配置文件索引
函数返回值：None
函数描述：None
**********************************************************************/
void changePidProfile(uint8_t pidProfileIndex)
{
    if (pidProfileIndex < PID_PROFILE_COUNT) {
        systemConfigMutable()->pidProfileIndex = pidProfileIndex;
		// 加载PID配置文件
        loadPidProfile();
		// PID初始化
        pidInit(currentPidProfile);
		// 初始化电调结束点
        initEscEndpoints();
    }
	// 发出给定数量的20ms哔哔声(200ms间隔)
    beeperConfirmationBeeps(pidProfileIndex + 1);
}

/**********************************************************************
函数名称：changePidProfileFromCellCount
函数功能：更改PID配置文件
函数形参：cellCount
函数返回值：None
函数描述：None
**********************************************************************/
void changePidProfileFromCellCount(uint8_t cellCount)
{
    if (currentPidProfile->auto_profile_cell_count == cellCount || currentPidProfile->auto_profile_cell_count == AUTO_PROFILE_CELL_COUNT_STAY) {
        return;
    }

	// 获取PID配置文件索引
    unsigned profileIndex = (systemConfig()->pidProfileIndex + 1) % PID_PROFILE_COUNT;
    int matchingProfileIndex = -1;
    while (profileIndex != systemConfig()->pidProfileIndex) {
        if (pidProfiles(profileIndex)->auto_profile_cell_count == cellCount) {
            matchingProfileIndex = profileIndex;
            break;
        } else if (matchingProfileIndex < 0 && pidProfiles(profileIndex)->auto_profile_cell_count == AUTO_PROFILE_CELL_COUNT_STAY) {
            matchingProfileIndex = profileIndex;
        }

        profileIndex = (profileIndex + 1) % PID_PROFILE_COUNT;
    }

    if (matchingProfileIndex >= 0) {
		// 更改PID配置文件
        changePidProfile(matchingProfileIndex);
    }
}

/**********************************************************************
函数名称：getCurrentPidProfileIndex
函数功能：获取当前速率配置文件索引
函数形参：None
函数返回值：当前速率配置文件索引
函数描述：None
**********************************************************************/
uint8_t getCurrentControlRateProfileIndex(void)
{
    return systemConfig()->activeRateProfile;
}

//-------------------------------------------------------------------------------------验证和修复配置相关API

/**********************************************************************
函数名称：adjustFilterLimit
函数功能：调整滤波限制
函数形参：parm，resetValue
函数返回值：None
函数描述：None
**********************************************************************/
static void adjustFilterLimit(uint16_t *parm, uint16_t resetValue)
{
    if (*parm > FILTER_FREQUENCY_MAX) {
        *parm = resetValue;
    }
}

/**********************************************************************
函数名称：validateAndFixGyroConfig
函数功能：验证和修复陀螺仪配置
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void validateAndFixGyroConfig(void)
{
	// -------------------------------------------------------------------修复陀螺滤波设置[限制]
    adjustFilterLimit(&gyroConfigMutable()->gyro_lowpass_hz, FILTER_FREQUENCY_MAX);
    adjustFilterLimit(&gyroConfigMutable()->gyro_lowpass2_hz, FILTER_FREQUENCY_MAX);
#ifdef USE_DYN_LPF
    // 防止无效的动态低通滤波器
    if (gyroConfig()->dyn_lpf_gyro_min_hz > gyroConfig()->dyn_lpf_gyro_max_hz) {
        gyroConfigMutable()->dyn_lpf_gyro_min_hz = 0;
    }
#endif

	// -------------------------------------------------------------------验证pid_process_denom(PID进程分母项)
    if (gyro.sampleRateHz > 0) {
		// 采样时间 = 1 / 8000 = 0.000125
        float samplingTime = 1.0f / gyro.sampleRateHz;

        // 检查基于电机协议的更新时间限制
        float motorUpdateRestriction;
        switch (motorConfig()->dev.motorPwmProtocol) {
	        default:				
	            motorUpdateRestriction = 0.00003125f;
	            break;
        }

		// PID循环时间 = 0.000125
        const float pidLooptime = samplingTime * pidConfig()->pid_process_denom;
		// 如果PID循环时间 < 电机协议的更新时间限制
        if (pidLooptime < motorUpdateRestriction) {
			// PID分母项最小值 = 0.00003125 / 0.000125 = 0.25
            uint8_t minPidProcessDenom = motorUpdateRestriction / samplingTime;
			// 0.00003125 / 0.000125 > 0.25
            if (motorUpdateRestriction / samplingTime > minPidProcessDenom) {
                // 如果有小数部分，则四舍五入
                minPidProcessDenom++;
            }
            minPidProcessDenom = constrain(minPidProcessDenom, 1, MAX_PID_PROCESS_DENOM);
            pidConfigMutable()->pid_process_denom = MAX(pidConfigMutable()->pid_process_denom, minPidProcessDenom);
        }
    }

	// -------------------------------------------------------------------加载速率配置文件
	// 验证速率配置文件索引合法性
    if (systemConfig()->activeRateProfile >= CONTROL_RATE_PROFILE_COUNT) {
        systemConfigMutable()->activeRateProfile = 0;
    }
    loadControlRateProfile();

	// -------------------------------------------------------------------加载Pid配置文件
	// 验证PID配置文件索引合法性
    if (systemConfig()->pidProfileIndex >= PID_PROFILE_COUNT) {
        systemConfigMutable()->pidProfileIndex = 0;
    }
    loadPidProfile();
}

/**********************************************************************
函数名称：validateAndFixConfig
函数功能：验证和修复配置
函数形参：None
函数返回值：None
函数描述：
	验证相关配置合法性，并清除不支持的特性。
**********************************************************************/
static void validateAndFixConfig(void)
{
	// -------------------------------------------------------------------验证串口配置是否有效
    if (!isSerialConfigValid(serialConfig())) {           
        pgResetFn_serialConfig(serialConfigMutable());
    }

	// -------------------------------------------------------------------验证GPS配置是否有效
#if defined(USE_GPS)
    const serialPortConfig_t *gpsSerial = findSerialPortConfig(FUNCTION_GPS);
#endif
    if (
#if defined(USE_GPS)
        !gpsSerial &&
#endif
        true) {
        featureDisableImmediate(FEATURE_GPS);
    }

	// -------------------------------------------------------------------验证PID配置是否有效
    for (unsigned i = 0; i < PID_PROFILE_COUNT; i++) {
        if (pidProfilesMutable(i)->motor_output_limit > 100 || pidProfilesMutable(i)->motor_output_limit == 0) {
            pidProfilesMutable(i)->motor_output_limit = 100;
        }
        if (pidProfilesMutable(i)->auto_profile_cell_count > MAX_AUTO_DETECT_CELL_COUNT || pidProfilesMutable(i)->auto_profile_cell_count < AUTO_PROFILE_CELL_COUNT_CHANGE) {
            pidProfilesMutable(i)->auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY;
        }
        // 如果任意轴的d_min值为>= D增益，则将d_min重置为0以保持配置器行为一致
        for (unsigned axis = 0; axis <= FD_YAW; axis++) {
            if (pidProfilesMutable(i)->d_min[axis] >= pidProfilesMutable(i)->pid[axis].D) {
                pidProfilesMutable(i)->d_min[axis] = 0;
            }
        }
		// 电池电压跌落补偿
#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
        if (batteryConfig()->voltageMeterSource != VOLTAGE_METER_ADC) {
            pidProfilesMutable(i)->vbat_sag_compensation = 0;
        }
#endif
    }
	
	// -------------------------------------------------------------------验证和修复陀螺仪配置
    validateAndFixGyroConfig();

#ifdef USE_ACC
    if (accelerometerConfig()->accZero.values.roll != 0 ||
        accelerometerConfig()->accZero.values.pitch != 0 ||
        accelerometerConfig()->accZero.values.yaw != 0) {
        accelerometerConfigMutable()->accZero.values.calibrationCompleted = 1;
    }
#endif // USE_ACC

	// -------------------------------------------------------------------验证串行接收机配置
    if (featureIsConfigured(FEATURE_RX_SERIAL)) {
		// 使能串行接收机特性
        featureEnableImmediate(DEFAULT_RX_FEATURE);
    }

	// -------------------------------------------------------------------验证RSSI通道配置
    if (rxConfigMutable()->rssi_channel) {
        rxConfigMutable()->rssi_src_frame_errors = false;
    }

	// -------------------------------------------------------------------验证RC平滑和插值配置
    if (!rcSmoothingIsEnabled() || rxConfig()->rcInterpolationChannels == INTERPOLATION_CHANNELS_T) {
        for (unsigned i = 0; i < PID_PROFILE_COUNT; i++) {
            pidProfilesMutable(i)->pid[PID_ROLL].F = 0;
            pidProfilesMutable(i)->pid[PID_PITCH].F = 0;
        }
    }

    if (!rcSmoothingIsEnabled() ||
        (rxConfig()->rcInterpolationChannels != INTERPOLATION_CHANNELS_RPY &&
         rxConfig()->rcInterpolationChannels != INTERPOLATION_CHANNELS_RPYT)) {
        for (unsigned i = 0; i < PID_PROFILE_COUNT; i++) {
            pidProfilesMutable(i)->pid[PID_YAW].F = 0;
        }
    }

#if defined(USE_THROTTLE_BOOST)
    if (!rcSmoothingIsEnabled() ||
        !(rxConfig()->rcInterpolationChannels == INTERPOLATION_CHANNELS_RPYT
        || rxConfig()->rcInterpolationChannels == INTERPOLATION_CHANNELS_T
        || rxConfig()->rcInterpolationChannels == INTERPOLATION_CHANNELS_RPT)) {
        for (unsigned i = 0; i < PID_PROFILE_COUNT; i++) {
            pidProfilesMutable(i)->throttle_boost = 0;
        }
    }
#endif

	// -------------------------------------------------------------------验证GPS救援配置
    if (!featureIsConfigured(FEATURE_GPS)
#if !defined(USE_GPS) || !defined(USE_GPS_RESCUE)
        || true
#endif
        ) {
#ifdef USE_GPS_RESCUE
        if (failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE) {
            failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_DROP_IT;
        }
#endif
        if (isModeActivationConditionPresent(BOXGPSRESCUE)) {
            removeModeActivationCondition(BOXGPSRESCUE);
        }
    }

	// -------------------------------------------------------------------验证模式激活配置
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);
    }

	// -------------------------------------------------------------------验证ADC相关配置
#ifdef USE_ADC
    adcConfigMutable()->vbat.enabled = (batteryConfig()->voltageMeterSource == VOLTAGE_METER_ADC);
    adcConfigMutable()->current.enabled = (batteryConfig()->currentMeterSource == CURRENT_METER_ADC);
#endif // USE_ADC


	// -------------------------------------------------------------------验证蜂鸣器配置
#if defined(USE_BEEPER)
#ifdef USE_TIMER
    if (beeperDevConfig()->frequency && !timerGetByTag(beeperDevConfig()->ioTag)) {
        beeperDevConfigMutable()->frequency = 0;
    }
#endif

    if (beeperConfig()->beeper_off_flags & ~BEEPER_ALLOWED_MODES) {
        beeperConfigMutable()->beeper_off_flags = 0;
    }

#ifdef USE_DSHOT
    if (beeperConfig()->dshotBeaconOffFlags & ~DSHOT_BEACON_ALLOWED_MODES) {
        beeperConfigMutable()->dshotBeaconOffFlags = 0;
    }

    if (beeperConfig()->dshotBeaconTone < DSHOT_CMD_BEACON1
        || beeperConfig()->dshotBeaconTone > DSHOT_CMD_BEACON5) {
        beeperConfigMutable()->dshotBeaconTone = DSHOT_CMD_BEACON1;
    }
#endif
#endif

	// -------------------------------------------------------------------验证OSD定时器配置
#if defined(USE_OSD)
    for (int i = 0; i < OSD_TIMER_COUNT; i++) {
         const uint16_t t = osdConfig()->timers[i];
         if (OSD_TIMER_SRC(t) >= OSD_TIMER_SRC_COUNT || OSD_TIMER_PRECISION(t) >= OSD_TIMER_PREC_COUNT) {
             osdConfigMutable()->timers[i] = osdTimerDefault[i];
         }
     }
#endif

	// -------------------------------------------------------------------验证RC速率是否在范围内
    for (unsigned i = 0; i < CONTROL_RATE_PROFILE_COUNT; i++) {
        switch (controlRateProfilesMutable(i)->rates_type) {
        case RATES_TYPE_DFLIGHT:
        default:
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                controlRateProfilesMutable(i)->rates[axis] = constrain(controlRateProfilesMutable(i)->rates[axis], 0, DFLIGHT_MAX_SRATE);
            }
            break;
        }
    }

	// -------------------------------------------------------------------清除不支持的特性
#ifndef USE_SERIAL_RX
	// 禁用串行接收机
    featureDisableImmediate(FEATURE_RX_SERIAL);
#endif
#if !defined(USE_SOFTSERIAL1) && !defined(USE_SOFTSERIAL2)
	// 禁用软串口
    featureDisableImmediate(FEATURE_SOFTSERIAL);
#endif
#ifndef USE_OSD
	// 禁用OSD 
    featureDisableImmediate(FEATURE_OSD);
#endif
	// 禁用动态低通滤波
    featureDisableImmediate(FEATURE_DYNAMIC_FILTER);
}

//-------------------------------------------------------------------------------------配置操作相关API

/**********************************************************************
函数名称：resetConfig
函数功能：复位所有配置
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void resetConfig(void)
{
	// 复位所有PG参数组寄存器
    pgResetAll();
}

/**********************************************************************
函数名称：activateConfig
函数功能：激活配置
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
static void activateConfig(void)
{
	// 选择调度程序优化率
    schedulerOptimizeRate(systemConfig()->schedulerOptimizeRate == SCHEDULER_OPTIMIZE_RATE_ON);
	// PID配置文件加载
	loadPidProfile();
	// RATE文件加载
	loadControlRateProfile();
	// RC进程初始化
    initRcProcessing();
	// PID初始化
    pidInit(currentPidProfile);
	// RC控制初始化
    rcControlsInit();
	// 失控保护复位
    failsafeReset();
#ifdef USE_ACC
	// 设置加速度计零偏
    setAccelerationTrims(&accelerometerConfigMutable()->accZero);
	// 初始化加速度计滤波
    accInitFilters();
#endif
	// IMU配置
    imuConfigure();
}

/**********************************************************************
函数名称：readEEPROM
函数功能：读取EEPROM
函数形参：None
函数返回值：完整性检查
函数描述：None
**********************************************************************/
bool readEEPROM(void)
{
    // 1.完整性检查 - 初始化EEPROM中所有配置记录
    bool success = loadEEPROM();
	// 2.特性初始化
    featureInit();
	// 3.验证和修复配置
    validateAndFixConfig();
	// 4.激活配置
    activateConfig();
    return success;
}

/**********************************************************************
函数名称：writeUnmodifiedConfigToEEPROM
函数功能：写入未修改的配置到EEPROM
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void writeUnmodifiedConfigToEEPROM(void)
{
	// 1.验证和修复配置
    validateAndFixConfig();   
	// 2.写入配置到EEPROM
    writeConfigToEEPROM();			  
    configIsDirty = false;
}

/**********************************************************************
函数名称：writeEEPROM
函数功能：写EEPROM
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void writeEEPROM(void)
{
    systemConfigMutable()->configurationState = CONFIGURATION_STATE_CONFIGURED;
	// 写入未修改的配置到EEPROM
    writeUnmodifiedConfigToEEPROM();
}

/**********************************************************************
函数名称：resetEEPROM
函数功能：复位EEPROM
函数形参：使用自定义默认值
函数返回值：复位成功与否
函数描述：None
**********************************************************************/
bool resetEEPROM(bool useCustomDefaults)
{
    UNUSED(useCustomDefaults);
	// 复位所有配置
    resetConfig();			
	// 写入未修改的配置到EEPROM
    writeUnmodifiedConfigToEEPROM();  
    return true;
}

/**********************************************************************
函数名称：ensureEEPROMStructureIsValid
函数功能：确保EEPROM结构是有效的
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void ensureEEPROMStructureIsValid(void)
{
	// 扫描EEPROM是否有效
    if (isEEPROMStructureValid()) {   
        return;
    }
	// 如果无效则复位EEPROM 
    resetEEPROM(false);				  
}

/**********************************************************************
函数名称：saveConfigAndNotify
函数功能：保存配置和通知
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void saveConfigAndNotify(void)
{
	// 写入配置
    writeEEPROM();
	// 读取配置 - 验证和修复配置
    readEEPROM();
	// 蜂鸣器通知
    beeperConfirmationBeeps(1);
}

//-------------------------------------------------------------------------------------重启需求相关API

/**********************************************************************
函数名称：setRebootRequired
函数功能：设置重启需求
函数形参：None
函数返回值：one
函数描述：None
**********************************************************************/
void setRebootRequired(void)
{
    rebootRequired = true;
	// 设置解锁禁用
    setArmingDisabled(ARMING_DISABLED_REBOOT_REQUIRED);
}

/**********************************************************************
函数名称：getRebootRequired
函数功能：获取重启需求
函数形参：None
函数返回值：重启需求
函数描述：None
**********************************************************************/
bool getRebootRequired(void)
{
    return rebootRequired;
}

