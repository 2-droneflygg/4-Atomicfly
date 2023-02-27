#pragma once

#include "common/time.h"

#define BEEPER_GET_FLAG(mode) (1 << (mode - 1))  // 获取模式标志位

#ifdef USE_DSHOT
#define DSHOT_BEACON_GUARD_DELAY_US 1200000      // DSHOT信标延迟，分开dshot信标和解锁/锁定事件 - 防止对电机方向指令的干扰
#endif

/* --------------------------蜂鸣器模式枚举-------------------------- */	
typedef enum {
    // 重要提示:为了向后兼容配置器，应该保留元素的顺序
    BEEPER_SILENCE = 0,             // 沉默,beeperSilence()
    BEEPER_GYRO_CALIBRATED,			// 陀螺仪校准完成
    BEEPER_RX_LOST,                 // 遥控器关闭或信号丢失时持续鸣叫直到信号恢复
    BEEPER_RX_LOST_LANDING,         // 解锁后遥控器关闭或信号丢失（自动降落/自动锁定）时鸣叫 SOS 信号
    BEEPER_DISARMING,               // 锁定飞控
    BEEPER_ARMING,                  // 解锁飞控
    BEEPER_ARMING_GPS_FIX,          // GPS定位成功后解锁飞控时鸣叫特殊音调
    BEEPER_BAT_CRIT_LOW,            // 当电池电压严重偏低时持续长鸣
    BEEPER_BAT_LOW,                 // 当电池电压偏低时重复鸣叫
    BEEPER_GPS_STATUS,              // 使用蜂鸣音的次数来表示找到了多少个 GPS 卫星
    BEEPER_RX_SET,                  // 通过辅助通道发出蜂鸣音
    BEEPER_ACC_CALIBRATION,         // 加速度计飞行中校准完成
    BEEPER_ACC_CALIBRATION_FAIL,    // 加速度计飞行中校准失败
    BEEPER_READY_BEEP,              // 当GPS锁定并准备好时，请发出提示音
    BEEPER_MULTI_BEEPS,             // 可变哔哔声
    BEEPER_DISARM_REPEAT,           // 摇杆保持在锁定位置时鸣叫
    BEEPER_ARMED,                   // 当飞控解锁且电机未转时，持续发出警告鸣叫直到上推油门或重新锁定
    BEEPER_SYSTEM_INIT,             // 飞控上电时鸣叫初始化音
    BEEPER_USB,                     // 通过 USB 连接飞控时启用蜂鸣器。不想在调试时听到鸣叫可禁用这个选项
    BEEPER_BLACKBOX_ERASE,          // 黑盒擦除完成时鸣叫
    BEEPER_CRASH_FLIP_MODE,         // 当处于反乌龟模式时发出蜂鸣音
    BEEPER_CAM_CONNECTION_OPEN,     // 当进入5键相机控制模式时发出蜂鸣音
    BEEPER_CAM_CONNECTION_CLOSE,    // 当退出5键相机控制模式时发出蜂鸣音
    BEEPER_RC_SMOOTHING_INIT_FAIL,  // 当解锁和rc平滑没有初始化滤波器时，警告哔声模式
    BEEPER_ARMING_GPS_NO_FIX,       // 当GPS无法定位启动时，发出特殊的哔声
    BEEPER_ALL,                     // 打开或关闭所有蜂鸣条件
} beeperMode_e;

// 蜂鸣器允许模式
#define BEEPER_ALLOWED_MODES ( \
      BEEPER_GET_FLAG(BEEPER_GYRO_CALIBRATED) \
    | BEEPER_GET_FLAG(BEEPER_RX_LOST) \
    | BEEPER_GET_FLAG(BEEPER_RX_LOST_LANDING) \
    | BEEPER_GET_FLAG(BEEPER_DISARMING) \
    | BEEPER_GET_FLAG(BEEPER_ARMING) \
    | BEEPER_GET_FLAG(BEEPER_ARMING_GPS_FIX) \
    | BEEPER_GET_FLAG(BEEPER_BAT_CRIT_LOW) \
    | BEEPER_GET_FLAG(BEEPER_BAT_LOW) \
    | BEEPER_GET_FLAG(BEEPER_GPS_STATUS) \
    | BEEPER_GET_FLAG(BEEPER_RX_SET) \
    | BEEPER_GET_FLAG(BEEPER_ACC_CALIBRATION) \
    | BEEPER_GET_FLAG(BEEPER_ACC_CALIBRATION_FAIL) \
    | BEEPER_GET_FLAG(BEEPER_READY_BEEP) \
    | BEEPER_GET_FLAG(BEEPER_MULTI_BEEPS) \
    | BEEPER_GET_FLAG(BEEPER_DISARM_REPEAT) \
    | BEEPER_GET_FLAG(BEEPER_ARMED) \
    | BEEPER_GET_FLAG(BEEPER_SYSTEM_INIT) \
    | BEEPER_GET_FLAG(BEEPER_USB) \
    | BEEPER_GET_FLAG(BEEPER_BLACKBOX_ERASE) \
    | BEEPER_GET_FLAG(BEEPER_CRASH_FLIP_MODE) \
    | BEEPER_GET_FLAG(BEEPER_CAM_CONNECTION_OPEN) \
    | BEEPER_GET_FLAG(BEEPER_CAM_CONNECTION_CLOSE) \
    | BEEPER_GET_FLAG(BEEPER_RC_SMOOTHING_INIT_FAIL) \
    | BEEPER_GET_FLAG(BEEPER_ARMING_GPS_NO_FIX) \
    )
// DSHOT信标允许的模式   
#define DSHOT_BEACON_ALLOWED_MODES ( \
      BEEPER_GET_FLAG(BEEPER_RX_LOST) \
    | BEEPER_GET_FLAG(BEEPER_RX_SET) )

void beeper(beeperMode_e mode);
void beeperSilence(void);
void beeperUpdate(timeUs_t currentTimeUs);
void beeperConfirmationBeeps(uint8_t beepCount);
void beeperWarningBeeps(uint8_t beepCount);
bool isBeeperOn(void);
timeUs_t getLastDshotBeaconCommandTimeUs(void);

