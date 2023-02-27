#include "common/utils.h"

// ---------------------------------------------------------相关宏定义
#define DISABLE_ARMING_FLAG(mask) (armingFlags &= ~(mask))    // 失能解锁标志位
#define ENABLE_ARMING_FLAG(mask) (armingFlags |= (mask))	  // 使能解锁标志位
#define ARMING_FLAG(mask) (armingFlags & (mask))              // 解锁标志位

#define DISABLE_STATE(mask) (stateFlags &= ~(mask))			  // 失能状态
#define ENABLE_STATE(mask) (stateFlags |= (mask))			  // 使能状态
#define STATE(mask) (stateFlags & (mask))					  // 状态标志位

#define DISABLE_FLIGHT_MODE(mask) disableFlightMode(mask)	  // 失能飞行模式标志位
#define ENABLE_FLIGHT_MODE(mask) enableFlightMode(mask)		  // 使能飞行模式标志位
#define FLIGHT_MODE(mask) (flightModeFlags & (mask))		  // 飞行模式         

/* --------------------------解锁标志位枚举-------------------------- */	
typedef enum {
    ARMED                       = (1 << 0),
    WAS_EVER_ARMED              = (1 << 1),
} armingFlag_e;

/* -----------------解锁禁用标志列出的顺序的严重程度----------------- */	
typedef enum {
    ARMING_DISABLED_NO_GYRO         = (1 << 0),
    ARMING_DISABLED_FAILSAFE        = (1 << 1),
    ARMING_DISABLED_RX_FAILSAFE     = (1 << 2),
    ARMING_DISABLED_BAD_RX_RECOVERY = (1 << 3),
    ARMING_DISABLED_BOXFAILSAFE     = (1 << 4),
    ARMING_DISABLED_RUNAWAY_TAKEOFF = (1 << 5),
    ARMING_DISABLED_CRASH_DETECTED  = (1 << 6),
    ARMING_DISABLED_THROTTLE        = (1 << 7),
    ARMING_DISABLED_ANGLE           = (1 << 8),
    ARMING_DISABLED_BOOT_GRACE_TIME = (1 << 9),
    ARMING_DISABLED_LOAD            = (1 << 10),
    ARMING_DISABLED_CALIBRATING     = (1 << 11),
    ARMING_DISABLED_CMS_MENU        = (1 << 12),
    ARMING_DISABLED_GPS             = (1 << 13),
    ARMING_DISABLED_RESC            = (1 << 14),
    ARMING_DISABLED_REBOOT_REQUIRED = (1 << 15),
    ARMING_DISABLED_ACC_CALIBRATION = (1 << 16),
    ARMING_DISABLED_MOTOR_PROTOCOL  = (1 << 17),
    ARMING_DISABLED_MSP             = (1 << 18),
    ARMING_DISABLED_ARM_SWITCH      = (1 << 19), 		
} armingDisableFlags_e;
// 解锁禁止标志位计数
#define ARMING_DISABLE_FLAGS_COUNT (LOG2(ARMING_DISABLED_ARM_SWITCH) + 1)  

/* ---------------------------飞行模式枚举--------------------------- */	
typedef enum {
    ANGLE_MODE      = (1 << 0),   		// 自稳模式
    MAG_MODE        = (1 << 1),	  		// 锁头模式
    FAILSAFE_MODE   = (1 << 2),  		// 失控保护
    GPS_RESCUE_MODE = (1 << 3)   		// GPS救援模式
} flightModeFlags_e;                                         

/* --------------------------标志位状态枚举-------------------------- */	
typedef enum {
    GPS_FIX_HOME   = (1 << 0),			// GPS家的位置确定
    GPS_FIX        = (1 << 1),			// GPS确定					
} stateFlags_t;

extern uint8_t armingFlags;
extern uint16_t flightModeFlags;
extern uint8_t stateFlags;
extern const char *armingDisableFlagNames[ARMING_DISABLE_FLAGS_COUNT];
void setArmingDisabled(armingDisableFlags_e flag);
void unsetArmingDisabled(armingDisableFlags_e flag);
bool isArmingDisabled(void);
armingDisableFlags_e getArmingDisableFlags(void);
uint16_t enableFlightMode(flightModeFlags_e mask);
uint16_t disableFlightMode(flightModeFlags_e mask);
bool sensors(uint32_t mask);
void sensorsSet(uint32_t mask);
void sensorsClear(uint32_t mask);
uint32_t sensorsMask(void);
void mwDisarm(void);

