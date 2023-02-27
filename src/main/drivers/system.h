#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------故障模式枚举------------------------------ */	
typedef enum {
    FAILURE_DEVELOPER = 0,
    FAILURE_MISSING_ACC,
    FAILURE_CONFIG_STORE_FAILURE,
    FAILURE_GYRO_INIT_FAILED,
} failureMode_e;
// 蜂鸣器警告序列时间相关定义
#define WARNING_FLASH_COUNT 5               // FLASH_警告数量
#define WARNING_FLASH_DURATION_MS 50        // FLASH_警告持续时间
#define WARNING_PAUSE_DURATION_MS 500       // FLASH_警告暂停时间
#define WARNING_CODE_DURATION_LONG_MS 250   // 警告长持续时间
#define WARNING_CODE_DURATION_SHORT_MS 50   // 警告短持续时间

/* ------------------------引导装载程序请求类型枚举------------------------ */	
typedef enum {
    BOOTLOADER_REQUEST_ROM,                 // ROM
} bootloaderRequestType_e;

// 当前晶振频率- 8或12MHz
extern uint32_t hse_value;
extern uint32_t cachedRccCsrValue;
void indicateFailure(failureMode_e mode, int repeatCount);
void failureMode(failureMode_e mode);
void systemReset(void);
void systemResetToBootloader(bootloaderRequestType_e requestType);
bool isMPUSoftReset(void);
void cycleCounterInit(void);
void systemInit(void);
void initialiseMemorySections(void);
void enableGPIOPowerUsageAndNoiseReductions(void);
typedef void extiCallbackHandlerFunc(void);
void registerExtiCallbackHandler(IRQn_Type irqn, extiCallbackHandlerFunc *fn);void unregisterExtiCallbackHandler(IRQn_Type irqn, extiCallbackHandlerFunc *fn);
void unusedPinsInit(void);
void delay(uint32_t ms);

