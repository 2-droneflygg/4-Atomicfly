#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "pg/pg.h"
#include "drivers/serial.h"

/* --------------------------端口共享枚举-------------------------- */	
typedef enum {
    PORTSHARING_UNUSED = 0,
    PORTSHARING_NOT_SHARED,
    PORTSHARING_SHARED
} portSharing_e;

/* --------------------------串口功能枚举-------------------------- */	
typedef enum {
    FUNCTION_NONE                = 0,
    FUNCTION_MSP                 = (1 << 0),  
    FUNCTION_GPS                 = (1 << 1),  
    FUNCTION_RX_SERIAL           = (1 << 2),  
    FUNCTION_VTX_SMARTAUDIO      = (1 << 3), 
    FUNCTION_DEBUG               = (1 << 5), 
} serialPortFunction_e;

/* ---------------------------波特率枚举--------------------------- */	
typedef enum {
    BAUD_AUTO = 0,
    BAUD_9600,
    BAUD_19200,
    BAUD_38400,
    BAUD_57600,
    BAUD_115200,
    BAUD_230400,
    BAUD_250000,
    BAUD_400000,
    BAUD_460800,
    BAUD_500000,
    BAUD_921600,
    BAUD_1000000,
    BAUD_1500000,
    BAUD_2000000,
    BAUD_2470000
} baudRate_e;
extern const uint32_t baudRates[];

/* -------------------------串口标识符枚举------------------------- */	
// 串行端口标识符现在是固定的，这些值由MSP命令使用。
typedef enum {
    SERIAL_PORT_NONE = -1,
    SERIAL_PORT_USART1 = 0,
    SERIAL_PORT_USART2,
    SERIAL_PORT_USART3,
    SERIAL_PORT_UART4,
    SERIAL_PORT_UART5,
    SERIAL_PORT_USART6,
    SERIAL_PORT_USART7,
    SERIAL_PORT_USART8,
    SERIAL_PORT_LPUART1,
    SERIAL_PORT_USB_VCP = 20,
    SERIAL_PORT_SOFTSERIAL1 = 30,
    SERIAL_PORT_SOFTSERIAL2,
    SERIAL_PORT_IDENTIFIER_MAX = SERIAL_PORT_SOFTSERIAL2
} serialPortIdentifier_e;
// 声明串口标识符枚举
extern const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT];
// 串口标识符转换宏
#define SERIAL_PORT_IDENTIFIER_TO_INDEX(x) (((x) < RESOURCE_SOFT_OFFSET) ? (x) : (RESOURCE_SOFT_OFFSET + ((x) - SERIAL_PORT_SOFTSERIAL1)))
#define SERIAL_PORT_IDENTIFIER_TO_UARTDEV(x) ((x) - SERIAL_PORT_USART1 + UARTDEV_1)

/* -------------------------串口使用结构体------------------------- */	
typedef struct serialPortUsage_s {
    serialPort_t *serialPort;							// 端口
    serialPortFunction_e function;						// 功能
    serialPortIdentifier_e identifier;					// 标识符
} serialPortUsage_t;

serialPort_t *findSharedSerialPort(uint16_t functionMask, serialPortFunction_e sharedWithFunction);

/* -------------------------串口配置结构体------------------------- */	
typedef struct serialPortConfig_s {
    uint32_t functionMask;								// 功能
    serialPortIdentifier_e identifier;					// 标识符
    uint8_t msp_baudrateIndex;                          // MSP波特率
    uint8_t gps_baudrateIndex;							// GPS波特率
} serialPortConfig_t;

/* -------------------------串口配置结构体------------------------- */	
typedef struct serialConfig_s {
    serialPortConfig_t portConfigs[SERIAL_PORT_COUNT];  // 端口配置	
    uint16_t serial_update_rate_hz;						// 更新速率
    uint8_t reboot_character;                           // 重启字符标识
} serialConfig_t;
// 声明串口配置结构体
PG_DECLARE(serialConfig_t, serialConfig);

/* ---------------------------打开串口----------------------------- */	
serialPort_t *openSerialPort(
    serialPortIdentifier_e identifier,
    serialPortFunction_e function,
    serialReceiveCallbackPtr rxCallback,
    void *rxCallbackData,
    uint32_t baudrate,
    portMode_e mode,
    portOptions_e options
);

extern serialPort_t *debugPort;
extern serialPort_t *USB_VCP_Port;
bool SerialDebug_Init(void);
bool USB_VCP_Init(void);
void serialInit(bool softserialEnabled, serialPortIdentifier_e serialPortToDisable);
bool isSerialConfigValid(const serialConfig_t *serialConfig);
const serialPortConfig_t *serialFindPortConfiguration(serialPortIdentifier_e identifier);
const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function);
const serialPortConfig_t *findNextSerialPortConfig(serialPortFunction_e function);
void pgResetFn_serialConfig(serialConfig_t *serialConfig); //!!TODO remove need for this
serialPortUsage_t *findSerialPortUsageByIdentifier(serialPortIdentifier_e identifier);
