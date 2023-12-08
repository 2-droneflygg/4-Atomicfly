/*******************************************************************************************
 增强了串行端口的灵活性，但其结果是配置稍微复杂一些. 
	具有函数(GPS、串行RX等)和端口(UARTx、SoftSerial x)的概念.
	由于硬件引脚映射、相互冲突的特性、硬件和软件，不是所有的功能都可以在所有端口上使用约束.	
*******************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#if defined(USE_VCP)
#include "drivers/serial_usb_vcp.h"
#endif
#if defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2)
#include "drivers/serial_softserial.h"
#endif
#include "drivers/light_led.h"

#include "config/config.h"

#include "io/serial.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"


// 串口使用数量
static uint8_t serialPortCount;
// 串口使用信息列表
static serialPortUsage_t serialPortUsageList[SERIAL_PORT_COUNT];

/* ---------------------------串口标识----------------------------- */	
const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT] = {
#ifdef USE_VCP
    SERIAL_PORT_USB_VCP,
#endif
#ifdef USE_UART1
    SERIAL_PORT_USART1,
#endif
#ifdef USE_UART2
    SERIAL_PORT_USART2,
#endif
#ifdef USE_UART3
    SERIAL_PORT_USART3,
#endif
#ifdef USE_UART6
    SERIAL_PORT_USART6,
#endif
#ifdef USE_SOFTSERIAL1
    SERIAL_PORT_SOFTSERIAL1,
#endif
#ifdef USE_SOFTSERIAL2
    SERIAL_PORT_SOFTSERIAL2,
#endif
};

/* ----------------------------波特率------------------------------ */	
const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
        400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000};
// 波特率数量宏
#define BAUD_RATE_COUNT (sizeof(baudRates) / sizeof(baudRates[0]))

PG_REGISTER_WITH_RESET_FN(serialConfig_t, serialConfig, PG_SERIAL_CONFIG, 1);
void pgResetFn_serialConfig(serialConfig_t *serialConfig)
{
    memset(serialConfig, 0, sizeof(serialConfig_t));
	// 遍历所有端口
    for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
        serialConfig->portConfigs[i].identifier = serialPortIdentifiers[i];// 标识符
        serialConfig->portConfigs[i].msp_baudrateIndex = BAUD_115200;      // MSP波特率
        serialConfig->portConfigs[i].gps_baudrateIndex = BAUD_115200;      // GPS波特率
    }
	// VCP - USB
    serialConfig->portConfigs[0].functionMask = FUNCTION_MSP;
    serialConfig->reboot_character = 'R';
    serialConfig->serial_update_rate_hz = 100;
	// USART1 - GPS
	serialConfig->portConfigs[1].functionMask = FUNCTION_GPS;
    // USART2 - 串行接收机
    serialConfig->portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
	// USART6 - 串行接收机
	//serialConfig->portConfigs[3].functionMask = FUNCTION_RX_SERIAL;
	// 软串口1 - 图传控制
	serialConfig->portConfigs[4].functionMask = FUNCTION_VTX_SMARTAUDIO;
}

/**********************************************************************
函数名称：findSerialPortUsageByIdentifier
函数功能：通过标识符查找串口信息
函数形参：串口标识符
函数返回值：串口信息（端口，功能，标识符）
函数描述：None 
**********************************************************************/
serialPortUsage_t *findSerialPortUsageByIdentifier(serialPortIdentifier_e identifier)
{
    uint8_t index;
	// 遍历所有端口
    for (index = 0; index < SERIAL_PORT_COUNT; index++) {
        serialPortUsage_t *candidate = &serialPortUsageList[index];
        // 如果找到所需标识符则返回
        if (candidate->identifier == identifier) {
            return candidate;
        }
    }
    return NULL;
}

/* ----------------------查找串口配置结构体------------------------ */	
typedef struct findSerialPortConfigState_s {
    uint8_t lastIndex;
} findSerialPortConfigState_t;

static findSerialPortConfigState_t findSerialPortConfigState;

/**********************************************************************
函数名称：findNextSerialPortConfig
函数功能：查找下一个串口配置
函数形参：串口功能
函数返回值：串口信息（端口，功能，标识符）
函数描述：None 
**********************************************************************/
const serialPortConfig_t *findNextSerialPortConfig(serialPortFunction_e function)
{
	// 遍历所有端口
    while (findSerialPortConfigState.lastIndex < SERIAL_PORT_COUNT) {
        const serialPortConfig_t *candidate = &serialConfig()->portConfigs[findSerialPortConfigState.lastIndex++];
        // 如果找到所需标识符则返回
        if (candidate->functionMask & function) {
            return candidate;
        }
    }
    return NULL;
}

/**********************************************************************
函数名称：findSerialPortConfig
函数功能：查找串口配置
函数形参：串口功能
函数返回值：串口信息（端口，功能，标识符）
函数描述：None 
**********************************************************************/
const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function)
{
    memset(&findSerialPortConfigState, 0, sizeof(findSerialPortConfigState));
    return findNextSerialPortConfig(function);
}

/**********************************************************************
函数名称：isSerialConfigValid
函数功能：查看串口配置是否有效
函数形参：serialConfig_t
函数返回值：是否有效
函数描述：None 
**********************************************************************/
bool isSerialConfigValid(const serialConfig_t *serialConfigToCheck)
{
    UNUSED(serialConfigToCheck);
	// 遍历所有端口
    for (int index = 0; index < SERIAL_PORT_COUNT; index++) {
		// 获取串口配置
        const serialPortConfig_t *portConfig = &serialConfigToCheck->portConfigs[index];
        uint8_t bitCount = BITCOUNT(portConfig->functionMask);
        if (bitCount > 1) {
            // 如果出现两位说明端口出现共享使用（冲突）
            if (bitCount > 2) {
                return false;
            }
        }
    }
    return true;
}

/**********************************************************************
函数名称：serialFindPortConfiguration
函数功能：查找串口配置
函数形参：serialPortIdentifier_e
函数返回值：serialPortConfig_t
函数描述：None 
**********************************************************************/
const serialPortConfig_t *serialFindPortConfiguration(serialPortIdentifier_e identifier)
{
    for (int index = 0; index < SERIAL_PORT_COUNT; index++) {
        const serialPortConfig_t *candidate = &serialConfig()->portConfigs[index];
        if (candidate->identifier == identifier) {
            return candidate;
        }
    }
    return NULL;
}

/**********************************************************************
函数名称：openSerialPort
函数功能：打开串口并进行初始化
函数形参：串口标识符，功能，RX回调，RX数据回调，波特率，模式，选择
函数返回值：serialPort_t（开启的串口结构体信息）
函数描述：None 
**********************************************************************/
serialPort_t *openSerialPort(serialPortIdentifier_e identifier,serialPortFunction_e function,serialReceiveCallbackPtr rxCallback,
    								void *rxCallbackData,uint32_t baudRate,portMode_e mode,portOptions_e options)
{
#if !(defined(USE_UART) || defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2))
    UNUSED(rxCallback);
    UNUSED(rxCallbackData);
    UNUSED(baudRate);
    UNUSED(mode);
    UNUSED(options);
#endif

    serialPortUsage_t *serialPortUsage = findSerialPortUsageByIdentifier(identifier);
    if (!serialPortUsage || serialPortUsage->function != FUNCTION_NONE) {
        // 串口已被使用/串口功能不为空
        return NULL;
    }

    serialPort_t *serialPort = NULL;
    switch (identifier) {
    // USB
#ifdef USE_VCP
        case SERIAL_PORT_USB_VCP:
            serialPort = usbVcpOpen();
            break;
#endif
	// 硬件串口
#if defined(USE_UART)
#ifdef USE_UART1
        case SERIAL_PORT_USART1:
#endif
#ifdef USE_UART2
        case SERIAL_PORT_USART2:
#endif
#ifdef USE_UART3
        case SERIAL_PORT_USART3:
#endif
#ifdef USE_UART6
        case SERIAL_PORT_USART6:
#endif
            serialPort = uartOpen(SERIAL_PORT_IDENTIFIER_TO_UARTDEV(identifier), rxCallback, rxCallbackData, baudRate, mode, options);
            break;
#endif
	// 软件串口
#ifdef USE_SOFTSERIAL1
        case SERIAL_PORT_SOFTSERIAL1:
            serialPort = openSoftSerial(SOFTSERIAL1, baudRate, mode, options);
            break;
#endif
#ifdef USE_SOFTSERIAL2
        case SERIAL_PORT_SOFTSERIAL2:
            serialPort = openSoftSerial(SOFTSERIAL2, baudRate, mode, options);
            break;
#endif
        default:
            break;
    }
    // 无串口配置
    if (!serialPort) {
        return NULL;
    }
    // 注册相关信息
    serialPort->identifier = identifier;
    serialPortUsage->function = function;
    serialPortUsage->serialPort = serialPort;

    return serialPort;
}

/**********************************************************************
函数名称：serialInit
函数功能：串口初始化
函数形参：软件串口是否启用，需要禁用的串口
函数返回值：None
函数描述：None 
**********************************************************************/
void serialInit(bool softserialEnabled, serialPortIdentifier_e serialPortToDisable)
{
#if !defined(USE_SOFTSERIAL1) && !defined(USE_SOFTSERIAL2)
    UNUSED(softserialEnabled);
#endif

	// 初始化串口使用数量
    serialPortCount = SERIAL_PORT_COUNT;
	// 清空使用列表
    memset(&serialPortUsageList, 0, sizeof(serialPortUsageList));
	// 遍历所有串口
    for (int index = 0; index < SERIAL_PORT_COUNT; index++) {
		// 获取串口标识符
        serialPortUsageList[index].identifier = serialPortIdentifiers[index];
		// 判断串口是否禁用
        if (serialPortToDisable != SERIAL_PORT_NONE) {
            if (serialPortUsageList[index].identifier == serialPortToDisable) {
				// 重置串口标识符
                serialPortUsageList[index].identifier = SERIAL_PORT_NONE;
				// 串口使用计数
                serialPortCount--;
            }
        }
		// 非软件串口
        else if (serialPortUsageList[index].identifier <= SERIAL_PORT_USART8) {
			// 获取索引
            int resourceIndex = SERIAL_PORT_IDENTIFIER_TO_INDEX(serialPortUsageList[index].identifier);
			// 检查硬件配置合法性
            if (!(serialPinConfig()->ioTagTx[resourceIndex] || serialPinConfig()->ioTagRx[resourceIndex])) {
				// 重置串口标识符
                serialPortUsageList[index].identifier = SERIAL_PORT_NONE;
				// 串口使用计数
                serialPortCount--;
            }
        }
		// 软件串口
        else if ((serialPortUsageList[index].identifier == SERIAL_PORT_SOFTSERIAL1
#ifdef USE_SOFTSERIAL1
            && !(softserialEnabled && (serialPinConfig()->ioTagTx[RESOURCE_SOFT_OFFSET + SOFTSERIAL1] || serialPinConfig()->ioTagRx[RESOURCE_SOFT_OFFSET + SOFTSERIAL1]))
#endif
           ) || (serialPortUsageList[index].identifier == SERIAL_PORT_SOFTSERIAL2
#ifdef USE_SOFTSERIAL2
            && !(softserialEnabled && (serialPinConfig()->ioTagTx[RESOURCE_SOFT_OFFSET + SOFTSERIAL2] || serialPinConfig()->ioTagRx[RESOURCE_SOFT_OFFSET + SOFTSERIAL2]))
#endif
            )) {
            // 重置串口标识符
            serialPortUsageList[index].identifier = SERIAL_PORT_NONE;
			// 串口使用计数
            serialPortCount--;
        }
    }
}

/**********************************************************************
函数名称：SerialDebug_Init
函数功能：调试用途串口初始化
函数形参：None
函数返回值：端口初始化状态
函数描述：None
**********************************************************************/
serialPort_t *debugPort = NULL;
bool SerialDebug_Init(void)
{
	// 打开串口并进行初始化 
    debugPort = openSerialPort(SERIAL_PORT_SOFTSERIAL2, FUNCTION_DEBUG, NULL, NULL, 9600, MODE_TX, SERIAL_BIDIR);
    if (!debugPort) {
        return false;
    }
    return true;
}

/**********************************************************************
函数名称：USB_VCP_Init
函数功能：USB初始化
函数形参：None  
函数返回值：端口初始化状态
函数描述：None 
**********************************************************************/
serialPort_t *USB_VCP_Port = NULL;
bool USB_VCP_Init(void)
{
	// 获取MSP端口
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_MSP);
    if(portConfig != NULL) {
        // 打开串口并进行初始化
        USB_VCP_Port = openSerialPort(portConfig->identifier, FUNCTION_MSP, NULL, NULL, baudRates[portConfig->msp_baudrateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
        if(!USB_VCP_Port) {
            return false;
        }
    } else {
        return false;
    }
    return true;
}
