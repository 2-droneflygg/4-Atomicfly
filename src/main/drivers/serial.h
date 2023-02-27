#pragma once

#include "drivers/io.h"
#include "drivers/io_types.h"
#include "drivers/resource.h"

#include "pg/pg.h"

// 串口最大索引
#if defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2)
#ifdef USE_SOFTSERIAL2
#define SERIAL_PORT_MAX_INDEX (RESOURCE_SOFT_OFFSET + 2)
#else
#define SERIAL_PORT_MAX_INDEX (RESOURCE_SOFT_OFFSET + 1)
#endif
#else
#define SERIAL_PORT_MAX_INDEX  RESOURCE_SOFT_OFFSET
#endif

/* ---------------------------------端口模式枚举--------------------------------- */	
typedef enum {
    MODE_RX = 1 << 0,								// RX					
    MODE_TX = 1 << 1,								// TX	
    MODE_RXTX = MODE_RX | MODE_TX					// TX && RX	
} portMode_e;

/* ---------------------------------端口选项枚举--------------------------------- */	
typedef enum {
    SERIAL_NOT_INVERTED  = 0 << 0,              	// 无反相
    SERIAL_INVERTED      = 1 << 0,					// 反相
    SERIAL_STOPBITS_1    = 0 << 1,					// 停止位 - 1位
    SERIAL_STOPBITS_2    = 1 << 1,					// 停止位 - 2位
    SERIAL_PARITY_NO     = 0 << 2,					// 无校验
    SERIAL_PARITY_EVEN   = 1 << 2,					// 奇偶校验
    SERIAL_UNIDIR        = 0 << 3,					// 单向
    SERIAL_BIDIR         = 1 << 3,					// 单线半双工 - VTX
    SERIAL_BIDIR_OD      = 0 << 4, 			    	// 单线半双工 - 开漏
    SERIAL_BIDIR_PP      = 1 << 4, 			    	// 单线半双工 - 推挽
    											   		// 确保第一个起始位被发送
    											   		// 前置一个0字节(0x00)到实际的数据字节
    SERIAL_BIDIR_NOPULL  = 1 << 5, 			    	// 单线半双工 - 无上下拉
} portOptions_e;

/* ----------------------------------串口结构体---------------------------------- */	
// 串行驱动程序用来将帧返回到应用
typedef void (*serialReceiveCallbackPtr)(uint16_t data, void *rxCallbackData);   
typedef void (*serialIdleCallbackPtr)();
typedef struct serialPort_s {
    const struct serialPortVTable *vTable;      	// 虚函数表
    portMode_e mode;								// 模式
    portOptions_e options;							// 选项
    uint32_t baudRate;								// 波特率
    uint32_t rxBufferSize;							// 接收缓存大小
    uint32_t txBufferSize;							// 发送缓存大小
    volatile uint8_t *rxBuffer;						// 接收缓存
    volatile uint8_t *txBuffer;						// 发送缓存
    uint32_t rxBufferHead;							// 接收缓存头
    uint32_t rxBufferTail;							// 接收缓存尾
    uint32_t txBufferHead;							// 发送缓存头
    uint32_t txBufferTail;							// 发送缓存尾
    serialReceiveCallbackPtr rxCallback;	    	// 接收回调函数
    void *rxCallbackData;							// 接收数据回调
    serialIdleCallbackPtr idleCallback;				// 空闲回调
    uint8_t identifier;								// 标识符
} serialPort_t;

/* ------------------------------串口引脚配置结构体------------------------------ */	
typedef struct serialPinConfig_s {
    ioTag_t ioTagTx[SERIAL_PORT_MAX_INDEX];			// TX
    ioTag_t ioTagRx[SERIAL_PORT_MAX_INDEX];			// RX
    ioTag_t ioTagInverter[SERIAL_PORT_MAX_INDEX];	// 反相器
} serialPinConfig_t;
// 声明串口引脚配置结构体
PG_DECLARE(serialPinConfig_t, serialPinConfig);

/* ------------------------------串口虚函数表结构体------------------------------ */	
struct serialPortVTable {
    void (*serialWrite)(serialPort_t *instance, uint8_t ch);
    uint32_t (*serialTotalRxWaiting)(const serialPort_t *instance);
    uint32_t (*serialTotalTxFree)(const serialPort_t *instance);
    uint8_t (*serialRead)(serialPort_t *instance);
    void (*serialSetBaudRate)(serialPort_t *instance, uint32_t baudRate);
    bool (*isSerialTransmitBufferEmpty)(const serialPort_t *instance);
    void (*setMode)(serialPort_t *instance, portMode_e mode);
    void (*writeBuf)(serialPort_t *instance, const void *data, int count);
    void (*beginWrite)(serialPort_t *instance);
    void (*endWrite)(serialPort_t *instance);
};

void serialWrite(serialPort_t *instance, uint8_t ch);
uint32_t serialRxBytesWaiting(const serialPort_t *instance);
uint32_t serialTxBytesFree(const serialPort_t *instance);
void serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count);
uint8_t serialRead(serialPort_t *instance);
void serialSetBaudRate(serialPort_t *instance, uint32_t baudRate);
void serialSetCtrlLineStateCb(serialPort_t *instance, void (*cb)(void *context, uint16_t ctrlLineState), void *context);
void serialSetBaudRateCb(serialPort_t *instance, void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context);
bool isSerialTransmitBufferEmpty(const serialPort_t *instance);
void waitForSerialPortToFinishTransmitting(serialPort_t *serialPort);
void serialPrint(serialPort_t *instance, const char *str);
void serialWriteBufShim(void *instance, const uint8_t *data, int count);
void serialBeginWrite(serialPort_t *instance);
void serialEndWrite(serialPort_t *instance);

