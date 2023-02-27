/*****************************************************************************
 软件模拟串口 - 单线半双工通信:
	波特率计算：
		假设数据传输的速率为960个字符每秒，每个字符由1个起始位、8个数据位、1个停止位构成，其传输的波特率为
		每个字符的位数：1 + 8 + 1 = 10
		每秒传输的位数：10 * 960 = 9600波特率

	定时器溢出中断（波特率）   	        	-> TX/RX进程
	定时器边沿中断（输入捕获）	       		-> 更改为RX状态
*****************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2)
#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/io.h"
#include "drivers/serial.h"
#include "drivers/timer.h"

#include "serial_softserial.h"

// ---------------------------------------------------------最大软件串口数
#if defined(USE_SOFTSERIAL1) && defined(USE_SOFTSERIAL2)
#define MAX_SOFTSERIAL_PORTS 2
#else
#define MAX_SOFTSERIAL_PORTS 1
#endif

// ---------------------------------------------------------定时器通道边沿触发
#define ICPOLARITY_RISING  true							 // 定时器通道上升沿触发
#define ICPOLARITY_FALLING false						 // 定时器通道下降沿触发

// ---------------------------------------------------------TX/RX每个字符的位数 
// 10（1个起始位、8个数据位、1个停止位）
#define RX_TOTAL_BITS 10
#define TX_TOTAL_BITS 10

/* ------------------------------定时器模式枚举------------------------------ */	
typedef enum {
    TIMER_MODE_SINGLE,  								 // 单定时器
    TIMER_MODE_DUAL,									 // 双定时器
} timerMode_e;

/* ------------------------------软件串口结构体------------------------------ */	
typedef struct softSerial_s {
    serialPort_t     port;								 // 端口
    IO_t rxIO;											 // RX引脚
    IO_t txIO;											 // TX引脚
    const timerHardware_t *timerHardware;				 // 定时器硬件	- 输入捕获
    const timerHardware_t *exTimerHardware;				 // 定时器硬件 - 输出比较
    volatile uint8_t rxBuffer[SOFTSERIAL_BUFFER_SIZE];	 // 接收缓存
    volatile uint8_t txBuffer[SOFTSERIAL_BUFFER_SIZE];	 // 发送缓存
    uint8_t          isSearchingForStartBit;			 // 是否正在搜索起始位
    uint8_t          rxBitIndex;						 // RX位索引					
    uint8_t          rxLastLeadingEdgeAtBitIndex;		 // RX位索引的上一个前缘
    uint8_t          rxEdge;							 // RX边缘
    uint8_t          rxActive;							 // 激活RX
    uint8_t          isTransmittingData;				 // 是否在传输数据
    int8_t           bitsLeftToTransmit;			     // 剩余位
    uint16_t         internalTxBuffer;  				 // 包括启动位和停止位
    uint16_t         internalRxBuffer;  				 // 包括启动位和停止位
    uint16_t         transmissionErrors;				 // 传输错误
    uint16_t         receiveErrors;						 // 接收错误
    uint8_t          softSerialPortIndex;				 // 软串口索引
    timerMode_e      timerMode;							 // 定时器模式
    timerOvrHandlerRec_t overCb;						 // 定时器溢出中断
    timerCCHandlerRec_t edgeCb;							 // 定时器通道中断回调
} softSerial_t;

// 串口虚函数表结构体变量 - 串行驱动标准API
static const struct serialPortVTable softSerialVTable;   // Forward
// 软件串口结构体变量
static softSerial_t softSerialPorts[MAX_SOFTSERIAL_PORTS];
// 函数声明
void onSerialTimerOverflow(timerOvrHandlerRec_t *cbRec, captureCompare_t capture);
void onSerialRxPinChange(timerCCHandlerRec_t *cbRec, captureCompare_t capture);

/**********************************************************************
函数名称：setTxSignal
函数功能：设置Tx信号
函数形参：softSerial，state
函数返回值：None
函数描述：None
**********************************************************************/
static void setTxSignal(softSerial_t *softSerial, uint8_t state)
{
    if (softSerial->port.options & SERIAL_INVERTED) {	 // 判断是否需要反相
        state = !state;
    }
    if (state) {										 // 设置IO电平
        IOHi(softSerial->txIO);
    } else {
        IOLo(softSerial->txIO);
    }
}

/**********************************************************************
函数名称：serialEnableCC
函数功能：使能串口定时器捕获比较
函数形参：softSerial
函数返回值：None
函数描述：None
**********************************************************************/
static void serialEnableCC(softSerial_t *softSerial)
{
	// 使能TIMx捕获比较通道x
    TIM_CCxCmd(softSerial->timerHardware->tim, softSerial->timerHardware->channel, TIM_CCx_Enable);
}

/**********************************************************************
函数名称：serialInputPortActivate
函数功能：串口输入端口激活
函数形参：softSerial
函数返回值：None
函数描述：None
**********************************************************************/
static void serialInputPortActivate(softSerial_t *softSerial)
{
	// 配置GPIO复用
    if (softSerial->port.options & SERIAL_INVERTED) {	 // 反相
        const uint8_t pinConfig = (softSerial->port.options & SERIAL_BIDIR_NOPULL) ? IOCFG_AF_PP : IOCFG_AF_PP_PD;
        IOConfigGPIOAF(softSerial->rxIO, pinConfig, softSerial->timerHardware->alternateFunction);
    } else {
        const uint8_t pinConfig = (softSerial->port.options & SERIAL_BIDIR_NOPULL) ? IOCFG_AF_PP : IOCFG_AF_PP_UP;
        IOConfigGPIOAF(softSerial->rxIO, pinConfig, softSerial->timerHardware->alternateFunction);
    }
    softSerial->rxActive = true;						 // 激活RX
    softSerial->isSearchingForStartBit = true;			 // 激活搜索起始位
    softSerial->rxBitIndex = 0;						     // 位索引重置为0
    // 使能定时器输入捕获
    serialEnableCC(softSerial);
}

/**********************************************************************
函数名称：serialInputPortDeActivate
函数功能：串口输入端口取消激活
函数形参：softSerial
函数返回值：None
函数描述：None
**********************************************************************/
static void serialInputPortDeActivate(softSerial_t *softSerial)
{
    // 使能TIMx捕获比较通道x
    TIM_CCxCmd(softSerial->timerHardware->tim, softSerial->timerHardware->channel, TIM_CCx_Disable);
	// 配置GPIO - 浮空输入
    IOConfigGPIO(softSerial->rxIO, IOCFG_IN_FLOATING);
    softSerial->rxActive = false;						 // 取消激活RX
}

/**********************************************************************
函数名称：serialOutputPortActivate
函数功能：串口输出端口激活
函数形参：softSerial
函数返回值：None
函数描述：None
**********************************************************************/
static void serialOutputPortActivate(softSerial_t *softSerial)
{
	// 配置GPIO（推挽输出） - 是否使用定时器
    if (softSerial->exTimerHardware)
        IOConfigGPIOAF(softSerial->txIO, IOCFG_OUT_PP, softSerial->exTimerHardware->alternateFunction);
    else
        IOConfigGPIO(softSerial->txIO, IOCFG_OUT_PP);
}

/**********************************************************************
函数名称：serialOutputPortDeActivate
函数功能：串口输出端口取消激活
函数形参：softSerial
函数返回值：None
函数描述：None
**********************************************************************/
static void serialOutputPortDeActivate(softSerial_t *softSerial)
{
	// 配置GPIO（浮空输入） - 是否使用定时器
    if (softSerial->exTimerHardware)
        IOConfigGPIOAF(softSerial->txIO, IOCFG_IN_FLOATING, softSerial->exTimerHardware->alternateFunction);
    else
        IOConfigGPIO(softSerial->txIO, IOCFG_IN_FLOATING);
}

/**********************************************************************
函数名称：isTimerPeriodTooLarge
函数功能：计时周期是否过大
函数形参：timerPeriod
函数返回值：result
函数描述：None
**********************************************************************/
static bool isTimerPeriodTooLarge(uint32_t timerPeriod)
{
	// 65535
    return timerPeriod > 0xFFFF;
}

/**********************************************************************
函数名称：serialTimerConfigureTimebase
函数功能：串口定时器配置时基 - 通过波特率计算定时器重装载值（PWM周期）
函数形参：timerHardwarePtr，波特率
函数返回值：None
函数描述：None
**********************************************************************/
static void serialTimerConfigureTimebase(const timerHardware_t *timerHardwarePtr, uint32_t baud)
{
	// 获取定时器时钟 - 频率
    uint32_t baseClock = timerClock(timerHardwarePtr->tim);  
    uint32_t clock = baseClock;
	// 定时器周期
    uint32_t timerPeriod;

	// 如果计时周期是否过大 - mhz保持不变…这将使波特率翻倍直到ok(但最低波特率< 1200)
    do {
		// 计算定时器周期 - 时钟 / 波特率
		// 168000000 / 9600 = 17500
        timerPeriod = clock / baud;
        if (isTimerPeriodTooLarge(timerPeriod)) {
            if (clock > 1) {
                clock = clock / 2;   
            }
        }
    } while (isTimerPeriodTooLarge(timerPeriod));
	// 配置定时器
    timerConfigure(timerHardwarePtr, timerPeriod, baseClock);
}

/**********************************************************************
函数名称：resetBuffers
函数功能：复位缓冲区
函数形参：softSerial
函数返回值：None
函数描述：None
**********************************************************************/
static void resetBuffers(softSerial_t *softSerial)
{
    softSerial->port.rxBufferSize = SOFTSERIAL_BUFFER_SIZE;
    softSerial->port.rxBuffer = softSerial->rxBuffer;
    softSerial->port.rxBufferTail = 0;
    softSerial->port.rxBufferHead = 0;

    softSerial->port.txBuffer = softSerial->txBuffer;
    softSerial->port.txBufferSize = SOFTSERIAL_BUFFER_SIZE;
    softSerial->port.txBufferTail = 0;
    softSerial->port.txBufferHead = 0;
}

/**********************************************************************
函数名称：openSoftSerial
函数功能：打开软件串口并进行初始化
函数形参：openSoftSerial，baud，mode，options
函数返回值：软串行端口
函数描述：None
**********************************************************************/
serialPort_t *openSoftSerial(softSerialPortIndex_e portIndex, uint32_t baud, portMode_e mode, portOptions_e options)
{
	// 获取软串口结构体
    softSerial_t *softSerial = &(softSerialPorts[portIndex]);    

	// 获取软串口引脚（TX）
    int pinCfgIndex = portIndex + RESOURCE_SOFT_OFFSET;
    ioTag_t tagTx = serialPinConfig()->ioTagTx[pinCfgIndex];
	IO_t txIO = IOGetByTag(tagTx);
	
	// 定义定时器引脚 
    const timerHardware_t *timerTx = timerAllocate(tagTx, OWNER_SERIAL_TX, RESOURCE_INDEX(portIndex + RESOURCE_SOFT_OFFSET));

	// IO初始化（TX）
    if (options & SERIAL_BIDIR) {
		// 为了与硬件uart的一致性，只使用TX pin，这个引脚必须有一个定时器，它不应该是n通道
        if (!timerTx || (timerTx->output & TIMER_OUTPUT_N_CHANNEL)) {
            return NULL;
        }
        softSerial->timerHardware = timerTx;
        softSerial->txIO = txIO;
        softSerial->rxIO = txIO;
        IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(portIndex + RESOURCE_SOFT_OFFSET));
    } 

    softSerial->port.vTable = &softSerialVTable;        // 虚函数表	
    softSerial->port.baudRate = baud;					// 波特率
    softSerial->port.mode = mode;						// 模式
    softSerial->port.options = options;					// 选择
    softSerial->port.rxCallback = NULL;			
    softSerial->port.rxCallbackData = NULL;	
    resetBuffers(softSerial);							// 重置缓冲区
    softSerial->softSerialPortIndex = portIndex;		// 端口索引
    softSerial->transmissionErrors = 0;
    softSerial->receiveErrors = 0;
    softSerial->rxActive = false;
    softSerial->isTransmittingData = false;

    // 配置串口定时器时基
    serialTimerConfigureTimebase(softSerial->timerHardware, baud);
    // 定时器通道中断服务回调函数初始化
    timerChCCHandlerInit(&softSerial->edgeCb, onSerialRxPinChange);
	// 定时器溢出中断服务回调函数初始化
    timerChOvrHandlerInit(&softSerial->overCb, onSerialTimerOverflow);
	// 设置通道的边沿和溢出回调
    softSerial->timerMode = TIMER_MODE_SINGLE;
	// 设置通道的边沿和溢出回调
    timerChConfigCallbacks(softSerial->timerHardware, &softSerial->edgeCb, &softSerial->overCb);
	
    return &softSerial->port;
}

/**********************************************************************
函数名称：processTxState
函数功能：TX进程状态
函数形参：softSerial
函数返回值：None
函数描述：
	由定时器溢出中断回调函数调用.
**********************************************************************/
void processTxState(softSerial_t *softSerial)
{
    uint8_t mask;
	// 判断是否正在传输数据
    if (!softSerial->isTransmittingData) {
		// 判断发送缓冲区是否为空 - 发送缓冲区为空说明数据发送完毕 - 关闭输出，切换到输入
        if (isSoftSerialTransmitBufferEmpty((serialPort_t *)softSerial)) {
            if (!softSerial->rxActive && softSerial->port.options & SERIAL_BIDIR) {
				// 串口输出端口取消激活 
                serialOutputPortDeActivate(softSerial);
				// 串口输入端口激活
                serialInputPortActivate(softSerial);
            }
            return;
        }

        // 数据发送 - 偏移缓冲区尾
        uint8_t byteToSend = softSerial->port.txBuffer[softSerial->port.txBufferTail++];
        if (softSerial->port.txBufferTail >= softSerial->port.txBufferSize) {
            softSerial->port.txBufferTail = 0;
        }

        // 建立内部缓冲区，MSB =停止位(1) + 数据位(MSB到LSB) + 起始位(0)LSB
        softSerial->internalTxBuffer = (1 << (TX_TOTAL_BITS - 1)) | (byteToSend << 1);
        softSerial->bitsLeftToTransmit = TX_TOTAL_BITS;
		// 数据正在传输 - 置位标志位
        softSerial->isTransmittingData = true;

        if (softSerial->rxActive && (softSerial->port.options & SERIAL_BIDIR)) {
            // 单线半双工: 关闭接收，切换到输出
            serialInputPortDeActivate(softSerial);
            serialOutputPortActivate(softSerial);
			// 开始发送下一个位时，因为端口操作需要时间，
			// 这里继续可能会导致位周期，在接收端的高速率下以减少导致的采样错误
			// 注意会有(略小于)1位的延迟;把它当作“转变的时间”，可以重新加载计数器并继续(未来的工作)
            return;
        }
    }

	// 发送数据
    if (softSerial->bitsLeftToTransmit) {
		// 电平是否反向
        mask = softSerial->internalTxBuffer & 1;
		// 数据移位
        softSerial->internalTxBuffer >>= 1;
		// 设置Tx引脚电平（信号）
        setTxSignal(softSerial, mask);
		// 位减1
        softSerial->bitsLeftToTransmit--;
        return;
    }
	// 数据发送完毕 - 复位标志位
    softSerial->isTransmittingData = false;
}

enum {
    TRAILING,
    LEADING
};

/**********************************************************************
函数名称：applyChangedBits
函数功能：应用变化位
函数形参：softSerial
函数返回值：None
函数描述：None
**********************************************************************/
void applyChangedBits(softSerial_t *softSerial)
{
    if (softSerial->rxEdge == TRAILING) {
        uint8_t bitToSet;
        for (bitToSet = softSerial->rxLastLeadingEdgeAtBitIndex; bitToSet < softSerial->rxBitIndex; bitToSet++) {
            softSerial->internalRxBuffer |= 1 << bitToSet;
        }
    }
}

/**********************************************************************
函数名称：prepareForNextRxByte
函数功能：准备下一个Rx字节
函数形参：softSerial
函数返回值：None
函数描述：None
**********************************************************************/
void prepareForNextRxByte(softSerial_t *softSerial)
{
    // 为下一个字节做准备
    softSerial->rxBitIndex = 0;
    softSerial->isSearchingForStartBit = true;
    if (softSerial->rxEdge == LEADING) {
        softSerial->rxEdge = TRAILING;
        timerChConfigIC(softSerial->timerHardware, (softSerial->port.options & SERIAL_INVERTED) ? ICPOLARITY_RISING : ICPOLARITY_FALLING, 0);
        serialEnableCC(softSerial);
    }
}

// 起始位掩码
#define START_BIT_MASK (1 << (RX_TOTAL_BITS - 1))    
// 停止位掩码
#define STOP_BIT_MASK  (1 << 0)

/**********************************************************************
函数名称：extractAndStoreRxByte
函数功能：提取并存储Rx字节
函数形参：softSerial
函数返回值：None
函数描述：None
**********************************************************************/
void extractAndStoreRxByte(softSerial_t *softSerial)
{
    if ((softSerial->port.mode & MODE_RX) == 0) {
        return;
    }

	// 起始位
    uint8_t haveStartBit = (softSerial->internalRxBuffer & START_BIT_MASK) == 0;  
	// 停止位
    uint8_t haveStopBit = (softSerial->internalRxBuffer & STOP_BIT_MASK) == 1;	   

	// 接收错误
    if (!haveStartBit || !haveStopBit) {
        softSerial->receiveErrors++;
        return;
    }

    uint8_t rxByte = (softSerial->internalRxBuffer >> 1) & 0xFF;

    if (softSerial->port.rxCallback) {
        softSerial->port.rxCallback(rxByte, softSerial->port.rxCallbackData);
    } else {
        softSerial->port.rxBuffer[softSerial->port.rxBufferHead] = rxByte;
        softSerial->port.rxBufferHead = (softSerial->port.rxBufferHead + 1) % softSerial->port.rxBufferSize;
    }
}

/**********************************************************************
函数名称：processRxState
函数功能：RX进程状态
函数形参：softSerial
函数返回值：None
函数描述：
	由定时器溢出中断回调函数调用.
**********************************************************************/
void processRxState(softSerial_t *softSerial)
{
    if (softSerial->isSearchingForStartBit) {
        return;
    }

    softSerial->rxBitIndex++;

    if (softSerial->rxBitIndex == RX_TOTAL_BITS - 1) {
		// 应用变化位
        applyChangedBits(softSerial);
        return;
    }

    if (softSerial->rxBitIndex == RX_TOTAL_BITS) {
        if (softSerial->rxEdge == TRAILING) {
            softSerial->internalRxBuffer |= STOP_BIT_MASK;
        }
		// 提取并存储Rx字节
        extractAndStoreRxByte(softSerial);
		// 准备下一个Rx字节
        prepareForNextRxByte(softSerial);
    }
}

// -----------------------------------------------------------------------------------定时器中断服务函数回调API

/**********************************************************************
函数名称：onSerialTimerOverflow
函数功能：串口定时器溢出
函数形参：cbRec，capture
函数返回值：None
函数描述：
	定时器溢出中断服务回调函数.
**********************************************************************/
void onSerialTimerOverflow(timerOvrHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(capture);
    softSerial_t *self = container_of(cbRec, softSerial_t, overCb);
    if (self->port.mode & MODE_TX)
        processTxState(self);
    if (self->port.mode & MODE_RX)
        processRxState(self);
}

/**********************************************************************
函数名称：onSerialRxPinChange
函数功能：串口引脚变换为RX
函数形参：cbRec，capture
函数返回值：None
函数描述：
	定时器边沿中断服务回调函数.
**********************************************************************/
void onSerialRxPinChange(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(capture);
    softSerial_t *self = container_of(cbRec, softSerial_t, edgeCb);
    bool inverted = self->port.options & SERIAL_INVERTED;

    if ((self->port.mode & MODE_RX) == 0) {
        return;
    }

	// 判断是否正在搜索起始位
    if (self->isSearchingForStartBit) {
		// 设置TIMx计数器寄存器值 - 同步位的时间，使它将中断在中心比特周期
        TIM_SetCounter(self->timerHardware->tim, self->timerHardware->tim->ARR / 2);

		// 传输错误统计
        if ((self->timerMode != TIMER_MODE_DUAL) && self->isTransmittingData) {
            self->transmissionErrors++;
        }

		// 配置输入捕获
        timerChConfigIC(self->timerHardware, inverted ? ICPOLARITY_FALLING : ICPOLARITY_RISING, 0);
        self->rxEdge = LEADING;
        self->rxBitIndex = 0;
        self->rxLastLeadingEdgeAtBitIndex = 0;
        self->internalRxBuffer = 0;
        self->isSearchingForStartBit = false;
        return;
    }

    if (self->rxEdge == LEADING) {
        self->rxLastLeadingEdgeAtBitIndex = self->rxBitIndex;
    }
	// 应用变化位
    applyChangedBits(self);
    if (self->rxEdge == TRAILING) {
        self->rxEdge = LEADING;
		// 配置输入捕获
        timerChConfigIC(self->timerHardware, inverted ? ICPOLARITY_FALLING : ICPOLARITY_RISING, 0);
    } else {
        self->rxEdge = TRAILING;
		// 配置输入捕获
        timerChConfigIC(self->timerHardware, inverted ? ICPOLARITY_RISING : ICPOLARITY_FALLING, 0);
    }
}

// -----------------------------------------------------------------------------------标准串行驱动程序API
/**********************************************************************
函数名称：softSerialRxBytesWaiting
函数功能：软串口RX字节等待
函数形参：instance
函数返回值：None
函数描述：None
**********************************************************************/
uint32_t softSerialRxBytesWaiting(const serialPort_t *instance)
{
    if ((instance->mode & MODE_RX) == 0) {
        return 0;
    }
    softSerial_t *s = (softSerial_t *)instance;
    return (s->port.rxBufferHead - s->port.rxBufferTail) & (s->port.rxBufferSize - 1);
}

/**********************************************************************
函数名称：softSerialTxBytesFree
函数功能：软串口TX字节释放
函数形参：instance
函数返回值：None
函数描述：None
**********************************************************************/
uint32_t softSerialTxBytesFree(const serialPort_t *instance)
{
    if ((instance->mode & MODE_TX) == 0) {
        return 0;
    }
    softSerial_t *s = (softSerial_t *)instance;
    uint8_t bytesUsed = (s->port.txBufferHead - s->port.txBufferTail) & (s->port.txBufferSize - 1);
    return (s->port.txBufferSize - 1) - bytesUsed;
}

/**********************************************************************
函数名称：softSerialReadByte
函数功能：软串口读字节
函数形参：instance
函数返回值：None
函数描述：None
**********************************************************************/
uint8_t softSerialReadByte(serialPort_t *instance)
{
    uint8_t ch;
    if ((instance->mode & MODE_RX) == 0) {
        return 0;
    }
    if (softSerialRxBytesWaiting(instance) == 0) {
        return 0;
    }
    ch = instance->rxBuffer[instance->rxBufferTail];
    instance->rxBufferTail = (instance->rxBufferTail + 1) % instance->rxBufferSize;
    return ch;
}

/**********************************************************************
函数名称：softSerialWriteByte
函数功能：软串口写字节
函数形参：s，ch
函数返回值：None
函数描述：None
**********************************************************************/
void softSerialWriteByte(serialPort_t *s, uint8_t ch)
{
    if ((s->mode & MODE_TX) == 0) {
        return;
    }
    s->txBuffer[s->txBufferHead] = ch;
    s->txBufferHead = (s->txBufferHead + 1) % s->txBufferSize;
}

/**********************************************************************
函数名称：softSerialSetBaudRate
函数功能：设置软串口波特率
函数形参：s，baudRate
函数返回值：None
函数描述：None
**********************************************************************/
void softSerialSetBaudRate(serialPort_t *s, uint32_t baudRate)
{
    softSerial_t *softSerial = (softSerial_t *)s;
    softSerial->port.baudRate = baudRate;
    serialTimerConfigureTimebase(softSerial->timerHardware, baudRate);
}

/**********************************************************************
函数名称：softSerialSetBaudRate
函数功能：设置软串口模式
函数形参：instance，mode
函数返回值：None
函数描述：None
**********************************************************************/
void softSerialSetMode(serialPort_t *instance, portMode_e mode)
{
    instance->mode = mode;
}

/**********************************************************************
函数名称：softSerialSetBaudRate
函数功能：软串行传输缓冲区是否为空
函数形参：instance
函数返回值：状态
函数描述：None
**********************************************************************/
bool isSoftSerialTransmitBufferEmpty(const serialPort_t *instance)
{
    return instance->txBufferHead == instance->txBufferTail;
}

// 软件串口虚函数表
static const struct serialPortVTable softSerialVTable = {
    .serialWrite = softSerialWriteByte,
    .serialTotalRxWaiting = softSerialRxBytesWaiting,
    .serialTotalTxFree = softSerialTxBytesFree,
    .serialRead = softSerialReadByte,
    .serialSetBaudRate = softSerialSetBaudRate,
    .isSerialTransmitBufferEmpty = isSoftSerialTransmitBufferEmpty,
    .setMode = softSerialSetMode,
};
#endif

