/*********************************************************************************
 提供一系列IO配置操作API。
*********************************************************************************/
#include "platform.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"

#include "common/utils.h"

/* ----------------------------IO端口时钟描述结构体---------------------------- */	
struct ioPortDef_s {
    rccPeriphTag_t rcc;
};
// 记录各个端口的时钟描述信息
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB1(GPIOA) },
    { RCC_AHB1(GPIOB) },
    { RCC_AHB1(GPIOC) },
    { RCC_AHB1(GPIOD) },
    { RCC_AHB1(GPIOE) },
    { RCC_AHB1(GPIOF) },
};

// ---------------------------------------------------------记录使用端口和每个端口偏移量
#if DEFIO_PORT_USED_COUNT > 0
static const uint16_t ioDefUsedMask[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_USED_LIST };
static const uint8_t  ioDefUsedOffset[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_OFFSET_LIST };
#else
// 避免-Wpedantic警告
static const uint16_t ioDefUsedMask[1] = {0};
static const uint8_t  ioDefUsedOffset[1] = {0};
#endif

// ---------------------------------------------------------IO描述信息块 - 根据使用引脚使用总数定义
#if DEFIO_IO_USED_COUNT
ioRec_t ioRecs[DEFIO_IO_USED_COUNT];
#else
// 避免-Wpedantic警告
ioRec_t ioRecs[1];
#endif

/**********************************************************************
函数名称：IOInitGlobal
函数功能：全部IO初始化
函数形参：None
函数返回值：None
函数描述：
	使用位掩码从ROM中初始化所有ioRec_t结构（IO描述信息块）。
**********************************************************************/
void IOInitGlobal(void)
{
    ioRec_t *ioRec = ioRecs;

	// 遍历GPIO端口  
    for (unsigned port = 0; port < ARRAYLEN(ioDefUsedMask); port++) {
		// 遍历GPIO引脚
        for (unsigned pin = 0; pin < sizeof(ioDefUsedMask[0]) * 8; pin++) {
			// 判断引脚是否使用
            if (ioDefUsedMask[port] & (1 << pin)) {
				// 端口间距为0x400，左移10位，100 0000 0000
                ioRec->gpio = (GPIO_TypeDef *)(GPIOA_BASE + (port << 10));   
                ioRec->pin = 1 << pin;
				// 偏移ioRec_t结构体信息块
                ioRec++;
            }
        }
    }
}

/**********************************************************************
函数名称：IOTraversePins
函数功能：遍历GPIO引脚为回调函数提供IO描述信息块
函数形参：回调函数（由unusedPinsInit调用）
函数返回值：None
函数描述：None
**********************************************************************/
void IOTraversePins(IOTraverseFuncPtr_t fnPtr)
{
	// 遍历GPIO引脚
    for (int i = 0; i < DEFIO_IO_USED_COUNT; i++) {
		// IO描述信息块
        fnPtr(&ioRecs[i]);
    }
}

/**********************************************************************
函数名称：IOGetByTag
函数功能：通过IO标签得到IO引脚
函数形参：IO标签
函数返回值：IO引脚
函数描述：None
**********************************************************************/
IO_t IOGetByTag(ioTag_t tag)
{//PB5
	// 获取端口索引 1
    const int portIdx = DEFIO_TAG_GPIOID(tag);
	// 获取引脚索引 5
    const int pinIdx = DEFIO_TAG_PIN(tag);
	// 检查端口合法性
    if (portIdx < 0 || portIdx >= DEFIO_PORT_USED_COUNT) {
        return NULL;
    }
    // 检查引脚合法性
    if (!(ioDefUsedMask[portIdx] & (1 << pinIdx))) {
        return NULL;
    }
	// _builtin_popcount() - 计算二进制中多少个1
	// 比如 ioDefUsedMask[1]   = TARGET_IO_PORTB	 // 1111 1111 1111 1101		
	// offset = 4
    int offset = __builtin_popcount(((1 << pinIdx) - 1) & ioDefUsedMask[portIdx]);
    // 加上端口偏移 - 6
    // offset = 10
    offset += ioDefUsedOffset[portIdx];
    return ioRecs + offset;
}

/**********************************************************************
函数名称：IO_Rec
函数功能：获取IO引脚
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset
函数返回值：该IO描述信息块
函数描述：None 
**********************************************************************/
ioRec_t* IO_Rec(IO_t io)
{
    return io;
}

/**********************************************************************
函数名称：IO_GPIO
函数功能：获取IO的GPIO端口
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset
函数返回值：该IO的GPIO端口
函数描述：None 
**********************************************************************/
GPIO_TypeDef* IO_GPIO(IO_t io)
{
	// 获取该IO描述信息块
    const ioRec_t *ioRec = IO_Rec(io);
	// 返回该IO的GPIO端口
    return ioRec->gpio;
}

/**********************************************************************
函数名称：IO_Pin
函数功能：获取IO的引脚
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset
函数返回值：该IO的GPIO引脚
函数描述：None 
**********************************************************************/
uint16_t IO_Pin(IO_t io)
{
	// 获取该IO描述信息块
    const ioRec_t *ioRec = IO_Rec(io);
	// 返回该IO的GPIO引脚
    return ioRec->pin;
}

/**********************************************************************
函数名称：IO_GPIOPortIdx
函数功能：获取IO的端口索引（ID）
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset
函数返回值：该IO的端口索引（ID）
函数描述：None
**********************************************************************/
int IO_GPIOPortIdx(IO_t io)
{
	// 判断端口是否存在
    if (!io) {
        return -1;
    }
	// 返回端口索引（ID）
    return (((size_t)IO_GPIO(io) - GPIOA_BASE) >> 10);     
}

/**********************************************************************
函数名称：IO_EXTI_PortSourceGPIO
函数功能：获取该外部中断IO的GPIO端口索引（ID）
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset
函数返回值：该IO的端口索引（ID）
函数描述：None
**********************************************************************/
int IO_EXTI_PortSourceGPIO(IO_t io)
{
    return IO_GPIOPortIdx(io);
}

/**********************************************************************
函数名称：IO_GPIOPinIdx
函数功能：获取IO的引脚索引（ID）
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset
函数返回值：引脚索引
函数描述：None
**********************************************************************/
int IO_GPIOPinIdx(IO_t io)
{
    if (!io) {
        return -1;
    }
	// __builtin_clz：返回前导的0的个数。
	// GPIO每个寄存器为32bit，占四个字节，这些寄存器都是按顺序依次排列在外设的基地址上。
	// 寄存器的位置都以相对该外设基地址的偏移地址来描述
    return 31 - __builtin_clz(IO_Pin(io));  
}

/**********************************************************************
函数名称：IO_EXTI_PinSource
函数功能：获取外部中断IO的引脚索引（ID）
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset
函数返回值：引脚索引
函数描述：None
**********************************************************************/
int IO_EXTI_PinSource(IO_t io)
{
    return IO_GPIOPinIdx(io);
}

/**********************************************************************
函数名称：IO_GPIO_PinSource
函数功能：获取IO的引脚索引（ID）
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset
函数返回值：引脚索引
函数描述：None
**********************************************************************/
int IO_GPIO_PinSource(IO_t io)
{
    return IO_GPIOPinIdx(io);
}

/**********************************************************************
函数名称：IO_GPIO_PinSource
函数功能：获取io的外部中断线
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset
函数返回值：该IO的外部中断线
函数描述：None
**********************************************************************/
uint32_t IO_EXTI_Line(IO_t io)
{
	// 检查IO合法性
    if (!io) {
        return 0;
    }
	// 将IO转换为EXTI线
	// EXTI_Line0       ((uint32_t)0x00001)   
	// EXTI_Line1       ((uint32_t)0x00002)
    return 1 << IO_GPIOPinIdx(io);
}

/**********************************************************************
函数名称：IORead
函数功能：读取IO电平
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset
函数返回值：IO读取到的电平
函数描述：None
**********************************************************************/
bool IORead(IO_t io)
{
	// 检查IO合法性
    if (!io) {
        return false;
    }
    return (IO_GPIO(io)->IDR & IO_Pin(io));
}

/**********************************************************************
函数名称：IOWrite
函数功能：写IO电平(推挽形式)
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset，高or低电平
函数返回值：None
函数描述：None
**********************************************************************/
void IOWrite(IO_t io, bool hi)
{
	// 检查IO合法性
    if (!io) {
        return;
    }
    if (hi) {
		// 高电平
        IO_GPIO(io)->BSRRL = IO_Pin(io);
    } else {
    	// 低电平
        IO_GPIO(io)->BSRRH = IO_Pin(io);
    }
}

/**********************************************************************
函数名称：IOHi
函数功能：写IO电平（只进行置位）
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset
函数返回值：None
函数描述：None
**********************************************************************/
void IOHi(IO_t io)
{
	// 检查IO合法性
    if (!io) {
        return;
    }
	// 高电平
    IO_GPIO(io)->BSRRL = IO_Pin(io);
}

/**********************************************************************
函数名称：IOLo
函数功能：写IO电平（只进行复位）
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset
函数返回值：None
函数描述：None
**********************************************************************/
void IOLo(IO_t io)
{
	// 检查IO合法性
    if (!io) {
        return;
    }
	// 低电平
    IO_GPIO(io)->BSRRH = IO_Pin(io);
}

/**********************************************************************
函数名称：IOToggle
函数功能：IO电平切换
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset
函数返回值：None
函数描述：None
**********************************************************************/
void IOToggle(IO_t io)
{
	// 检查IO合法性
    if (!io) {
        return;
    }
	// 获取IO掩码
    uint32_t mask = IO_Pin(io);
	// 电平取反
    if (IO_GPIO(io)->ODR & mask) {
        IO_GPIO(io)->BSRRH = mask;
    } else {
        IO_GPIO(io)->BSRRL = mask;
    }
}

/**********************************************************************
函数名称：IOInit
函数功能：IO初始化
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset，owner，index
函数返回值：None
函数描述：
	声明IO pin，设置所有者和资源
**********************************************************************/
void IOInit(IO_t io, resourceOwner_e owner, uint8_t index)
{
	// 检查IO合法性
    if (!io) {
        return;
    }
	// 获取该IO描述信息块
    ioRec_t *ioRec = IO_Rec(io);
	// 注册所有者
    ioRec->owner = owner; 
	// 注册所有者索引
    ioRec->index = index;  
}

/**********************************************************************
函数名称：IOGetOwner
函数功能：获取IO所有者
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset
函数返回值：该IO所有者
函数描述：None
**********************************************************************/
resourceOwner_e IOGetOwner(IO_t io)
{
	// 检查IO合法性
    if (!io) {
        return OWNER_FREE;
    }
	// 获取该IO描述信息块
    const ioRec_t *ioRec = IO_Rec(io);
	// 返回该IO所有者
    return ioRec->owner;
}

/**********************************************************************
函数名称：IORelease
函数功能：释放IO所有者
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset
函数返回值：None
函数描述：None
**********************************************************************/
void IORelease(IO_t io)
{
	// 检查IO合法性
    if (!io) {
        return;
    }
	// 获取该IO描述信息块
    ioRec_t *ioRec = IO_Rec(io);
	// 释放该IO所有者
    ioRec->owner = OWNER_FREE;
}

/**********************************************************************
函数名称：IOIsFreeOrPreinit
函数功能：获取IO是否为空闲或者是预初始化
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset
函数返回值：该IO是否为空闲或者是预初始化
函数描述：None
**********************************************************************/
bool IOIsFreeOrPreinit(IO_t io)
{
	// 获取该IO所有者
    resourceOwner_e owner = IOGetOwner(io);
	// 判断是否为空闲或者是预初始化
    if (owner == OWNER_FREE || owner == OWNER_PREINIT) {
        return true;
    }
    return false;
}

/**********************************************************************
函数名称：IOConfigGPIO
函数功能：配置GPIO
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset，配置
函数返回值：None
函数描述：None
**********************************************************************/
void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
	// 检查IO合法性
    if (!io) {
        return;
    }
	// 获取GPIO端口时钟
    const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
	// 使能时钟
    RCC_ClockCmd(rcc, ENABLE);
	// GPIO配置
    GPIO_InitTypeDef init = {
        .GPIO_Pin = IO_Pin(io),                // 引脚
        .GPIO_Mode = (cfg >> 0) & 0x03,        // 模式
        .GPIO_Speed = (cfg >> 2) & 0x03,       // 速度
        .GPIO_OType = (cfg >> 4) & 0x01,       // 类型
        .GPIO_PuPd = (cfg >> 5) & 0x03,        // 上下拉
    };
	// GPIO初始化
    GPIO_Init(IO_GPIO(io), &init);
}

/**********************************************************************
函数名称：IOConfigGPIO
函数功能：GPIO引脚复用配置
函数形参：通过IOGetByTag函数将IO标签转换 - ioRecs + 该引脚offset，配置，复用功能
函数返回值：None
函数描述：None
**********************************************************************/
void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
{
	// 检查IO合法性
    if (!io) {
        return;
    }
	// 获取GPIO端口时钟
    const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
	// 使能时钟
    RCC_ClockCmd(rcc, ENABLE);
	// 复用IO
    GPIO_PinAFConfig(IO_GPIO(io), IO_GPIO_PinSource(io), af);
	// GPIO配置
    GPIO_InitTypeDef init = {
        .GPIO_Pin = IO_Pin(io),                // 引脚
        .GPIO_Mode = (cfg >> 0) & 0x03,        // 模式
        .GPIO_Speed = (cfg >> 2) & 0x03,       // 速度
        .GPIO_OType = (cfg >> 4) & 0x01,       // 类型
        .GPIO_PuPd = (cfg >> 5) & 0x03,        // 上下拉
    };
	// GPIO初始化
    GPIO_Init(IO_GPIO(io), &init);
}

