/*********************************************************************************
 提供MAX7456相关驱动及配置API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_MAX7456
#include "pg/max7456.h"
#include "pg/vcd.h"

#include "drivers/bus_spi.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/max7456.h"
#include "drivers/nvic.h"
#include "drivers/osd.h"
#include "drivers/osd_symbols.h"
#include "drivers/time.h"

// ---------------------------------------------------------MAX7456寄存器地址
#define MAX7456ADD_READ         0x80    // 视频模式0读地址
#define MAX7456ADD_VM0          0x00    // 视频模式0写地址 
#define MAX7456ADD_VM1          0x01	// 视频模式1写地址
#define MAX7456ADD_HOS          0x02	// 水平偏移写地址
#define MAX7456ADD_VOS          0x03	// 垂直偏移写地址
#define MAX7456ADD_DMM          0x04	// 显示存储器模式
#define MAX7456ADD_DMAH         0x05	// 显示存储器地址高
#define MAX7456ADD_DMAL         0x06	// 显示存储器地址低
#define MAX7456ADD_DMDI         0x07	// 显示存储器数据输入
#define MAX7456ADD_CMM          0x08	// 字符存储器模式
#define MAX7456ADD_CMAH         0x09	// 字符存储器地址高
#define MAX7456ADD_CMAL         0x0a	// 字符存储器地址低
#define MAX7456ADD_CMDI         0x0b	// 字符存储器数据输入
#define MAX7456ADD_OSDM         0x0c	// OSD插入叠加
#define MAX7456ADD_RB0          0x10	// 行0亮度
#define MAX7456ADD_RB1          0x11	// 行1亮度
#define MAX7456ADD_RB2          0x12	// 行2亮度
#define MAX7456ADD_RB3          0x13	// 行3亮度
#define MAX7456ADD_RB4          0x14	// 行4亮度
#define MAX7456ADD_RB5          0x15	// 行5亮度
#define MAX7456ADD_RB6          0x16	// 行6亮度
#define MAX7456ADD_RB7          0x17	// 行7亮度
#define MAX7456ADD_RB8          0x18	// 行8亮度
#define MAX7456ADD_RB9          0x19	// 行9亮度
#define MAX7456ADD_RB10         0x1a	// 行10亮度
#define MAX7456ADD_RB11         0x1b	// 行11亮度
#define MAX7456ADD_RB12         0x1c	// 行12亮度
#define MAX7456ADD_RB13         0x1d	// 行13亮度
#define MAX7456ADD_RB14         0x1e	// 行14亮度
#define MAX7456ADD_RB15         0x1f	// 行15亮度
#define MAX7456ADD_OSDBL        0x6c	// OSD黑电平
#define MAX7456ADD_STAT         0xA0	// 状态寄存器
#define WRITE_NVR               0xA0	// 字符存储器模式寄存器 - 从影子存储器写入到字符存储器

// ---------------------------------------------------------MAX7456和OSD使能状态
#define MAX7456_RESET               0x02
#define OSD_ENABLE                  0x08

// ---------------------------------------------------------视频模式
#define VIDEO_MODE_PAL              0x40
#define VIDEO_MODE_NTSC             0x00
#define VIDEO_MODE_MASK             0x40
#define VIDEO_MODE_IS_PAL(val)      (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_PAL)
#define VIDEO_MODE_IS_NTSC(val)     (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_NTSC)

// ---------------------------------------------------------等待输入稳定的时间
#define VIDEO_SIGNAL_DEBOUNCE_MS    100 						   

// ---------------------------------------------------------状态寄存器 && 获取状态宏
#define STAT_PAL      0x01
#define STAT_NTSC     0x02
#define STAT_LOS      0x04
#define STAT_NVR_BUSY 0x20
#define STAT_IS_PAL(val)  ((val) & STAT_PAL)					   // 获取状态是否为PAL
#define STAT_IS_NTSC(val) ((val) & STAT_NTSC)					   // 获取状态是否为NTSC
#define STAT_IS_LOS(val)  ((val) & STAT_LOS)
#define VIN_IS_PAL(val)  (!STAT_IS_LOS(val) && STAT_IS_PAL(val))   // 获取视频输入是否为PAL制式
#define VIN_IS_NTSC(val)  (!STAT_IS_LOS(val) && STAT_IS_NTSC(val)) // 获取视频输入是否为NTSC制式

// ---------------------------------------------------------Kluege警告!
// 即使使用!LOS 
// 当这种情况发生时，STAT寄存器的低3位被读为0
// 为了处理这种情况，这个宏将!LOS && !PAL定义为NTSC
#define VIN_IS_NTSC_alt(val)  (!STAT_IS_LOS(val) && !STAT_IS_PAL(val))

// ---------------------------------------------------------MAX7456信号检查间隔MS
#define MAX7456_SIGNAL_CHECK_INTERVAL_MS 1000 	
// ---------------------------------------------------------MAX7456失速检查间隔MS
#define MAX7456_STALL_CHECK_INTERVAL_MS  1000 					   

// ---------------------------------------------------------DMM特殊位
#define CLEAR_DISPLAY           0x04    // 清除显示存储器
#define INVERT_PIXEL_COLOR 		0x08	// 颜色反转

// ---------------------------------------------------------结束字符
#define END_STRING         		0xff	// 255(0-255个字符)

// ---------------------------------------------------------设备类型
#define MAX7456_DEVICE_TYPE_MAX 0
#define MAX7456_DEVICE_TYPE_AT  1

// ---------------------------------------------------------每行字符数量（总：30*16 = 480）
#define CHARS_PER_LINE      30          						  

// ---------------------------------------------------------MAX7456支持的层数
#define MAX7456_SUPPORTED_LAYER_COUNT (DISPLAYPORT_LAYER_BACKGROUND + 1)

// ---------------------------------------------------------显示层结构体定义 - 前景、背景
typedef struct max7456Layer_s {
	// 显存
    uint8_t buffer[VIDEO_BUFFER_CHARS_PAL];
} max7456Layer_t;
// 显示层显存
static max7456Layer_t displayLayers[MAX7456_SUPPORTED_LAYER_COUNT];
// 初始化激活层 - 前景
static displayPortLayer_e activeLayer = DISPLAYPORT_LAYER_FOREGROUND;  

// ---------------------------------------------------------影子缓存 - 将所有内容写入活动层，然后进行比较 - 使用shadowBuffer只更新已更改的字符，这个解决方案比重绘整个屏幕更快
static uint8_t shadowBuffer[VIDEO_BUFFER_CHARS_PAL];
// ---------------------------------------------------------SPI缓存
// 一次空闲时要更新的最大字符数
#define MAX_CHARS2UPDATE    100
static uint8_t spiBuff[MAX_CHARS2UPDATE*6];

// ---------------------------------------------------------MAX7456设备初始化
busDevice_t max7456BusDevice;
busDevice_t *busdev = &max7456BusDevice;

// ---------------------------------------------------------MAX7456设备检测初始化
static bool max7456DeviceDetected = false;						 
static uint16_t max7456SpiClock = MAX7456_SPI_CLK;

// ---------------------------------------------------------在共享SPI总线上，改变片选线选中器件
#define __spiBusTransactionBegin(busdev)     {spiBusSetDivisor(busdev, max7456SpiClock);IOLo((busdev)->busdev_u.spi.csnPin);}
#define __spiBusTransactionEnd(busdev)       {IOHi((busdev)->busdev_u.spi.csnPin);spiSetDivisor((busdev)->busdev_u.spi.instance, MAX7456_RESTORE_CLK);}

// ---------------------------------------------------------定义屏幕最大尺寸 - PAL
uint16_t maxScreenSize = VIDEO_BUFFER_CHARS_PAL;

// ---------------------------------------------------------视频信号配置
static uint8_t  videoSignalCfg;					// 视频信号配置
static uint8_t  videoSignalReg  = OSD_ENABLE; 	// 视频信号寄存器 - OSD_ENABLE需要触发第一个ReInit
static uint8_t  displayMemoryModeReg = 0;		// 显存模式寄存器		
static uint8_t  hosRegValue; 				    // HOS(水平偏移寄存器)值
static uint8_t  vosRegValue; 				  	// VOS(垂直偏移寄存器)值

// ---------------------------------------------------------字符加载状态
static bool fontIsLoading = false;			

// ---------------------------------------------------------max7456设备类型
static uint8_t max7456DeviceType;

// ---------------------------------------------------------以前的状态初始化在有效范围之外，以在第一次调用时强制更新
#define INVALID_PREVIOUS_REGISTER_STATE 255
static uint8_t previousBlackWhiteRegister = INVALID_PREVIOUS_REGISTER_STATE;
static uint8_t previousInvertRegister = INVALID_PREVIOUS_REGISTER_STATE;

// ---------------------------------------------------------函数声明
static void max7456DrawScreenSlow(void);

//-------------------------------------------------------------------------------------显示层操作相关API

/**********************************************************************
函数名称：getLayerBuffer
函数功能：获取层缓存
函数形参：前景或背景
函数返回值：None
函数描述：None
**********************************************************************/
static uint8_t *getLayerBuffer(displayPortLayer_e layer)
{
    return displayLayers[layer].buffer;
}

/**********************************************************************
函数名称：getActiveLayerBuffer
函数功能：获取激活层缓存
函数形参：None
函数返回值：激活层缓存数组
函数描述：None
**********************************************************************/
static uint8_t *getActiveLayerBuffer(void)
{
    return getLayerBuffer(activeLayer);
}

/**********************************************************************
函数名称：max7456ClearLayer
函数功能：MAX7456清除层
函数形参：layer
函数返回值：None
函数描述：
	缓冲区由空白字符(0x20)填充。
**********************************************************************/
static void max7456ClearLayer(displayPortLayer_e layer)
{
    memset(getLayerBuffer(layer), 0x20, VIDEO_BUFFER_CHARS_PAL);
}

/**********************************************************************
函数名称：max7456LayerSupported
函数功能：MAX7456是否支持该层
函数形参：layer
函数返回值：result
函数描述：None
**********************************************************************/
bool max7456LayerSupported(displayPortLayer_e layer)
{
    if (layer == DISPLAYPORT_LAYER_FOREGROUND || layer == DISPLAYPORT_LAYER_BACKGROUND) {
        return true;
    } else {
        return false;
    }
}

/**********************************************************************
函数名称：max7456LayerSupported
函数功能：获取MAX7456是否支持该层
函数形参：layer
函数返回值：状态
函数描述：None
**********************************************************************/
bool max7456LayerSelect(displayPortLayer_e layer)
{
    if (max7456LayerSupported(layer)) {
        activeLayer = layer;
        return true;
    } else {
        return false;
    }
}

/**********************************************************************
函数名称：max7456LayerCopy
函数功能：MAX7456复制层
函数形参：destLayer，sourceLayer
函数返回值：状态
函数描述：None
**********************************************************************/
bool max7456LayerCopy(displayPortLayer_e destLayer, displayPortLayer_e sourceLayer)
{
    if ((sourceLayer != destLayer) && max7456LayerSupported(sourceLayer) && max7456LayerSupported(destLayer)) {
        memcpy(getLayerBuffer(destLayer), getLayerBuffer(sourceLayer), VIDEO_BUFFER_CHARS_PAL);
        return true;
    } else {
        return false;
    }
}

/**********************************************************************
函数名称：max7456ClearScreen
函数功能：MAX7456清除屏幕
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void max7456ClearScreen(void)
{
    max7456ClearLayer(activeLayer);
}

//-------------------------------------------------------------------------------------字符操作相关API

/**********************************************************************
函数名称：max7456WriteChar
函数功能：MAX7456写字符
函数形参：x，y，c
函数返回值：None
函数描述：None
**********************************************************************/
void max7456WriteChar(uint8_t x, uint8_t y, uint8_t c)
{
    uint8_t *buffer = getActiveLayerBuffer();
    if (x < CHARS_PER_LINE && y < VIDEO_LINES_PAL) {
		// 写入显存对应位置
        buffer[y * CHARS_PER_LINE + x] = c;
    }
}

/**********************************************************************
函数名称：max7456Write
函数功能：MAX7456写字符串
函数形参：x，y，*buff
函数返回值：None
函数描述：None
**********************************************************************/
void max7456Write(uint8_t x, uint8_t y, const char *buff)
{
    if (y < VIDEO_LINES_PAL) {
        uint8_t *buffer = getActiveLayerBuffer();
        for (int i = 0; buff[i] && x + i < CHARS_PER_LINE; i++) {
			// 写入显存对应位置 - +i字符
            buffer[y * CHARS_PER_LINE + x + i] = buff[i];
        }
    }
}

//-------------------------------------------------------------------------------------MAX7456数据传输相关API

/**********************************************************************
函数名称：max7456Send
函数功能：MAX7456发送数据
函数形参：txByte，寄存器
函数返回值：返回的数据
函数描述：None
**********************************************************************/
static uint8_t max7456Send(uint8_t add, uint8_t data)
{
    spiTransferByte(busdev->busdev_u.spi.instance, add);
    return spiTransferByte(busdev->busdev_u.spi.instance, data);
}

/**********************************************************************
函数名称：max7456DmaInProgress
函数功能：MAX7456 DMA是否正在运行
函数形参：None
函数返回值：状态
函数描述：None
**********************************************************************/
bool max7456DmaInProgress(void)
{
    return false;
}

//-------------------------------------------------------------------------------------MAX7456显存操作相关API

/**********************************************************************
函数名称：max7456ClearShadowBuffer
函数功能：MAX7456清除影子缓存
函数形参：None
函数返回值：None
函数描述：
	当清除影子缓冲区时，填充0，以便字符将与在层缓冲区中使用的0x20相比，被标记为改变。
**********************************************************************/
static void max7456ClearShadowBuffer(void)
{
    memset(shadowBuffer, 0, maxScreenSize);
}

/**********************************************************************
函数名称：max7456BuffersSynced
函数功能：获取MAX745显示层缓存是否同步
函数形参：None
函数返回值：同步状态
函数描述：
	比对显存与影子缓存数据是否一致 - 如果不一致则刷新。
**********************************************************************/
bool max7456BuffersSynced(void)
{
	// 遍历所有显示层
    for (int i = 0; i < maxScreenSize; i++) {
		// 判断激活层显存是否与影子缓存一致
        if (displayLayers[DISPLAYPORT_LAYER_FOREGROUND].buffer[i] != shadowBuffer[i]) {
            return false;
        }
    }
    return true;
}

//-------------------------------------------------------------------------------------MAX7456像素设置相关API
 
/**********************************************************************
函数名称：max7456Invert
函数功能：MAX7456设置黑白像素的反转
函数形参：invert
函数返回值：None
函数描述：None
**********************************************************************/
void max7456Invert(bool invert)
{
    if (invert) {
        displayMemoryModeReg |= INVERT_PIXEL_COLOR;
    } else {
        displayMemoryModeReg &= ~INVERT_PIXEL_COLOR;
    }

    if (displayMemoryModeReg != previousInvertRegister) {
		// 清除影子缓冲区，使所有字符以适当的反转状态重绘
        max7456ClearShadowBuffer();
        previousInvertRegister = displayMemoryModeReg;
        __spiBusTransactionBegin(busdev);
        max7456Send(MAX7456ADD_DMM, displayMemoryModeReg);
        __spiBusTransactionEnd(busdev);
    }
}

/**********************************************************************
函数名称：max7456Brightness
函数功能：MAX7456设置黑白像素的亮度
函数形参：黑亮度(0- 3,0是最暗的),white白色亮度(0- 3,0是最暗的)
函数返回值：None
函数描述：None
**********************************************************************/
void max7456Brightness(uint8_t black, uint8_t white)
{
    const uint8_t reg = (black << 2) | (3 - white);

    if (reg != previousBlackWhiteRegister) {
        previousBlackWhiteRegister = reg;
        __spiBusTransactionBegin(busdev);
        for (int i = MAX7456ADD_RB0; i <= MAX7456ADD_RB15; i++) {
            max7456Send(i, reg);
        }
        __spiBusTransactionEnd(busdev);
    }
}

//-------------------------------------------------------------------------------------MAX7456绘制屏幕相关API

/**********************************************************************
函数名称：max7456DrawScreen
函数功能：MAX7456绘制屏幕
函数形参：None
函数返回值：None
函数描述：
	获取显存 - 与影子缓存做比对，只更新有改变的缓存部分。
**********************************************************************/
void max7456DrawScreen(void)
{
    static uint16_t pos = 0;

    if (!fontIsLoading) {
        // MAX7456是否需要重新初始化
        max7456ReInitIfRequired(false);

		// ----------------获取激活层缓存
        uint8_t *buffer = getActiveLayerBuffer();

        int buff_len = 0;
		// ----------------遍历所有字符
        for (int k = 0; k < MAX_CHARS2UPDATE; k++) {
			// 只更新改变的缓存
            if (buffer[pos] != shadowBuffer[pos]) {
                spiBuff[buff_len++] = MAX7456ADD_DMAH;
                spiBuff[buff_len++] = pos >> 8;				// 显示存储器地址高 - DMAH[0]为显示存储器地址的最高位		
                spiBuff[buff_len++] = MAX7456ADD_DMAL;
                spiBuff[buff_len++] = pos & 0xff;			// 显示存储器地址低 - 决定了字符的显示区域，由DMAH[0]控制
                spiBuff[buff_len++] = MAX7456ADD_DMDI;
                spiBuff[buff_len++] = buffer[pos];			// 显示存储器数据输入 - 字符地址与字符属性字节一同存储
                shadowBuffer[pos] = buffer[pos];			// 更新影子缓存
            }
			// 判断是否超出屏幕尺寸
            if (++pos >= maxScreenSize) {
                pos = 0;
                break;
            }
        }
		// ----------------数据传输
        if (buff_len) {
			// 拉低片选 - 开始通信
            __spiBusTransactionBegin(busdev);
			// SPI传输
            spiTransfer(busdev->busdev_u.spi.instance, spiBuff, NULL, buff_len);
			// 拉高片选 - 结束通信
            __spiBusTransactionEnd(busdev);
        }
    }
}

/**********************************************************************
函数名称：max7456DrawScreenSlow
函数功能：MAX7456缓慢绘制屏幕
函数形参：None
函数返回值：None
函数描述：
	8位模式：
			DMAH[1]=0， DMDI 为字符地址
			DMAH[1]=1， DMDI 为字符属性
	16位模式：
			B7-0＝字符地址
**********************************************************************/
static void max7456DrawScreenSlow(void)
{
    bool escapeCharFound = false;
	// 获取激活层缓存
    uint8_t *buffer = getActiveLayerBuffer();

    __spiBusTransactionBegin(busdev);
	// 启用自动递增模式并更新激活缓冲区中的每个字符
	// “转义”字符0xFF必须被跳过，因为它会导致MAX7456退出自动递增模式
    max7456Send(MAX7456ADD_DMAH, 0);
    max7456Send(MAX7456ADD_DMAL, 0);
    max7456Send(MAX7456ADD_DMM, displayMemoryModeReg | 1);

    for (int xx = 0; xx < maxScreenSize; xx++) {
		// 判断是否为最后一个字符
        if (buffer[xx] == END_STRING) {
            escapeCharFound = true;
			// 在第一次传递中将0xFF字符替换为空格，以避免终止自动递增
            max7456Send(MAX7456ADD_DMDI, ' ');  
        } else {
            max7456Send(MAX7456ADD_DMDI, buffer[xx]);       // 显示存储器数据寄存器，字符地址
        }
        shadowBuffer[xx] = buffer[xx];
    }

    max7456Send(MAX7456ADD_DMDI, END_STRING);
    max7456Send(MAX7456ADD_DMM, displayMemoryModeReg);

	// 如果发现任何“转义”字符0xFF，则进行第二次传递，用直接寻址去更新这个字符
    if (escapeCharFound) {
        for (int xx = 0; xx < maxScreenSize; xx++) {
            if (buffer[xx] == END_STRING) {
                max7456Send(MAX7456ADD_DMAH, xx >> 8);		// 显示存储器地址高 - DMAH[0]为显示存储器地址的最高位
                max7456Send(MAX7456ADD_DMAL, xx & 0xFF);    // 显示存储器地址低 - 决定了字符的显示区域，由DMAH[0]控制
                max7456Send(MAX7456ADD_DMDI, END_STRING);   // 显示存储器数据输入 - 字符地址与字符属性字节一同存储
            }
        }
    }
    __spiBusTransactionEnd(busdev);
}

/**********************************************************************
函数名称：max7456RefreshAll
函数功能：MAX7456刷新全部
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void max7456RefreshAll(void)
{
	// MAX7456重新初始化
    max7456ReInitIfRequired(true);
	// MAX7456缓慢绘制屏幕
    max7456DrawScreenSlow();
}

//-------------------------------------------------------------------------------------MAX7456字库字符写入相关API

/**********************************************************************
函数名称：write_max7456_font
函数功能：写一个OSD字符到MAX7456内部EEPROM字库
函数形参：字库地址，字符数据
函数返回值：状态
函数描述：
	字库地址（0-255),可以存储256字符。
**********************************************************************/
bool write_max7456_font(uint16_t char_address, unsigned char *font_data)
{
    if (!max7456DeviceDetected) {
        return false;
    }
    __spiBusTransactionBegin(busdev);
    // 禁用OSD显示
    fontIsLoading = true;
    max7456Send(MAX7456ADD_VM0, 0);
    max7456Send(MAX7456ADD_CMAH, char_address);     // 字库存储地址高位寄存器(字符存储器地址位) - 选择字库地址
    
	// 54 个像素块全部导入影子存储器
    for (int x = 0; x < 54; x++) {				
        max7456Send(MAX7456ADD_CMAL, x);            // (字符存储器地址)6bit(0-5) 代表着一个字符中的四个像素点，64 字节（实际只用 54 字节）
        max7456Send(MAX7456ADD_CMDI, font_data[x]); // 字库存储器数据寄存器 - 设置每一像素点
#ifdef LED0_TOGGLE
        LED0_TOGGLE;
#endif
    }

    // 从影子ram传输54字节到NVM
    max7456Send(MAX7456ADD_CMM, WRITE_NVR);			// 字符存储器模式寄存器 - 从影子存储器写入到字符存储器

    // 等待直到状态寄存器中的第5位返回0 (12ms)
    while ((max7456Send(MAX7456ADD_STAT, 0x00) & STAT_NVR_BUSY) != 0x00);
    __spiBusTransactionEnd(busdev);
    return true;
}

//-------------------------------------------------------------------------------------MAX7456获取制式行数相关API

/**********************************************************************
函数名称：max7456GetRowsCount
函数功能：MAX7456获取行数
函数形参：None
函数返回值：行数
函数描述：None
**********************************************************************/
uint8_t max7456GetRowsCount(void)
{
    return (videoSignalReg & VIDEO_MODE_PAL) ? VIDEO_LINES_PAL : VIDEO_LINES_NTSC;
}

//-------------------------------------------------------------------------------------MAX7456设备检测相关API

/**********************************************************************
函数名称：max7456IsDeviceDetected
函数功能：获取MAX7456设备检测结果
函数形参：None
函数返回值：状态
函数描述：None
**********************************************************************/
bool max7456IsDeviceDetected(void)
{
    return max7456DeviceDetected;
}

//-------------------------------------------------------------------------------------MAX7456初始化相关API

/**********************************************************************
函数名称：max7456PreInit
函数功能：MAX7456预初始化
函数形参：max7456Config
函数返回值：None
函数描述：None
**********************************************************************/
void max7456PreInit(const max7456Config_t *max7456Config)
{
	// 预初始化SPI注册表
    spiPreinitRegister(max7456Config->csTag, max7456Config->preInitOPU ? IOCFG_OUT_PP : IOCFG_IPU, 1);
}

/**********************************************************************
函数名称：max7456Init
函数功能：MAX7456初始化
函数形参：max7456Config,pVcdProfile,cpuOverclock
函数返回值：状态
函数描述：
	只初始化CS，并尝试初始化MAX7456,检测设备类型。
**********************************************************************/
bool max7456Init(const max7456Config_t *max7456Config, const vcdProfile_t *pVcdProfile, bool cpuOverclock)
{
    max7456DeviceDetected = false;

    // -------------------------------------------------清除所有层
    for (unsigned i = 0; i < MAX7456_SUPPORTED_LAYER_COUNT; i++) {
        max7456ClearLayer(i);
    }

	// -------------------------------------------------检测配置合法性
    if (!max7456Config->csTag || !max7456Config->spiDevice) {
        return false;
    }

	// -------------------------------------------------获取片选线IO标签
    busdev->busdev_u.spi.csnPin = IOGetByTag(max7456Config->csTag);
	// 获取IO是否为空闲或者是预初始化
    if (!IOIsFreeOrPreinit(busdev->busdev_u.spi.csnPin)) {
        return false;
    }

	// -------------------------------------------------初始化片选线IO 
    IOInit(busdev->busdev_u.spi.csnPin, OWNER_OSD_CS, 0);
    IOConfigGPIO(busdev->busdev_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(busdev->busdev_u.spi.csnPin);

	// -------------------------------------------------实例SPI总线
    spiBusSetInstance(busdev, spiInstanceByDevice(SPI_CFG_TO_DEV(max7456Config->spiDevice)));

	// -------------------------------------------------检测MAX7456设备类型,为了安全起见，速度要放慢一半
	// 通过读取OSDM (OSD插入MUX)寄存器检测MAX7456和兼容设备
	// 这个寄存器在这个驱动程序中没有被修改，因此确保保持它的默认值(0x1B)
    spiSetDivisor(busdev->busdev_u.spi.instance, MAX7456_SPI_CLK * 2);
    __spiBusTransactionBegin(busdev);
    uint8_t osdm = max7456Send(MAX7456ADD_OSDM|MAX7456ADD_READ, 0xff);
    __spiBusTransactionEnd(busdev);
    if (osdm != 0x1B) {
        IOConfigGPIO(busdev->busdev_u.spi.csnPin, IOCFG_IPU);
        return false;
    }

    // -------------------------------------------------声明片选线的所有者
    max7456DeviceDetected = true;
    IOInit(busdev->busdev_u.spi.csnPin, OWNER_OSD_CS, 0);

	// -------------------------------------------------通过在CMAL[6]上读写CA[8]位来检测设备类型，用于访问字符字形存储的后半部分，仅支持AT变体
    __spiBusTransactionBegin(busdev);
    max7456Send(MAX7456ADD_CMAL, (1 << 6)); 			// CA[8] bit
    if (max7456Send(MAX7456ADD_CMAL|MAX7456ADD_READ, 0xff) & (1 << 6)) {
        max7456DeviceType = MAX7456_DEVICE_TYPE_AT;
    } else {
        max7456DeviceType = MAX7456_DEVICE_TYPE_MAX;
    }
    __spiBusTransactionEnd(busdev);

#if defined(USE_OVERCLOCK)
    // -------------------------------------------------根据配置和设备类型确定SPI时钟分频
    switch (max7456Config->clockConfig) {
	    case MAX7456_CLOCK_CONFIG_HALF:
	        max7456SpiClock = MAX7456_SPI_CLK * 2;
	        break;
	    case MAX7456_CLOCK_CONFIG_OC:
	        max7456SpiClock = (cpuOverclock && (max7456DeviceType == MAX7456_DEVICE_TYPE_MAX)) ? MAX7456_SPI_CLK * 2 : MAX7456_SPI_CLK;
	        break;
	    case MAX7456_CLOCK_CONFIG_FULL:
	        max7456SpiClock = MAX7456_SPI_CLK;
	        break;
    }
#else
    UNUSED(max7456Config);
    UNUSED(cpuOverclock);
#endif
	// -------------------------------------------------设置spi总线分频
    spiBusSetDivisor(busdev, max7456SpiClock);

    // -------------------------------------------------强制软复位Max7456
    __spiBusTransactionBegin(busdev);
    max7456Send(MAX7456ADD_VM0, MAX7456_RESET);
    __spiBusTransactionEnd(busdev);

    // -------------------------------------------------设置写入寄存器的值
    videoSignalCfg = pVcdProfile->video_system;
    hosRegValue = 32 - pVcdProfile->h_offset;
    vosRegValue = 16 - pVcdProfile->v_offset;

    // -------------------------------------------------真正的init将在稍后驱动程序检测到空闲时进行
    return true;
}

/**********************************************************************
函数名称：max7456ReInit
函数功能：MAX7456重新初始化
函数形参：None
函数返回值：None
函数描述：
	初始化视频制式和屏幕大小。
**********************************************************************/
void max7456ReInit(void)
{
    uint8_t srdata = 0;
    static bool firstInit = true;

	// 拉低片选开始通信
    __spiBusTransactionBegin(busdev);
	// -------------------------------------------------视频制式配置
    switch (videoSignalCfg) {
	    case VIDEO_SYSTEM_PAL:							// PAL
	        videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
	        break;
	    case VIDEO_SYSTEM_NTSC:							// NTSC
	        videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
	        break;
	    case VIDEO_SYSTEM_AUTO:							// 自动
	    	// 获取视频制式
	        srdata = max7456Send(MAX7456ADD_STAT, 0x00);
	        if (VIN_IS_NTSC(srdata)) {
	            videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
	        } else if (VIN_IS_PAL(srdata)) {
	            videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
	        } else {
	            // 没有有效的输入信号，回退到默认(XXX NTSC暂时)
	            videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
	        }
	        break;
    }

	// -------------------------------------------------初始化屏幕大小
    if (videoSignalReg & VIDEO_MODE_PAL) { 				// PAL
        maxScreenSize = VIDEO_BUFFER_CHARS_PAL;
    } else {                               				// NTSC
        maxScreenSize = VIDEO_BUFFER_CHARS_NTSC;
    }

    // -------------------------------------------------设置所有行为相同的字符黑/白级别
    previousBlackWhiteRegister = INVALID_PREVIOUS_REGISTER_STATE;
	// 设置黑白像素的亮度
    max7456Brightness(0, 2);
	
    // 重新启用MAX7456(最后一次函数调用禁用它)
    __spiBusTransactionBegin(busdev);
    // -------------------------------------------------确保Max7456已启用
    max7456Send(MAX7456ADD_VM0, videoSignalReg);
    max7456Send(MAX7456ADD_HOS, hosRegValue);
    max7456Send(MAX7456ADD_VOS, vosRegValue);
    max7456Send(MAX7456ADD_DMM, displayMemoryModeReg | CLEAR_DISPLAY);
	// 拉高片选结束通信
    __spiBusTransactionEnd(busdev);

    // 清除阴影，以强制重绘所有屏幕在非dma模式
    max7456ClearShadowBuffer();
    if (firstInit) {
        max7456DrawScreenSlow();
        firstInit = false;
    }
}

/**********************************************************************
函数名称：max7456BuffersSynced
函数功能：MAX7456重新初始化
函数形参：是否需要重新初始化
函数返回值：状态
函数描述：None
**********************************************************************/
void max7456ReInitIfRequired(bool forceStallCheck)
{
    static timeMs_t lastSigCheckMs = 0;
    static timeMs_t videoDetectTimeMs = 0;
    static uint16_t reInitCount = 0;
	// 偏置，使其与信号检查不一致
    static timeMs_t lastStallCheckMs = MAX7456_STALL_CHECK_INTERVAL_MS / 2;   

	// 获取当前时间节拍
    const timeMs_t nowMs = millis();

    bool stalled = false;
    if (forceStallCheck || (lastStallCheckMs + MAX7456_STALL_CHECK_INTERVAL_MS < nowMs)) {
        lastStallCheckMs = nowMs;
        __spiBusTransactionBegin(busdev);
        stalled = (max7456Send(MAX7456ADD_VM0|MAX7456ADD_READ, 0x00) != videoSignalReg);
        __spiBusTransactionEnd(busdev);
    }
    if (stalled) {
		// MAX7456重新初始化 - 不一致，重新初始化视频制式和屏幕大小
        max7456ReInit();
    } else if ((videoSignalCfg == VIDEO_SYSTEM_AUTO)
              && ((nowMs - lastSigCheckMs) > MAX7456_SIGNAL_CHECK_INTERVAL_MS)) {

        // 根据当前输入格式调整输出格式
        __spiBusTransactionBegin(busdev);
        const uint8_t videoSense = max7456Send(MAX7456ADD_STAT, 0x00);
        __spiBusTransactionEnd(busdev);

        if (videoSense & STAT_LOS) {
            videoDetectTimeMs = 0;
        } else {
            if ((VIN_IS_PAL(videoSense) && VIDEO_MODE_IS_NTSC(videoSignalReg))
              || (VIN_IS_NTSC_alt(videoSense) && VIDEO_MODE_IS_PAL(videoSignalReg))) {
                if (videoDetectTimeMs) {
                    if (millis() - videoDetectTimeMs > VIDEO_SIGNAL_DEBOUNCE_MS) {
						// MAX7456重新初始化 - 重新初始化视频制式和屏幕大小
                        max7456ReInit();
                    }
                } else {
                    // 等待信号稳定
                    videoDetectTimeMs = millis();
                }
            }
        }

		// 更新检查时间
        lastSigCheckMs = nowMs;
    }
}
#endif // USE_MAX7456

