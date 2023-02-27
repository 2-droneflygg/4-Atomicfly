#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_MAX7456

#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/max7456.h"
#include "drivers/osd.h"

#include "config/config.h"

#include "io/displayport_max7456.h"

#include "osd/osd.h"

#include "pg/displayport_profiles.h"
#include "pg/max7456.h"
#include "pg/vcd.h"

displayPort_t max7456DisplayPort;

/**********************************************************************
函数名称：grab
函数功能：抓取
函数形参：displayPort
函数返回值：0
函数描述：None
**********************************************************************/
static int grab(displayPort_t *displayPort)
{
    UNUSED(displayPort);
#ifdef USE_OSD
    resumeRefreshAt = 0;
#endif
    return 0;
}

/**********************************************************************
函数名称：release
函数功能：释放
函数形参：displayPort
函数返回值：0
函数描述：None
**********************************************************************/
static int release(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

/**********************************************************************
函数名称：clearScreen
函数功能：清除屏幕
函数形参：displayPort
函数返回值：0
函数描述：None
**********************************************************************/
static int clearScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    max7456Invert(displayPortProfileMax7456()->invert);
    max7456Brightness(displayPortProfileMax7456()->blackBrightness, displayPortProfileMax7456()->whiteBrightness);
    max7456ClearScreen();
    return 0;
}

/**********************************************************************
函数名称：drawScreen
函数功能：绘制屏幕
函数形参：displayPort
函数返回值：0
函数描述：None
**********************************************************************/
static int drawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    max7456DrawScreen();
    return 0;
}

/**********************************************************************
函数名称：drawScreen
函数功能：获取屏幕大小
函数形参：displayPort
函数返回值：0
函数描述：None
**********************************************************************/
static int screenSize(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return maxScreenSize;
}

/**********************************************************************
函数名称：writeString
函数功能：写字符串
函数形参：displayPort，x,y,attr,s
函数返回值：0
函数描述：None
**********************************************************************/
static int writeString(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, const char *s)
{
    UNUSED(displayPort);
    UNUSED(attr);
    max7456Write(x, y, s);
    return 0;
}

/**********************************************************************
函数名称：writeChar
函数功能：写字符
函数形参：displayPort，x,y,attr,c
函数返回值：0
函数描述：None
**********************************************************************/
static int writeChar(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, uint8_t c)
{
    UNUSED(displayPort);
    UNUSED(attr);
    max7456WriteChar(x, y, c);
    return 0;
}

/**********************************************************************
函数名称：isTransferInProgress
函数功能：获取传输是否正在进行中
函数形参：displayPort
函数返回值：状态
函数描述：None
**********************************************************************/
static bool isTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return max7456DmaInProgress();
}

/**********************************************************************
函数名称：isSynced
函数功能：是否同步
函数形参：displayPort
函数返回值：状态
函数描述：None
**********************************************************************/
static bool isSynced(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return max7456BuffersSynced();
}

/**********************************************************************
函数名称：resync
函数功能：重新同步
函数形参：displayPort
函数返回值：None
函数描述：
	进行缓慢绘制屏幕并计算行列。
**********************************************************************/
static void resync(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    max7456RefreshAll();
    displayPort->rows = max7456GetRowsCount() + displayPortProfileMax7456()->rowAdjust;
    displayPort->cols = 30 + displayPortProfileMax7456()->colAdjust;
}

/**********************************************************************
函数名称：txBytesFree
函数功能：释放发送字节
函数形参：displayPort
函数返回值：UINT32_MAX
函数描述：None
**********************************************************************/
static uint32_t txBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

/**********************************************************************
函数名称：layerSupported
函数功能：支持层
函数形参：displayPort，layer
函数返回值：状态
函数描述：None
**********************************************************************/
static bool layerSupported(displayPort_t *displayPort, displayPortLayer_e layer)
{
    UNUSED(displayPort);
    return max7456LayerSupported(layer);
}

/**********************************************************************
函数名称：layerSelect
函数功能：选择层
函数形参：displayPort，layer
函数返回值：状态
函数描述：None
**********************************************************************/
static bool layerSelect(displayPort_t *displayPort, displayPortLayer_e layer)
{
    UNUSED(displayPort);
    return max7456LayerSelect(layer);
}

/**********************************************************************
函数名称：layerCopy
函数功能：复制层
函数形参：displayPort，destLayer，sourceLayer
函数返回值：状态
函数描述：None
**********************************************************************/
static bool layerCopy(displayPort_t *displayPort, displayPortLayer_e destLayer, displayPortLayer_e sourceLayer)
{
    UNUSED(displayPort);
    return max7456LayerCopy(destLayer, sourceLayer);
}

/**********************************************************************
函数名称：isReady
函数功能：就绪状态
函数形参：instance
函数返回值：状态
函数描述：None
**********************************************************************/
static bool isReady(displayPort_t *instance)
{
    UNUSED(instance);
	// MAX7456设备检测结果
    return max7456IsDeviceDetected();
}

/**********************************************************************
函数名称：writeFontcharacter
函数功能：写OSD字库字符
函数形参：instance，地址，字符数据
函数返回值：状态
函数描述：None
**********************************************************************/
static bool writeFontcharacter(displayPort_t *instance, uint16_t addr, const struct osdCharacter_s *chr)
{
    UNUSED(instance);
	// MAX7456设备检测结果
    return write_max7456_font(addr,(unsigned char *)chr);
}

// 注册函数表 - MAX7456操作函数 -> 显示端口虚函数接口
static const displayPortVTable_t max7456VTable = {
    .grab = grab,
    .release = release,
    .clearScreen = clearScreen,
    .drawScreen = drawScreen,
    .screenSize = screenSize,
    .writeString = writeString,
    .writeChar = writeChar,
    .isTransferInProgress = isTransferInProgress,
    .resync = resync,
    .isSynced = isSynced,
    .txBytesFree = txBytesFree,
    .layerSupported = layerSupported,
    .layerSelect = layerSelect,
    .layerCopy = layerCopy,
    .isReady = isReady,
    .writeFontCharacter = writeFontcharacter,
};

/**********************************************************************
函数名称：max7456DisplayPortInit
函数功能：max7456显示端口初始化
函数形参：vcdProfile
函数返回值：max7456DisplayPort
函数描述：None
**********************************************************************/
displayPort_t *max7456DisplayPortInit(const vcdProfile_t *vcdProfile)
{
	// MAX7456初始化
    if ( !max7456Init(max7456Config(), vcdProfile, systemConfig()->cpu_overclock) ) {
        return NULL;
    }
	// 显示初始化 
    displayInit(&max7456DisplayPort, &max7456VTable);
	// 重新同步 - 进行缓慢绘制屏幕并计算行列
    resync(&max7456DisplayPort);
    return &max7456DisplayPort;
}
#endif // USE_MAX7456

