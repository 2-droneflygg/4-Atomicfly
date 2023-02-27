/*********************************************************************************
 提供一系列显示接口API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/osd.h"

#include "display.h"

/**********************************************************************
函数名称：displayClearScreen
函数功能：清除屏幕
函数形参：instance
函数返回值：None
函数描述：None
**********************************************************************/
void displayClearScreen(displayPort_t *instance)
{
    instance->vTable->clearScreen(instance);
    instance->cleared = true;
    instance->cursorRow = -1;
}

/**********************************************************************
函数名称：displayDrawScreen
函数功能：绘制屏幕
函数形参：instance
函数返回值：None
函数描述：None
**********************************************************************/
void displayDrawScreen(displayPort_t *instance)
{
    instance->vTable->drawScreen(instance);
}

/**********************************************************************
函数名称：displayGrab
函数功能：抓取显示设备
函数形参：instance
函数返回值：None
函数描述：None
**********************************************************************/
void displayGrab(displayPort_t *instance)
{
    instance->vTable->grab(instance);
    instance->vTable->clearScreen(instance);
    ++instance->grabCount;
}

/**********************************************************************
函数名称：displayRelease
函数功能：显示释放
函数形参：instance
函数返回值：None
函数描述：None
**********************************************************************/
void displayRelease(displayPort_t *instance)
{
    instance->vTable->release(instance);
    --instance->grabCount;
}

/**********************************************************************
函数名称：displayReleaseAll
函数功能：显示是否获取
函数形参：instance
函数返回值：状态
函数描述：None
**********************************************************************/
bool displayIsGrabbed(const displayPort_t *instance)
{
    return (instance && instance->grabCount > 0);
}

/**********************************************************************
函数名称：displayWrite
函数功能：显示写入字符串
函数形参：instance,x,y，attr，s
函数返回值：0
函数描述：None
**********************************************************************/
int displayWrite(displayPort_t *instance, uint8_t x, uint8_t y, uint8_t attr, const char *s)
{
    instance->posX = x + strlen(s);
    instance->posY = y;
    return instance->vTable->writeString(instance, x, y, attr, s);
}

/**********************************************************************
函数名称：displayWriteChar
函数功能：显示写字符
函数形参：显示端口,x,y，attr，字符
函数返回值：0
函数描述：None
**********************************************************************/
int displayWriteChar(displayPort_t *instance, uint8_t x, uint8_t y, uint8_t attr, uint8_t c)
{
    instance->posX = x + 1;
    instance->posY = y;
    return instance->vTable->writeChar(instance, x, y, attr, c);
}

/**********************************************************************
函数名称：displayResync
函数功能：显示重新同步
函数形参：instance
函数返回值：None
函数描述：None
**********************************************************************/
void displayResync(displayPort_t *instance)
{
    instance->vTable->resync(instance);
}

/**********************************************************************
函数名称：displayTxBytesFree
函数功能：显示发送字节释放
函数形参：instance
函数返回值：UINT32_MAX
函数描述：None
**********************************************************************/
uint16_t displayTxBytesFree(const displayPort_t *instance)
{
    return instance->vTable->txBytesFree(instance);
}

/**********************************************************************
函数名称：displayLayerSupported
函数功能：获取显示层是否支持
函数形参：instance，layer
函数返回值：状态
函数描述：None
**********************************************************************/
bool displayLayerSupported(displayPort_t *instance, displayPortLayer_e layer)
{
    if (layer == DISPLAYPORT_LAYER_FOREGROUND) {
        // 每个设备必须支持前景(默认)层
        return true;
    } else if (layer < DISPLAYPORT_LAYER_COUNT && instance->vTable->layerSupported) {
        return instance->vTable->layerSupported(instance, layer);
    }
    return false;
}

/**********************************************************************
函数名称：displayLayerSelect
函数功能：显示层选择
函数形参：instance，layer
函数返回值：状态
函数描述：None
**********************************************************************/
bool displayLayerSelect(displayPort_t *instance, displayPortLayer_e layer)
{
    if (instance->vTable->layerSelect) {
        return instance->vTable->layerSelect(instance, layer);
    }
    return false;
}

/**********************************************************************
函数名称：displayLayerCopy
函数功能：显示层复制
函数形参：instance，destLayer，sourceLayer
函数返回值：状态
函数描述：None
**********************************************************************/
bool displayLayerCopy(displayPort_t *instance, displayPortLayer_e destLayer, displayPortLayer_e sourceLayer)
{
    if (instance->vTable->layerCopy && sourceLayer != destLayer) {
        return instance->vTable->layerCopy(instance, destLayer, sourceLayer);
    }
    return false;
}

/**********************************************************************
函数名称：displayIsReady
函数功能：显示是否就绪
函数形参：instance
函数返回值：状态
函数描述：None
**********************************************************************/
bool displayIsReady(displayPort_t *instance)
{
	// 初始化阶段设备检测结果
    if (instance->vTable->isReady) {
        return instance->vTable->isReady(instance);
    }
	// 不提供isReady方法的驱动程序
	// 假设立即准备好(either by actually开始准备非常快或通过阻塞)
    return true;
}

/**********************************************************************
函数名称：displayInit
函数功能：显示初始化
函数形参：instance，vTable
函数返回值：None
函数描述：None
**********************************************************************/
void displayInit(displayPort_t *instance, const displayPortVTable_t *vTable)
{
    instance->vTable = vTable;
    instance->vTable->clearScreen(instance);
    instance->useFullscreen = false;
    instance->cleared = true;
    instance->grabCount = 0;
    instance->cursorRow = -1;
}

/**********************************************************************
函数名称：displayInit
函数功能：显示设备写OSD字符
函数形参：instance，地址，字符
函数返回值：None
函数描述：None
**********************************************************************/
bool displayWriteFontCharacter(displayPort_t *instance, uint16_t addr, const osdCharacter_t *chr)
{
    if (instance->vTable->writeFontCharacter) {
        return instance->vTable->writeFontCharacter(instance, addr, chr);
    }
    return false;
}

