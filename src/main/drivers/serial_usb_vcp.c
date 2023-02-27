#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_VCP
#include "build/build_config.h"
#include "common/utils.h"
#include "drivers/io.h"
#include "drivers/time.h"
#include "serial.h"
#include "serial_usb_vcp.h"
#include "usb_core.h"
#include "usbd_cdc_vcp.h"

// ---------------------------------------------------------USB超时时间
#define USB_TIMEOUT  50
// ---------------------------------------------------------VCP端口
static vcpPort_t vcpPort;

/**********************************************************************
函数名称：usbVcpFlush
函数功能：USB_VCP刷新
函数形参：VCP端口
函数返回值：缓冲区是否全部发送完毕
函数描述：None
**********************************************************************/
static bool usbVcpFlush(vcpPort_t *port)
{
	// 获取待发送字节
    uint32_t count = port->txAt;
	// 刷新重置
    port->txAt = 0;
    if (count == 0) {
        return true;
    }
	// 检查USB状态合法性
    if (!usbIsConnected() || !usbIsConfigured()) {
        return false;
    }
	// 获取起始时间
    uint32_t start = millis();
	// 获取发送缓冲区
    uint8_t *p = port->txBuf;
	// 等待数据发送完毕
    while (count > 0) {
		// 发送数据
        uint32_t txed = CDC_Send_DATA(p, count);
		// 递减
        count -= txed;
		// 偏移下一字节
        p += txed;
		// 超时检测
        if (millis() - start > USB_TIMEOUT) {
            break;
        }
    }
    return count == 0;
}

/**********************************************************************
函数名称：usbVcpWrite
函数功能：USB_VCP数据发送
函数形参：串口标识，字符
函数返回值：None
函数描述：None
**********************************************************************/
static void usbVcpWrite(serialPort_t *instance, uint8_t c)
{
	// 获取VCP端口
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
	// 写入发送缓冲区
    port->txBuf[port->txAt++] = c;
	// 非大容量写模式 || 发送缓冲区满则进行刷新
    if (!port->buffering || port->txAt >= ARRAYLEN(port->txBuf)) {
        usbVcpFlush(port);
    }
}

/**********************************************************************
函数名称：usbVcpRead
函数功能：USB_VCP数据接收
函数形参：串口标识
函数返回值：缓冲区是否全部发送完毕
函数描述：None
**********************************************************************/
static uint8_t usbVcpRead(serialPort_t *instance)
{
    UNUSED(instance);
    uint8_t buf[1];
    while (true) {
        if (CDC_Receive_DATA(buf, 1))
            return buf[0];
    }
}

/**********************************************************************
函数名称：usbVcpAvailable
函数功能：USB_VCP数据接收
函数形参：串口标识
函数返回值：接收缓冲区字节数
函数描述：None
**********************************************************************/
static uint32_t usbVcpAvailable(const serialPort_t *instance)
{
    UNUSED(instance);
    return CDC_Receive_BytesAvailable();
}

/**********************************************************************
函数名称：usbTxBytesFree
函数功能：USB_VCP数据接收
函数形参：串口标识
函数返回值：发送缓冲区空闲字节数
函数描述：None
**********************************************************************/
static uint32_t usbTxBytesFree(const serialPort_t *instance)
{
    UNUSED(instance);
    return CDC_Send_FreeBytes();
}

/**********************************************************************
函数名称：usbVcpWriteBuf
函数功能：USB_VCP写（发送）缓冲区
函数形参：串口标识，数据，数据大小
函数返回值：None
函数描述：None
**********************************************************************/
static void usbVcpWriteBuf(serialPort_t *instance, const void *data, int count)
{
    UNUSED(instance);
    // 检查USB状态合法性
    if (!(usbIsConnected() && usbIsConfigured())) {
        return;
    }
    // 获取起始时间
    uint32_t start = millis();
    // 获取数据指针
    const uint8_t *p = data;
    while (count > 0) {
        // 发送数据
        uint32_t txed = CDC_Send_DATA(p, count);
        count -= txed;
        p += txed;
        // 检查是否超时
        if (millis() - start > USB_TIMEOUT) {
            break;
        }
    }
}

/**********************************************************************
函数名称：usbVcpBeginWrite
函数功能：USB_VCP开启大容量写模式
函数形参：串口标识
函数返回值：None
函数描述：None
**********************************************************************/
static void usbVcpBeginWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    // 开启大容量写模式
    port->buffering = true;
}

/**********************************************************************
函数名称：usbVcpEndWrite
函数功能：USB_VCP关闭大容量写模式
函数形参：串口标识
函数返回值：None
函数描述：None
**********************************************************************/
static void usbVcpEndWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    // 关闭大容量写模式
    port->buffering = false;
    // 刷新缓冲区 - 发送因大容量写模式正在阻塞未发送USB缓冲区数据
    usbVcpFlush(port);
}

/**********************************************************************
函数名称：isUsbVcpTransmitBufferEmpty
函数功能：获取传输缓冲区是否为空(传输完成)
函数形参：串口标识
函数返回值：None
函数描述：None
**********************************************************************/
static bool isUsbVcpTransmitBufferEmpty(const serialPort_t *instance)
{
    UNUSED(instance);
    return true;
}

// 虚函数表
static const struct serialPortVTable usbVTable[] = {
    {
        .serialWrite = usbVcpWrite,
        .serialTotalRxWaiting = usbVcpAvailable,
        .serialTotalTxFree = usbTxBytesFree,
        .serialRead = usbVcpRead,
        .isSerialTransmitBufferEmpty = isUsbVcpTransmitBufferEmpty,
        .writeBuf = usbVcpWriteBuf,
        .beginWrite = usbVcpBeginWrite,
        .endWrite = usbVcpEndWrite,  
    }
};

/**********************************************************************
函数名称：usbVcpOpen
函数功能：打开USB_VCP
函数形参：None
函数返回值：串口配置
函数描述：None
**********************************************************************/
serialPort_t *usbVcpOpen(void)
{
    vcpPort_t *s;
	// IO初始化
    IOInit(IOGetByTag(IO_TAG(PA11)), OWNER_USB, 0);
    IOInit(IOGetByTag(IO_TAG(PA12)), OWNER_USB, 0);
    IO_t usbPin = IOGetByTag(IO_TAG(PA12));
    IOConfigGPIO(usbPin, IOCFG_OUT_OD);
    // 拉低DP创建USB断开脉冲
    IOLo(usbPin);
    delay(200);
    IOHi(usbPin);
	// USB初始化
    USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
	// 注册相关信息
    s = &vcpPort;
	// 注册虚函数表
    s->port.vTable = usbVTable;
    return (serialPort_t *)s;
}
#endif

