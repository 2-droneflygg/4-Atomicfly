/*********************************************************************************
 提供串口引脚配置API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART

#include "build/build_config.h"

#include "drivers/rcc.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

FAST_RAM_ZERO_INIT uartDevice_t uartDevice[UARTDEV_COUNT];      // target.h中配置的串口
FAST_RAM_ZERO_INIT uartDevice_t *uartDevmap[UARTDEV_COUNT_MAX]; // 串口设备信息记录

/**********************************************************************
函数名称：uartPinConfigure
函数功能：串口引脚配置 ,根据pSerialPinConfig的配置，如果配置正确，则注册硬件
函数形参：serialPinConfig_t结构体
函数返回值：None
函数描述：None
**********************************************************************/
void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig)
{
    uartDevice_t *uartdev = uartDevice;

	// 遍历所有串口
    for (size_t hindex = 0; hindex < UARTDEV_COUNT; hindex++) {
		// 获取串口设备硬件配置
        const uartHardware_t *hardware = &uartHardware[hindex];		
		// 获取串口设备 
        const UARTDevice_e device = hardware->device;				
		// 遍历所有引脚
        for (int pindex = 0 ; pindex < UARTHARDWARE_MAX_PINS ; pindex++) {
			// 	RX引脚配置
            if (hardware->rxPins[pindex].pin == pSerialPinConfig->ioTagRx[device]) {
                uartdev->rx = hardware->rxPins[pindex];
            }
			// 	TX引脚配置
            if (hardware->txPins[pindex].pin == pSerialPinConfig->ioTagTx[device]) {
                uartdev->tx = hardware->txPins[pindex];
            }
        }
		// 硬件注册
        if (uartdev->rx.pin || uartdev->tx.pin) {
            uartdev->hardware = hardware;
            uartDevmap[device] = uartdev++;
        }
    }
}
#endif

