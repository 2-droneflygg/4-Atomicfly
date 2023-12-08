/*********************************************************************************
 提供串口引脚配置信息。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_UART

#include "build/build_config.h"

#include "io/serial.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "pg/pg_ids.h"

// ---------------------------------------------------------如果未定义串口引脚则置为None
#ifdef USE_UART1
# if !defined(UART1_RX_PIN)
#  define UART1_RX_PIN NONE
# endif
# if !defined(UART1_TX_PIN)
#  define UART1_TX_PIN NONE
# endif
# if !defined(INVERTER_PIN_UART1)
#  define INVERTER_PIN_UART1 NONE
# endif
#endif
#ifdef USE_UART2
# if !defined(UART2_RX_PIN)
#  define UART2_RX_PIN NONE
# endif
# if !defined(UART2_TX_PIN)
#  define UART2_TX_PIN NONE
# endif
# if !defined(INVERTER_PIN_UART2)
#  define INVERTER_PIN_UART2 NONE
# endif
#endif
#ifdef USE_UART3
# if !defined(UART3_RX_PIN)
#  define UART3_RX_PIN NONE
# endif
# if !defined(UART3_TX_PIN)
#  define UART3_TX_PIN NONE
# endif
# if !defined(INVERTER_PIN_UART3)
#  define INVERTER_PIN_UART3 NONE
# endif
#endif
#ifdef USE_UART6
# if !defined(UART6_RX_PIN)
#  define UART6_RX_PIN NONE
# endif
# if !defined(UART6_TX_PIN)
#  define UART6_TX_PIN NONE
# endif
# if !defined(INVERTER_PIN_UART6)
#  define INVERTER_PIN_UART6 NONE
# endif
#endif
#ifdef USE_SOFTSERIAL1
# if !defined(SOFTSERIAL1_RX_PIN)
#  define SOFTSERIAL1_RX_PIN NONE
# endif
# if !defined(SOFTSERIAL1_TX_PIN)
#  define SOFTSERIAL1_TX_PIN NONE
# endif
#endif
#ifdef USE_SOFTSERIAL2
# if !defined(SOFTSERIAL2_RX_PIN)
#  define SOFTSERIAL2_RX_PIN NONE
# endif
# if !defined(SOFTSERIAL2_TX_PIN)
#  define SOFTSERIAL2_TX_PIN NONE
# endif
#endif

#if defined(USE_UART) || defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2)
// ---------------------------------------------------------串口默认引脚结构体
typedef struct serialDefaultPin_s {
    serialPortIdentifier_e ident;					// 串口标识符
    ioTag_t rxIO, txIO, inverterIO;					// IO引脚
} serialDefaultPin_t;

// ---------------------------------------------------------串口默认引脚配置
static const serialDefaultPin_t serialDefaultPin[] = {
#ifdef USE_UART1
    { SERIAL_PORT_USART1, IO_TAG(UART1_RX_PIN), IO_TAG(UART1_TX_PIN), IO_TAG(INVERTER_PIN_UART1) },
#endif
#ifdef USE_UART2
    { SERIAL_PORT_USART2, IO_TAG(UART2_RX_PIN), IO_TAG(UART2_TX_PIN), IO_TAG(INVERTER_PIN_UART2) },
#endif
#ifdef USE_UART3
    { SERIAL_PORT_USART3, IO_TAG(UART3_RX_PIN), IO_TAG(UART3_TX_PIN), IO_TAG(INVERTER_PIN_UART3) },
#endif
#ifdef USE_UART6
    { SERIAL_PORT_USART6, IO_TAG(UART6_RX_PIN), IO_TAG(UART6_TX_PIN), IO_TAG(INVERTER_PIN_UART6) },
#endif
#ifdef USE_SOFTSERIAL1
    { SERIAL_PORT_SOFTSERIAL1, IO_TAG(SOFTSERIAL1_RX_PIN), IO_TAG(SOFTSERIAL1_TX_PIN), IO_TAG(NONE) },
#endif
#ifdef USE_SOFTSERIAL2
    { SERIAL_PORT_SOFTSERIAL2, IO_TAG(SOFTSERIAL2_RX_PIN), IO_TAG(SOFTSERIAL2_TX_PIN), IO_TAG(NONE) },
#endif
};

PG_REGISTER_WITH_RESET_FN(serialPinConfig_t, serialPinConfig, PG_SERIAL_PIN_CONFIG, 1);
void pgResetFn_serialPinConfig(serialPinConfig_t *serialPinConfig)
{
	// 遍历所有串口引脚配置信息
    for (size_t index = 0 ; index < ARRAYLEN(serialDefaultPin) ; index++) {
		// 获取引脚配置信息
        const serialDefaultPin_t *defpin = &serialDefaultPin[index];
		// 将引脚配置信息注册到串口引脚配置
        serialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_INDEX(defpin->ident)] = defpin->rxIO;
        serialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_INDEX(defpin->ident)] = defpin->txIO;
        serialPinConfig->ioTagInverter[SERIAL_PORT_IDENTIFIER_TO_INDEX(defpin->ident)] = defpin->inverterIO;
    }
}
#endif
#endif

