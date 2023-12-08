#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_UART
#include "common/utils.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/serial_uart.h"

#include "drivers/io_types.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

typedef struct uartDmaopt_s {
    UARTDevice_e device;
    int8_t txDmaopt;
    int8_t rxDmaopt;
} uartDmaopt_t;

static uartDmaopt_t uartDmaopt[] = {
#ifdef USE_UART1
    { UARTDEV_1, UART1_TX_DMA_OPT, UART1_RX_DMA_OPT },
#endif
#ifdef USE_UART2
    { UARTDEV_2, UART2_TX_DMA_OPT, UART2_RX_DMA_OPT },
#endif
#ifdef USE_UART3
    { UARTDEV_3, UART3_TX_DMA_OPT, UART3_RX_DMA_OPT },
#endif
#ifdef USE_UART6
    { UARTDEV_6, UART6_TX_DMA_OPT, UART6_RX_DMA_OPT },
#endif
};

PG_REGISTER_ARRAY_WITH_RESET_FN(serialUartConfig_t, UARTDEV_CONFIG_MAX, serialUartConfig, PG_SERIAL_UART_CONFIG, 0);
void pgResetFn_serialUartConfig(serialUartConfig_t *config)
{
    for (unsigned i = 0; i < UARTDEV_CONFIG_MAX; i++) {
        config[i].txDmaopt = -1;
        config[i].rxDmaopt = -1;
    }

    for (unsigned i = 0; i < ARRAYLEN(uartDmaopt); i++) {
        UARTDevice_e device = uartDmaopt[i].device;
        config[device].txDmaopt = uartDmaopt[i].txDmaopt;
        config[device].rxDmaopt = uartDmaopt[i].rxDmaopt;
    }
}
#endif

