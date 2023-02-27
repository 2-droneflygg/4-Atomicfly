#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "pg/pg.h"
#include "drivers/io_types.h"
#include "drivers/dma_reqmap.h"

#define UARTDEV_CONFIG_MAX 8 // UARTDEV_COUNT_MAX的替代选项，它需要serial_uart_imp.h

/* -----------------------------串口配置结构体--------------------------- */	
typedef struct serialUartConfig_s {
    int8_t txDmaopt;
    int8_t rxDmaopt;
} serialUartConfig_t;
// 声明串口配置结构体
PG_DECLARE_ARRAY(serialUartConfig_t, UARTDEV_CONFIG_MAX, serialUartConfig);

