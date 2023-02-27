#pragma once

#include "drivers/bus_spi.h"
#include "drivers/io_types.h"
#include "drivers/dma_reqmap.h"

#include "pg/pg.h"

/* --------------------------SPI配置结构体-------------------------- */	
typedef struct spiPinConfig_s {
    ioTag_t ioTagSck;
    ioTag_t ioTagMiso;
    ioTag_t ioTagMosi;
    int8_t txDmaopt;
    int8_t rxDmaopt;
} spiPinConfig_t;
// 声明SPI配置结构体
PG_DECLARE_ARRAY(spiPinConfig_t, SPIDEV_COUNT, spiPinConfig);

