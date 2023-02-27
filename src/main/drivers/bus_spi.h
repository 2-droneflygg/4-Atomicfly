#pragma once

#include "drivers/bus.h"
#include "drivers/io_types.h"
#include "drivers/bus.h"
#include "drivers/rcc_types.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

// ---------------------------------------------------------SPI设备数量
#define SPIDEV_COUNT 3

// ---------------------------------------------------------设备配置转换宏
#define SPI_CFG_TO_DEV(x)   ((x) - 1)
#define SPI_DEV_TO_CFG(x)   ((x) + 1)

// ---------------------------------------------------------SPI_IO配置宏
#define SPI_IO_AF_CFG         IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define SPI_IO_AF_SCK_CFG     IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_DOWN)
#define SPI_IO_AF_MISO_CFG    IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP)
#define SPI_IO_CS_CFG         IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)

/* ---------------------------SPI时钟分频枚举---------------------------- */	
typedef enum {
    SPI_CLOCK_INITIALIZATION = 256,
    SPI_CLOCK_SLOW          = 128, 		// 00.65625 MHz
    SPI_CLOCK_STANDARD      = 8,   		// 10.50000 MHz
    SPI_CLOCK_FAST          = 4,   		// 21.00000 MHz
    SPI_CLOCK_ULTRAFAST     = 2    		// 42.00000 MHz
} SPIClockDivider_e;

/* --------------------------SPI设备（ID）枚举--------------------------- */	
typedef enum SPIDevice {
    SPIINVALID = -1,
    SPIDEV_1   = 0,
    SPIDEV_2,
    SPIDEV_3,
} SPIDevice;
	
void spiPreinit(void);
void spiPreinitRegister(ioTag_t iotag, uint8_t iocfg, uint8_t init);
void spiPreinitByIO(IO_t io);
void spiPreinitByTag(ioTag_t tag);
bool spiInit(SPIDevice device, bool leadingEdge);
void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor);
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t data);
bool spiIsBusBusy(SPI_TypeDef *instance);
bool spiTransfer(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len);
SPIDevice spiDeviceByInstance(SPI_TypeDef *instance);
SPI_TypeDef *spiInstanceByDevice(SPIDevice device);
bool spiBusTransfer(const busDevice_t *bus, const uint8_t *txData, uint8_t *rxData, int length);
bool spiBusWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data);
bool spiBusRawReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length);
bool spiBusReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length);
uint8_t spiBusRawReadRegister(const busDevice_t *bus, uint8_t reg);
uint8_t spiBusReadRegister(const busDevice_t *bus, uint8_t reg);
void spiBusSetInstance(busDevice_t *bus, SPI_TypeDef *instance);
void spiBusSetDivisor(busDevice_t *bus, SPIClockDivider_e divider);
struct spiPinConfig_s;
void spiPinConfigure(const struct spiPinConfig_s *pConfig);

