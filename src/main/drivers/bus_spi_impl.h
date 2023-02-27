/*********************************************************************************
 提供一系列SPI总线配置信息。
*********************************************************************************/
#pragma once

/* -----------------------------SPI引脚定义结构体------------------------------ */	
typedef struct spiPinDef_s {
    ioTag_t pin;
} spiPinDef_t;

/* -----------------------------SPI硬件信息结构体------------------------------ */	
// 最大引脚选择数量 - 两套方案
#define MAX_SPI_PIN_SEL 2
typedef struct spiHardware_s {
    SPIDevice device;							 // SPI设备ID
    SPI_TypeDef *reg;							 // SPIx
    spiPinDef_t sckPins[MAX_SPI_PIN_SEL];		 // SCK
    spiPinDef_t misoPins[MAX_SPI_PIN_SEL];		 // MISO
    spiPinDef_t mosiPins[MAX_SPI_PIN_SEL];		 // MOSI
    uint8_t af;									 // 复用功能
    rccPeriphTag_t rcc;							 // 时钟
} spiHardware_t;
// 声明SPI硬件信息结构体
extern const spiHardware_t spiHardware[];

/* -----------------------------SPI设备信息结构体------------------------------ */	
typedef struct SPIDevice_s {
    SPI_TypeDef *dev;							 // SPI设备			
    ioTag_t sck;								 // SCK引脚
    ioTag_t miso;								 // MISO引脚
    ioTag_t mosi;								 // MOSI引脚
    uint8_t af;									 // 复用功能
    rccPeriphTag_t rcc;							 // 时钟
    volatile uint16_t errorCount;				 // 错误数量
    bool leadingEdge;							 // 边沿 - 4种传输方式（上升沿、下降沿、前沿、后沿）
    											 // 根据SPI通信的时钟极性（CPOL）以及时钟相位（CPHA）的选择不同而不同
} spiDevice_t;
// 声明SPI设备信息结构体
extern spiDevice_t spiDevice[SPIDEV_COUNT];

void spiInitDevice(SPIDevice device, bool leadingEdge);
uint32_t spiTimeoutUserCallback(SPI_TypeDef *instance);

