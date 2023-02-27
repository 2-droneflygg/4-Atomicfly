#pragma once

/* ---------------------------------资源所有者枚举--------------------------------- */	
typedef enum {
    OWNER_FREE = 0,
    OWNER_MOTOR,
    OWNER_LED,
    OWNER_ADC,
    OWNER_ADC_BATT,
    OWNER_ADC_CURR,
    OWNER_SERIAL_TX,
    OWNER_SERIAL_RX,
    OWNER_TIMER,
    OWNER_SYSTEM,
    OWNER_SPI_SCK,
    OWNER_SPI_MISO,
    OWNER_SPI_MOSI,
    OWNER_I2C_SCL,
    OWNER_I2C_SDA,
    OWNER_SDCARD,
    OWNER_SDCARD_CS,
    OWNER_SDCARD_DETECT,
    OWNER_BARO_CS,
    OWNER_GYRO_CS,
    OWNER_OSD_CS,
    OWNER_INVERTER,
    OWNER_SPI_CS,
    OWNER_GYRO_EXTI,
    OWNER_COMPASS_EXTI,
    OWNER_USB,
    OWNER_USB_DETECT,
    OWNER_BEEPER,
    OWNER_OSD,
    OWNER_LED_STRIP,
    OWNER_TIMUP,
    OWNER_PREINIT,
    OWNER_PULLUP,
    OWNER_PULLDOWN,
    OWNER_SWD,
    OWNER_TOTAL_COUNT
} resourceOwner_e;

/* ------------------------------资源所有者配置结构体------------------------------ */	
typedef struct resourceOwner_s {
    resourceOwner_e owner;
    uint8_t resourceIndex;
} resourceOwner_t;

#define RESOURCE_INDEX(x) (x + 1)     // 资源索引
#define RESOURCE_SOFT_OFFSET    10	  // 资源软件偏移
