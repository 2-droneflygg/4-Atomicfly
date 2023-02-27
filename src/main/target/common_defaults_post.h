/**********************************************************************
 飞控功能相关默认配置预定义：
**********************************************************************/

//-----------------------------------------------------------配置电机数量
#define MAX_SUPPORTED_MOTORS 4
//-----------------------------------------------------------配置混控方式
#ifndef DEFAULT_MIXER
#define DEFAULT_MIXER    MIXER_QUADX
#endif

//-----------------------------------------------------------配置SPI总线
// 配置引脚
#ifndef SPI1_SCK_PIN
#define SPI1_SCK_PIN    PA5
#define SPI1_MISO_PIN   PA6
#define SPI1_MOSI_PIN   PA7
#endif
#ifndef SPI2_SCK_PIN
#define SPI2_SCK_PIN    PB13
#define SPI2_MISO_PIN   PB14
#define SPI2_MOSI_PIN   PB15
#endif
#ifndef SPI3_SCK_PIN
#define SPI3_SCK_PIN    PB3
#define SPI3_MISO_PIN   PB4
#define SPI3_MOSI_PIN   PB5
#endif
#ifndef SPI4_SCK_PIN
#define SPI4_SCK_PIN    NONE
#define SPI4_MISO_PIN   NONE
#define SPI4_MOSI_PIN   NONE
#endif
#ifndef SPI5_SCK_PIN
#define SPI5_SCK_PIN    NONE
#define SPI5_MISO_PIN   NONE
#define SPI5_MOSI_PIN   NONE
#endif
#ifndef SPI6_SCK_PIN
#define SPI6_SCK_PIN    NONE
#define SPI6_MISO_PIN   NONE
#define SPI6_MOSI_PIN   NONE
#endif
// SPI_DMA
#ifdef USE_SPI
#ifdef USE_SPI_DEVICE_1
#ifndef SPI1_TX_DMA_OPT
#define SPI1_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef SPI1_RX_DMA_OPT
#define SPI1_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif
#ifdef USE_SPI_DEVICE_2
#ifndef SPI2_TX_DMA_OPT
#define SPI2_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef SPI2_RX_DMA_OPT
#define SPI2_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif
#ifdef USE_SPI_DEVICE_3
#ifndef SPI3_TX_DMA_OPT
#define SPI3_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef SPI3_RX_DMA_OPT
#define SPI3_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif
#endif

//-----------------------------------------------------------配置IIC总线
// IIC引脚
#ifndef I2C1_SCL
#define I2C1_SCL PB6
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB7
#endif
#ifndef I2C2_SCL
#define I2C2_SCL PB10
#endif
#ifndef I2C2_SDA
#define I2C2_SDA PB11
#endif
#ifndef I2C3_SCL
#define I2C3_SCL PA8
#endif
#ifndef I2C3_SDA
#define I2C3_SDA PC9
#endif
// IIC超频
#ifndef I2C1_OVERCLOCK
#define I2C1_OVERCLOCK false
#endif
#ifndef I2C2_OVERCLOCK
#define I2C2_OVERCLOCK false
#endif
#ifndef I2C3_OVERCLOCK
#define I2C3_OVERCLOCK false
#endif
#ifndef I2C4_OVERCLOCK
#define I2C4_OVERCLOCK false
#endif
// IIC上拉引脚
#if defined(USE_I2C_PULLUP)
#define I2C1_PULLUP true
#define I2C2_PULLUP true
#define I2C3_PULLUP true
#define I2C4_PULLUP true
#else
#define I2C1_PULLUP false
#define I2C2_PULLUP false
#define I2C3_PULLUP false
#define I2C4_PULLUP false
#endif

//-----------------------------------------------------------配置ADC
#ifdef USE_ADC
#if !defined(USE_UNIFIED_TARGET) && !defined(ADC_INSTANCE)
// ADC设备实例
#define ADC_INSTANCE ADC1
#ifndef ADC1_DMA_OPT
// ADC1_DMA选项
#define ADC1_DMA_OPT 1
#endif
#endif
// ADC_DMA
#if !defined(ADC1_DMA_OPT)
#define ADC1_DMA_OPT (DMA_OPT_UNUSED)
#endif
#if !defined(ADC2_DMA_OPT)
#define ADC2_DMA_OPT (DMA_OPT_UNUSED)
#endif
#if !defined(ADC3_DMA_OPT)
#define ADC3_DMA_OPT (DMA_OPT_UNUSED)
#endif
#if !defined(ADC4_DMA_OPT)
#define ADC4_DMA_OPT (DMA_OPT_UNUSED)
#endif
#if !defined(ADC5_DMA_OPT)
#define ADC5_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif // USE_ADC

//-----------------------------------------------------------配置USART
// 使用串口功能
#if defined(USE_UART1) || defined(USE_UART2) || defined(USE_UART3) || defined(USE_UART4) || defined(USE_UART5) || defined(USE_UART6) || defined(USE_UART7) || defined(USE_UART8)
#define USE_UART
#endif
// 配置串口DMA
#ifdef USE_UART1
#ifndef UART1_TX_DMA_OPT
#define UART1_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART1_RX_DMA_OPT
#define UART1_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif
#ifdef USE_UART3
#ifndef UART3_TX_DMA_OPT
#define UART3_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART3_RX_DMA_OPT
#define UART3_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif
#ifdef USE_UART6
#ifndef UART6_TX_DMA_OPT
#define UART6_TX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#ifndef UART6_RX_DMA_OPT
#define UART6_RX_DMA_OPT (DMA_OPT_UNUSED)
#endif
#endif

//-----------------------------------------------------------配置TIMER管理
#ifdef USE_TIMER_MGMT
#ifndef MAX_TIMER_PINMAP_COUNT
// 最大的已知F405RG定时器引脚数量 
#define MAX_TIMER_PINMAP_COUNT 21 
#endif
#endif

//-----------------------------------------------------------配置陀螺仪
// 陀螺仪引脚
#if !defined(GYRO_1_SPI_INSTANCE)
#define GYRO_1_SPI_INSTANCE     NULL
#endif
#if !defined(GYRO_1_CS_PIN)
#define GYRO_1_CS_PIN           NONE
#endif
#if !defined(GYRO_1_EXTI_PIN)
#define GYRO_1_EXTI_PIN         NONE
#endif
// 最大陀螺仪、加速度计数量
#define MAX_GYRODEV_COUNT 1
#define MAX_ACCDEV_COUNT 1
// 陀螺仪板对齐
#if !defined(GYRO_1_ALIGN)
#define GYRO_1_ALIGN            CW0_DEG
#endif
#if !defined(GYRO_2_ALIGN)
#define GYRO_2_ALIGN            CW0_DEG
#endif

//-----------------------------------------------------------配置气压计
#if defined(USE_BARO)
// 配置引脚
#ifndef BARO_SPI_INSTANCE
#define BARO_SPI_INSTANCE       NULL
#endif
#ifndef BARO_CS_PIN
#define BARO_CS_PIN             NONE
#endif
#ifndef BARO_I2C_INSTANCE
#define BARO_I2C_INSTANCE       I2C_DEVICE
#endif
#ifndef BARO_XCLR_PIN
#define BARO_XCLR_PIN           NONE
#endif
#endif

//-----------------------------------------------------------配置磁力计
#if defined(USE_MAG)
// 配置引脚
#ifndef MAG_SPI_INSTANCE
#define MAG_SPI_INSTANCE        NULL
#endif
#ifndef MAG_CS_PIN
#define MAG_CS_PIN              NONE
#endif
#ifndef MAG_I2C_INSTANCE
#define MAG_I2C_INSTANCE        I2C_DEVICE
#endif
#endif
#ifndef MAG_INT_EXTI
#define MAG_INT_EXTI            NONE
#endif

//-----------------------------------------------------------配置接收机
// RX通道 - 从rx/rx.c和rx/rx.h中提取
#define RX_MAPPABLE_CHANNEL_COUNT 8
// SBUS
#ifndef SERIALRX_PROVIDER
#define SERIALRX_PROVIDER 2    
#endif
// RX通道值
#define RX_MIN_USEC 885
#define RX_MAX_USEC 2115
#define RX_MID_USEC 1500

//-----------------------------------------------------------配置MAX7456
#ifdef  USE_MAX7456
#ifndef MAX7456_CLOCK_CONFIG_DEFAULT
#define MAX7456_CLOCK_CONFIG_DEFAULT    MAX7456_CLOCK_CONFIG_OC
#endif
#ifndef MAX7456_SPI_CLK
#define MAX7456_SPI_CLK                 (SPI_CLOCK_STANDARD)
#endif
#ifndef MAX7456_RESTORE_CLK
#define MAX7456_RESTORE_CLK             (SPI_CLOCK_FAST)
#endif
#ifndef MAX7456_SPI_CS_PIN
#define MAX7456_SPI_CS_PIN              NONE
#endif
#ifndef MAX7456_SPI_INSTANCE
#define MAX7456_SPI_INSTANCE            NULL
#endif
#endif

