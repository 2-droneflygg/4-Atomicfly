/**********************************************************************
 飞控硬件配置[DFLIGHTF4]：
**********************************************************************/
#pragma once

// ---------------------------------------------------------------------------------目标飞控板标识
#define TARGET_BOARD_IDENTIFIER "DFLIGHT"

// ---------------------------------------------------------------------------------默认特性
#define DEFAULT_FEATURES        (FEATURE_OSD)

// ---------------------------------------------------------------------------------配置使用的GPIO端口资源
// GPIOA - 不使用13、14引脚 
#define TARGET_IO_PORTA (0xffff & ~(BIT(14)|BIT(13)))      
// GPIOB - 不使用2引脚 
#define TARGET_IO_PORTB (0xffff & ~(BIT(2)))				  
// GPIOC - 不使用13、14、15引脚 
#define TARGET_IO_PORTC (0xffff & ~(BIT(15)|BIT(14)|BIT(13))) 
// GPIOD - 使用1、2引脚 
#define TARGET_IO_PORTD BIT(2)

// ---------------------------------------------------------------------------------配置SPI总线
// 使用SPI
#define USE_SPI
// 配置SPI1端口
#define USE_SPI_DEVICE_1
// 配置SPI2端口
#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15
// 配置SPI3端口
#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

// ---------------------------------------------------------------------------------配置IIC总线
// 使用IIC
#define USE_I2C
// 使用IIC2
#define USE_I2C_DEVICE_2
// 配置IIC2引脚
#define I2C2_SCL                PB10  // PB10, shared with UART3TX
#define I2C2_SDA                PB11  // PB11, shared with UART3RX
// IIC默认设备选择IIC2
#define I2C_DEVICE              (I2CDEV_2)

// ---------------------------------------------------------------------------------配置外部EXTI
#define USE_EXTI

// ---------------------------------------------------------------------------------配置ADC
// 使用ADC
//#define USE_ADC
// 使用ADC2
#define ADC_INSTANCE            ADC2
// DMA 2 Stream 3 Channel 1 (compat default)
#define ADC2_DMA_OPT            1     
// 电流计ADC端口配置
#define CURRENT_METER_ADC_PIN   PC1   // 由电调的电流传感器提供
// 电池电压检测端口
#define VBAT_ADC_PIN            PC2   // 11:1 (10K + 1K) divider
// 默认电压计ADC源
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
// 默认电流计ADC源
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

// ---------------------------------------------------------------------------------配置串口
// 配置模拟USB通信
#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN          PC5
// 配置串口 1
#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9
// 配置串口 3
#define USE_UART3
#define UART3_RX_PIN            NONE
#define UART3_TX_PIN            NONE
// 配置串口 6
#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6
// 使用软件模拟串口 1、2
#define USE_SOFTSERIAL1			
#define SOFTSERIAL1_TX_PIN      PA1
#define SOFTSERIAL1_RX_PIN      PA1
#define USE_SOFTSERIAL2
#define SOFTSERIAL2_TX_PIN      PA8
#define SOFTSERIAL2_RX_PIN      PA8
// 串行通信端口数量 - VCP, USART1, USART3, USART6, SOFTSERIAL x 2
#define SERIAL_PORT_COUNT       6  

// ---------------------------------------------------------------------------------配置定时器
// 使用定时器
#define USED_TIMERS (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(9) | TIM_N(10) | TIM_N(12))
// 配置可用的定时器通道数量
#define USABLE_TIMER_CHANNEL_COUNT 15

// ---------------------------------------------------------------------------------板载LED引脚
#define LED0_PIN                PB5

// ---------------------------------------------------------------------------------蜂鸣器引脚
#define USE_BEEPER
#define BEEPER_PIN              PB4
#define BEEPER_INVERTED			// 开漏输出

// ---------------------------------------------------------------------------------配置陀螺仪加速度计
// 使用陀螺仪[MPU6000]
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
// 使用加速度计[MPU6000]
#define USE_ACC
#define USE_ACC_SPI_MPU6000
// 陀螺仪引脚配置
#define GYRO_1_CS_PIN           PC2
#define GYRO_1_SPI_INSTANCE     SPI1
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC3
#define USE_MPU_DATA_READY_SIGNAL
// 板对齐方式
#define GYRO_1_ALIGN            CW270_DEG

// ---------------------------------------------------------------------------------配置气压计[BMP280]
// 使用气压计
#define USE_BARO
#define USE_BARO_SPI_BMP280
// 气压计引脚配置
#define BARO_SPI_INSTANCE       SPI2
#define BARO_CS_PIN             PB3 // v1
#define USE_BARO_BMP280
#define BARO_I2C_INSTANCE       (I2CDEV_2)
// 默认气压计SPI_BMP280
#define DEFAULT_BARO_SPI_BMP280

// ---------------------------------------------------------------------------------配置磁力计[HMC5883L]
// 使用磁力计
#define USE_MAG
#define USE_MAG_HMC5883
// 板对齐方式
#define MAG_HMC5883_ALIGN       CW180_DEG

// ---------------------------------------------------------------------------------配置OSD
// 使用MAX7456
#define USE_MAX7456
// 配置MAX7456引脚
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PA15

// ---------------------------------------------------------------------------------使用DMA驱动DSHOT
#define ENABLE_DSHOT_DMAR       DSHOT_DMAR_ON

