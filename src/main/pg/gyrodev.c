#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/gyrodev.h"

#include "drivers/io.h"
#include "drivers/bus_spi.h"
#include "drivers/sensor.h"
#include "sensors/gyro.h"

static void gyroResetCommonDeviceConfig(gyroDeviceConfig_t *devconf, ioTag_t extiTag, uint8_t alignment)
{
	// 外部中断线
    devconf->extiTag = extiTag;
	// 板对齐
    devconf->alignment = alignment;			
}

static void gyroResetSpiDeviceConfig(gyroDeviceConfig_t *devconf, SPI_TypeDef *instance, ioTag_t csnTag, ioTag_t extiTag, uint8_t alignment)
{
	// 使用SPI总线
    devconf->bustype = BUSTYPE_SPI;
	// SPI设备
    devconf->spiBus = SPI_DEV_TO_CFG(spiDeviceByInstance(instance));
	// 片选线
    devconf->csnTag = csnTag;
	// 配置陀螺仪通用设备配置
    gyroResetCommonDeviceConfig(devconf, extiTag, alignment);
}

PG_REGISTER_ARRAY_WITH_RESET_FN(gyroDeviceConfig_t, MAX_GYRODEV_COUNT, gyroDeviceConfig, PG_GYRO_DEVICE_CONFIG, 0);
void pgResetFn_gyroDeviceConfig(gyroDeviceConfig_t *devconf)
{
    devconf[0].index = 0;
    // 陀螺仪配置
    gyroResetSpiDeviceConfig(&devconf[0], GYRO_1_SPI_INSTANCE, IO_TAG(GYRO_1_CS_PIN), IO_TAG(GYRO_1_EXTI_PIN), GYRO_1_ALIGN);
}

