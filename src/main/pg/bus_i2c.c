#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C)
#include "common/utils.h"

#include "drivers/io.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/bus_i2c.h"

typedef struct i2cDefaultConfig_s {
    I2CDevice device;
    ioTag_t ioTagScl, ioTagSda;
    bool overClock;
    bool pullUp;
} i2cDefaultConfig_t;

static const i2cDefaultConfig_t i2cDefaultConfig[] = {
#ifdef USE_I2C_DEVICE_2
    { I2CDEV_2, IO_TAG(I2C2_SCL), IO_TAG(I2C2_SDA), I2C2_OVERCLOCK, I2C2_PULLUP },
#endif
};

PG_REGISTER_ARRAY_WITH_RESET_FN(i2cConfig_t, I2CDEV_COUNT, i2cConfig, PG_I2C_CONFIG, 0);
void pgResetFn_i2cConfig(i2cConfig_t *i2cConfig)
{
    memset(i2cConfig, 0, sizeof(*i2cConfig));

    for (size_t index = 0 ; index < ARRAYLEN(i2cDefaultConfig) ; index++) {
        const i2cDefaultConfig_t *defconf = &i2cDefaultConfig[index];
        int device = defconf->device;
        i2cConfig[device].ioTagScl = defconf->ioTagScl;
        i2cConfig[device].ioTagSda = defconf->ioTagSda;
        i2cConfig[device].overClock = defconf->overClock;
        i2cConfig[device].pullUp = defconf->pullUp;
    }
}
#endif // defined(USE_I2C)

