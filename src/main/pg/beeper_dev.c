#include "platform.h"

#ifdef USE_BEEPER
#include "drivers/io.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "beeper_dev.h"

#ifdef BEEPER_INVERTED
#define IS_OPEN_DRAIN   false   // 引脚推挽输出
#define IS_INVERTED     true	// 引脚开漏输出
#else
#define IS_OPEN_DRAIN   true    // 引脚推挽输出
#define IS_INVERTED     false	// 引脚开漏输出
#endif
#define BEEPER_PWM_HZ   0

PG_REGISTER_WITH_RESET_TEMPLATE(beeperDevConfig_t, beeperDevConfig, PG_BEEPER_DEV_CONFIG, 0);
PG_RESET_TEMPLATE(beeperDevConfig_t, beeperDevConfig,
    .isOpenDrain = IS_OPEN_DRAIN,
    .isInverted = IS_INVERTED,
    .ioTag = IO_TAG(BEEPER_PIN),
    .frequency = BEEPER_PWM_HZ
);
#endif

