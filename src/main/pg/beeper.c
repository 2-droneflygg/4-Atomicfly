#include "platform.h"

#ifdef USE_BEEPER
#include "io/beeper.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "beeper.h"

PG_REGISTER_WITH_RESET_TEMPLATE(beeperConfig_t, beeperConfig, PG_BEEPER_CONFIG, 2);
PG_RESET_TEMPLATE(beeperConfig_t, beeperConfig,
    .dshotBeaconTone = 1,		// DSHOT信标音调
    .dshotBeaconOffFlags = 0,   // 开启RX_LOST和RX_SET(遥控器失去信号蜂鸣器持续鸣叫直至信号恢复和通过辅助通道发出蜂鸣音)
    .beeper_off_flags = true,   // 蜂鸣器静音
);
#endif

