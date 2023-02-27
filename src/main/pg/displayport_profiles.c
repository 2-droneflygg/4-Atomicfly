
#include <stdbool.h>

#include "platform.h"

#include "pg/displayport_profiles.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"


#if defined(USE_MAX7456)
PG_REGISTER_WITH_RESET_FN(displayPortProfile_t, displayPortProfileMax7456, PG_DISPLAY_PORT_MAX7456_CONFIG, 0);
void pgResetFn_displayPortProfileMax7456(displayPortProfile_t *displayPortProfile)
{
    displayPortProfile->colAdjust = 0;
    displayPortProfile->rowAdjust = 0;

    // 根据MAX7456数据表设置默认值
    displayPortProfile->invert = false;
    displayPortProfile->blackBrightness = 0;
    displayPortProfile->whiteBrightness = 2;
}
#endif

