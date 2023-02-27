#include "platform.h"

#include "drivers/osd.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "vcd.h"


PG_REGISTER_WITH_RESET_FN(vcdProfile_t, vcdProfile, PG_VCD_CONFIG, 0);
void pgResetFn_vcdProfile(vcdProfile_t *vcdProfile)
{
	vcdProfile->video_system = VIDEO_SYSTEM_NTSC;  // 默认视频制式NTSC
}

