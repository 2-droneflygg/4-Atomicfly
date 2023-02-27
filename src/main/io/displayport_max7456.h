#pragma once

#include "drivers/display.h"

#include "pg/displayport_profiles.h"

struct vcdProfile_s;
displayPort_t *max7456DisplayPortInit(const struct vcdProfile_s *vcdProfile);

