/**********************************************************************
RATE配置文件：
	飞控可以保存3个不同的Rate配置文件
	Rate配置文件包括：RC rate， Rate，RC expo，油门和TPA
	配置文件可以通过OSD菜单切换

	RC_EXPO: 固定曲线值 - 改变曲线的曲率，并不会改变两端的曲率
	RC_Rate：线性角速度 - 放大倍数
	Rate:	 曲线角速度 - 非线性，既改了曲线又改了rate
	
	陀螺仪： 物体运动角位移的时间变化率叫瞬时角速度（亦称即时角速度），单位是弧度/秒(rad/s)
	deg/s:   度(角度)每秒
**********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/axis.h"

#include "config/config_reset.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"

controlRateConfig_t *currentControlRateProfile;

PG_REGISTER_ARRAY_WITH_RESET_FN(controlRateConfig_t, CONTROL_RATE_PROFILE_COUNT, controlRateProfiles, PG_CONTROL_RATE_PROFILES, 2);
void pgResetFn_controlRateProfiles(controlRateConfig_t *controlRateConfig)
{
    for (int i = 0; i < CONTROL_RATE_PROFILE_COUNT; i++) {
        RESET_CONFIG(controlRateConfig_t, &controlRateConfig[i],
            .thrMid8 = 50,
            .thrExpo8 = 0,
            .dynThrPID = 65,
            .tpa_breakpoint = 1350,
            .rates_type = RATES_TYPE_DFLIGHT,
            .rcRates[FD_ROLL] = 100,
            .rcRates[FD_PITCH] = 100,
            .rcRates[FD_YAW] = 100,
            .rcExpo[FD_ROLL] = 10,
            .rcExpo[FD_PITCH] = 10,
            .rcExpo[FD_YAW] = 0,
            .rates[FD_ROLL] = 71,
            .rates[FD_PITCH] = 71,
            .rates[FD_YAW] = 70,
            .throttle_limit_type = THROTTLE_LIMIT_TYPE_SCALE,
            .throttle_limit_percent = 75,
            .rate_limit[FD_ROLL] = CONTROL_RATE_CONFIG_RATE_LIMIT_MAX,
            .rate_limit[FD_PITCH] = CONTROL_RATE_CONFIG_RATE_LIMIT_MAX,
            .rate_limit[FD_YAW] = CONTROL_RATE_CONFIG_RATE_LIMIT_MAX,
            .tpaMode = TPA_MODE_D,
            .profileName = { 0 },
        );
    }
}

/**********************************************************************
函数名称：loadControlRateProfile
函数功能：加载速率配置文件
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void loadControlRateProfile(void)
{
    currentControlRateProfile = controlRateProfilesMutable(systemConfig()->activeRateProfile);
}

/**********************************************************************
函数名称：changeControlRateProfile
函数功能：更改控制速率配置文件
函数形参：controlRateProfileIndex
函数返回值：None
函数描述：None
**********************************************************************/
void changeControlRateProfile(uint8_t controlRateProfileIndex)
{
    if (controlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT) {
        systemConfigMutable()->activeRateProfile = controlRateProfileIndex;
    }
	// 加载速率配置文件
    loadControlRateProfile();
	// 初始化RC进程
    initRcProcessing();
}

/**********************************************************************
函数名称：copyControlRateProfile
函数功能：复制控制速率配置文件
函数形参：dstControlRateProfileIndex，dstControlRateProfileIndex
函数返回值：None
函数描述：None
**********************************************************************/
void copyControlRateProfile(const uint8_t dstControlRateProfileIndex, const uint8_t srcControlRateProfileIndex) {
    if ((dstControlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT && srcControlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT)
        && dstControlRateProfileIndex != srcControlRateProfileIndex
    ) {
        memcpy(controlRateProfilesMutable(dstControlRateProfileIndex), controlRateProfiles(srcControlRateProfileIndex), sizeof(controlRateConfig_t));
    }
}

