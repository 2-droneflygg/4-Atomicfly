// 菜单内容为PID，率，RC预览，杂项

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS
#include "build/version.h"
#include "build/build_config.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_imu.h"

#include "common/utils.h"

#include "config/feature.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "drivers/motor.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/pg.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"


// Iterm释放表
const char * const lookupTableItermRelax[] = {
    "OFF", "RP", "RPY", "RP_INC", "RPY_INC"
};
// Iterm释放类型表
const char * const lookupTableItermRelaxType[] = {
    "GYRO", "SETPOINT"
};
// 插值定位点表
const char* const lookupTableInterpolatedSetpoint[] = {
    "OFF", "ON", "AVERAGED_2", "AVERAGED_3", "AVERAGED_4"
};
// 油门限制类型OSD表
static const char * const osdTableThrottleLimitType[] = {
    "OFF", "SCALE", "CLIP"
};

// PID
static uint8_t tmpPidProfileIndex;
static uint8_t pidProfileIndex;
static char pidProfileIndexString[MAX_PROFILE_NAME_LENGTH + 5];
static uint8_t tempPid[3][3];
static uint16_t tempPidF[3];

static uint8_t tmpRateProfileIndex;
static uint8_t rateProfileIndex;
static char rateProfileIndexString[MAX_RATE_PROFILE_NAME_LENGTH + 5];
static controlRateConfig_t rateProfile;


/**********************************************************************
函数名称：setProfileIndexString
函数功能：设置配置文件索引字符串
函数形参：profileString，profileIndex，profileName
函数返回值：None 
函数描述：None 
**********************************************************************/
static void setProfileIndexString(char *profileString, int profileIndex, char *profileName)
{
    int charIndex = 0;
    profileString[charIndex++] = '1' + profileIndex;

#ifdef USE_PROFILE_NAMES
    const int profileNameLen = strlen(profileName);

    if (profileNameLen > 0) {
        profileString[charIndex++] = ' ';
        profileString[charIndex++] = '(';
        for (int i = 0; i < profileNameLen; i++) {
            profileString[charIndex++] = toupper(profileName[i]);
        }
        profileString[charIndex++] = ')';
    }
#else
    UNUSED(profileName);
#endif

    profileString[charIndex] = '\0';
}

/**********************************************************************
函数名称：cmsx_menuImu_onEnter
函数功能：进入IMU菜单
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_menuImu_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    pidProfileIndex = getCurrentPidProfileIndex();
    tmpPidProfileIndex = pidProfileIndex + 1;

    rateProfileIndex = getCurrentControlRateProfileIndex();
    tmpRateProfileIndex = rateProfileIndex + 1;

    return NULL;
}

/**********************************************************************
函数名称：cmsx_menuImu_onExit
函数功能：退出IMU菜单
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_menuImu_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    changePidProfile(pidProfileIndex);
    changeControlRateProfile(rateProfileIndex);

    return NULL;
}

/**********************************************************************
函数名称：cmsx_profileIndexOnChange
函数功能：改变配置文件索引
函数形参：displayPort，ptr
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_profileIndexOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(displayPort);
    UNUSED(ptr);

    pidProfileIndex = tmpPidProfileIndex - 1;
    changePidProfile(pidProfileIndex);

    return NULL;
}

/**********************************************************************
函数名称：cmsx_rateProfileIndexOnChange
函数功能：改变速率配置文件索引
函数形参：displayPort，ptr
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_rateProfileIndexOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(displayPort);
    UNUSED(ptr);

    rateProfileIndex = tmpRateProfileIndex - 1;
    changeControlRateProfile(rateProfileIndex);

    return NULL;
}

/**********************************************************************
函数名称：cmsx_PidRead
函数功能：读取pid
函数形参：None 
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_PidRead(void)
{

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);
    for (uint8_t i = 0; i < 3; i++) {
        tempPid[i][0] = pidProfile->pid[i].P;
        tempPid[i][1] = pidProfile->pid[i].I;
        tempPid[i][2] = pidProfile->pid[i].D;
        tempPidF[i] = pidProfile->pid[i].F;
    }

    return NULL;
}

/**********************************************************************
函数名称：cmsx_PidOnEnter
函数功能：进入pid菜单
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_PidOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    setProfileIndexString(pidProfileIndexString, pidProfileIndex, currentPidProfile->profileName);
    cmsx_PidRead();

    return NULL;
}

/**********************************************************************
函数名称：cmsx_PidWriteback
函数功能：退出pid菜单
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_PidWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = currentPidProfile;
    for (uint8_t i = 0; i < 3; i++) {
        pidProfile->pid[i].P = tempPid[i][0];
        pidProfile->pid[i].I = tempPid[i][1];
        pidProfile->pid[i].D = tempPid[i][2];
        pidProfile->pid[i].F = tempPidF[i];
    }
    pidInitConfig(currentPidProfile);

    return NULL;
}

// PID菜单管理内容
static const OSD_Entry cmsx_menuPidEntries[] =
{
    { "-- PID --", OME_Label, NULL, pidProfileIndexString, 0},

    { "ROLL  P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][0],  0, 200, 1 }, 0 },
    { "ROLL  I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][1],  0, 200, 1 }, 0 },
    { "ROLL  D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][2],  0, 200, 1 }, 0 },
    { "ROLL  F", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_ROLL],  0, 2000, 1 }, 0 },

    { "PITCH P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][0], 0, 200, 1 }, 0 },
    { "PITCH I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][1], 0, 200, 1 }, 0 },
    { "PITCH D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][2], 0, 200, 1 }, 0 },
    { "PITCH F", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_PITCH], 0, 2000, 1 }, 0 },

    { "YAW   P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][0],   0, 200, 1 }, 0 },
    { "YAW   I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][1],   0, 200, 1 }, 0 },
    { "YAW   D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][2],   0, 200, 1 }, 0 },
    { "YAW   F", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_YAW],   0, 2000, 1 }, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

// PID菜单配置
static CMS_Menu cmsx_menuPid = {
    .onEnter = cmsx_PidOnEnter,
    .onExit = cmsx_PidWriteback,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuPidEntries
};

/**********************************************************************
函数名称：cmsx_RateProfileRead
函数功能：读取速率配置文件
函数形参：None 
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_RateProfileRead(void)
{
    memcpy(&rateProfile, controlRateProfiles(rateProfileIndex), sizeof(controlRateConfig_t));

    return NULL;
}

/**********************************************************************
函数名称：cmsx_RateProfileWriteback
函数功能：写入速率配置退出
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_RateProfileWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    memcpy(controlRateProfilesMutable(rateProfileIndex), &rateProfile, sizeof(controlRateConfig_t));

    return NULL;
}

/**********************************************************************
函数名称：cmsx_RateProfileOnEnter
函数功能：进入速率配置文件
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_RateProfileOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    setProfileIndexString(rateProfileIndexString, rateProfileIndex, controlRateProfilesMutable(rateProfileIndex)->profileName);
    cmsx_RateProfileRead();

    return NULL;
}

// 速率配置文件菜单管理内容
static const OSD_Entry cmsx_menuRateProfileEntries[] =
{
    { "-- RATE --", OME_Label, NULL, rateProfileIndexString, 0 },

    { "RC R RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_ROLL],    1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1, 10 }, 0 },
    { "RC P RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_PITCH],    1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1, 10 }, 0 },
    { "RC Y RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_YAW], 1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1, 10 }, 0 },

    { "ROLL SUPER",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_ROLL],   0, CONTROL_RATE_CONFIG_RATE_MAX, 1, 10 }, 0 },
    { "PITCH SUPER", OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_PITCH],   0, CONTROL_RATE_CONFIG_RATE_MAX, 1, 10 }, 0 },
    { "YAW SUPER",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_YAW],   0, CONTROL_RATE_CONFIG_RATE_MAX, 1, 10 }, 0 },

    { "RC R EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_ROLL],    0, 100, 1, 10 }, 0 },
    { "RC P EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_PITCH],    0, 100, 1, 10 }, 0 },
    { "RC Y EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_YAW], 0, 100, 1, 10 }, 0 },

    { "THR MID",     OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.thrMid8,           0,  100,  1}, 0 },
    { "THR EXPO",    OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.thrExpo8,          0,  100,  1}, 0 },
    { "THRPID ATT",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.dynThrPID,         0,  100,  1, 10}, 0 },
    { "TPA BRKPT",   OME_UINT16, NULL, &(OSD_UINT16_t){ &rateProfile.tpa_breakpoint, 1000, 2000, 10}, 0 },

    { "THR LIM TYPE",OME_TAB,    NULL, &(OSD_TAB_t)   { &rateProfile.throttle_limit_type, THROTTLE_LIMIT_TYPE_COUNT - 1, osdTableThrottleLimitType}, 0 },
    { "THR LIM %",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.throttle_limit_percent, 25,  100,  1}, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

// 速率配置文件菜单配置
static CMS_Menu cmsx_menuRateProfile = {
    .onEnter = cmsx_RateProfileOnEnter,
    .onExit = cmsx_RateProfileWriteback,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuRateProfileEntries
};

static uint8_t  cmsx_feedForwardTransition;
static uint8_t  cmsx_ff_boost;
static uint8_t  cmsx_angleStrength;
static uint8_t  cmsx_throttleBoost;
static uint16_t cmsx_itermAcceleratorGain;
static uint16_t cmsx_itermThrottleThreshold;
static uint8_t  cmsx_motorOutputLimit;
static int8_t   cmsx_autoProfileCellCount;
#ifdef USE_D_MIN
static uint8_t  cmsx_d_min[XYZ_AXIS_COUNT];
static uint8_t  cmsx_d_min_gain;
static uint8_t  cmsx_d_min_advance;
#endif

#ifdef USE_ITERM_RELAX
static uint8_t cmsx_iterm_relax;
static uint8_t cmsx_iterm_relax_type;
static uint8_t cmsx_iterm_relax_cutoff;
#endif

#ifdef USE_INTERPOLATED_SP
static uint8_t cmsx_ff_interpolate_sp;
static uint8_t cmsx_ff_smooth_factor;
#endif

/**********************************************************************
函数名称：cmsx_RateProfileOnEnter
函数功能：进入速率配置文件
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_profileOtherOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    setProfileIndexString(pidProfileIndexString, pidProfileIndex, currentPidProfile->profileName);

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);

    cmsx_feedForwardTransition  = pidProfile->feedForwardTransition;
    cmsx_ff_boost = pidProfile->ff_boost;

    cmsx_angleStrength =     pidProfile->pid[PID_LEVEL].P;

    cmsx_itermAcceleratorGain   = pidProfile->itermAcceleratorGain;
    cmsx_itermThrottleThreshold = pidProfile->itermThrottleThreshold;

    cmsx_throttleBoost = pidProfile->throttle_boost;
    cmsx_motorOutputLimit = pidProfile->motor_output_limit;
    cmsx_autoProfileCellCount = pidProfile->auto_profile_cell_count;

#ifdef USE_D_MIN
    for (unsigned i = 0; i < XYZ_AXIS_COUNT; i++) {
        cmsx_d_min[i]  = pidProfile->d_min[i];
    }
    cmsx_d_min_gain = pidProfile->d_min_gain;
    cmsx_d_min_advance = pidProfile->d_min_advance;
#endif

#ifdef USE_ITERM_RELAX
    cmsx_iterm_relax = pidProfile->iterm_relax;
    cmsx_iterm_relax_type = pidProfile->iterm_relax_type;
    cmsx_iterm_relax_cutoff = pidProfile->iterm_relax_cutoff;
#endif

#ifdef USE_INTERPOLATED_SP
    cmsx_ff_interpolate_sp = pidProfile->ff_interpolate_sp;
    cmsx_ff_smooth_factor = pidProfile->ff_smooth_factor;
#endif

    return NULL;
}

/**********************************************************************
函数名称：cmsx_profileOtherOnExit
函数功能：退出速率配置文件
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_profileOtherOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);
    pidProfile->feedForwardTransition = cmsx_feedForwardTransition;
    pidInitConfig(currentPidProfile);
    pidProfile->ff_boost = cmsx_ff_boost;

    pidProfile->pid[PID_LEVEL].P = cmsx_angleStrength;

    pidProfile->itermAcceleratorGain   = cmsx_itermAcceleratorGain;
    pidProfile->itermThrottleThreshold = cmsx_itermThrottleThreshold;

    pidProfile->throttle_boost = cmsx_throttleBoost;
    pidProfile->motor_output_limit = cmsx_motorOutputLimit;
    pidProfile->auto_profile_cell_count = cmsx_autoProfileCellCount;

#ifdef USE_D_MIN
    for (unsigned i = 0; i < XYZ_AXIS_COUNT; i++) {
        pidProfile->d_min[i] = cmsx_d_min[i];
    }
    pidProfile->d_min_gain = cmsx_d_min_gain;
    pidProfile->d_min_advance = cmsx_d_min_advance;
#endif

#ifdef USE_ITERM_RELAX
    pidProfile->iterm_relax = cmsx_iterm_relax;
    pidProfile->iterm_relax_type = cmsx_iterm_relax_type;
    pidProfile->iterm_relax_cutoff = cmsx_iterm_relax_cutoff;
#endif

#ifdef USE_INTERPOLATED_SP
    pidProfile->ff_interpolate_sp = cmsx_ff_interpolate_sp;
    pidProfile->ff_smooth_factor = cmsx_ff_smooth_factor;
#endif

    initEscEndpoints();
    return NULL;
}

// 配置文件其他菜单管理内容
static const OSD_Entry cmsx_menuProfileOtherEntries[] = {
    { "-- OTHER PP --", OME_Label, NULL, pidProfileIndexString, 0 },

    { "ANGLE STR",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_angleStrength,          0,    200,   1  }   , 0 },

#ifdef USE_THROTTLE_BOOST
    { "THR BOOST",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_throttleBoost,          0,    100,   1  }   , 0 },
#endif

    { "FF TRANS",      OME_FLOAT,  NULL, &(OSD_FLOAT_t)  { &cmsx_feedForwardTransition,  0,    100,   1, 10 }, 0 },
#ifdef USE_INTERPOLATED_SP
    { "FF MODE",       OME_TAB,    NULL, &(OSD_TAB_t)    { &cmsx_ff_interpolate_sp,  4, lookupTableInterpolatedSetpoint}, 0 },
    { "FF SMOOTHNESS", OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_ff_smooth_factor,     0,     75,   1  }   , 0 },
#endif
    { "FF BOOST",    OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_ff_boost,               0,     50,   1  }   , 0 },

#ifdef USE_ITERM_RELAX
    { "I_RELAX",         OME_TAB,    NULL, &(OSD_TAB_t)     { &cmsx_iterm_relax,        ITERM_RELAX_COUNT - 1,      lookupTableItermRelax       }, 0 },
    { "I_RELAX TYPE",    OME_TAB,    NULL, &(OSD_TAB_t)     { &cmsx_iterm_relax_type,   ITERM_RELAX_TYPE_COUNT - 1, lookupTableItermRelaxType   }, 0 },
    { "I_RELAX CUTOFF",  OME_UINT8,  NULL, &(OSD_UINT8_t)   { &cmsx_iterm_relax_cutoff, 1, 50, 1 }, 0 },
#endif
    { "AG GAIN",     OME_UINT16, NULL, &(OSD_UINT16_t) { &cmsx_itermAcceleratorGain,   ITERM_ACCELERATOR_GAIN_OFF, ITERM_ACCELERATOR_GAIN_MAX, 10 }   , 0 },
    { "AG THR",      OME_UINT16, NULL, &(OSD_UINT16_t) { &cmsx_itermThrottleThreshold, 20,   1000,  1  }   , 0 },
    
#ifdef USE_D_MIN
    { "D_MIN ROLL",  OME_UINT8,  NULL, &(OSD_UINT8_t) { &cmsx_d_min[FD_ROLL],      0, 100, 1 }, 0 },
    { "D_MIN PITCH", OME_UINT8,  NULL, &(OSD_UINT8_t) { &cmsx_d_min[FD_PITCH],     0, 100, 1 }, 0 },
    { "D_MIN YAW",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &cmsx_d_min[FD_YAW],       0, 100, 1 }, 0 },
    { "D_MIN GAIN",  OME_UINT8,  NULL, &(OSD_UINT8_t) { &cmsx_d_min_gain,          0, 100, 1 }, 0 },
    { "D_MIN ADV",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &cmsx_d_min_advance,       0, 200, 1 }, 0 },
#endif

    { "MTR OUT LIM %",OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_motorOutputLimit, MOTOR_OUTPUT_LIMIT_PERCENT_MIN,  MOTOR_OUTPUT_LIMIT_PERCENT_MAX,  1}, 0 },

    { "AUTO CELL CNT", OME_INT8, NULL, &(OSD_INT8_t) { &cmsx_autoProfileCellCount, AUTO_PROFILE_CELL_COUNT_CHANGE, MAX_AUTO_DETECT_CELL_COUNT, 1}, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

// 配置文件其他菜单配置
static CMS_Menu cmsx_menuProfileOther = {
    .onEnter = cmsx_profileOtherOnEnter,
    .onExit = cmsx_profileOtherOnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuProfileOtherEntries,
};


static uint16_t gyroConfig_gyro_lowpass_hz;
static uint16_t gyroConfig_gyro_lowpass2_hz;
static uint8_t  gyroConfig_gyro_to_use;

/**********************************************************************
函数名称：cmsx_menuGyro_onEnter
函数功能：进入陀螺仪菜单
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_menuGyro_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    gyroConfig_gyro_lowpass_hz =  gyroConfig()->gyro_lowpass_hz;
    gyroConfig_gyro_lowpass2_hz =  gyroConfig()->gyro_lowpass2_hz;

    return NULL;
}

/**********************************************************************
函数名称：cmsx_menuGyro_onExit
函数功能：退出陀螺仪菜单
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_menuGyro_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    gyroConfigMutable()->gyro_lowpass_hz =  gyroConfig_gyro_lowpass_hz;
    gyroConfigMutable()->gyro_lowpass2_hz =  gyroConfig_gyro_lowpass2_hz;

    return NULL;
}

// 滤波菜单管理内容
static const OSD_Entry cmsx_menuFilterGlobalEntries[] =
{
    { "-- FILTER GLB  --", OME_Label, NULL, NULL, 0 },

    { "GYRO LPF",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_lowpass_hz, 0, FILTER_FREQUENCY_MAX, 1 }, 0 },
#ifdef USE_GYRO_LPF2
    { "GYRO LPF2",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_lowpass2_hz,  0, FILTER_FREQUENCY_MAX, 1 }, 0 },
#endif
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

// 滤波菜单配置
static CMS_Menu cmsx_menuFilterGlobal = {
    .onEnter = cmsx_menuGyro_onEnter,
    .onExit = cmsx_menuGyro_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuFilterGlobalEntries,
};

#if (defined(USE_DYN_LPF)) && defined(USE_EXTENDED_CMS_MENUS)
#ifdef USE_DYN_LPF
static uint16_t dynFiltGyroMin;
static uint16_t dynFiltGyroMax;
static uint16_t dynFiltDtermMin;
static uint16_t dynFiltDtermMax;
static uint8_t dynFiltDtermExpo;
#endif

/**********************************************************************
函数名称：cmsx_menuDynFilt_onEnter
函数功能：进入动态低通滤波菜单
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_menuDynFilt_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

#ifdef USE_DYN_LPF
    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);
    dynFiltGyroMin  = gyroConfig()->dyn_lpf_gyro_min_hz;
    dynFiltGyroMax  = gyroConfig()->dyn_lpf_gyro_max_hz;
    dynFiltDtermMin = pidProfile->dyn_lpf_dterm_min_hz;
    dynFiltDtermMax = pidProfile->dyn_lpf_dterm_max_hz;
    dynFiltDtermExpo = pidProfile->dyn_lpf_curve_expo;
#endif

    return NULL;
}

/**********************************************************************
函数名称：cmsx_menuDynFilt_onExit
函数功能：退出动态低通滤波菜单
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_menuDynFilt_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

#ifdef USE_DYN_LPF
    pidProfile_t *pidProfile = currentPidProfile;
    gyroConfigMutable()->dyn_lpf_gyro_min_hz = dynFiltGyroMin;
    gyroConfigMutable()->dyn_lpf_gyro_max_hz = dynFiltGyroMax;
    pidProfile->dyn_lpf_dterm_min_hz         = dynFiltDtermMin;
    pidProfile->dyn_lpf_dterm_max_hz         = dynFiltDtermMax;
    pidProfile->dyn_lpf_curve_expo           = dynFiltDtermExpo;
#endif

    return NULL;
}

// 动态低通滤波菜单管理内容
static const OSD_Entry cmsx_menuDynFiltEntries[] =
{
    { "-- DYN FILT --", OME_Label, NULL, NULL, 0 },

#ifdef USE_DYN_LPF
    { "LPF GYRO MIN",   OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltGyroMin,  0, 1000, 1 }, 0 },
    { "LPF GYRO MAX",   OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltGyroMax,  0, 1000, 1 }, 0 },
    { "DTERM DLPF MIN", OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltDtermMin, 0, 1000, 1 }, 0 },
    { "DTERM DLPF MAX", OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltDtermMax, 0, 1000, 1 }, 0 },
    { "DTERM DLPF EXPO", OME_UINT8, NULL, &(OSD_UINT8_t) { &dynFiltDtermExpo, 0, 10, 1 }, 0 },
#endif

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

// 动态低通滤波菜单配置
static CMS_Menu cmsx_menuDynFilt = {
    .onEnter = cmsx_menuDynFilt_onEnter,
    .onExit = cmsx_menuDynFilt_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuDynFiltEntries,
};

#endif

static uint16_t cmsx_dterm_lowpass_hz;
static uint16_t cmsx_dterm_lowpass2_hz;
static uint16_t cmsx_yaw_lowpass_hz;

/**********************************************************************
函数名称：cmsx_FilterPerProfileRead
函数功能：读取滤波配置文件
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_FilterPerProfileRead(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);

    cmsx_dterm_lowpass_hz   = pidProfile->dterm_lowpass_hz;
    cmsx_dterm_lowpass2_hz  = pidProfile->dterm_lowpass2_hz;

    return NULL;
}

/**********************************************************************
函数名称：cmsx_FilterPerProfileWriteback
函数功能：写入滤波配置文件退出
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_FilterPerProfileWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = currentPidProfile;

    pidProfile->dterm_lowpass_hz   = cmsx_dterm_lowpass_hz;
    pidProfile->dterm_lowpass2_hz  = cmsx_dterm_lowpass2_hz;

    return NULL;
}

// 滤波配置文件菜单管理内容
static const OSD_Entry cmsx_menuFilterPerProfileEntries[] =
{
    { "-- FILTER PP  --", OME_Label, NULL, NULL, 0 },

    { "DTERM LPF",  OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_lowpass_hz,     0, FILTER_FREQUENCY_MAX, 1 }, 0 },
    { "DTERM LPF2", OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_lowpass2_hz,    0, FILTER_FREQUENCY_MAX, 1 }, 0 },
	
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

// 滤波配置文件菜单配置
static CMS_Menu cmsx_menuFilterPerProfile = {
    .onEnter = cmsx_FilterPerProfileRead,
    .onExit = cmsx_FilterPerProfileWriteback,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuFilterPerProfileEntries,
};

#ifdef USE_EXTENDED_CMS_MENUS
static uint8_t cmsx_dstPidProfile;
static uint8_t cmsx_dstControlRateProfile;

// 配置文件名称
static const char * const cmsx_ProfileNames[] = {
    "-",
    "1",
    "2",
    "3"
};

static OSD_TAB_t cmsx_PidProfileTable = { &cmsx_dstPidProfile, 3, cmsx_ProfileNames };
static OSD_TAB_t cmsx_ControlRateProfileTable = { &cmsx_dstControlRateProfile, 3, cmsx_ProfileNames };

/**********************************************************************
函数名称：cmsx_menuCopyProfile_onEnter
函数功能：进入配置文件复制
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_menuCopyProfile_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_dstPidProfile = 0;
    cmsx_dstControlRateProfile = 0;

    return NULL;
}

/**********************************************************************
函数名称：cmsx_CopyPidProfile
函数功能：复制PID配置文件
函数形参：pDisplay，ptr
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_CopyPidProfile(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(pDisplay);
    UNUSED(ptr);

    if (cmsx_dstPidProfile > 0) {
        pidCopyProfile(cmsx_dstPidProfile - 1, getCurrentPidProfileIndex());
    }

    return NULL;
}

/**********************************************************************
函数名称：cmsx_CopyControlRateProfile
函数功能：复制控制速率配置文件
函数形参：pDisplay，ptr
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_CopyControlRateProfile(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(pDisplay);
    UNUSED(ptr);

    if (cmsx_dstControlRateProfile > 0) {
        copyControlRateProfile(cmsx_dstControlRateProfile - 1, getCurrentControlRateProfileIndex());
    }

    return NULL;
}

// 复制配置文件菜单管理内容
static const OSD_Entry cmsx_menuCopyProfileEntries[] =
{
    { "-- COPY PROFILE --", OME_Label, NULL, NULL, 0},

    { "CPY PID PROF TO",   OME_TAB,      NULL,                        &cmsx_PidProfileTable, 0 },
    { "COPY PP",           OME_Funcall,  cmsx_CopyPidProfile,         NULL, 0 },
    { "CPY RATE PROF TO",  OME_TAB,      NULL,                        &cmsx_ControlRateProfileTable, 0 },
    { "COPY RP",           OME_Funcall,  cmsx_CopyControlRateProfile, NULL, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

// 复制配置文件菜单配置
CMS_Menu cmsx_menuCopyProfile = {
    .onEnter = cmsx_menuCopyProfile_onEnter,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuCopyProfileEntries,
};

#endif

// IMU菜单管理内容
static const OSD_Entry cmsx_menuImuEntries[] =
{
    { "-- PROFILE --", OME_Label, NULL, NULL, 0},

    {"PID PROF",  OME_UINT8,   cmsx_profileIndexOnChange,     &(OSD_UINT8_t){ &tmpPidProfileIndex, 1, PID_PROFILE_COUNT, 1},    0},
    {"PID",       OME_Submenu, cmsMenuChange,                 &cmsx_menuPid,                                                 0},

    {"RATE PROF", OME_UINT8,   cmsx_rateProfileIndexOnChange, &(OSD_UINT8_t){ &tmpRateProfileIndex, 1, CONTROL_RATE_PROFILE_COUNT, 1}, 0},
    {"RATE",      OME_Submenu, cmsMenuChange,                 &cmsx_menuRateProfile,                                         0},

    {"MISC PP",   OME_Submenu, cmsMenuChange,                 &cmsx_menuProfileOther,                                        0},

    {"GYRO LPF",  OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterGlobal,                                        0},
    {"DTERM LPF",   OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterPerProfile,                                    0},

#if  defined(USE_DYN_LPF) && defined(USE_EXTENDED_CMS_MENUS)
    {"DYN LPF",  OME_Submenu, cmsMenuChange,                 &cmsx_menuDynFilt,                                             0},
#endif

#ifdef USE_EXTENDED_CMS_MENUS
    {"COPY PROF", OME_Submenu, cmsMenuChange,                 &cmsx_menuCopyProfile,                                         0},
#endif /* USE_EXTENDED_CMS_MENUS */

    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

// IMU菜单配置
CMS_Menu cmsx_menuImu = {
    .onEnter = cmsx_menuImu_onEnter,
    .onExit = cmsx_menuImu_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuImuEntries,
};

#endif // CMS
