// 固件相关菜单内容和支持功能

#include <ctype.h>

#include <stdbool.h>

#include "platform.h"

#ifdef USE_CMS
#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "common/printf.h"

#include "config/config.h"

#include "drivers/system.h"

#include "fc/runtime_config.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"

#include "cms_menu_firmware.h"

// 校准状态最大长度
#define CALIBRATION_STATUS_MAX_LENGTH  6
#define CALIBRATION_STATUS_OFF " --- "
#define CALIBRATION_STATUS_NOK " NOK "
#define CALIBRATION_STATUS_WAIT "WAIT "
#define CALIBRATION_STATUS_OK "  OK "

static char gyroCalibrationStatus[CALIBRATION_STATUS_MAX_LENGTH];
#if defined(USE_ACC)
static char accCalibrationStatus[CALIBRATION_STATUS_MAX_LENGTH];
#endif
#if defined(USE_BARO)
static char baroCalibrationStatus[CALIBRATION_STATUS_MAX_LENGTH];
#endif
#if defined(USE_MAG)
static char magCalibrationStatus[CALIBRATION_STATUS_MAX_LENGTH];
#endif

/**********************************************************************
函数名称：cmsx_CalibrationOnDisplayUpdate
函数功能：校准显示更新
函数形参：pDisp，selected
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsx_CalibrationOnDisplayUpdate(displayPort_t *pDisp, const OSD_Entry *selected)
{
    UNUSED(pDisp);
    UNUSED(selected);

    tfp_sprintf(gyroCalibrationStatus, sensors(SENSOR_GYRO) ? gyroIsCalibrationComplete() ? CALIBRATION_STATUS_OK : CALIBRATION_STATUS_WAIT: CALIBRATION_STATUS_OFF);
#if defined(USE_ACC)
    tfp_sprintf(accCalibrationStatus, sensors(SENSOR_ACC) ? accIsCalibrationComplete() ? accHasBeenCalibrated() ? CALIBRATION_STATUS_OK : CALIBRATION_STATUS_NOK : CALIBRATION_STATUS_WAIT: CALIBRATION_STATUS_OFF);
#endif
#if defined(USE_BARO)
    tfp_sprintf(baroCalibrationStatus, sensors(SENSOR_BARO) ? baroIsCalibrationComplete() ? CALIBRATION_STATUS_OK : CALIBRATION_STATUS_WAIT: CALIBRATION_STATUS_OFF);
#endif
#if defined(USE_MAG)
	tfp_sprintf(magCalibrationStatus, sensors(SENSOR_MAG) ? compassIsCalibrationComplete() ? CALIBRATION_STATUS_OK : CALIBRATION_STATUS_WAIT: CALIBRATION_STATUS_OFF);
#endif

    return NULL;
}

/**********************************************************************
函数名称：cmsCalibrateGyro
函数功能：校准陀螺仪
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *cmsCalibrateGyro(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (sensors(SENSOR_GYRO)) {
        gyroStartCalibration(false);
    }

    return NULL;
}

/**********************************************************************
函数名称：cmsCalibrateAcc
函数功能：校准加速度计
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
#if defined(USE_ACC)
static const void *cmsCalibrateAcc(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (sensors(SENSOR_ACC)) {
        accStartCalibration();
    }

    return MENU_CHAIN_BACK;
}
#endif

/**********************************************************************
函数名称：cmsCalibrateBaro
函数功能：校准气压计
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
#if defined(USE_BARO)
static const void *cmsCalibrateBaro(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (sensors(SENSOR_BARO)) {
        baroStartCalibration();
    }

    return NULL;
}
#endif

/**********************************************************************
函数名称：cmsCalibrateMag
函数功能：校准磁力计
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
#if defined(USE_MAG)
static const void *cmsCalibrateMag(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (sensors(SENSOR_MAG)) {
        compassStartCalibration();
    }

    return NULL;
}
#endif


#if defined(USE_ACC)
// 加速度计校准菜单管理内容
static const OSD_Entry menuCalibrateAccEntries[] = {
    { "--- CALIBRATE ACC ---", OME_Label, NULL, NULL, 0 },
    { "PLACE ON A LEVEL SURFACE", OME_Label, NULL, NULL, 0},
    { "MAKE SURE CRAFT IS STILL", OME_Label, NULL, NULL, 0},
    { " ", OME_Label, NULL, NULL, 0},
    { "START CALIBRATION",  OME_Funcall, cmsCalibrateAcc, NULL, 0 },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

// 加速度计校准菜单配置
CMS_Menu cmsx_menuCalibrateAcc = {
    .onEnter = NULL,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = menuCalibrateAccEntries
};

/**********************************************************************
函数名称：cmsCalibrateAccMenu
函数功能：加速度计校准菜单
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
const void *cmsCalibrateAccMenu(displayPort_t *pDisp, const void *self)
{
    UNUSED(self);

    if (sensors(SENSOR_ACC)) {
        cmsMenuChange(pDisp, &cmsx_menuCalibrateAcc);
    }

    return NULL;
}
#endif

// 校准菜单管理内容
static const OSD_Entry menuCalibrationEntries[] = {
    { "--- CALIBRATE ---", OME_Label, NULL, NULL, 0 },
    { "GYRO", OME_Funcall, cmsCalibrateGyro, gyroCalibrationStatus, DYNAMIC },
#if defined(USE_ACC)
    { "ACC",  OME_Funcall, cmsCalibrateAccMenu, accCalibrationStatus, DYNAMIC },
#endif
#if defined(USE_BARO)
    { "BARO", OME_Funcall, cmsCalibrateBaro, baroCalibrationStatus, DYNAMIC },
#endif
#if defined(USE_MAG)
	 { "MAG", OME_Funcall, cmsCalibrateMag, magCalibrationStatus, DYNAMIC },
#endif
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

// 校准菜单配置
static CMS_Menu cmsx_menuCalibration = {
    .onEnter = NULL,
    .onExit = NULL,
    .onDisplayUpdate = cmsx_CalibrationOnDisplayUpdate,
    .entries = menuCalibrationEntries
};

static char infoTargetName[] = __TARGET__;

// 固件菜单管理内容
static const OSD_Entry menuFirmwareEntries[] = {
    { "--- INFO ---", OME_Label, NULL, NULL, 0 },
    { "FWID", OME_String, NULL, FC_FIRMWARE_IDENTIFIER, 0 },
    { "FWVER", OME_String, NULL, FC_VERSION_STRING, 0 },
    { "TARGET", OME_String, NULL, infoTargetName, 0 },
    { "--- SETUP ---", OME_Label, NULL, NULL, 0 },
    { "CALIBRATE",     OME_Submenu, cmsMenuChange, &cmsx_menuCalibration, 0},
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

// 固件菜单配置
CMS_Menu cmsx_menuFirmware = {
    .onEnter = NULL,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = menuFirmwareEntries
};
#endif

