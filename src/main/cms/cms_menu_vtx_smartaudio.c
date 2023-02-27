#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <drivers/vtx_table.h>

#include "platform.h"

#if defined(USE_CMS) && defined(USE_VTX_SMARTAUDIO)
#include "common/printf.h"
#include "common/utils.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_vtx_smartaudio.h"

#include "drivers/vtx_common.h"

#include "config/config.h"

#include "io/vtx_smartaudio.h"
#include "io/vtx.h"

// 接口CMS
// 操作模型和射频模式(CMS)
#define SACMS_OPMODEL_UNDEF        0 // Not known yet
#define SACMS_OPMODEL_FREE         1 // Freestyle model: Power up transmitting
#define SACMS_OPMODEL_RACE         2 // Race model: Power up in pit mode

uint8_t  saCmsOpmodel = SACMS_OPMODEL_UNDEF;

#define SACMS_TXMODE_NODEF         0
#define SACMS_TXMODE_PIT_OUTRANGE  1
#define SACMS_TXMODE_PIT_INRANGE   2
#define SACMS_TXMODE_ACTIVE        3

uint8_t  saCmsRFState;               // 射频状态;激活，PIR, POR XXX目前没有使用

uint8_t  saCmsBand = 0;
uint8_t  saCmsChan = 0;
uint8_t  saCmsPower = 0;

// 由信道表导出的频率(用于频带/信道模式参考)
uint16_t saCmsFreqRef = 0;

uint16_t saCmsDeviceFreq = 0;

uint8_t  saCmsPower;
uint8_t  saCmsPit;
uint8_t  saCmsPitFMode;          // Undef(0), In-Range(1) or Out-Range(2)

uint8_t  saCmsFselMode;          // Channel(0) or User defined(1)
uint8_t  saCmsFselModeNew;       // Channel(0) or User defined(1)

uint16_t saCmsORFreq = 0;        // POR frequency
uint16_t saCmsORFreqNew;         // POR frequency

uint16_t saCmsUserFreq = 0;      // User defined frequency
uint16_t saCmsUserFreqNew;       // User defined frequency

/**********************************************************************
函数名称：saUpdateStatusString
函数功能：smartaudio更新菜单
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void saCmsUpdate(void)
{
    if (saCmsOpmodel == SACMS_OPMODEL_UNDEF) {
        // 这是对GET_SETTINGS的第一个有效响应。
        saCmsOpmodel = saDevice.willBootIntoPitMode ? SACMS_OPMODEL_RACE : SACMS_OPMODEL_FREE;

        saCmsFselMode = (saDevice.mode & SA_MODE_GET_FREQ_BY_FREQ) && vtxSettingsConfig()->band == 0 ? 1 : 0;

        saCmsBand = vtxSettingsConfig()->band;
        saCmsChan = vtxSettingsConfig()->channel;
        saCmsFreqRef = vtxSettingsConfig()->freq;
        saCmsDeviceFreq = saCmsFreqRef;

        if ((saDevice.mode & SA_MODE_GET_PITMODE) == 0) {
            saCmsRFState = SACMS_TXMODE_ACTIVE;
        } else if (saDevice.mode & SA_MODE_GET_IN_RANGE_PITMODE) {
            saCmsRFState = SACMS_TXMODE_PIT_INRANGE;
        } else {
            saCmsRFState = SACMS_TXMODE_PIT_OUTRANGE;
        }

        saCmsPower = vtxSettingsConfig()->power;

        // if user-freq mode then track possible change
        if (saCmsFselMode && vtxSettingsConfig()->freq) {
            saCmsUserFreq = vtxSettingsConfig()->freq;
        }

        saCmsFselModeNew = saCmsFselMode;   //init mode for menu
    }

    saUpdateStatusString();
}

char saCmsStatusString[31] = "- -- ---- ---";
//                            m bc ffff ppp
//                            0123456789012
static const void *saCmsConfigOpmodelByGvar(displayPort_t *, const void *self);
static const void *saCmsConfigPitFModeByGvar(displayPort_t *, const void *self);
static const void *saCmsConfigBandByGvar(displayPort_t *, const void *self);
static const void *saCmsConfigChanByGvar(displayPort_t *, const void *self);
static const void *saCmsConfigPowerByGvar(displayPort_t *, const void *self);
static const void *saCmsConfigPitByGvar(displayPort_t *, const void *self);

/**********************************************************************
函数名称：saUpdateStatusString
函数功能：smartaudio更新状态字符串
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void saUpdateStatusString(void)
{
    if (saDevice.version == 0)
        return;
	
    if (saCmsORFreq == 0 && saDevice.orfreq != 0)
        saCmsORFreq = saDevice.orfreq;
    if (saCmsUserFreq == 0 && saDevice.freq != 0)
        saCmsUserFreq = saDevice.freq;

    if (saDevice.version == 2) {
        if (saDevice.mode & SA_MODE_GET_OUT_RANGE_PITMODE)
            saCmsPitFMode = 1;
        else
            saCmsPitFMode = 0;
    } else if (saDevice.version == 3) {
        saCmsPitFMode = 1;//V2.1 only supports PIR
    }

    const vtxDevice_t *device = vtxCommonDevice();

    saCmsStatusString[0] = "-FR"[saCmsOpmodel];

    if (saCmsFselMode == 0) {
        uint8_t band;
        uint8_t channel;
        vtxCommonGetBandAndChannel(device, &band, &channel);
        saCmsStatusString[2] = vtxCommonLookupBandLetter(device, band);
        saCmsStatusString[3] = vtxCommonLookupChannelName(device, channel)[0];
    } else {
        saCmsStatusString[2] = 'U';
        saCmsStatusString[3] = 'F';
    }

    if ((saDevice.mode & SA_MODE_GET_PITMODE)
        && (saDevice.mode & SA_MODE_GET_OUT_RANGE_PITMODE)) {
        tfp_sprintf(&saCmsStatusString[5], "%4d", saDevice.orfreq);
    } else {
        uint16_t freq = 0;
        vtxCommonGetFrequency(device, &freq);
        tfp_sprintf(&saCmsStatusString[5], "%4d", freq);
    }

    saCmsStatusString[9] = ' ';

    if (saDevice.mode & SA_MODE_GET_PITMODE) {
        saCmsPit = 2;
        saCmsStatusString[10] = 'P';
        if (saDevice.mode & SA_MODE_GET_OUT_RANGE_PITMODE) {
            saCmsStatusString[11] = 'O';
        } else {
            saCmsStatusString[11] = 'I';
        }
        saCmsStatusString[12] = 'R';
        saCmsStatusString[13] = 0;
    } else {
        saCmsPit = 1;
        uint8_t powerIndex = 0;
        bool powerFound = vtxCommonGetPowerIndex(device, &powerIndex);
        tfp_sprintf(&saCmsStatusString[10], "%s", powerFound ? vtxCommonLookupPowerName(device, powerIndex) : "???");
    }

    if (vtxTableBandCount == 0 || vtxTablePowerLevels == 0) {
        strncpy(saCmsStatusString, "PLEASE CONFIGURE VTXTABLE", sizeof(saCmsStatusString));
    }
}

/**********************************************************************
函数名称：saCmsResetOpmodel
函数功能：smartaudio重置操作码
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
void saCmsResetOpmodel()
{
    // 在'saCmsUpdate()'中触发数据刷新
    saCmsOpmodel = SACMS_OPMODEL_UNDEF;
}

/**********************************************************************
函数名称：saCmsConfigBandByGvar
函数功能：smartaudio配置频组
函数形参：pDisp，self
函数返回值：NULL 
函数描述：None 
**********************************************************************/
static const void *saCmsConfigBandByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (saDevice.version == 0) {
        // Bounce back; not online yet
        saCmsBand = 0;
        return NULL;
    }

    if (saCmsBand == 0) {
        // Bouce back, no going back to undef state
        saCmsBand = 1;
        return NULL;
    }

    if ((saCmsOpmodel == SACMS_OPMODEL_FREE) && !saDeferred) {
        vtxCommonSetBandAndChannel(vtxCommonDevice(), saCmsBand, saCmsChan);
    }

    saCmsFreqRef = vtxCommonLookupFrequency(vtxCommonDevice(), saCmsBand, saCmsChan);

    return NULL;
}

/**********************************************************************
函数名称：saCmsConfigChanByGvar
函数功能：smartaudio配置频点
函数形参：pDisp，self
函数返回值：NULL 
函数描述：None 
**********************************************************************/
static const void *saCmsConfigChanByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (saDevice.version == 0) {
        // Bounce back; not online yet
        saCmsChan = 0;
        return NULL;
    }

    if (saCmsChan == 0) {
        // Bounce back; no going back to undef state
        saCmsChan = 1;
        return NULL;
    }

    if ((saCmsOpmodel == SACMS_OPMODEL_FREE) && !saDeferred) {
        vtxCommonSetBandAndChannel(vtxCommonDevice(), saCmsBand, saCmsChan);
    }

    saCmsFreqRef = vtxCommonLookupFrequency(vtxCommonDevice(), saCmsBand, saCmsChan);

    return NULL;
}

/**********************************************************************
函数名称：saCmsConfigPitByGvar
函数功能：smartaudio配置PIT
函数形参：pDisp，self
函数返回值：NULL 
函数描述：None 
**********************************************************************/
static const void *saCmsConfigPitByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    dprintf(("saCmsConfigPitByGvar: saCmsPit %d\r\n", saCmsPit));

    if (saDevice.version == 0) {
        // Bounce back; not online yet
        saCmsPit = 0;
        return NULL;
    }

    if (saCmsPit == 0) {               // 试图回到undef状态;反弹
        saCmsPit = 1;
        return NULL;
    }
    if (saDevice.power != saCmsPower) {// 我们不能同时改变功率和PIT模式
        saCmsPower = saDevice.power;
    }

    return NULL;
}

/**********************************************************************
函数名称：saCmsConfigPowerByGvar
函数功能：smartaudio配置功率
函数形参：pDisp，self
函数返回值：NULL 
函数描述：None 
**********************************************************************/
static const void *saCmsConfigPowerByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (saDevice.version == 0) {
        // Bounce back; not online yet
        saCmsPower = 0;
        return NULL;
    }

    if (saCmsPower == 0) {
        // Bouce back; no going back to undef state
        saCmsPower = 1;
        return NULL;
    }

    if (saCmsPit > 0 && saCmsPit != 1 ) {
        saCmsPit = 1;
    }

    if (saCmsOpmodel == SACMS_OPMODEL_FREE && !saDeferred) {
        vtxSettingsConfigMutable()->power = saCmsPower;
    }
    dprintf(("saCmsConfigPowerByGvar: power index is now %d\r\n", saCmsPower));

    return NULL;
}

static OSD_TAB_t saCmsEntBand;
static OSD_TAB_t saCmsEntChan;
static OSD_TAB_t saCmsEntPower;

/**********************************************************************
函数名称：saCmsInitNames
函数功能：smartaudio初始化名称
函数形参：None 
函数返回值：None 
函数描述：None 
**********************************************************************/
static void saCmsInitNames(void)
{
    saCmsEntBand.val = &saCmsBand;
    saCmsEntBand.max = vtxTableBandCount;
    saCmsEntBand.names = vtxTableBandNames;

    saCmsEntChan.val = &saCmsChan;
    saCmsEntChan.max = vtxTableChannelCount;
    saCmsEntChan.names = vtxTableChannelNames;

    saCmsEntPower.val = &saCmsPower;
    saCmsEntPower.max = vtxTablePowerLevels;
    saCmsEntPower.names = vtxTablePowerLabels;
}

static OSD_UINT16_t saCmsEntFreqRef = { &saCmsFreqRef, 5600, 5900, 0 };

static const char * const saCmsPitNames[] = {
    "---",
    "OFF",
    "ON ",
};


static OSD_TAB_t saCmsEntPit = {&saCmsPit, 2, saCmsPitNames};

static const void *sacms_SetupTopMenu(displayPort_t *pDisp); // Forward

/**********************************************************************
函数名称：saCmsCommence
函数功能：smartaudio SET相关操作
函数形参：pDisp，self
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *saCmsCommence(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

	// 获取最新设置
    const vtxSettingsConfig_t prevSettings = {
        .band = vtxSettingsConfig()->band,
        .channel = vtxSettingsConfig()->channel,
        .freq = vtxSettingsConfig()->freq,
        .power = vtxSettingsConfig()->power,
        .lowPowerDisarm = vtxSettingsConfig()->lowPowerDisarm,
    };
    vtxSettingsConfig_t newSettings = prevSettings;

    if (saCmsOpmodel == SACMS_OPMODEL_RACE) {
        // Race model
        // Setup band, freq and power.

        newSettings.band = saCmsBand;
        newSettings.channel = saCmsChan;
        newSettings.freq = vtxCommonLookupFrequency(vtxCommonDevice(), saCmsBand, saCmsChan);
    } else {
        // Freestyle model
        // Setup band and freq / user freq
        if (saCmsFselModeNew == 0) {
            newSettings.band = saCmsBand;
            newSettings.channel = saCmsChan;
            newSettings.freq = vtxCommonLookupFrequency(vtxCommonDevice(), saCmsBand, saCmsChan);
        } else {
            saSetMode(0);    //make sure FREE mode is setup
            newSettings.band = 0;
            newSettings.freq = saCmsUserFreq;
        }
    }

    if (newSettings.power == saCmsPower && saCmsPit > 0) {
        vtxCommonSetPitMode(vtxCommonDevice(), saCmsPit == 2);
    }
    newSettings.power = saCmsPower;

    if (memcmp(&prevSettings, &newSettings, sizeof(vtxSettingsConfig_t))) {
        vtxSettingsConfigMutable()->band = newSettings.band;
        vtxSettingsConfigMutable()->channel = newSettings.channel;
        vtxSettingsConfigMutable()->power = newSettings.power;
        vtxSettingsConfigMutable()->freq = newSettings.freq;
        saveConfigAndNotify();
    }

    return MENU_CHAIN_BACK;
}

// 开始菜单管理内容
static const OSD_Entry saCmsMenuCommenceEntries[] = {
    { "CONFIRM", OME_Label,   NULL,          NULL, 0 },
    { "YES",     OME_Funcall, saCmsCommence, NULL, 0 },
    { "NO",    OME_Back, NULL, NULL, 0 },
    { NULL,      OME_END, NULL, NULL, 0 }
};

// 开始菜单配置
static CMS_Menu saCmsMenuCommence = {
    .onEnter = NULL,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = saCmsMenuCommenceEntries,
};

// 顶层菜单管理内容 - 设置通道模式
static const OSD_Entry saCmsMenuChanModeEntries[] =
{
    { "- SMARTAUDIO -", OME_Label, NULL, NULL, 0 },

    { "",       OME_Label,   NULL,                   saCmsStatusString,  DYNAMIC },
    { "BAND",   OME_TAB,     saCmsConfigBandByGvar,  &saCmsEntBand,      0 },
    { "CHAN",   OME_TAB,     saCmsConfigChanByGvar,  &saCmsEntChan,      0 },
    { "(FREQ)", OME_UINT16,  NULL,                   &saCmsEntFreqRef,   DYNAMIC },
    { "POWER",  OME_TAB,     saCmsConfigPowerByGvar, &saCmsEntPower,     DYNAMIC },
    { "PIT",    OME_TAB,     saCmsConfigPitByGvar,   &saCmsEntPit,     DYNAMIC },
    { "SET",    OME_Submenu, cmsMenuChange,          &saCmsMenuCommence, 0 },

    { "BACK",   OME_Back, NULL, NULL, 0 },
    { NULL,     OME_END, NULL, NULL, 0 }
};

// 顶层菜单管理内容 - 离线
static const OSD_Entry saCmsMenuOfflineEntries[] =
{
    { "- VTX SMARTAUDIO -", OME_Label, NULL, NULL, 0 },
    { "",      OME_Label,   NULL,          saCmsStatusString, DYNAMIC },
    { "BACK",  OME_Back, NULL, NULL, 0 },
    { NULL,    OME_END, NULL, NULL, 0 }
};

/**********************************************************************
函数名称：sacms_SetupTopMenu
函数功能：设置顶部菜单
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
CMS_Menu cmsx_menuVtxSmartAudio; 				// Forward
static const void *sacms_SetupTopMenu(displayPort_t *pDisp)
{
    UNUSED(pDisp);
    cmsx_menuVtxSmartAudio.entries = saCmsMenuChanModeEntries;
    saCmsInitNames();
    return NULL;
}

// 黑羊图传协议菜单配置
CMS_Menu cmsx_menuVtxSmartAudio = {
    .onEnter = sacms_SetupTopMenu,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = saCmsMenuOfflineEntries,
};
#endif // CMS

