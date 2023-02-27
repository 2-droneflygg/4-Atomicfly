#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>

#include "platform.h"

#if defined(USE_CMS) && defined(USE_VTX_CONTROL) && (defined(USE_VTX_TRAMP) || defined(USE_VTX_SMARTAUDIO) || defined(USE_VTX_RTC6705))
#include "common/printf.h"

#include "cms/cms.h"
#include "cms/cms_menu_vtx_smartaudio.h"
#include "cms/cms_types.h"

#include "drivers/vtx_common.h"

#include "cms_menu_vtx_common.h"


#define MAX_STATUS_LINE_LENGTH 21

static char statusLine1[MAX_STATUS_LINE_LENGTH] = "";
static char statusLine2[MAX_STATUS_LINE_LENGTH] = "";

/**********************************************************************
函数名称：setStatusMessage
函数功能：设置状态信息
函数形参：pDisp
函数返回值：NULL
函数描述：None 
**********************************************************************/
static const void *setStatusMessage(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    vtxDevice_t *device = vtxCommonDevice();

    statusLine1[0] = 0;
    statusLine2[0] = 0;

    if (!device) {
        tfp_sprintf(&statusLine1[0], "VTX NOT RESPONDING");
        tfp_sprintf(&statusLine2[0], "OR NOT CONFIGURED");
    } else {
        vtxDevType_e vtxType = vtxCommonGetDeviceType(device);
        if (vtxType == VTXDEV_UNSUPPORTED) {
            tfp_sprintf(&statusLine1[0], "UNSUPPORTED VTX TYPE");
        } else {
            tfp_sprintf(&statusLine1[0], "UNKNOWN VTX TYPE");
        }
    }
    return NULL;
}

// vtxError菜单管理内容
static const OSD_Entry vtxErrorMenuEntries[] =
{
    { "",     OME_Label, NULL, statusLine1,  DYNAMIC },
    { "",     OME_Label, NULL, statusLine2,  DYNAMIC },
    { "",     OME_Label, NULL, NULL, 0 },
    { "BACK", OME_Back,  NULL, NULL, 0 },
    { NULL,   OME_END,   NULL, NULL, 0 }
};

// vtxError菜单配置
static CMS_Menu cmsx_menuVtxError = {
    .onEnter = setStatusMessage,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = vtxErrorMenuEntries,
};

/**********************************************************************
函数名称：cmsSelectVtx
函数功能：根据vtx设备类型重定向到适当的菜单
函数形参：pDisp
函数返回值：NULL
函数描述：
	如果设备不是有效的或不支持的类型，那么不要重定向，而不是显示本地信息菜单
**********************************************************************/
const void *cmsSelectVtx(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(ptr);

    vtxDevice_t *device = vtxCommonDevice();

    if (device) {
        vtxDevType_e vtxType = vtxCommonGetDeviceType(device);

        switch (vtxType) {
#if defined(USE_VTX_SMARTAUDIO)
        case VTXDEV_SMARTAUDIO:
            cmsMenuChange(pDisplay, &cmsx_menuVtxSmartAudio);
            break;
#endif
        default:
            cmsMenuChange(pDisplay, &cmsx_menuVtxError);
            break;
        }
    } else {
        cmsMenuChange(pDisplay, &cmsx_menuVtxError);
    }

    return NULL;
}
#endif

