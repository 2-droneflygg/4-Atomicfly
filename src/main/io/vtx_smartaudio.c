#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include "platform.h"

#if defined(USE_VTX_SMARTAUDIO) && defined(USE_VTX_CONTROL)

#include "cms/cms.h"
#include "cms/cms_menu_vtx_smartaudio.h"

#include "common/maths.h"
#include "common/printf.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"

#include "io/serial.h"
#include "io/vtx.h"
#include "io/vtx_control.h"
#include "io/vtx_smartaudio.h"


// 定时参数
// 注意vtxSAProcess()通常以200ms的间隔调用
#define SMARTAUDIO_CMD_TIMEOUT       120    // 直到该命令被认为已丢失
#define SMARTAUDIO_POLLING_INTERVAL  150    // SMARTAUDIO最小轮询间隔
#define SMARTAUDIO_POLLING_WINDOW   1000    // 命令轮询状态更改后的时间窗口

static serialPort_t *smartAudioSerialPort = NULL;

smartAudioDevice_t saDevice;

// vtxSmartAudio
#ifdef USE_VTX_COMMON
static const vtxVTable_t saVTable;          // Forward
static vtxDevice_t vtxSmartAudio = {
    .vTable = &saVTable,
};
#endif

/* -----------------------smartaaudio命令和响应代码枚举----------------------- */	
enum {
    SA_CMD_NONE = 0x00,
    SA_CMD_GET_SETTINGS = 0x01,
    SA_CMD_SET_POWER,
    SA_CMD_SET_CHAN,
    SA_CMD_SET_FREQ,
    SA_CMD_SET_MODE,
    SA_CMD_GET_SETTINGS_V2 = 0x09,          // 只响应
    SA_CMD_GET_SETTINGS_V21 = 0x11,
} smartAudioCommand_e;

// 这不是一个好的设计;这样不能区分命令和响应
#define SACMD(cmd) (((cmd) << 1) | 1)

#define SA_IS_PITMODE(n) ((n) & SA_MODE_GET_PITMODE)
#define SA_IS_PIRMODE(n) (((n) & SA_MODE_GET_PITMODE) && ((n) & SA_MODE_GET_IN_RANGE_PITMODE))
#define SA_IS_PORMODE(n) (((n) & SA_MODE_GET_PITMODE) && ((n) & SA_MODE_GET_OUT_RANGE_PITMODE))

// 在“saDevice之间进行转换。通道和波段/通道值
#define SA_DEVICE_CHVAL_TO_BAND(val) ((val) / (uint8_t)vtxTableChannelCount) + 1
#define SA_DEVICE_CHVAL_TO_CHANNEL(val) ((val) % (uint8_t)vtxTableChannelCount) + 1
#define SA_BANDCHAN_TO_DEVICE_CHVAL(band, channel) ((band - 1) * (uint8_t)vtxTableChannelCount + (channel - 1))

/* ----------------------统计计数器，用于用户侧的故障排除--------------------- */	
smartAudioStat_t saStat = {
    .pktsent = 0,
    .pktrcvd = 0,
    .badpre = 0,
    .badlen = 0,
    .crc = 0,
    .ooopresp = 0,
    .badcode = 0,
};

/* ------------------------最后一次接收设备('hard')状态----------------------- */	
smartAudioDevice_t saDevice = {
    .version = 0,
    .channel = -1,
    .power = 0,
    .mode = 0,
    .freq = 0,
    .orfreq = 0,
    .willBootIntoPitMode = false,
};
	
/* ------------------------------smartaaudio设备------------------------------ */	
static smartAudioDevice_t saDevicePrev = {
    .version = 0,
};

static uint8_t saLockMode = SA_MODE_SET_UNLOCK;

#define VTX_SMARTAUDIO_POWER_COUNT   4
static char *saSupportedPowerLabels[VTX_SMARTAUDIO_POWER_COUNT + 1] = {"---", "25 ", "200", "500", "800"};
static char *saSupportedPowerLabelPointerArray[VTX_SMARTAUDIO_POWER_COUNT + 1];
static uint8_t saSupportedNumPowerLevels = VTX_SMARTAUDIO_POWER_COUNT;
static uint16_t saSupportedPowerValues[VTX_SMARTAUDIO_POWER_COUNT];

bool saDeferred = true; 

// 接收帧重组缓冲区
#define SA_MAX_RCVLEN    21
static uint8_t sa_rbuf[SA_MAX_RCVLEN + 4]; // 删除4字节保护

/**********************************************************************
函数名称：CRC8
函数功能：CRC8计算
函数形参：data，len
函数返回值：crc
函数描述：None
**********************************************************************/
#define POLYGEN 0xd5
static uint8_t CRC8(const uint8_t *data, const int8_t len)
{
    uint8_t crc = 0; /* start with 0 so first byte can be 'xored' in */
    uint8_t currByte;

    for (int i = 0 ; i < len ; i++) {
        currByte = data[i];

        crc ^= currByte; /* XOR-in the next input byte */

        for (int i = 0; i < 8; i++) {
            if ((crc & 0x80) != 0) {
                crc = (uint8_t)((crc << 1) ^ POLYGEN);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**********************************************************************
函数名称：saAutobaud
函数功能：Smartaaudio自动波特率
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
#define SMARTBAUD_MIN 4800
#define SMARTBAUD_MAX 4950
uint16_t sa_smartbaud = SMARTBAUD_MIN;
static int sa_adjdir = 1;    // -1=下降，1=上升
static int sa_baudstep = 50;
static void saAutobaud(void)
{
    if (saStat.pktsent < 10) {
        // 样本收集不足
        return;
    }

    if (((saStat.pktrcvd * 100) / saStat.pktsent) >= 70) {
        // This is okay
        saStat.pktsent = 0; // Should be more moderate?
        saStat.pktrcvd = 0;
        return;
    }

    dprintf(("autobaud: adjusting\r\n"));

    if ((sa_adjdir == 1) && (sa_smartbaud == SMARTBAUD_MAX)) {
        sa_adjdir = -1;
        dprintf(("autobaud: now going down\r\n"));
    } else if ((sa_adjdir == -1 && sa_smartbaud == SMARTBAUD_MIN)) {
        sa_adjdir = 1;
        dprintf(("autobaud: now going up\r\n"));
    }

    sa_smartbaud += sa_baudstep * sa_adjdir;

    dprintf(("autobaud: %d\r\n", sa_smartbaud));

    smartAudioSerialPort->vTable->serialSetBaudRate(smartAudioSerialPort, sa_smartbaud);

    saStat.pktsent = 0;
    saStat.pktrcvd = 0;
}

// 传输层变量
static timeUs_t sa_lastTransmissionMs = 0;
static uint8_t sa_outstanding = SA_CMD_NONE; // 未处理命令
static uint8_t sa_osbuf[32]; 				 // 重传命令帧
static int sa_oslen;         				 // 和关联长度

/**********************************************************************
函数名称：saProcessResponse
函数功能：Smartaaudio进程响应
函数形参：buf，len
函数返回值：None
函数描述：None
**********************************************************************/
static void saProcessResponse(uint8_t *buf, int len)
{
    uint8_t resp = buf[0];

    if (resp == sa_outstanding) {
        sa_outstanding = SA_CMD_NONE;
    } else if ((resp == SA_CMD_GET_SETTINGS_V2 ||
                resp == SA_CMD_GET_SETTINGS_V21) &&
               (sa_outstanding == SA_CMD_GET_SETTINGS)) {
        sa_outstanding = SA_CMD_NONE;
    } else {
        saStat.ooopresp++;
        dprintf(("processResponse: outstanding %d got %d\r\n", sa_outstanding, resp));
    }

    switch (resp) {
    case SA_CMD_GET_SETTINGS_V21: // Version 2.1 Get Settings
    case SA_CMD_GET_SETTINGS_V2:  // Version 2 Get Settings
    case SA_CMD_GET_SETTINGS:     // Version 1 Get Settings
        dprintf(("received settings\r\n"));
        if (len < 7) {
            break;
        }

		// From spec: "Bit 7-3持有智能音频版本，0是V1, 1是V2, 2是V2.1"
		// saDevice.version = 0表示未知，1表示智能音频V1, 2表示智能音频V2, 3表示智能音频V2.1
        saDevice.version = (buf[0] == SA_CMD_GET_SETTINGS) ? 1 : ((buf[0] == SA_CMD_GET_SETTINGS_V2) ? 2 : 3);
        saDevice.channel = buf[2];
        uint8_t rawPowerValue = buf[3];
        saDevice.mode = buf[4];
        saDevice.freq = (buf[5] << 8) | buf[6];

        // read pir and por flags to detect if the device will boot into pitmode.
        // note that "quit pitmode without unsetting the pitmode flag" clears pir and por flags but the device will still boot into pitmode.
        // therefore we ignore the pir and por flags while the device is not in pitmode
        // actually, this is the whole reason the variable saDevice.willBootIntoPitMode exists.
        // otherwise we could use saDevice.mode directly
        if (saDevice.mode & SA_MODE_GET_PITMODE) {
            bool newBootMode = (saDevice.mode & SA_MODE_GET_IN_RANGE_PITMODE) || (saDevice.mode & SA_MODE_GET_OUT_RANGE_PITMODE);
            if (newBootMode != saDevice.willBootIntoPitMode) {
                dprintf(("saProcessResponse: willBootIntoPitMode is now %s\r\n", newBootMode ? "true" : "false"));
            }
            saDevice.willBootIntoPitMode = newBootMode;
        }

        if (saDevice.version == 3) {
            //read dbm based power levels
            //aaaaaand promptly forget them. todo: write them into vtxtable pg and load it
            if (len < 10) { //current power level in dbm field missing or power level length field missing or zero power levels reported
                dprintf(("processResponse: V2.1 vtx didn't report any power levels\r\n"));
                break;
            }
            saSupportedNumPowerLevels = constrain((int8_t)buf[8], 0, VTX_TABLE_MAX_POWER_LEVELS);
            //SmartAudio seems to report buf[8] + 1 power levels, but one of them is zero.
            //zero is indeed a valid power level to set the vtx to, but it activates pit mode.
            //crucially, after sending 0 dbm, the vtx does NOT report its power level to be 0 dbm.
            //instead, it reports whatever value was set previously and it reports to be in pit mode.
            //for this reason, zero shouldn't be used as a normal power level in betaflight.
            for ( int8_t i = 0; i < saSupportedNumPowerLevels; i++ ) {
                saSupportedPowerValues[i] = buf[9 + i + 1];//+ 1 to skip the first power level, as mentioned above
            }

            dprintf(("processResponse: %d power values: %d, %d, %d, %d\r\n",
                     vtxTablePowerLevels, vtxTablePowerValues[0], vtxTablePowerValues[1],
                     vtxTablePowerValues[2], vtxTablePowerValues[3]));
            //dprintf(("processResponse: V2.1 received vtx power value %d\r\n",buf[7]));
            rawPowerValue = buf[7];
        }
        saDevice.power = 0;//set to unknown power level if the reported one doesnt match any of the known ones
        dprintf(("processResponse: rawPowerValue is %d, legacy power is %d\r\n", rawPowerValue, buf[3]));
        for (int8_t i = 0; i < vtxTablePowerLevels; i++) {
            if (rawPowerValue == vtxTablePowerValues[i]) {
                saDevice.power = i + 1;

            }
        }
        break;

    case SA_CMD_SET_POWER: // Set Power
        break;

    case SA_CMD_SET_CHAN: // Set Channel
        break;

    case SA_CMD_SET_FREQ: // Set Frequency
        if (len < 5) {
            break;
        }

        const uint16_t freq = (buf[2] << 8) | buf[3];

        if (freq & SA_FREQ_GETPIT) {
            saDevice.orfreq = freq & SA_FREQ_MASK;
            dprintf(("saProcessResponse: GETPIT freq %d\r\n", saDevice.orfreq));
        } else if (freq & SA_FREQ_SETPIT) {
            saDevice.orfreq = freq & SA_FREQ_MASK;
            dprintf(("saProcessResponse: SETPIT freq %d\r\n", saDevice.orfreq));
        } else {
            saDevice.freq = freq;
            dprintf(("saProcessResponse: SETFREQ freq %d\r\n", freq));
        }
        break;

    case SA_CMD_SET_MODE: // Set Mode
        dprintf(("saProcessResponse: SET_MODE 0x%x (pir %s, por %s, pitdsbl %s, %s)\r\n",
                 buf[2], (buf[2] & 1) ? "on" : "off", (buf[2] & 2) ? "on" : "off", (buf[3] & 4) ? "on" : "off",
                 (buf[4] & 8) ? "unlocked" : "locked"));
        break;

    default:
        saStat.badcode++;
        return;
    }

    if (memcmp(&saDevice, &saDevicePrev, sizeof(smartAudioDevice_t))) {
#ifdef USE_CMS    //if changes then trigger saCms update
        saCmsResetOpmodel();
#endif
    }
    saDevicePrev = saDevice;

#ifdef USE_VTX_COMMON
    // Todo: Update states in saVtxDevice?
#endif

#ifdef USE_CMS
    // Export current device status for CMS
    saCmsUpdate();
    saUpdateStatusString();
#endif
}

/**********************************************************************
函数名称：saReceiveFrame
函数功能：Smartaaudio接收帧
函数形参：c
函数返回值：None
函数描述：None
**********************************************************************/
static void saReceiveFrame(uint8_t c)
{

    static enum saFramerState_e {
        S_WAITPRE1, // Waiting for preamble 1 (0xAA)
        S_WAITPRE2, // Waiting for preamble 2 (0x55)
        S_WAITRESP, // Waiting for response code
        S_WAITLEN,  // Waiting for length
        S_DATA,     // Receiving data
        S_WAITCRC,  // Waiting for CRC
    } state = S_WAITPRE1;

    static int len;
    static int dlen;

    switch (state) {
    case S_WAITPRE1:
        if (c == 0xAA) {
            state = S_WAITPRE2;
        } else {
            state = S_WAITPRE1; // Don't need this (no change)
        }
        break;

    case S_WAITPRE2:
        if (c == 0x55) {
            state = S_WAITRESP;
        } else {
            saStat.badpre++;
            state = S_WAITPRE1;
        }
        break;

    case S_WAITRESP:
        sa_rbuf[0] = c;
        state = S_WAITLEN;
        break;

    case S_WAITLEN:
        sa_rbuf[1] = c;
        len = c;

        if (len > SA_MAX_RCVLEN - 2) {
            saStat.badlen++;
            state = S_WAITPRE1;
        } else if (len == 0) {
            state = S_WAITCRC;
        } else {
            dlen = 0;
            state = S_DATA;
        }
        break;

    case S_DATA:
        // XXX Should check buffer overflow (-> saerr_overflow)
        sa_rbuf[2 + dlen] = c;
        if (++dlen == len) {
            state = S_WAITCRC;
        }
        break;

    case S_WAITCRC:
        if (CRC8(sa_rbuf, 2 + len) == c) {
            // Got a response
            saProcessResponse(sa_rbuf, len + 2);
            saStat.pktrcvd++;
        } else if (sa_rbuf[0] & 1) {
            // Command echo
            // XXX There is an exceptional case (V2 response)
            // XXX Should check crc in the command format?
        } else {
            saStat.crc++;
        }
        state = S_WAITPRE1;
        break;
    }
}

/**********************************************************************
函数名称：saSendFrame
函数功能：Smartaaudio发送帧
函数形参：buf，len
函数返回值：None
函数描述：None
**********************************************************************/
static void saSendFrame(uint8_t *buf, int len)
{
    switch (smartAudioSerialPort->identifier) {
    case SERIAL_PORT_SOFTSERIAL1:
    case SERIAL_PORT_SOFTSERIAL2:
        break;
    default:
        serialWrite(smartAudioSerialPort, 0x00); // Generate 1st start bit
        break;
    }

    for (int i = 0 ; i < len ; i++) {
        serialWrite(smartAudioSerialPort, buf[i]);
    }

    saStat.pktsent++;
 

    sa_lastTransmissionMs = millis();
}

/**********************************************************************
函数名称：saResendCmd
函数功能：Smartaaudio重新发送命令
函数形参：buf，len
函数返回值：None
函数描述：
 重传和命令排队：
	传输级别支持包括响应超时时的重传和命令队列。

 重新发送缓冲区:
	smartaaudio返回有效命令帧的响应超过60毫秒，所以需要重发缓冲区

 命令排队:
	驱动程序自动发送GetSettings命令自动bauding，
	异步到用户发起的命令;另一个命令发出时未执行的命令必须排队等待后续处理
	队列也处理多个命令的情况需要实现一个用户级命令
**********************************************************************/
static void saResendCmd(void)
{
    saSendFrame(sa_osbuf, sa_oslen);
}

/**********************************************************************
函数名称：saSendCmd
函数功能：Smartaaudio发送命令
函数形参：buf，len
函数返回值：None
函数描述：None
**********************************************************************/
static void saSendCmd(uint8_t *buf, int len)
{
    for (int i = 0 ; i < len ; i++) {
        sa_osbuf[i] = buf[i];
    }

    sa_oslen = len;
    sa_outstanding = (buf[2] >> 1);

    saSendFrame(sa_osbuf, sa_oslen);
}

/* -----------------------smartaaudio命令队列管理结构体----------------------- */	
typedef struct saCmdQueue_s {
    uint8_t *buf;
    int len;
} saCmdQueue_t;

#define SA_QSIZE 6       // 1心跳(GetSettings) + 2命令+ 1 slack
static saCmdQueue_t sa_queue[SA_QSIZE];
static uint8_t sa_qhead = 0;
static uint8_t sa_qtail = 0;

/**********************************************************************
函数名称：saQueueEmpty
函数功能：Smartaaudio队列为空
函数形参：None
函数返回值：状态
函数描述：None
**********************************************************************/
static bool saQueueEmpty(void)
{
    return sa_qhead == sa_qtail;
}

/**********************************************************************
函数名称：saQueueFull
函数功能：Smartaaudio队列满
函数形参：None
函数返回值：状态
函数描述：None
**********************************************************************/
static bool saQueueFull(void)
{
    return ((sa_qhead + 1) % SA_QSIZE) == sa_qtail;
}

/**********************************************************************
函数名称：saQueueCmd
函数功能：Smartaaudio队列命令
函数形参：buf，len
函数返回值：状态
函数描述：None
**********************************************************************/
static void saQueueCmd(uint8_t *buf, int len)
{
    if (saQueueFull()) {
        return;
    }
    sa_queue[sa_qhead].buf = buf;
    sa_queue[sa_qhead].len = len;
    sa_qhead = (sa_qhead + 1) % SA_QSIZE;
}

/**********************************************************************
函数名称：saSendQueue
函数功能：Smartaaudio发送队列
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
static void saSendQueue(void)
{
    if (saQueueEmpty()) {
        return;
    }

    saSendCmd(sa_queue[sa_qtail].buf, sa_queue[sa_qtail].len);
    sa_qtail = (sa_qtail + 1) % SA_QSIZE;
}

/**********************************************************************
函数名称：saGetSettings
函数功能：Smartaaudio获取设置
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
static void saGetSettings(void)
{
    static uint8_t bufGetSettings[5] = {0xAA, 0x55, SACMD(SA_CMD_GET_SETTINGS), 0x00, 0x9F};

    dprintf(("smartAudioGetSettings\r\n"));
    saQueueCmd(bufGetSettings, 5);
}

/**********************************************************************
函数名称：saValidateFreq
函数功能：Smartaaudio验证频率
函数形参：freq
函数返回值：result
函数描述：None
**********************************************************************/
static bool saValidateFreq(uint16_t freq)
{
    return (freq >= VTX_SMARTAUDIO_MIN_FREQUENCY_MHZ && freq <= VTX_SMARTAUDIO_MAX_FREQUENCY_MHZ);
}

/**********************************************************************
函数名称：saSetFreq
函数功能：Smartaaudio设置频率
函数形参：freq
函数返回值：None
函数描述：None
**********************************************************************/
void saSetFreq(uint16_t freq)
{
    static uint8_t buf[7] = { 0xAA, 0x55, SACMD(SA_CMD_SET_FREQ), 2 };
    static uint8_t switchBuf[7];

    if (freq & SA_FREQ_GETPIT) {
        dprintf(("smartAudioSetFreq: GETPIT\r\n"));
    } else if (freq & SA_FREQ_SETPIT) {
        dprintf(("smartAudioSetFreq: SETPIT %d\r\n", freq & SA_FREQ_MASK));
    } else {
        dprintf(("smartAudioSetFreq: SET %d\r\n", freq));
    }

    buf[4] = (freq >> 8) & 0xff;
    buf[5] = freq & 0xff;
    buf[6] = CRC8(buf, 6);

	// 从“channel”切换到“smartaaudio”时，需要解决明显的smartaaudio bug
	// 到'user-freq'模式，其中set-freq命令将失败，如果频率值与之前的'user-freq'模式相同
    if ((saDevice.mode & SA_MODE_GET_FREQ_BY_FREQ) == 0 && freq == saDevice.freq) {
        memcpy(&switchBuf, &buf, sizeof(buf));
        const uint16_t switchFreq = freq + ((freq == VTX_SMARTAUDIO_MAX_FREQUENCY_MHZ) ? -1 : 1);
        switchBuf[4] = (switchFreq >> 8);
        switchBuf[5] = switchFreq & 0xff;
        switchBuf[6] = CRC8(switchBuf, 6);

        saQueueCmd(switchBuf, 7);

        // 需要在“set”命令之间执行“get”，以保持对变量的同步跟踪
        saGetSettings();
    }

    saQueueCmd(buf, 7);
}

/**********************************************************************
函数名称：saSetPitFreq
函数功能：Smartaaudio设置PIT频率
函数形参：freq
函数返回值：None
函数描述：None
**********************************************************************/
void saSetPitFreq(uint16_t freq)
{
    saSetFreq(freq | SA_FREQ_SETPIT);
}

/**********************************************************************
函数名称：saSetMode
函数功能：Smartaaudio设置模式
函数形参：mode
函数返回值：None
函数描述：None
**********************************************************************/
void saSetMode(int mode)
{
    static uint8_t buf[6] = { 0xAA, 0x55, SACMD(SA_CMD_SET_MODE), 1 };

    buf[4] = (mode & 0x3f) | saLockMode;
    if (saDevice.version >= 3 && (mode & SA_MODE_CLR_PITMODE) &&
        ((mode & SA_MODE_SET_IN_RANGE_PITMODE) || (mode & SA_MODE_SET_OUT_RANGE_PITMODE))) {
        saDevice.willBootIntoPitMode = true;// 退出pitmode而不取消设置标志。
		// 响应只会说pit=off，但是设备在重启时仍然会进入pitmode
		// 因此我们必须记住这个变化
    }
    dprintf(("saSetMode(0x%x): pir=%s por=%s pitdsbl=%s %s\r\n", mode, (mode & 1) ? "on " : "off", (mode & 2) ? "on " : "off",
            (mode & 4)? "on " : "off", (mode & 8) ? "locked" : "unlocked"));
    buf[5] = CRC8(buf, 5);

    saQueueCmd(buf, 6);
}

/**********************************************************************
函数名称：vtxSmartAudioInit
函数功能：Smartaaudio初始化
函数形参：mode
函数返回值：None
函数描述：None
**********************************************************************/
bool vtxSmartAudioInit(void)
{
    for (int8_t i = 0; i < VTX_SMARTAUDIO_POWER_COUNT + 1; i++) {
        saSupportedPowerLabelPointerArray[i] = saSupportedPowerLabels[i];
    }

	// 查找配置为SMARTAUDIO功能的串口
    const serialPortConfig_t *SmartaudioPortConfig = findSerialPortConfig(FUNCTION_VTX_SMARTAUDIO);
    if (!SmartaudioPortConfig) {
        return false;
    }
	
    portOptions_e portOptions = SERIAL_STOPBITS_2 | SERIAL_BIDIR_NOPULL;
#if defined(USE_VTX_COMMON)
    portOptions = portOptions | (vtxConfig()->halfDuplex ? SERIAL_BIDIR | SERIAL_BIDIR_PP : SERIAL_UNIDIR);
#endif

	// 打开串口并进行初始化 - 单线半双工
    smartAudioSerialPort = openSerialPort(SmartaudioPortConfig->identifier, FUNCTION_VTX_SMARTAUDIO, NULL, NULL, 4800, MODE_RXTX, portOptions);
    if (!smartAudioSerialPort) {
        return false;
    }

    for (int8_t i = 0; i < VTX_SMARTAUDIO_POWER_COUNT; i++) {
        saSupportedPowerValues[i] = 1;
    }

    dprintf(("vtxSmartAudioInit %d power levels recorded\r\n", vtxTablePowerLevels));

	// 设置设备
    vtxCommonSetDevice(&vtxSmartAudio);
	// 设置工厂频组
    vtxTableSetFactoryBands(true);
	// vtx初始化
    vtxInit();

    return true;
}

/**********************************************************************
函数名称：vtxSAProcess
函数功能：Smartaaudio进程
函数形参：vtxDevice，currentTimeUs
函数返回值：None
函数描述：None
**********************************************************************/
#define SA_INITPHASE_START         0
#define SA_INITPHASE_WAIT_SETTINGS 1 // 发送了SA_CMD_GET_SETTINGS并等待应答
#define SA_INITPHASE_WAIT_PITFREQ  2 // 频率GETPIT发送和等待应答
#define SA_INITPHASE_DONE          3
static void vtxSAProcess(vtxDevice_t *vtxDevice, timeUs_t currentTimeUs)
{
    UNUSED(vtxDevice);
    UNUSED(currentTimeUs);

    static char initPhase = SA_INITPHASE_START;

    if (smartAudioSerialPort == NULL) {
        return;
    }

    while (serialRxBytesWaiting(smartAudioSerialPort) > 0) {
        uint8_t c = serialRead(smartAudioSerialPort);
        saReceiveFrame((uint16_t)c);
    }

    // 每帧接收后重新评估波特率
    saAutobaud();

    switch (initPhase) {
    case SA_INITPHASE_START:
        saGetSettings();
        //saSendQueue();
        initPhase = SA_INITPHASE_WAIT_SETTINGS;
        break;

    case SA_INITPHASE_WAIT_SETTINGS:
		// 不发送SA_FREQ_GETPIT到V1设备;它作为普通的SA_CMD_SET_FREQ
		// 并将设备置于未初始化频率的用户频率模式，出于同样的原因，也不要将它发送到V2.1
        if (saDevice.version) {
            if (saDevice.version == 2) {
                saSetFreq(SA_FREQ_GETPIT);
                initPhase = SA_INITPHASE_WAIT_PITFREQ;
            } else {
                initPhase = SA_INITPHASE_DONE;
            }

            if (saDevice.version == 1) {
                saSupportedPowerValues[0] = 7;
                saSupportedPowerValues[1] = 16;
                saSupportedPowerValues[2] = 25;
                saSupportedPowerValues[3] = 40;
            } else if (saDevice.version == 2) {
                saSupportedPowerValues[0] = 0;
                saSupportedPowerValues[1] = 1;
                saSupportedPowerValues[2] = 2;
                saSupportedPowerValues[3] = 3;
            }

            // 如果没有USE_VTX_TABLE，用默认设置填充vtxTable变量(而不是从PG加载)
            vtxTablePowerLevels = constrain(saSupportedNumPowerLevels, 0, VTX_SMARTAUDIO_POWER_COUNT);
            if (saDevice.version >= 3) {
                for (int8_t i = 0; i < vtxTablePowerLevels; i++) {
                    //ideally we would convert dbm to mW here
                    tfp_sprintf(saSupportedPowerLabels[i + 1], "%3d", constrain(saSupportedPowerValues[i], 0, 999));
                }
            }
            for (int8_t i = 0; i < vtxTablePowerLevels; i++) {
                vtxTablePowerValues[i] = saSupportedPowerValues[i];
            }
			// 功率标签
            for (int8_t i = 0; i < vtxTablePowerLevels + 1; i++) {
                vtxTablePowerLabels[i] = saSupportedPowerLabels[i];
            }
            dprintf(("vtxSAProcess init phase vtxTablePowerLevels set to %d\r\n", vtxTablePowerLevels));

            if (saDevice.version >= 2 ) {
                saDevice.willBootIntoPitMode = (saDevice.mode & SA_MODE_GET_PITMODE) ? true : false;
                dprintf(("sainit: willBootIntoPitMode is %s\r\n", saDevice.willBootIntoPitMode ? "true" : "false"));
            }
        }
        break;

    case SA_INITPHASE_WAIT_PITFREQ:
        if (saDevice.orfreq) {
            initPhase = SA_INITPHASE_DONE;
        }
        break;

    case SA_INITPHASE_DONE:
        break;
    }

    //命令队列控制
    timeMs_t nowMs = millis();             // 不要用“currentTimeUs / 1000”代替;sa_lasttransmitms基于millis()。
    static timeMs_t lastCommandSentMs = 0; // 发送的最后一个非get设置

    if ((sa_outstanding != SA_CMD_NONE) && (nowMs - sa_lastTransmissionMs > SMARTAUDIO_CMD_TIMEOUT)) {
        // Last command timed out
        dprintf(("process: resending 0x%x\r\n", sa_outstanding));
        // XXX Todo: Resend termination and possible offline transition
        saResendCmd();
        lastCommandSentMs = nowMs;
    } else if (!saQueueEmpty()) {
        // Command pending. Send it.
        dprintf(("process: sending queue\r\n"));
        saSendQueue();
        lastCommandSentMs = nowMs;
    } else if ((nowMs - lastCommandSentMs < SMARTAUDIO_POLLING_WINDOW)
               && (nowMs - sa_lastTransmissionMs >= SMARTAUDIO_POLLING_INTERVAL)) {
        dprintf(("process: sending status change polling\r\n"));
        saGetSettings();
        saSendQueue();
    }
}

#ifdef USE_VTX_COMMON
// *************************************************************通用VTX_API接口
/**********************************************************************
函数名称：vtxSAGetDeviceType
函数功能：Smartaaudio获取设备类型
函数形参：vtxDevice
函数返回值：VTXDEV_SMARTAUDIO
函数描述：None
**********************************************************************/
vtxDevType_e vtxSAGetDeviceType(const vtxDevice_t *vtxDevice)
{
    UNUSED(vtxDevice);
    return VTXDEV_SMARTAUDIO;
}

/**********************************************************************
函数名称：vtxSAIsReady
函数功能：Smartaaudio是否就绪
函数形参：vtxDevice
函数返回值：状态
函数描述：None
**********************************************************************/
static bool vtxSAIsReady(const vtxDevice_t *vtxDevice)
{
    if (vtxDevice != NULL && saDevice.power == 0) {
        return false;
		// 等待功率读取存在
		// 在收到第一个设置后，powervalues被加载到vtxTableXXX中
		// 因此，当processResponse第一次运行它的索引查找时，它不会找到任何东西，也不会记录功率
		// 这个函数会一直等待，直到第二次接收到设置后才会发现一些东西
		// USE_VTX_TABLE不需要检查，因为在smartaaudio启动之前，该表就已经从pg加载了
		// 事实上，使用USE_VTX_TABLE，如果用户已经使用了USE_VTX_TABLE，那么这种检查可能会导致smartaaudio实现瘫痪
		// 选择忽略一个电源状态，但vtx碰巧在那个状态下，不管什么原因
    }
    return vtxDevice != NULL && saDevice.version != 0;
}

/**********************************************************************
函数名称：saValidateBandAndChannel
函数功能：Smartaaudio验证频组和通道
函数形参：band，channel
函数返回值：状态
函数描述：None
**********************************************************************/
static bool saValidateBandAndChannel(uint8_t band, uint8_t channel)
{
    return (band >= VTX_SMARTAUDIO_MIN_BAND && band <= vtxTableBandCount &&
            channel >= VTX_SMARTAUDIO_MIN_CHANNEL && channel <= vtxTableChannelCount);
}

/**********************************************************************
函数名称：vtxSASetBandAndChannel
函数功能：Smartaaudio设置频组和通道
函数形参：vtxDevice，band，channel
函数返回值：None
函数描述：None
**********************************************************************/
static void vtxSASetBandAndChannel(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel)
{
    UNUSED(vtxDevice);
    if (saValidateBandAndChannel(band, channel)) {
        static uint8_t buf[6] = { 0xAA, 0x55, SACMD(SA_CMD_SET_CHAN), 1 };

        buf[4] = SA_BANDCHAN_TO_DEVICE_CHVAL(band, channel);
        buf[5] = CRC8(buf, 5);
        dprintf(("vtxSASetBandAndChannel set index band %d channel %d value sent 0x%x\r\n", band, channel, buf[4]));

        // 这将清除saDevice，模式& SA_MODE_GET_FREQ_BY_FREQ
        saQueueCmd(buf, 6);
    }
}

/**********************************************************************
函数名称：vtxSASetPowerByIndex
函数功能：Smartaaudio通过索引设置功率
函数形参：vtxDevice，index
函数返回值：None
函数描述：None
**********************************************************************/
static void vtxSASetPowerByIndex(vtxDevice_t *vtxDevice, uint8_t index)
{
    static uint8_t buf[6] = { 0xAA, 0x55, SACMD(SA_CMD_SET_POWER), 1 };

    if (!vtxSAIsReady(vtxDevice)) {
        return;
    }

    uint16_t powerValue = 0;
    if (!vtxCommonLookupPowerValue(vtxDevice, index, &powerValue)) {
        dprintf(("saSetPowerByIndex: cannot get power level %d, only levels 1 through %d supported", index,
                 vtxTablePowerLevels));
        return;
    }

    buf[4] = powerValue;
    dprintf(("saSetPowerByIndex: index %d, value %d\r\n", index, buf[4]));
    if (saDevice.version == 3) {
        buf[4] |= 128;	// 设置MSB表示dbm设置的功率
    }
    buf[5] = CRC8(buf, 5);
    saQueueCmd(buf, 6);
}

/**********************************************************************
函数名称：vtxSASetPitMode
函数功能：Smartaaudio设置PIT模式
函数形参：vtxDevice，onoff
函数返回值：None
函数描述：None
**********************************************************************/
static void vtxSASetPitMode(vtxDevice_t *vtxDevice, uint8_t onoff)
{
    if (!vtxSAIsReady(vtxDevice) || saDevice.version < 2) {
        return;
    }

    if (onoff && saDevice.version < 3) {
        // 在V2.1之前的智能音频不能通过软件打开PIT模式。
        return;
    }

    if (saDevice.version >= 3 && !saDevice.willBootIntoPitMode) {
        if (onoff) {
			// 使用SET_POWER命令在0 dbm下启用pitmode。
			// 启用pitmode模式，而不会导致设备在下次上电时启动进入pitmode模式
            static uint8_t buf[6] = { 0xAA, 0x55, SACMD(SA_CMD_SET_POWER), 1 };
            buf[4] = 0 | 128;
            buf[5] = CRC8(buf, 5);
            saQueueCmd(buf, 6);
            dprintf(("vtxSASetPitMode: set power to 0 dbm\r\n"));
        } else {
            saSetMode(SA_MODE_CLR_PITMODE);
            dprintf(("vtxSASetPitMode: clear pitmode permanently"));
        }
        return;
    }

    uint8_t newMode = onoff ? 0 : SA_MODE_CLR_PITMODE;

    if (saDevice.mode & SA_MODE_GET_OUT_RANGE_PITMODE) {
        newMode |= SA_MODE_SET_OUT_RANGE_PITMODE;
    }

    if ((saDevice.mode & SA_MODE_GET_IN_RANGE_PITMODE) || (onoff && newMode == 0)) {
        // 确保当打开PIT模式时，PIT模式得到实际启用
        newMode |= SA_MODE_SET_IN_RANGE_PITMODE;
    }
    dprintf(("vtxSASetPitMode %s with stored mode 0x%x por %s, pir %s, newMode 0x%x\r\n", onoff ? "on" : "off", saDevice.mode,
            (saDevice.mode & SA_MODE_GET_OUT_RANGE_PITMODE) ? "on" : "off",
            (saDevice.mode & SA_MODE_GET_IN_RANGE_PITMODE) ? "on" : "off" , newMode));


    saSetMode(newMode);

    return;
}

/**********************************************************************
函数名称：vtxSASetFreq
函数功能：Smartaaudio设置频率
函数形参：vtxDevice，freq
函数返回值：None
函数描述：None
**********************************************************************/
static void vtxSASetFreq(vtxDevice_t *vtxDevice, uint16_t freq)
{
    if (!vtxSAIsReady(vtxDevice)) {
        return;
    }

    if (saValidateFreq(freq)) {
        saSetFreq(freq);
    }
}

/**********************************************************************
函数名称：vtxSAGetBandAndChannel
函数功能：Smartaaudio获取频组和通道
函数形参：vtxDevice，pBand，pChannel
函数返回值：状态
函数描述：None
**********************************************************************/
static bool vtxSAGetBandAndChannel(const vtxDevice_t *vtxDevice, uint8_t *pBand, uint8_t *pChannel)
{
    if (!vtxSAIsReady(vtxDevice)) {
        return false;
    }

    if (saDevice.mode & SA_MODE_GET_FREQ_BY_FREQ) {
        *pBand = 0;
        *pChannel = 0;
        return true;
    } else {
        *pBand = SA_DEVICE_CHVAL_TO_BAND(saDevice.channel);
        *pChannel = SA_DEVICE_CHVAL_TO_CHANNEL(saDevice.channel);
        return true;
    }
}

/**********************************************************************
函数名称：vtxSAGetPowerIndex
函数功能：Smartaaudio获取功率索引
函数形参：vtxDevice，pIndex
函数返回值：状态
函数描述：None
**********************************************************************/
static bool vtxSAGetPowerIndex(const vtxDevice_t *vtxDevice, uint8_t *pIndex)
{
    if (!vtxSAIsReady(vtxDevice)) {
        return false;
    }

    *pIndex = saDevice.power;// 功率级别是基于1的，以使vtxtables多一个标签而不是值
    return true;
}

/**********************************************************************
函数名称：vtxSAGetFreq
函数功能：Smartaaudio获取频率
函数形参：vtxDevice，pFreq
函数返回值：状态
函数描述：None
**********************************************************************/
static bool vtxSAGetFreq(const vtxDevice_t *vtxDevice, uint16_t *pFreq)
{
    if (!vtxSAIsReady(vtxDevice)) {
        return false;
    }

    // 如果不是在用户频率模式，然后转换频带/chan频率
    *pFreq = (saDevice.mode & SA_MODE_GET_FREQ_BY_FREQ) ? saDevice.freq :
             vtxCommonLookupFrequency(&vtxSmartAudio,
                                      SA_DEVICE_CHVAL_TO_BAND(saDevice.channel),
                                      SA_DEVICE_CHVAL_TO_CHANNEL(saDevice.channel));
    return true;
}

/**********************************************************************
函数名称：vtxSAGetStatus
函数功能：Smartaaudio获取状态
函数形参：vtxDevice，status
函数返回值：状态
函数描述：None
**********************************************************************/
static bool vtxSAGetStatus(const vtxDevice_t *vtxDevice, unsigned *status)
{
    if (!vtxSAIsReady(vtxDevice) || saDevice.version < 2) {
        return false;
    }

    *status = (saDevice.mode & SA_MODE_GET_PITMODE) ? VTX_STATUS_PIT_MODE : 0;

    return true;
}

/**********************************************************************
函数名称：vtxSAGetPowerLevels
函数功能：Smartaaudio获取功率标准
函数形参：vtxDevice，levels，powers
函数返回值：saSupportedNumPowerLevels
函数描述：None
**********************************************************************/
static uint8_t vtxSAGetPowerLevels(const vtxDevice_t *vtxDevice, uint16_t *levels, uint16_t *powers)
{
    if (!vtxSAIsReady(vtxDevice) || saDevice.version < 2) {
        return 0;
    }

    for (uint8_t i = 0; i < saSupportedNumPowerLevels; i++) {
        levels[i] = saSupportedPowerValues[i];
        uint16_t power = (uint16_t)pow(10.0,levels[i]/10.0);

        if (levels[i] > 14) {
            // 对于大于25mW的功率，可以四舍五入到50的倍数，以满足预期
            power = 50 * ((power + 25) / 50);
        }

        powers[i] = power;
    }
    return saSupportedNumPowerLevels;
}

// 注册函数表
static const vtxVTable_t saVTable = {
    .process = vtxSAProcess,
    .getDeviceType = vtxSAGetDeviceType,
    .isReady = vtxSAIsReady,
    .setBandAndChannel = vtxSASetBandAndChannel,
    .setPowerByIndex = vtxSASetPowerByIndex,
    .setPitMode = vtxSASetPitMode,
    .setFrequency = vtxSASetFreq,
    .getBandAndChannel = vtxSAGetBandAndChannel,
    .getPowerIndex = vtxSAGetPowerIndex,
    .getFrequency = vtxSAGetFreq,
    .getStatus = vtxSAGetStatus,
    .getPowerLevels = vtxSAGetPowerLevels,
};
#endif // VTX_COMMON
#endif // VTX_SMARTAUDIO

