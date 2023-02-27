#include "platform.h"

#include "common/crc.h"

#include "smartaudio_protocol.h"

#define SMARTAUDIO_SYNC_BYTE            0xAA
#define SMARTAUDIO_HEADER_BYTE          0x55
#define SMARTAUDIO_START_CODE           SMARTAUDIO_SYNC_BYTE + SMARTAUDIO_HEADER_BYTE
#define SMARTAUDIO_GET_PITMODE_FREQ     (1 << 14)
#define SMARTAUDIO_SET_PITMODE_FREQ     (1 << 15)
#define SMARTAUDIO_FREQUENCY_MASK       0x3FFF

#define SMARTAUDIO_CMD_GET_SETTINGS     0x03
#define SMARTAUDIO_CMD_SET_POWER        0x05
#define SMARTAUDIO_CMD_SET_CHANNEL      0x07
#define SMARTAUDIO_CMD_SET_FREQUENCY    0x09
#define SMARTAUDIO_CMD_SET_MODE         0x0B

#define SMARTAUDIO_RSP_GET_SETTINGS_V1  SMARTAUDIO_CMD_GET_SETTINGS >> 1
#define SMARTAUDIO_RSP_GET_SETTINGS_V2  (SMARTAUDIO_CMD_GET_SETTINGS >> 1) | 0x08
#define SMARTAUDIO_RSP_SET_POWER        SMARTAUDIO_CMD_SET_POWER >> 1
#define SMARTAUDIO_RSP_SET_CHANNEL      SMARTAUDIO_CMD_SET_CHANNEL >> 1
#define SMARTAUDIO_RSP_SET_FREQUENCY    SMARTAUDIO_CMD_SET_FREQUENCY >> 1
#define SMARTAUDIO_RSP_SET_MODE         SMARTAUDIO_CMD_SET_MODE >> 1

#define SMARTAUDIO_BANDCHAN_TO_INDEX(band, channel) (band * 8 + (channel))
#define U16BIGENDIAN(bytes) (bytes << 8) | ((bytes >> 8) & 0xFF)

/**********************************************************************
函数名称：smartaudioFrameInit
函数功能：Smartaudio帧初始化
函数形参：command，header，payloadLength
函数返回值：None
函数描述：None
**********************************************************************/
static void smartaudioFrameInit(const uint8_t command, smartaudioFrameHeader_t *header, const uint8_t payloadLength)
{
    header->startCode = SMARTAUDIO_START_CODE;
    header->length = payloadLength;
    header->command = command;
}

/**********************************************************************
函数名称：smartaudioUnpackOperationMode
函数功能：Smartaudio解析操作模式
函数形参：command，operationMode，settingsResponse
函数返回值：None
函数描述：None
**********************************************************************/
static void smartaudioUnpackOperationMode(smartaudioSettings_t *settings, const uint8_t operationMode, const bool settingsResponse)
{
    if (settingsResponse) {
        // “获取设置”和“设置模式”响应之间的操作模式位序不同
        settings->userFrequencyMode = operationMode & 0x01;
        settings->pitmodeEnabled = operationMode & 0x02;
        settings->pitmodeInRangeActive = operationMode & 0x04;
        settings->pitmodeOutRangeActive = operationMode & 0x08;
        settings->unlocked = operationMode & 0x10;
    } else {
        settings->pitmodeInRangeActive = operationMode & 0x01;
        settings->pitmodeOutRangeActive = operationMode & 0x02;
        settings->pitmodeEnabled = operationMode & 0x04;
        settings->unlocked = operationMode & 0x08;
    }
}

/**********************************************************************
函数名称：smartaudioUnpackFrequency
函数功能：Smartaudio解析频率
函数形参：settings，frequency
函数返回值：None
函数描述：None
**********************************************************************/
static void smartaudioUnpackFrequency(smartaudioSettings_t *settings, const uint16_t frequency)
{
    if (frequency & SMARTAUDIO_GET_PITMODE_FREQ) {
        settings->pitmodeFrequency = U16BIGENDIAN(frequency & SMARTAUDIO_FREQUENCY_MASK);
    } else {
        settings->frequency = U16BIGENDIAN(frequency & SMARTAUDIO_FREQUENCY_MASK);
    }
}

/**********************************************************************
函数名称：smartaudioUnpackSettings
函数功能：Smartaudio解析设置
函数形参：settings，frame
函数返回值：None
函数描述：None
**********************************************************************/
static void smartaudioUnpackSettings(smartaudioSettings_t *settings, const smartaudioSettingsResponseFrame_t *frame)
{
    settings->channel = frame->channel;
    settings->power = frame->power;
    smartaudioUnpackFrequency(settings, frame->frequency);
    smartaudioUnpackOperationMode(settings, frame->operationMode, true);
}

/**********************************************************************
函数名称：smartaudioPackOperationMode
函数功能：Smartaudio操作模式包
函数形参：settings
函数返回值：operationMode
函数描述：None
**********************************************************************/
static uint8_t smartaudioPackOperationMode(const smartaudioSettings_t *settings)
{
    uint8_t operationMode = 0;
    operationMode |= settings->pitmodeInRangeActive << 0;
    operationMode |= settings->pitmodeOutRangeActive << 1;
    operationMode |= settings->pitmodeEnabled << 2;
    operationMode |= settings->unlocked << 3;
    return operationMode;
}

/**********************************************************************
函数名称：smartaudioPackOperationMode
函数功能：Smartaudio获取设置帧
函数形参：smartaudioFrame
函数返回值：大小
函数描述：None
**********************************************************************/
size_t smartaudioFrameGetSettings(smartaudioFrame_t *smartaudioFrame)
{
    smartaudioCommandOnlyFrame_t *frame = (smartaudioCommandOnlyFrame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_GET_SETTINGS, &frame->header, 0);
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioCommandOnlyFrame_t) - sizeof(frame->crc));
    return sizeof(smartaudioCommandOnlyFrame_t);
}

/**********************************************************************
函数名称：smartaudioPackOperationMode
函数功能：Smartaudio获取PIT模式频率帧
函数形参：smartaudioFrame
函数返回值：大小
函数描述：None
**********************************************************************/
size_t smartaudioFrameGetPitmodeFrequency(smartaudioFrame_t *smartaudioFrame)
{
    smartaudioU16Frame_t *frame = (smartaudioU16Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_FREQUENCY, &frame->header, sizeof(frame->payload));
    frame->payload = SMARTAUDIO_SET_PITMODE_FREQ;
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU16Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU16Frame_t);
}

/**********************************************************************
函数名称：smartaudioFrameSetPower
函数功能：Smartaudio设置功率帧
函数形参：smartaudioFrame，power
函数返回值：大小
函数描述：None
**********************************************************************/
size_t smartaudioFrameSetPower(smartaudioFrame_t *smartaudioFrame, const uint8_t power)
{
    smartaudioU8Frame_t *frame = (smartaudioU8Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_POWER, &frame->header, sizeof(frame->payload));
    frame->payload = power;
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU8Frame_t);
}

/**********************************************************************
函数名称：smartaudioFrameSetBandChannel
函数功能：Smartaudio设置频组频点帧
函数形参：smartaudioFrame，band，channel
函数返回值：大小
函数描述：None
**********************************************************************/
size_t smartaudioFrameSetBandChannel(smartaudioFrame_t *smartaudioFrame, const uint8_t band, const uint8_t channel)
{
    smartaudioU8Frame_t *frame = (smartaudioU8Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_CHANNEL, &frame->header, sizeof(frame->payload));
    frame->payload = SMARTAUDIO_BANDCHAN_TO_INDEX(band, channel);
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU8Frame_t);
}

/**********************************************************************
函数名称：smartaudioFrameSetFrequency
函数功能：Smartaudio设置频率帧
函数形参：smartaudioFrame，frequency，pitmodeFrequency
函数返回值：大小
函数描述：None
**********************************************************************/
size_t smartaudioFrameSetFrequency(smartaudioFrame_t *smartaudioFrame, const uint16_t frequency, const bool pitmodeFrequency)
{
    smartaudioU16Frame_t *frame = (smartaudioU16Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_FREQUENCY, &frame->header, sizeof(frame->payload));
    frame->payload = U16BIGENDIAN(frequency | (pitmodeFrequency ? SMARTAUDIO_SET_PITMODE_FREQ : 0x00));
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU16Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU16Frame_t);
}

/**********************************************************************
函数名称：smartaudioFrameSetOperationMode
函数功能：Smartaudio设置操作模式帧
函数形参：smartaudioFrame，settings
函数返回值：大小
函数描述：None
**********************************************************************/
size_t smartaudioFrameSetOperationMode(smartaudioFrame_t *smartaudioFrame, const smartaudioSettings_t *settings)
{
    smartaudioU8Frame_t *frame = (smartaudioU8Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_MODE, &frame->header, sizeof(frame->payload));
    frame->payload = smartaudioPackOperationMode(settings);
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU8Frame_t);
}

/**********************************************************************
函数名称：smartaudioParseResponseBuffer
函数功能：Smartaudio解析响应缓冲区
函数形参：settings，buffer
函数返回值：状态
函数描述：None
**********************************************************************/
bool smartaudioParseResponseBuffer(smartaudioSettings_t *settings, const uint8_t *buffer)
{
    const smartaudioFrameHeader_t *header = (const smartaudioFrameHeader_t *)buffer;
    const uint8_t fullFrameLength = sizeof(smartaudioFrameHeader_t) + header->length;
    const uint8_t headerPayloadLength = fullFrameLength - 1; // subtract crc byte from length
    const uint8_t *endPtr = buffer + fullFrameLength;

    if (crc8_dvb_s2_update(*endPtr, buffer, headerPayloadLength) || header->startCode != SMARTAUDIO_START_CODE) {
        return false;
    }
    switch (header->command) {
        case SMARTAUDIO_RSP_GET_SETTINGS_V1: {
                const smartaudioSettingsResponseFrame_t *resp = (const smartaudioSettingsResponseFrame_t *)buffer;
                settings->version = 1;
                smartaudioUnpackSettings(settings, resp);
            }
            break;
        case SMARTAUDIO_RSP_GET_SETTINGS_V2: {
                const smartaudioSettingsResponseFrame_t *resp = (const smartaudioSettingsResponseFrame_t *)buffer;
                settings->version = 2;
                smartaudioUnpackSettings(settings, resp);
            }
            break;
        case SMARTAUDIO_RSP_SET_POWER: {
                const smartaudioU16ResponseFrame_t *resp = (const smartaudioU16ResponseFrame_t *)buffer;
                settings->channel = (resp->payload >> 8) & 0xFF;
                settings->power = resp->payload & 0xFF;
            }
            break;
        case SMARTAUDIO_RSP_SET_CHANNEL: {
                const smartaudioU8ResponseFrame_t *resp = (const smartaudioU8ResponseFrame_t *)buffer;
                settings->channel = resp->payload;
            }
            break;
        case SMARTAUDIO_RSP_SET_FREQUENCY: {
                const smartaudioU16ResponseFrame_t *resp = (const smartaudioU16ResponseFrame_t *)buffer;
                smartaudioUnpackFrequency(settings, resp->payload);
            }
            break;
        case SMARTAUDIO_RSP_SET_MODE: {
                const smartaudioU8ResponseFrame_t *resp = (const smartaudioU8ResponseFrame_t*)buffer;
                smartaudioUnpackOperationMode(settings, resp->payload, false);
            }
            break;
        default:
            return false;
    }
    return true;
}

