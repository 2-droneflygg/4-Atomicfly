#pragma once

#include "common/streambuf.h"
#include "common/time.h"
#include "io/serial.h"
#include "drivers/time.h"
#include "drivers/system.h"

// -----------------------MSP端口输入输出缓冲区大小
#define MSP_PORT_INBUF_SIZE 256
#define MSP_PORT_OUTBUF_SIZE 256

/* -----------------------评估非MSP数据枚举----------------------- */
typedef enum {
    MSP_EVALUATE_NON_MSP_DATA,
    MSP_SKIP_NON_MSP_DATA
} mspEvaluateNonMspData_e;

/* ------------------------MSP挂起请求枚举------------------------ */
typedef enum {
    MSP_PENDING_NONE,
    MSP_PENDING_BOOTLOADER_ROM,
    MSP_PENDING_BOOTLOADER_FLASH,
} mspPendingSystemRequest_e;

/* --------------------------MSP状态枚举-------------------------- */	
typedef enum {
    MSP_IDLE,                                   // 空闲状态
    MSP_HEADER,                                 // 数据头
    MSP_PAYLOAD,                                // 有效负载
    MSP_CHECKSUM,                               // 校验
    MSP_COMMAND_RECEIVED,                       // 命令处理
} mspState_e;

/* --------------------------MSP结果枚举-------------------------- */	
// ACK返回正，错误返回负，无应答返回零
typedef enum {
    MSP_RESULT_ACK = 1,                         // 应答
    MSP_RESULT_ERROR = -1,                      // 错误
    MSP_RESULT_NO_REPLY = 0,                    // 无应答
    MSP_RESULT_CMD_UNKNOWN = -2,                // 命令未知
} mspResult_e;

/* -------------------------MSP报头结构体------------------------- */
typedef struct __attribute__((packed)) {
    uint8_t size;
    uint8_t cmd;
} mspHeader_t;

/* -----------------------MSP报头（大型）结构体-------------------- */
// 用于有效负载较大时存储负载长度
typedef struct __attribute__((packed)) {
    uint16_t size;
} mspHeaderJUMBO_t;

/* --------------------------MSP包结构体-------------------------- */	
typedef struct mspPacket_s {
    sbuf_t buf;                                 // 数据包
    int16_t cmd;                                // 命令
    int16_t result;                             // 应答状态
} mspPacket_t;

/* -------------------------MSP端口结构体------------------------- */
struct serialPort_s;
typedef struct mspPort_s {
    struct serialPort_s *port;                  // 端口
    timeMs_t lastActivityMs;                    // 上一次活动时间
    mspPendingSystemRequest_e pendingRequest;   // MSP挂起请求
    mspState_e c_state;                         // MSP状态
    uint8_t inBuf[MSP_PORT_INBUF_SIZE];         // 输入缓冲区
    uint16_t cmdMSP;                            // MSP命令
    uint_fast16_t offset;                       // 偏移
    uint_fast16_t dataSize;                     // 数据大小
    uint8_t checksum;                           // 校验字节
} mspPort_t; 

// 后置处理进程（用于优雅地处理重新启动等）
struct serialPort_s;
typedef void (*mspPostProcessFnPtr)(struct serialPort_s *port); 
// 命令处理进程
typedef mspResult_e (*mspProcessCommandFnPtr)(mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *mspPostProcessFn);

void mspSerialProcess(mspEvaluateNonMspData_e evaluateNonMspData, mspProcessCommandFnPtr mspProcessCommandFn);
bool mspSerialProcessReceivedData(mspPort_t *mspPort, uint8_t c);
mspResult_e mspFcProcessCommand(mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *mspPostProcessFn);
mspPostProcessFnPtr mspSerialProcessReceivedCommand(mspPort_t *msp, mspProcessCommandFnPtr mspProcessCommandFn);
int mspSerialEncode(mspPort_t *msp, mspPacket_t *packet);
uint8_t mspSerialChecksumBuf(uint8_t checksum, const uint8_t *data, int len);
static int mspSerialSendFrame(mspPort_t *msp, const uint8_t * hdr, int hdrLen, const uint8_t * data, int dataLen, const uint8_t * crc, int crcLen);

