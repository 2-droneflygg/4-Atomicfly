#include "platform.h"
#include "msp.h"
#include "msp_command.h"
#include "drivers/serial.h"

/**********************************************************************
函数名称：mspSerialProcessReceivedData
函数功能：MSP接收数据进程
函数形参：MSP端口，字符
函数返回值：true：接收完成，false：非MSP数据
函数描述：None 
**********************************************************************/
bool mspSerialProcessReceivedData(mspPort_t *mspPort, uint8_t c)
{
    // 数据包：开始标志（$），数据头（有效负载大小，命令），有效负载，校验字节
    switch (mspPort->c_state) {
        case MSP_IDLE:      
            // -------------------------------------等待'$'字符开始接收MSP数据包
            if (c == '$') {
                mspPort->c_state = MSP_HEADER;
				// 复位相关变量
	            mspPort->offset = 0;
	            mspPort->checksum = 0;
				//serialWrite(mspPort->port,0x34);
            } else {
                return false;
            }
        break;
        case MSP_HEADER: 
            // -------------------------------------1.数据头
            // 添加字符到数据缓冲区
            mspPort->inBuf[mspPort->offset++] = c;
            // 异或数据校验
            mspPort->checksum ^= c;
            // 判断数据头是否接收完成
            if (mspPort->offset == sizeof(mspHeader_t)) {
                mspHeader_t *hdr = (mspHeader_t *)&mspPort->inBuf[0];
                // 检查是否超出输入缓冲区容量
                if (hdr->size > MSP_PORT_INBUF_SIZE) {
                    mspPort->c_state = MSP_IDLE;
                } else {
                    mspPort->dataSize = hdr->size;
                    mspPort->cmdMSP = hdr->cmd;
                    mspPort->offset = 0;               
                    // 如果没有有效负载-跳到校验和字节
                    mspPort->c_state = mspPort->dataSize > 0 ? MSP_PAYLOAD : MSP_CHECKSUM;    
                }
            } 
        break;
        case MSP_PAYLOAD: 
            // -------------------------------------2.有效负载
            // 添加字符到数据缓冲区
            mspPort->inBuf[mspPort->offset++] = c;
			//serialWrite(mspPort->port,c);
            // 异或数据校验
            mspPort->checksum ^= c;
            // 判断有效负载是否接收完成
            if (mspPort->offset == mspPort->dataSize) {
                mspPort->c_state = MSP_CHECKSUM;
            }
        break;
        case MSP_CHECKSUM: 
            // -------------------------------------3.校验字节
            serialWrite(mspPort->port,mspPort->checksum);
            if (mspPort->checksum == c) {
                // 进入处理命令环节 - 命令处理进程
                mspPort->c_state = MSP_COMMAND_RECEIVED;
            } else {
                mspPort->c_state = MSP_IDLE;
            }
        break;
    }
    return true;
}

/**********************************************************************
函数名称：mspSerialChecksumBuf
函数功能：获取MSP串行端口数据校验和
函数形参：已有校验和，数据指针，数据长度
函数返回值：校验和
函数描述：None 
**********************************************************************/
uint8_t mspSerialChecksumBuf(uint8_t checksum, const uint8_t *data, int len)
{
    while (len-- > 0) {
        // 异或校验
        checksum ^= *data++;
    }
    return checksum;
}

/**********************************************************************
函数名称：mspSerialSendFrame
函数功能：MSP串行端口发送数据帧
函数形参：MSP端口，数据头，数据头长度，有效负载，负载长度，校验字节，校验长度
函数返回值：成功：数据帧总长度，失败：0
函数描述：None 
**********************************************************************/
#define JUMBO_FRAME_SIZE_LIMIT 255
int mspSerialSendFrame(mspPort_t *msp, 
							  const uint8_t *hdr, int hdrLen, 
							  const uint8_t *data, int dataLen, 
							  const uint8_t *crc, int crcLen)
{
	// ---------------------------------------------计算数据帧总长度
    const int totalFrameLength = hdrLen + dataLen + crcLen;
    // ---------------------------------------------传输缓冲区为空(传输完成) && 缓冲区空闲字节足够承载数据帧
    if (!isSerialTransmitBufferEmpty(msp->port) && ((int)serialTxBytesFree(msp->port) < totalFrameLength)) {
        return 0;
    }
    // ---------------------------------------------传输帧
	serialBeginWrite(msp->port);                // 开始写入缓冲区
    serialWriteBuf(msp->port, hdr, hdrLen);     // 写入数据头
    serialWriteBuf(msp->port, data, dataLen);   // 写入有效负载
    serialWriteBuf(msp->port, crc, crcLen);     // 写入CRC校验和
    serialEndWrite(msp->port);                  // 结束写入
    return totalFrameLength;
}

/**********************************************************************
函数名称：mspSerialEncode
函数功能：MSP数据包编码并进行发送
函数形参：MSP端口，数据包
函数返回值：发送状态
函数描述：None 
**********************************************************************/
int mspSerialEncode(mspPort_t *msp, mspPacket_t *packet)
{
    // 获取数据长度
    const int dataLen = sbufBytesRemaining(&packet->buf);
    // 数据头缓冲区 - 预加载起始字节     
    uint8_t hdrBuf[16] = {'$'};
    // CRC缓冲区  
    uint8_t crcBuf[2];
    // 校验和缓冲区
    uint8_t checksum;
    // 数据头长度 - 预偏移起始字节
    int hdrLen = 1;
    // CRC长度
    int crcLen = 0;
    // 校验和起始点
    #define CHECKSUM_STARTPOS 1
    // ---------------------------------------------数据头
    // 获取数据头缓冲区并偏移
    mspHeader_t *hdr = (mspHeader_t *)&hdrBuf[hdrLen];
    // 获取数据头大小
    hdrLen += sizeof(mspHeader_t);
    // 获取命令
    hdr->cmd = packet->cmd;
    // 必要时添加巨帧报头
    if (dataLen >= JUMBO_FRAME_SIZE_LIMIT) {
        mspHeaderJUMBO_t * hdrJUMBO = (mspHeaderJUMBO_t *)&hdrBuf[hdrLen];
        hdrLen += sizeof(mspHeaderJUMBO_t);
        hdr->size = JUMBO_FRAME_SIZE_LIMIT;
        hdrJUMBO->size = dataLen;
    } else {
        hdr->size = dataLen;
    }
    // ---------------------------------------------预计算CRC校验和
    checksum = mspSerialChecksumBuf(0, hdrBuf + CHECKSUM_STARTPOS, hdrLen - CHECKSUM_STARTPOS);
    checksum = mspSerialChecksumBuf(checksum, sbufPtr(&packet->buf), dataLen);
    crcBuf[crcLen++] = checksum;
    // ---------------------------------------------发送数据包帧
    return mspSerialSendFrame(msp, hdrBuf, hdrLen, sbufPtr(&packet->buf), dataLen, crcBuf, crcLen);   
}

/**********************************************************************
函数名称：mspFcProcessCommand
函数功能：MSP命令处理进程
函数形参：命令，命令回复包缓存，后置处理进程（如重启等操作）
函数返回值：命令应答状态
函数描述：None 
**********************************************************************/
mspResult_e mspFcProcessCommand(mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *mspPostProcessFn)
{
    // 默认初始化应答
    int ret = MSP_RESULT_ACK;
    // 获取回复包缓存
    sbuf_t *dst = &reply->buf;
    // 获取命令包缓存
    sbuf_t *src = &cmd->buf;
    // 获取命令
    const int16_t cmdMSP = cmd->cmd;
    reply->cmd = cmd->cmd;
    // ---------------------------------------------执行命令处理[命令输入输出]
    if (mspProcessOutCommand(cmdMSP, dst, mspPostProcessFn)) { 
        // 上位机索要数据需要进行应答数据包
        ret = MSP_RESULT_ACK;
    } else {
        ret = mspProcessInCommand(cmdMSP, src, mspPostProcessFn);
    }
    reply->result = ret;
    return ret;
}

/**********************************************************************
函数名称：mspSerialProcessReceivedCommand
函数功能：MSP接收命令进程
函数形参：MSP端口，MSP命令处理进程
函数返回值：后置处理进程
函数描述：None 
**********************************************************************/
mspPostProcessFnPtr mspSerialProcessReceivedCommand(mspPort_t *msp, mspProcessCommandFnPtr mspProcessCommandFn)
{
    static uint8_t outBuf[MSP_PORT_OUTBUF_SIZE];
    // ---------------------------------------------命令回复包
    mspPacket_t reply = {
        .buf = { .ptr = outBuf, .end = ARRAYEND(outBuf)},             
        .cmd = -1,                                         
    };
    uint8_t *outBufHead = reply.buf.ptr;
    // ---------------------------------------------命令包
    mspPacket_t command = {
        .buf = { .ptr = msp->inBuf, .end = msp->inBuf + msp->dataSize},
        .cmd = msp->cmdMSP,
    };
    // 后置处理进程
    mspPostProcessFnPtr mspPostProcessFn = NULL;
    // 执行命令处理进程 - 获取命令对应数据包（reply）、注册对应后置处理进程
    const mspResult_e status = mspProcessCommandFn(&command, &reply, &mspPostProcessFn);
    // 应答
    if (status != MSP_RESULT_NO_REPLY) {
        // 改变缓冲流方向
        sbufSwitchToReader(&reply.buf, outBufHead); 
        // 数据包编码后进行发送帧
        mspSerialEncode(msp, &reply);
    }
    // 后置处理进程
    return mspPostProcessFn;
}

/**********************************************************************
函数名称：mspSerialProcess
函数功能：串行数据处理
函数形参：评估MSP数据/非MSP数据，MSP命令进程指针
函数返回值：None
函数描述：None
**********************************************************************/
void mspSerialProcess(mspEvaluateNonMspData_e evaluateNonMspData, mspProcessCommandFnPtr mspProcessCommandFn)
{
    mspPort_t mspPort = {
        // 注册USB端口
        .port = USB_VCP_Port,  
        // 初始化MSP为空闲状态
        .c_state =  MSP_IDLE,
    };
    mspPostProcessFnPtr mspPostProcessFn = NULL;
	// -------------------------------------------------判断是否有字节传入(接收缓冲区存在数据)
	if (serialRxBytesWaiting(mspPort.port)) {
        mspPort.lastActivityMs = millis();
        mspPort.pendingRequest = MSP_PENDING_NONE;
		// ---------------------------------------------等待接收缓冲区字节处理完成
        while (serialRxBytesWaiting(mspPort.port)) {
			// ----------------------------读取字符
            const uint8_t c = serialRead(mspPort.port);
			// ----------------------------处理字符 - 接收一帧数据包
            const bool consumed = mspSerialProcessReceivedData(&mspPort, c);
            // ----------------------------处理非MSP数据
            if (!consumed && evaluateNonMspData == MSP_EVALUATE_NON_MSP_DATA) {
                // 未解锁 - 检查重启到BOOTLODER需求
                if (c == serialConfig()->reboot_character) {
                    systemResetToBootloader(BOOTLOADER_REQUEST_ROM);
                }
            }
            // ----------------------------命令处理
            if (mspPort.c_state == MSP_COMMAND_RECEIVED) {
                // 命令处理进程 - 并获取后置处理进程
                mspPostProcessFn = mspSerialProcessReceivedCommand(&mspPort, mspProcessCommandFn);
                // 一次处理一个命令，以免阻塞
                mspPort.c_state = MSP_IDLE;
                break; 
            }
		}
        // --------------------------------------------------后置处理进程
        if (mspPostProcessFn) {
            // ----------------------------等待端口完成传输
            waitForSerialPortToFinishTransmitting(mspPort.port);
            // ----------------------------执行后置处理进程
            mspPostProcessFn(mspPort.port);
        }
	} 
}

