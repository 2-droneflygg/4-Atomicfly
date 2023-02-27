#pragma once

#define SOFTSERIAL_BUFFER_SIZE 256

/* ------------------------------软件串口索引枚举------------------------------ */	
typedef enum {
    SOFTSERIAL1 = 0,
    SOFTSERIAL2
} softSerialPortIndex_e;

serialPort_t *openSoftSerial(softSerialPortIndex_e portIndex, uint32_t baud, portMode_e mode, portOptions_e options);
void softSerialWriteByte(serialPort_t *instance, uint8_t ch);
uint32_t softSerialRxBytesWaiting(const serialPort_t *instance);
uint32_t softSerialTxBytesFree(const serialPort_t *instance);
uint8_t softSerialReadByte(serialPort_t *instance);
void softSerialSetBaudRate(serialPort_t *s, uint32_t baudRate);
bool isSoftSerialTransmitBufferEmpty(const serialPort_t *s);


