#pragma once

struct sbuf_s;

uint16_t crc16_ccitt(uint16_t crc, unsigned char a);
uint16_t crc16_ccitt_update(uint16_t crc, const void *data, uint32_t length);
uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a);
uint8_t crc8_dvb_s2_update(uint8_t crc, const void *data, uint32_t length);

