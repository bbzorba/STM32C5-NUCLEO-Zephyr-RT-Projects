#include <zephyr/device.h>
#include "../inc/mavlink_tx.h"
#include "../inc/serial_com.h"

static uint8_t tx_seq;

static void crc_feed(uint16_t *crc, uint8_t b)
{
	uint8_t t = b ^ (uint8_t)(*crc);

	t ^= t << 4;
	*crc = (*crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ (t >> 4);
}

void send_msg(uint8_t msgid, const uint8_t *p, uint8_t plen, uint8_t crc_extra)
{
	uint8_t f[64];

	f[0] = 0xFE; f[1] = plen; f[2] = tx_seq++; f[3] = 255; f[4] = 0; f[5] = msgid;
	memcpy(&f[6], p, plen);

	uint16_t crc = 0xFFFF;

	for (uint8_t i = 1; i < 6u + plen; i++) crc_feed(&crc, f[i]);
	crc_feed(&crc, crc_extra);
	f[6 + plen] = crc & 0xFF;
	f[7 + plen] = crc >> 8;
	serial_write(f, 8u + plen);
}

void send_heartbeat(void)
{
	uint8_t p[9] = { 0, 0, 0, 0, 6, 8, 0, 0, 3 };

	send_msg(0, p, 9, 50);
}

 void request_imu_stream(void)
{
	/* REQUEST_DATA_STREAM (msgid=66): RAW_SENSORS group at 10 Hz */
	uint8_t p[6] = { 10, 0, 1, 0, 1, 1 };

	send_msg(66, p, 6, 148);
}