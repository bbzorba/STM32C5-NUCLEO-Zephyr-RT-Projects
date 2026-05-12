#include "../inc/mavlink_tx.h"
#include "../inc/serial_com.h"
#include <string.h>

static uint8_t tx_seq;

static void crc_accum(uint16_t *crc, uint8_t x)
{
	uint8_t t = x ^ (uint8_t)(*crc & 0xFF);

	t ^= t << 4;
	*crc = (*crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ ((uint16_t)t >> 4);
}

static void send_frame(uint8_t msgid, const uint8_t *payload, uint8_t plen, uint8_t crc_extra)
{
	uint8_t f[64];

	f[0] = 0xFE; f[1] = plen; f[2] = tx_seq++; f[3] = 255; f[4] = 0; f[5] = msgid;
	memcpy(&f[6], payload, plen);

	uint16_t crc = 0xFFFF;

	for (uint8_t i = 1; i < 6u + plen; i++) crc_accum(&crc, f[i]);
	crc_accum(&crc, crc_extra);
	f[6 + plen] = (uint8_t)(crc & 0xFF);
	f[7 + plen] = (uint8_t)(crc >> 8);
	serial_write(f, 8u + plen);
}

void send_heartbeat(void)
{
	/* HEARTBEAT (msgid=0, plen=9, crc_extra=50)
	 * type=MAV_TYPE_GCS(6), autopilot=MAV_AUTOPILOT_INVALID(8), mavlink_version=3 */
	uint8_t p[9] = { 0, 0, 0, 0, 6, 8, 0, 0, 3 };

	send_frame(0, p, 9, 50);
}

void send_command_long(uint16_t cmd,
		       float p1, float p2, float p3,
		       float p4, float p5, float p6, float p7)
{
	/* COMMAND_LONG (msgid=76, plen=33, crc_extra=152) */
	uint8_t p[33];

	memcpy(&p[ 0], &p1, 4); memcpy(&p[ 4], &p2, 4); memcpy(&p[ 8], &p3, 4);
	memcpy(&p[12], &p4, 4); memcpy(&p[16], &p5, 4); memcpy(&p[20], &p6, 4);
	memcpy(&p[24], &p7, 4);
	p[28] = (uint8_t)(cmd & 0xFF);
	p[29] = (uint8_t)(cmd >> 8);
	p[30] = 1;   /* target_system    = 1 (PX4) */
	p[31] = 1;   /* target_component = 1       */
	p[32] = 0;   /* confirmation               */
	send_frame(76, p, 33, 152);
}
