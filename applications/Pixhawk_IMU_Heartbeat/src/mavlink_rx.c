#include "../inc/mavlink_rx.h"

/* Shared state (extern in mavlink_rx.h) */
struct px4_hb_t px4_hb;
struct imu_t    imu;

/* Parser state — private to this translation unit */
static enum { PS_STX, PS_LEN, PS_INC, PS_COM, PS_SEQ, PS_SYS, PS_CMP,
              PS_ID0, PS_ID1, PS_ID2, PS_PAY, PS_C1,  PS_C2 } ps;
static bool     ps_v2;
static uint8_t  ps_plen, ps_pidx, ps_pay[64], ps_c1;
static uint32_t ps_mid;
static uint16_t ps_crc;

static void crc_feed(uint16_t *crc, uint8_t b)
{
	uint8_t t = b ^ (uint8_t)(*crc);

	t ^= t << 4;
	*crc = (*crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ (t >> 4);
}

static uint8_t msg_crc_extra(uint32_t id)
{
	switch (id) {
	case  0: return  50;   /* HEARTBEAT */
	case 27: return 144;   /* RAW_IMU   */
	default: return   0;
	}
}

static const char *mav_autopilot_str(uint8_t a)
{
	switch (a) {
	case  0: return "GENERIC";
	case  3: return "ARDUPILOT";
	case 12: return "PX4";
	default: return "OTHER";
	}
}

static const char *mav_type_str(uint8_t t)
{
	switch (t) {
	case  0: return "GENERIC";
	case  1: return "FIXED_WING";
	case  2: return "QUADROTOR";
	case 13: return "HEXAROTOR";
	case 14: return "OCTOROTOR";
	default: return "OTHER";
	}
}

void parse_byte(uint8_t b)
{
	switch (ps) {
	case PS_STX:
		if      (b == 0xFE) { ps_v2 = false; ps_crc = 0xFFFF; ps = PS_LEN; }
		else if (b == 0xFD) { ps_v2 = true;  ps_crc = 0xFFFF; ps = PS_LEN; }
		break;
	case PS_LEN: crc_feed(&ps_crc, b); ps_plen = b; ps = ps_v2 ? PS_INC : PS_SEQ; break;
	case PS_INC: crc_feed(&ps_crc, b); ps = b ? PS_STX : PS_COM;                   break;
	case PS_COM: crc_feed(&ps_crc, b); ps = PS_SEQ;                                 break;
	case PS_SEQ: crc_feed(&ps_crc, b); ps = PS_SYS;                                 break;
	case PS_SYS: crc_feed(&ps_crc, b); ps = PS_CMP;                                 break;
	case PS_CMP: crc_feed(&ps_crc, b); ps = PS_ID0;                                 break;
	case PS_ID0:
		crc_feed(&ps_crc, b); ps_mid = b; ps_pidx = 0;
		ps = ps_v2 ? PS_ID1 : (ps_plen ? PS_PAY : PS_C1);
		break;
	case PS_ID1: crc_feed(&ps_crc, b); ps_mid |= (uint32_t)b << 8;  ps = PS_ID2; break;
	case PS_ID2: crc_feed(&ps_crc, b); ps_mid |= (uint32_t)b << 16; ps = ps_plen ? PS_PAY : PS_C1; break;
	case PS_PAY:
		crc_feed(&ps_crc, b);
		if (ps_pidx < sizeof(ps_pay)) ps_pay[ps_pidx] = b;
		if (++ps_pidx >= ps_plen) ps = PS_C1;
		break;
	case PS_C1: ps_c1 = b; ps = PS_C2; break;
	case PS_C2: {
		uint16_t fc = ps_crc;

		crc_feed(&fc, msg_crc_extra(ps_mid));
		if ((ps_c1 | ((uint16_t)b << 8)) == fc) on_message(ps_mid, ps_pay);
		ps = PS_STX;
		break;
	}
	}
}

void on_message(uint32_t id, const uint8_t *p)
{
	if (id == 0) {
		/* HEARTBEAT: custom_mode[0-3], type[4], autopilot[5], base_mode[6] */
		px4_hb.type      = p[4];
		px4_hb.autopilot = p[5];
		px4_hb.base_mode = p[6];
		printk("[RX-HB] type=%s  autopilot=%s  armed=%s  base_mode=0x%02x\n",
		       mav_type_str(px4_hb.type),
		       mav_autopilot_str(px4_hb.autopilot),
		       (px4_hb.base_mode & 0x80) ? "YES" : "NO",
		       px4_hb.base_mode);
	} else if (id == 27) {
		/* RAW_IMU: time_usec[0-7], xacc[8], yacc[10], zacc[12],
		 *          xgyro[14], ygyro[16], zgyro[18],
		 *          xmag[20],  ymag[22],  zmag[24]           */
		memcpy(&imu.ax, &p[ 8], 2); memcpy(&imu.ay, &p[10], 2); memcpy(&imu.az, &p[12], 2);
		memcpy(&imu.gx, &p[14], 2); memcpy(&imu.gy, &p[16], 2); memcpy(&imu.gz, &p[18], 2);
		memcpy(&imu.mx, &p[20], 2); memcpy(&imu.my, &p[22], 2); memcpy(&imu.mz, &p[24], 2);
	}
}