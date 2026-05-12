#include "../inc/mavlink_rx.h"
#include <zephyr/kernel.h>
#include <string.h>

/* Definitions of extern variables declared in mavlink_rx.h */
volatile int32_t  g_rel_alt_mm;
volatile bool     g_ack_ready;
volatile uint16_t g_ack_cmd;
volatile uint8_t  g_ack_result;
struct px4_hb_t   g_px4_hb;

/* Parser state — private to this file */
static enum {
	RX_STX,
	RX_LEN,
	RX_V2_INCOMPAT, RX_V2_COMPAT,       /* MAVLink v2 only          */
	RX_SEQ, RX_SYS, RX_COMP,
	RX_MSGID, RX_V2_MSGID_1, RX_V2_MSGID_2,
	RX_PAYLOAD, RX_CRC1, RX_CRC2
} rx_state;

static bool     rx_is_v2;
static uint8_t  rx_plen, rx_payload[64], rx_pidx, rx_crc1_got;
static uint32_t rx_msgid;
static uint16_t rx_crc_run;

static void crc_accum(uint16_t *crc, uint8_t x)
{
	uint8_t t = x ^ (uint8_t)(*crc & 0xFF);

	t ^= t << 4;
	*crc = (*crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ ((uint16_t)t >> 4);
}

static uint8_t crc_extra_for(uint32_t msgid)
{
	switch (msgid) {
	case  0: return  50;   /* HEARTBEAT            */
	case 33: return 104;   /* GLOBAL_POSITION_INT  */
	case 76: return 152;   /* COMMAND_LONG         */
	case 77: return 143;   /* COMMAND_ACK          */
	default: return   0;
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

static const char *mav_autopilot_str(uint8_t a)
{
	switch (a) {
	case  0: return "GENERIC";
	case  3: return "ARDUPILOT";
	case 12: return "PX4";
	default: return "OTHER";
	}
}

static void mav_dispatch(uint32_t msgid, const uint8_t *payload)
{
	switch (msgid) {
	case 0:
		/* HEARTBEAT: type[4], autopilot[5], base_mode[6] */
		g_px4_hb.type      = payload[4];
		g_px4_hb.autopilot = payload[5];
		g_px4_hb.base_mode = payload[6];
		printk("[RX-HB] type=%s  autopilot=%s  armed=%s  base_mode=0x%02x\n",
		       mav_type_str(g_px4_hb.type),
		       mav_autopilot_str(g_px4_hb.autopilot),
		       (g_px4_hb.base_mode & 0x80) ? "YES" : "NO",
		       g_px4_hb.base_mode);
		break;

	case 33: {
		/* GLOBAL_POSITION_INT: relative_alt at byte offset 16 (int32, mm) */
		int32_t tmp;

		memcpy(&tmp, &payload[16], 4);
		g_rel_alt_mm = tmp;
		break;
	}

	case 77: {
		/* COMMAND_ACK: command[0-1], result[2] */
		uint16_t cmd;

		memcpy(&cmd, &payload[0], 2);
		g_ack_cmd    = cmd;
		g_ack_result = payload[2];
		g_ack_ready  = true;
		break;
	}

	default:
		break;
	}
}

void mav_parse_byte(uint8_t b)
{
	switch (rx_state) {
	case RX_STX:
		if      (b == 0xFE) { rx_is_v2 = false; rx_crc_run = 0xFFFF; rx_state = RX_LEN; }
		else if (b == 0xFD) { rx_is_v2 = true;  rx_crc_run = 0xFFFF; rx_state = RX_LEN; }
		break;

	case RX_LEN:
		crc_accum(&rx_crc_run, b);
		rx_plen  = b;
		rx_state = rx_is_v2 ? RX_V2_INCOMPAT : RX_SEQ;
		break;

	case RX_V2_INCOMPAT:
		crc_accum(&rx_crc_run, b);
		rx_state = (b == 0) ? RX_V2_COMPAT : RX_STX;
		break;

	case RX_V2_COMPAT:
		crc_accum(&rx_crc_run, b);
		rx_state = RX_SEQ;
		break;

	case RX_SEQ:
		crc_accum(&rx_crc_run, b);
		rx_state = RX_SYS;
		break;

	case RX_SYS:
		crc_accum(&rx_crc_run, b);
		rx_state = RX_COMP;
		break;

	case RX_COMP:
		crc_accum(&rx_crc_run, b);
		rx_state = RX_MSGID;
		break;

	case RX_MSGID:
		crc_accum(&rx_crc_run, b);
		rx_msgid = b;
		rx_pidx  = 0;
		rx_state = rx_is_v2 ? RX_V2_MSGID_1
				    : (rx_plen > 0 ? RX_PAYLOAD : RX_CRC1);
		break;

	case RX_V2_MSGID_1:
		crc_accum(&rx_crc_run, b);
		rx_msgid |= ((uint32_t)b << 8);
		rx_state  = RX_V2_MSGID_2;
		break;

	case RX_V2_MSGID_2:
		crc_accum(&rx_crc_run, b);
		rx_msgid |= ((uint32_t)b << 16);
		rx_state  = (rx_plen > 0) ? RX_PAYLOAD : RX_CRC1;
		break;

	case RX_PAYLOAD:
		crc_accum(&rx_crc_run, b);
		if (rx_pidx < sizeof(rx_payload)) rx_payload[rx_pidx] = b;
		rx_pidx++;
		if (rx_pidx >= rx_plen) rx_state = RX_CRC1;
		break;

	case RX_CRC1:
		rx_crc1_got = b;
		rx_state    = RX_CRC2;
		break;

	case RX_CRC2: {
		uint16_t final_crc = rx_crc_run;

		crc_accum(&final_crc, crc_extra_for(rx_msgid));
		uint16_t got_crc = rx_crc1_got | ((uint16_t)b << 8);
		if (final_crc == got_crc) mav_dispatch(rx_msgid, rx_payload);
		rx_state = RX_STX;
		break;
	}
	}
}
