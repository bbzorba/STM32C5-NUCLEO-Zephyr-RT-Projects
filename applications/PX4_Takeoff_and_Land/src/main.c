/*
 * ============================================================
 * PX4 Takeoff-and-Land via MAVLink — Zephyr RTOS
 * ============================================================
 *
 * Hardware connection:
 *   NUCLEO-C562RE USART1  (PB14 = TX,  PB15 = RX)
 *   → PX4 flight controller TELEM1 port
 *   Baud: 57600 · 8-N-1  (must match PX4 parameter SER_TEL1_BAUD)
 *   Wiring: PB14 → TELEM_RX,  PB15 → TELEM_TX,  GND → GND
 *
 * Mission:
 *   1. Set PX4 to AUTO mode    (MAV_CMD_DO_SET_MODE,         cmd=176)
 *   2. Arm the motors          (MAV_CMD_COMPONENT_ARM_DISARM, cmd=400)
 *   3. Take off to 10 m        (MAV_CMD_NAV_TAKEOFF,          cmd=22)
 *   4. Hover for 3 seconds
 *   5. Land at same position   (MAV_CMD_NAV_LAND,             cmd=21)
 *
 * Scheduling — cooperative Earliest-Deadline-First (EDF):
 *
 *   The scheduler runs in a single thread and loops every 1 ms.
 *   Each iteration it:
 *     1. Activates every task whose period has elapsed, recording its
 *        absolute deadline = (activation time) + (relative deadline).
 *     2. Picks the ready task with the smallest absolute deadline (EDF).
 *     3. Runs it to completion (cooperative scheduling).
 *     4. Rearms the task for the next period.
 *     5. Prints a warning if the task overran its deadline.
 *
 *   Task        Period    Rel-deadline   Purpose
 *   ---------   -------   -----------    -----------------------------
 *   heartbeat   1000 ms    900 ms         Keep MAVLink link alive
 *   rx_parse      50 ms     40 ms         Drain UART buffer & parse
 *   mission      500 ms    450 ms         Advance mission state machine
 * ============================================================
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

/* ─────────────────────────────────────────────────────────────
 * UART  (USART1 — see boards/nucleo_c562re.overlay)
 * ───────────────────────────────────────────────────────────── */

#define MAVLINK_UART_NODE  DT_NODELABEL(usart1)

static const struct device *uart_dev;

/* Power-of-2 ring buffer shared between the ISR (writer) and
 * the rx_parse task (reader).  Only one side writes its pointer,
 * so no mutex is needed on Cortex-M.                           */
#define RX_BUF_SIZE  256u
#define RX_BUF_MASK  (RX_BUF_SIZE - 1u)

static uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint32_t rx_head;          /* written by ISR            */
static volatile uint32_t g_rx_bytes_total; /* raw byte counter (ISR)    */
static uint32_t rx_tail;                   /* written by rx_parse_task  */

/* ISR: push every received byte into the ring buffer */
static void uart_isr(const struct device *dev, void *ud)
{
	ARG_UNUSED(ud);
	while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
		uint8_t c;
		if (uart_fifo_read(dev, &c, 1) == 1) {
			rx_buf[rx_head & RX_BUF_MASK] = c;
			rx_head++;
			g_rx_bytes_total++;
		}
	}
}

/* Consume one byte from the ring buffer.  Returns false if empty. */
static bool rx_get(uint8_t *b)
{
	if (rx_tail == rx_head) {
		return false;
	}
	*b = rx_buf[rx_tail & RX_BUF_MASK];
	rx_tail++;
	return true;
}

/* Blocking byte-by-byte TX.  Frames are at most 41 bytes, so
 * blocking is acceptable at 57600 baud (< 6 ms per frame).    */
static void uart_send(const uint8_t *data, uint8_t len)
{
	for (uint8_t i = 0; i < len; i++) {
		uart_poll_out(uart_dev, data[i]);
	}
}

/* ─────────────────────────────────────────────────────────────
 * MAVLink v1 — minimal encoder
 * ───────────────────────────────────────────────────────────── */

/*
 * CRC-16/MCRF4XX — accumulate one byte into *crc.
 * Call for every byte from 'len' through end of payload, then
 * call once more with the message-specific CRC_EXTRA to finalise.
 */
static void crc_accum(uint16_t *crc, uint8_t x)
{
	uint8_t t = x ^ (uint8_t)(*crc & 0xFF);
	t ^= t << 4;
	*crc = (*crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ ((uint16_t)t >> 4);
}

static uint8_t g_seq;   /* rolling MAVLink frame sequence counter */

/*
 * Build one complete MAVLink v1 frame into 'out'.
 * Frame layout:  0xFE | len | seq | sys=255 | comp=0 | msgid
 *                | payload… | crc_lo | crc_hi
 * Returns total byte count written to 'out'.
 */
static uint8_t mav_frame(uint8_t *out, uint8_t msgid,
			  const uint8_t *payload, uint8_t plen,
			  uint8_t crc_extra)
{
	out[0] = 0xFE;        /* MAVLink v1 start byte          */
	out[1] = plen;
	out[2] = g_seq++;
	out[3] = 255;         /* system-ID:    255 = GCS        */
	out[4] = 0;           /* component-ID: 0                */
	out[5] = msgid;
	memcpy(&out[6], payload, plen);

	/* CRC over bytes [1 .. 5+plen], then mix in CRC_EXTRA  */
	uint16_t crc = 0xFFFF;
	for (uint8_t i = 1; i < 6u + plen; i++) {
		crc_accum(&crc, out[i]);
	}
	crc_accum(&crc, crc_extra);

	out[6 + plen] = (uint8_t)(crc & 0xFF);
	out[7 + plen] = (uint8_t)(crc >> 8);
	return 8u + plen;
}

/* HEARTBEAT  (msgid=0, plen=9, crc_extra=50) */
static uint8_t mav_heartbeat(uint8_t *out)
{
	const uint8_t p[9] = {
		0, 0, 0, 0,   /* custom_mode = 0  (uint32 LE)          */
		6,            /* type       = MAV_TYPE_GCS (6)          */
		8,            /* autopilot  = MAV_AUTOPILOT_INVALID (8) */
		0,            /* base_mode                              */
		0,            /* system_status                          */
		3,            /* mavlink_version = 3                    */
	};
	return mav_frame(out, 0, p, sizeof(p), 50);
}

/* COMMAND_LONG  (msgid=76, plen=33, crc_extra=152) */
static uint8_t mav_command_long(uint8_t *out, uint16_t cmd,
				float p1, float p2, float p3,
				float p4, float p5, float p6, float p7)
{
	uint8_t p[33];
	memcpy(&p[ 0], &p1, 4);   /* param1 */
	memcpy(&p[ 4], &p2, 4);   /* param2 */
	memcpy(&p[ 8], &p3, 4);   /* param3 */
	memcpy(&p[12], &p4, 4);   /* param4 */
	memcpy(&p[16], &p5, 4);   /* param5 */
	memcpy(&p[20], &p6, 4);   /* param6 */
	memcpy(&p[24], &p7, 4);   /* param7 */
	p[28] = (uint8_t)(cmd & 0xFF);   /* command lo          */
	p[29] = (uint8_t)(cmd >> 8);     /* command hi          */
	p[30] = 1;                        /* target_system    = 1 (PX4) */
	p[31] = 1;                        /* target_component = 1       */
	p[32] = 0;                        /* confirmation               */
	return mav_frame(out, 76, p, sizeof(p), 152);
}

/* ─────────────────────────────────────────────────────────────
 * MAVLink v1 — minimal decoder
 * ───────────────────────────────────────────────────────────── */

/* Data parsed from incoming MAVLink frames; updated by rx_parse_task,
 * read by mission_task.  Both run in the same thread — no locking needed. */
static int32_t  g_rel_alt_mm;   /* Relative altitude in mm (GLOBAL_POSITION_INT) */
static bool     g_ack_ready;    /* True when a fresh COMMAND_ACK was received     */
static uint16_t g_ack_cmd;      /* Which command was acknowledged                 */
static uint8_t  g_ack_result;   /* 0 = MAV_RESULT_ACCEPTED                        */

/* Last received Pixhawk heartbeat fields */
static struct {
	uint8_t type, autopilot, base_mode;
} g_px4_hb;

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

/* CRC_EXTRA is a per-message-type seed byte defined in the MAVLink spec */
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

/* Called once a complete, CRC-verified frame has been received */
static void mav_dispatch(uint32_t msgid, const uint8_t *payload)
{
	switch (msgid) {
	case 0:
		/* HEARTBEAT from vehicle
		 * payload[4] = type, payload[5] = autopilot, payload[6] = base_mode */
		g_px4_hb.type      = payload[4];
		g_px4_hb.autopilot = payload[5];
		g_px4_hb.base_mode = payload[6];
		printk("[RX-HB] type=%s  autopilot=%s  armed=%s  base_mode=0x%02x\n",
		       mav_type_str(g_px4_hb.type),
		       mav_autopilot_str(g_px4_hb.autopilot),
		       (g_px4_hb.base_mode & 0x80) ? "YES" : "NO",
		       g_px4_hb.base_mode);
		break;
	case 33:
		/* GLOBAL_POSITION_INT
		 * relative_alt field: int32_t at byte offset 16, unit = mm */
		memcpy(&g_rel_alt_mm, &payload[16], 4);
		break;

	case 77:
		/* COMMAND_ACK
		 * command: uint16_t at offset 0
		 * result:  uint8_t  at offset 2  (0 = accepted) */
		memcpy(&g_ack_cmd, &payload[0], 2);
		g_ack_result = payload[2];
		g_ack_ready  = true;
		break;

	default:
		break;
	}
}

/* Parser state — module-level so it persists across rx_parse_task calls */
static enum {
	RX_STX, RX_LEN,
	RX_V2_INCOMPAT, RX_V2_COMPAT,           /* MAVLink v2 only         */
	RX_SEQ, RX_SYS, RX_COMP,
	RX_MSGID, RX_V2_MSGID_1, RX_V2_MSGID_2, /* v2 has 24-bit message ID */
	RX_PAYLOAD, RX_CRC1, RX_CRC2
} rx_state;

static bool     rx_is_v2;
static uint8_t  rx_plen, rx_payload[64], rx_pidx, rx_crc1_got;
static uint32_t rx_msgid;   /* 8-bit for v1, 24-bit for v2 */
static uint16_t rx_crc_run;

/* Feed one received byte into the MAVLink v1/v2 parser state machine */
static void mav_parse_byte(uint8_t b)
{
	switch (rx_state) {

	case RX_STX:
		if (b == 0xFE) {        /* MAVLink v1 */
			rx_is_v2   = false;
			rx_crc_run = 0xFFFF;
			rx_state   = RX_LEN;
		} else if (b == 0xFD) { /* MAVLink v2 */
			rx_is_v2   = true;
			rx_crc_run = 0xFFFF;
			rx_state   = RX_LEN;
		}
		break;

	case RX_LEN:
		crc_accum(&rx_crc_run, b);
		rx_plen  = b;
		rx_state = rx_is_v2 ? RX_V2_INCOMPAT : RX_SEQ;
		break;

	case RX_V2_INCOMPAT:
		crc_accum(&rx_crc_run, b);
		/* Non-zero incompat flags (e.g. signing) = can't parse; resync */
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
		rx_msgid = b;           /* bits 0-7 (complete for v1) */
		rx_pidx  = 0;
		if (rx_is_v2) {
			rx_state = RX_V2_MSGID_1;
		} else {
			rx_state = (rx_plen > 0) ? RX_PAYLOAD : RX_CRC1;
		}
		break;

	case RX_V2_MSGID_1:
		crc_accum(&rx_crc_run, b);
		rx_msgid |= ((uint32_t)b << 8);   /* bits 8-15 */
		rx_state  = RX_V2_MSGID_2;
		break;

	case RX_V2_MSGID_2:
		crc_accum(&rx_crc_run, b);
		rx_msgid |= ((uint32_t)b << 16);  /* bits 16-23 */
		rx_state  = (rx_plen > 0) ? RX_PAYLOAD : RX_CRC1;
		break;

	case RX_PAYLOAD:
		crc_accum(&rx_crc_run, b);
		if (rx_pidx < sizeof(rx_payload)) {
			rx_payload[rx_pidx] = b;
		}
		rx_pidx++;
		if (rx_pidx >= rx_plen) {
			rx_state = RX_CRC1;
		}
		break;

	case RX_CRC1:
		rx_crc1_got = b;
		rx_state    = RX_CRC2;
		break;

	case RX_CRC2: {
		uint16_t final_crc = rx_crc_run;
		crc_accum(&final_crc, crc_extra_for(rx_msgid));
		uint16_t got_crc = rx_crc1_got | ((uint16_t)b << 8);
		if (final_crc == got_crc) {
			mav_dispatch(rx_msgid, rx_payload);
		}
		rx_state = RX_STX;
		break;
	}
	} /* switch */
}

/* ─────────────────────────────────────────────────────────────
 * EDF Scheduler
 * ───────────────────────────────────────────────────────────── */

struct edf_task {
	const char *name;
	void       (*run)(void);    /* task function (cooperative: runs to completion) */
	int64_t     period_ms;      /* how often the task activates                    */
	int64_t     deadline_ms;    /* relative deadline counted from activation time  */
	int64_t     next_wake_ms;   /* next absolute activation time (ms since boot)   */
	int64_t     abs_deadline;   /* absolute deadline for the current instance      */
	bool        ready;          /* activated and waiting to run                    */
};

/*
 * One EDF scheduling step:
 *   1. Activate every task whose period has elapsed.
 *   2. Among all ready tasks, select the one with the earliest
 *      absolute deadline  (Earliest-Deadline-First rule).
 *   3. Run it.
 *   4. Rearm for the next period.
 *   5. Warn on deadline miss.
 */
static void edf_tick(struct edf_task *tasks, int n)
{
	int64_t now = k_uptime_get();

	/* Step 1 — activate newly due tasks */
	for (int i = 0; i < n; i++) {
		if (!tasks[i].ready && now >= tasks[i].next_wake_ms) {
			tasks[i].ready        = true;
			tasks[i].abs_deadline = now + tasks[i].deadline_ms;
		}
	}

	/* Step 2 — EDF selection */
	int sel = -1;

	for (int i = 0; i < n; i++) {
		if (!tasks[i].ready) {
			continue;
		}
		if (sel < 0 || tasks[i].abs_deadline < tasks[sel].abs_deadline) {
			sel = i;
		}
	}

	if (sel < 0) {
		return;   /* nothing ready */
	}

	/* Step 3 — run */
	int64_t deadline = tasks[sel].abs_deadline;

	tasks[sel].run();

	/* Step 4 — rearm */
	tasks[sel].ready        = false;
	tasks[sel].next_wake_ms = k_uptime_get() + tasks[sel].period_ms;

	/* Step 5 — deadline miss check */
	int64_t finish = k_uptime_get();

	if (finish > deadline) {
		printk("[EDF] *** DEADLINE MISSED: %s (+%lld ms late) ***\n",
		       tasks[sel].name, (long long)(finish - deadline));
	}
}

/* ─────────────────────────────────────────────────────────────
 * Task 1 — Heartbeat  (T=1000 ms, D=900 ms)
 *
 * PX4 drops the MAVLink connection if it does not receive a
 * heartbeat within 1 second.  Sending one every 1000 ms with a
 * 900 ms deadline keeps the link alive with margin.
 * ───────────────────────────────────────────────────────────── */
static void heartbeat_task(void)
{
	uint8_t frame[17];   /* max HEARTBEAT frame = 8 header + 9 payload */
	uint8_t len = mav_heartbeat(frame);

	uart_send(frame, len);
	printk("[HBEAT] sent\n");
}

/* ─────────────────────────────────────────────────────────────
 * Task 2 — RX parse  (T=50 ms, D=40 ms)
 *
 * Drains all bytes accumulated in the ring buffer by the UART ISR
 * and feeds them through the MAVLink byte parser.
 * ───────────────────────────────────────────────────────────── */
static void rx_parse_task(void)
{
	static uint32_t last_report_bytes;
	static int64_t  last_report_ms;
	uint8_t b;

	while (rx_get(&b)) {
		mav_parse_byte(b);
	}

	/* Print once per second when new bytes have arrived */
	uint32_t cur = g_rx_bytes_total;
	int64_t  now = k_uptime_get();

	if (cur != last_report_bytes && (now - last_report_ms) >= 1000) {
		printk("[RX] %u bytes total from Pixhawk\n", (unsigned)cur);
		last_report_bytes = cur;
		last_report_ms    = now;
	}
}

/* ─────────────────────────────────────────────────────────────
 * Task 3 — Mission state machine  (T=500 ms, D=450 ms)
 * ───────────────────────────────────────────────────────────── */

/* PX4 MAVLink command IDs */
#define CMD_DO_SET_MODE   176u   /* MAV_CMD_DO_SET_MODE            */
#define CMD_ARM_DISARM    400u   /* MAV_CMD_COMPONENT_ARM_DISARM   */
#define CMD_TAKEOFF        22u   /* MAV_CMD_NAV_TAKEOFF             */
#define CMD_LAND           21u   /* MAV_CMD_NAV_LAND                */

/* Mission parameters */
#define TAKEOFF_ALT_M     10     /* target altitude (m)             */
#define TAKEOFF_ALT_MM  9000     /* "at altitude" threshold (mm)    */
#define LANDED_ALT_MM    500     /* "landed" threshold (mm)         */
#define HOVER_MS        3000     /* hover duration before landing   */
#define MAX_CMD_RETRIES    3     /* give up after this many transient rejections */

static enum {
	M_IDLE,
	M_SET_MODE,          /* send DO_SET_MODE → PX4 AUTO mode    */
	M_WAIT_MODE_ACK,     /* wait for COMMAND_ACK                */
	M_ARM,               /* send ARM command                    */
	M_WAIT_ARM_ACK,      /* wait for COMMAND_ACK                */
	M_TAKEOFF,           /* send TAKEOFF command                */
	M_CLIMBING,          /* monitor altitude until >= 9 m       */
	M_HOVERING,          /* wait HOVER_MS at altitude           */
	M_LAND,              /* send LAND command                   */
	M_DESCENDING,        /* monitor altitude until <= 0.5 m     */
	M_DONE               /* mission complete                    */
} mission_state = M_IDLE;

static int64_t hover_start_ms;

static const char *mav_result_str(uint8_t r)
{
	switch (r) {
	case 0: return "ACCEPTED";
	case 1: return "TEMPORARILY_REJECTED";
	case 2: return "DENIED";
	case 3: return "UNSUPPORTED";
	case 4: return "FAILED (prearm checks)";
	case 5: return "IN_PROGRESS";
	default: return "UNKNOWN";
	}
}

static void mission_task(void)
{
	uint8_t frame[64];
	uint8_t len;

	switch (mission_state) {

	/* ── Step 0: kick off ────────────────────────────────── */
	case M_IDLE:
		printk("[MISSION] Starting — setting AUTO mode\n");
		mission_state = M_SET_MODE;
		break;

	/* ── Step 1: Set PX4 to AUTO mode ───────────────────── */
	case M_SET_MODE:
		g_ack_ready = false;
		/*
		 * MAV_CMD_DO_SET_MODE (176)
		 *   param1 = 1.0  → MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
		 *   param2 = 4.0  → PX4 main mode 4 = AUTO
		 *   param3 = 0.0  → sub-mode: let PX4 choose
		 */
		len = mav_command_long(frame, CMD_DO_SET_MODE,
				       1.0f, 4.0f, 0.0f,
				       0.0f, 0.0f, 0.0f, 0.0f);
		uart_send(frame, len);
		printk("[MISSION] Sent: SET_MODE AUTO\n");
		mission_state = M_WAIT_MODE_ACK;
		break;

	case M_WAIT_MODE_ACK: {
		static int mode_retries;

		if (g_ack_ready && g_ack_cmd == CMD_DO_SET_MODE) {
			g_ack_ready = false;
			if (g_ack_result == 0) {
				printk("[MISSION] Mode ACK OK\n");
				mode_retries  = 0;
				mission_state = M_ARM;
			} else {
				mode_retries++;
				printk("[MISSION] SET_MODE result=%d (%s), retry %d/%d\n",
				       g_ack_result, mav_result_str(g_ack_result),
				       mode_retries, MAX_CMD_RETRIES);
				if (mode_retries >= MAX_CMD_RETRIES) {
					printk("[MISSION] SET_MODE failed — aborting.\n");
					mode_retries  = 0;
					mission_state = M_DONE;
				} else {
					mission_state = M_SET_MODE;
				}
			}
		}
		break;
	}

	/* ── Step 2: Arm motors ──────────────────────────────── */
	case M_ARM:
		g_ack_ready = false;
		/*
		 * MAV_CMD_COMPONENT_ARM_DISARM (400)
		 *   param1 = 1.0  → ARM
		 *   param2 = 0.0  → normal arm (safety checks enabled)
		 *            Use 21196.0 to force-arm in simulation / SITL.
		 */
		len = mav_command_long(frame, CMD_ARM_DISARM,
				       1.0f, 0.0f, 0.0f,
				       0.0f, 0.0f, 0.0f, 0.0f);
		uart_send(frame, len);
		printk("[MISSION] Sent: ARM\n");
		mission_state = M_WAIT_ARM_ACK;
		break;

	case M_WAIT_ARM_ACK: {
		static int arm_retries;

		if (g_ack_ready && g_ack_cmd == CMD_ARM_DISARM) {
			g_ack_ready = false;
			if (g_ack_result == 0) {
				printk("[MISSION] ARM ACK OK\n");
				arm_retries   = 0;
				mission_state = M_TAKEOFF;
			} else if (g_ack_result == 4 /* MAV_RESULT_FAILED */) {
				/* Prearm checks are blocking arm — retrying won't help
				 * until hardware/config issues are resolved.            */
				printk("[MISSION] ARM FAILED — prearm checks not passing.\n");
				printk("[MISSION] Fix all QGC warnings (ESCs, GPS, RC) "
				       "before arming. Aborting mission.\n");
				arm_retries   = 0;
				mission_state = M_DONE;
			} else {
				/* Transient rejection (e.g. IN_PROGRESS) — retry */
				arm_retries++;
				printk("[MISSION] ARM result=%d (%s), retry %d/%d\n",
				       g_ack_result, mav_result_str(g_ack_result),
				       arm_retries, MAX_CMD_RETRIES);
				if (arm_retries >= MAX_CMD_RETRIES) {
					printk("[MISSION] ARM failed after %d attempts "
					       "— aborting.\n", arm_retries);
					arm_retries   = 0;
					mission_state = M_DONE;
				} else {
					mission_state = M_ARM;
				}
			}
		}
		break;
	}

	/* ── Step 3: Take off to 10 m ───────────────────────── */
	case M_TAKEOFF:
		g_ack_ready = false;
		/*
		 * MAV_CMD_NAV_TAKEOFF (22)
		 *   param7 = target altitude in metres, relative to home
		 */
		len = mav_command_long(frame, CMD_TAKEOFF,
				       0.0f, 0.0f, 0.0f,
				       0.0f, 0.0f, 0.0f, (float)TAKEOFF_ALT_M);
		uart_send(frame, len);
		printk("[MISSION] Sent: TAKEOFF to %d m\n", TAKEOFF_ALT_M);
		mission_state = M_CLIMBING;
		break;

	case M_CLIMBING:
		printk("[MISSION] Climbing — rel_alt = %d mm\n", (int)g_rel_alt_mm);
		if (g_rel_alt_mm >= TAKEOFF_ALT_MM) {
			hover_start_ms = k_uptime_get();
			printk("[MISSION] Reached %d m — hovering for %d s\n",
			       TAKEOFF_ALT_M, HOVER_MS / 1000);
			mission_state = M_HOVERING;
		}
		break;

	/* ── Step 4: Hover ───────────────────────────────────── */
	case M_HOVERING:
		if ((k_uptime_get() - hover_start_ms) >= HOVER_MS) {
			printk("[MISSION] Hover complete — initiating landing\n");
			mission_state = M_LAND;
		}
		break;

	/* ── Step 5: Land at same position ──────────────────── */
	case M_LAND:
		g_ack_ready = false;
		/*
		 * MAV_CMD_NAV_LAND (21)
		 *   All params = 0 → land at current (home) position
		 */
		len = mav_command_long(frame, CMD_LAND,
				       0.0f, 0.0f, 0.0f,
				       0.0f, 0.0f, 0.0f, 0.0f);
		uart_send(frame, len);
		printk("[MISSION] Sent: LAND\n");
		mission_state = M_DESCENDING;
		break;

	case M_DESCENDING:
		printk("[MISSION] Descending — rel_alt = %d mm\n", (int)g_rel_alt_mm);
		if (g_rel_alt_mm <= LANDED_ALT_MM) {
			printk("[MISSION] Landed — mission complete!\n");
			mission_state = M_DONE;
		}
		break;

	case M_DONE:
		/* Nothing left to do */
		break;
	}
}

/* ─────────────────────────────────────────────────────────────
 * main
 * ───────────────────────────────────────────────────────────── */
int main(void)
{
	/* Initialise the MAVLink UART */
	uart_dev = DEVICE_DT_GET(MAVLINK_UART_NODE);
	if (!device_is_ready(uart_dev)) {
		printk("ERROR: MAVLink UART (USART1) not ready — check boards/nucleo_c562re.overlay\n");
		return -1;
	}
	uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
	uart_irq_rx_enable(uart_dev);

	printk("PX4 MAVLink link ready — USART1 at 57600 baud (PB14=TX, PB15=RX)\n");

	/*
	 * EDF task set.
	 * next_wake_ms = 0 means all tasks fire immediately at boot.
	 * EDF will order the first burst by deadline: rx_parse (40ms)
	 * first, then mission (450ms), then heartbeat (900ms).
	 */
	struct edf_task tasks[] = {
		/* name          run fn           T(ms)  D(ms)  next  abs_dl  ready */
		{ "heartbeat", heartbeat_task, 1000,  900,    0,    0,     false },
		{ "rx_parse",  rx_parse_task,    50,   40,    0,    0,     false },
		{ "mission",   mission_task,    500,  450,    0,    0,     false },
	};

	const int num_tasks = ARRAY_SIZE(tasks);

	printk("EDF scheduler started (%d tasks)\n", num_tasks);

	/* Main loop — 1 ms polling granularity */
	while (true) {
		edf_tick(tasks, num_tasks);
		k_msleep(1);
	}

	return 0;
}
