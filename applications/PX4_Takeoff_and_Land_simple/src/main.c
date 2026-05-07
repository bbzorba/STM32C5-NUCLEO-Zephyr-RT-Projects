#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

/* UART ring buffer ------------------------------------------- */

static const struct device *uart_dev;

#define RX_BUF  256u
#define RX_MASK (RX_BUF - 1u)

static uint8_t           rx_buf[RX_BUF];
static volatile uint32_t rx_head;
static          uint32_t rx_tail;

static void uart_isr(const struct device *dev, void *ud)
{
	ARG_UNUSED(ud);
	uint8_t c;

	while (uart_irq_update(dev) && uart_irq_rx_ready(dev))
		if (uart_fifo_read(dev, &c, 1) == 1)
			rx_buf[rx_head++ & RX_MASK] = c;
}

static bool rx_get(uint8_t *b)
{
	if (rx_tail == rx_head) return false;
	*b = rx_buf[rx_tail++ & RX_MASK];
	return true;
}

static void serial_write(const uint8_t *d, uint8_t n)
{
	for (uint8_t i = 0; i < n; i++) uart_poll_out(uart_dev, d[i]);
}

/* MAVLink encoder ------------------------------------------ */

static uint8_t tx_seq;

static void crc_feed(uint16_t *crc, uint8_t b)
{
	uint8_t t = b ^ (uint8_t)(*crc);

	t ^= t << 4;
	*crc = (*crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ (t >> 4);
}

static void send_msg(uint8_t msgid, const uint8_t *p, uint8_t plen, uint8_t crc_extra)
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

static void send_heartbeat(void)
{
	uint8_t p[9] = { 0, 0, 0, 0, 6, 8, 0, 0, 3 };

	send_msg(0, p, 9, 50);
}

static void send_command(uint16_t cmd, float p1, float p2, float p7)
{
	uint8_t p[33] = {0};

	memcpy(&p[0],  &p1, 4);
	memcpy(&p[4],  &p2, 4);
	memcpy(&p[24], &p7, 4);
	p[28] = cmd & 0xFF;
	p[29] = cmd >> 8;
	p[30] = 1;
	p[31] = 1;
	send_msg(76, p, 33, 152);
}

/* Ask Pixhawk to stream RAW_SENSORS group (includes RAW_IMU) at 10 Hz.
 * Must be sent periodically — PX4 stops streaming if the GCS goes silent. */
static void request_raw_sensor_stream(void)
{
	uint8_t p[6];
	uint16_t rate = 10;

	p[0] = rate & 0xFF;
	p[1] = rate >> 8;
	p[2] = 1;   /* target_system              */
	p[3] = 0;   /* target_component (all)     */
	p[4] = 1;   /* MAV_DATA_STREAM_RAW_SENSORS */
	p[5] = 1;   /* 1 = start                  */
	send_msg(66, p, 6, 148);
}

/* MAVLink decoder ------------------------------------------ */

static bool     ack_ready;
static uint16_t ack_cmd;
static uint8_t  ack_result;
static int32_t  rel_alt_mm;

static struct {
	int16_t ax, ay, az;   /* accelerometer (RAW_IMU msgid=27) */
	int16_t gx, gy, gz;   /* gyroscope                        */
	int16_t mx, my, mz;   /* magnetometer                     */
} imu;

static uint8_t msg_crc_extra(uint32_t id)
{
	switch (id) {
	case  0: return  50;   /* HEARTBEAT           */
	case 27: return 144;   /* RAW_IMU             */
	case 33: return 104;   /* GLOBAL_POSITION_INT */
	case 77: return 143;   /* COMMAND_ACK         */
	default: return   0;
	}
}

static void on_message(uint32_t id, const uint8_t *p)
{
	if (id == 27) {
		memcpy(&imu.ax, &p[ 8], 2); memcpy(&imu.ay, &p[10], 2); memcpy(&imu.az, &p[12], 2);
		memcpy(&imu.gx, &p[14], 2); memcpy(&imu.gy, &p[16], 2); memcpy(&imu.gz, &p[18], 2);
		memcpy(&imu.mx, &p[20], 2); memcpy(&imu.my, &p[22], 2); memcpy(&imu.mz, &p[24], 2);
	} else if (id == 33) {
		memcpy(&rel_alt_mm, &p[16], 4);
	} else if (id == 77) {
		memcpy(&ack_cmd, &p[0], 2);
		ack_result = p[2];
		ack_ready  = true;
	}
}

/* v1 (0xFE) and v2 (0xFD) parser state machine */
static enum { PS_STX, PS_LEN, PS_INC, PS_COM, PS_SEQ, PS_SYS, PS_CMP,
	      PS_ID0, PS_ID1, PS_ID2, PS_PAY, PS_C1,  PS_C2 } ps;

static bool     ps_v2;
static uint8_t  ps_plen, ps_pidx, ps_pay[64], ps_c1;
static uint32_t ps_mid;
static uint16_t ps_crc;

static void parse_byte(uint8_t b)
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

/* Mission state machine ------------------------------------- */

#define CMD_SET_MODE  176u
#define CMD_ARM       400u
#define CMD_TAKEOFF    22u
#define CMD_LAND       21u

static enum {
	M_IDLE,
	M_SET_MODE, M_WAIT_MODE_ACK,
	M_ARM,      M_WAIT_ARM_ACK,
	M_TAKEOFF,  M_CLIMBING,
	M_HOVERING,
	M_LAND,
	M_DONE
} mission_state = M_IDLE;

static int64_t hover_start_ms;

static void run_mission(void)
{
	switch (mission_state) {

	case M_IDLE:
		printk("[MISSION] Starting\n");
		mission_state = M_SET_MODE;
		break;

	case M_SET_MODE:
		ack_ready = false;
		send_command(CMD_SET_MODE, 1.0f, 4.0f, 0.0f);
		printk("[MISSION] SET_MODE AUTO\n");
		mission_state = M_WAIT_MODE_ACK;
		break;

	case M_WAIT_MODE_ACK:
		if (ack_ready && ack_cmd == CMD_SET_MODE) {
			ack_ready = false;
			printk("[MISSION] Mode ACK result=%d\n", ack_result);
			mission_state = (ack_result == 0) ? M_ARM : M_DONE;
		}
		break;

	case M_ARM:
		ack_ready = false;
		send_command(CMD_ARM, 1.0f, 0.0f, 0.0f);
		printk("[MISSION] ARM\n");
		mission_state = M_WAIT_ARM_ACK;
		break;

	case M_WAIT_ARM_ACK:
		if (ack_ready && ack_cmd == CMD_ARM) {
			ack_ready = false;
			printk("[MISSION] ARM ACK result=%d\n", ack_result);
			if (ack_result == 4) {
				printk("[MISSION] Prearm checks failed - fix QGC warnings first\n");
			}
			mission_state = (ack_result == 0) ? M_TAKEOFF : M_DONE;
		}
		break;

	case M_TAKEOFF:
		ack_ready = false;
		send_command(CMD_TAKEOFF, 0.0f, 0.0f, 10.0f);
		printk("[MISSION] TAKEOFF to 10m\n");
		mission_state = M_CLIMBING;
		break;

	case M_CLIMBING:
		printk("[MISSION] Climbing - alt=%d mm\n", (int)rel_alt_mm);
		if (rel_alt_mm >= 9000) {
			hover_start_ms = k_uptime_get();
			printk("[MISSION] Reached 10m - hovering\n");
			mission_state = M_HOVERING;
		}
		break;

	case M_HOVERING:
		if (k_uptime_get() - hover_start_ms >= 3000) {
			printk("[MISSION] Hover done - landing\n");
			mission_state = M_LAND;
		}
		break;

	case M_LAND:
		send_command(CMD_LAND, 0.0f, 0.0f, 0.0f);
		printk("[MISSION] LAND\n");
		mission_state = M_DONE;
		break;

	case M_DONE:
		break;
	}
}

/* main ------------------------------------------------------ */

int main(void)
{
	uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart1));
	if (!device_is_ready(uart_dev)) {
		printk("UART not ready\n");
		return -1;
	}
	uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
	uart_irq_rx_enable(uart_dev);

	printk("MAVLink ready - USART1 57600 baud (PB14=TX, PB15=RX)\n");

	request_raw_sensor_stream();

	int64_t last_heartbeat_ms  = 0;
	int64_t last_mission_ms    = 0;
	int64_t last_stream_req_ms = 0;

	while (true) {
		uint8_t b;

		while (rx_get(&b)) parse_byte(b);

		int64_t now = k_uptime_get();

		if (now - last_heartbeat_ms >= 1000) {
			last_heartbeat_ms = now;
			send_heartbeat();
			printk("[HBEAT] accel=%d,%d,%d  gyro=%d,%d,%d  mag=%d,%d,%d\n",
			       imu.ax, imu.ay, imu.az,
			       imu.gx, imu.gy, imu.gz,
			       imu.mx, imu.my, imu.mz);
		}

		if (now - last_mission_ms >= 500) {
			last_mission_ms = now;
			run_mission();
		}

		if (now - last_stream_req_ms >= 5000) {
			last_stream_req_ms = now;
			request_raw_sensor_stream();
		}

		k_msleep(10);
	}

	return 0;
}
