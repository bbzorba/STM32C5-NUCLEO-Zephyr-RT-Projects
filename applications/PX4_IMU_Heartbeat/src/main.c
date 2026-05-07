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

/* MAVLink encoder -------------------------------------------- */

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

static void request_imu_stream(void)
{
	/* REQUEST_DATA_STREAM (msgid=66): RAW_SENSORS group at 10 Hz */
	uint8_t p[6] = { 10, 0, 1, 0, 1, 1 };

	send_msg(66, p, 6, 148);
}

/* MAVLink decoder -------------------------------------------- */

/* Pixhawk heartbeat fields (msgid=0) */
static struct {
	uint8_t type, autopilot, base_mode;
} px4_hb;

/* IMU raw data (msgid=27 RAW_IMU) */
static struct {
	int16_t ax, ay, az;   /* accelerometer (RAW_IMU msgid=27) */
	int16_t gx, gy, gz;   /* gyroscope                        */
	int16_t mx, my, mz;   /* magnetometer                     */
} imu;

static uint8_t msg_crc_extra(uint32_t id)
{
	switch (id) {
	case  0: return  50;   /* HEARTBEAT */
	case 27: return 144;   /* RAW_IMU   */
	default: return   0;
	}
}

static void on_message(uint32_t id, const uint8_t *p)
{
	if (id == 0) {
		/* HEARTBEAT: custom_mode[0-3], type[4], autopilot[5], base_mode[6] */
		px4_hb.type      = p[4];
		px4_hb.autopilot = p[5];
		px4_hb.base_mode = p[6];
		printk("[RX-HB] type=%u  autopilot=%u  base_mode=0x%02x\n",
		       px4_hb.type, px4_hb.autopilot, px4_hb.base_mode);
	} else if (id == 27) {
		/* RAW_IMU: time_usec[0-7], xacc[8], yacc[10], zacc[12],
		 *          xgyro[14], ygyro[16], zgyro[18],
		 *          xmag[20],  ymag[22],  zmag[24]           */
		memcpy(&imu.ax, &p[ 8], 2); memcpy(&imu.ay, &p[10], 2); memcpy(&imu.az, &p[12], 2);
		memcpy(&imu.gx, &p[14], 2); memcpy(&imu.gy, &p[16], 2); memcpy(&imu.gz, &p[18], 2);
		memcpy(&imu.mx, &p[20], 2); memcpy(&imu.my, &p[22], 2); memcpy(&imu.mz, &p[24], 2);
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

/* Scheduler: Rate Monotonic Scheduling (RMS) -----------------
 *
 * RMS is optimal for fixed-period tasks: assign static priorities
 * inversely proportional to period (shorter period = higher priority).
 * Zephyr's preemptive thread scheduler implements RMS when priorities
 * are set this way. Lower number = higher priority in Zephyr.
 *
 *   Thread      Period   Deadline   Priority   Purpose
 *   rx_parse    20 ms    18 ms      5          Drain UART, parse MAVLink
 *   report      1000 ms  900 ms     7          Send heartbeat, print data
 *
 * Utilization: (1/20) + (1/100) ≈ 6%  <<  RMS bound of 82.8% for 2 tasks.
 * ------------------------------------------------------------ */

#define RX_PERIOD_MS      20
#define RX_DEADLINE_MS    18
#define REPORT_PERIOD_MS  1000
#define REPORT_DEADLINE_MS 900

#define RX_PRIORITY     5
#define REPORT_PRIORITY 7
#define STACK_SIZE      1024

/* Thread 1 — RX parse (highest priority, shortest period) */
static void rx_parse_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	while (true) {
		int64_t t0 = k_uptime_get();
		uint8_t byte;

		while (rx_get(&byte)) parse_byte(byte);

		int64_t elapsed = k_uptime_get() - t0;

		if (elapsed > RX_DEADLINE_MS) {
			printk("[SCHED] rx_parse deadline miss (%lld ms)\n", (long long)elapsed);
		}

		k_msleep(RX_PERIOD_MS - MIN(elapsed, RX_PERIOD_MS));
	}
}

/* Thread 2 — Report + heartbeat TX (lower priority, longer period) */
static void report_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	int stream_tick = 0;

	while (true) {
		int64_t t0 = k_uptime_get();

		send_heartbeat();
		printk("[TX-HB] sent\n");

		printk("[IMU] accel=%d,%d,%d\n", imu.ax, imu.ay, imu.az);
		printk("[IMU]  gyro=%d,%d,%d\n", imu.gx, imu.gy, imu.gz);
		printk("[IMU]   mag=%d,%d,%d\n", imu.mx, imu.my, imu.mz);
		printk("\n");

		/* Re-request stream every 5 s so Pixhawk keeps sending after reboot */
		if (++stream_tick >= 5) {
			stream_tick = 0;
			request_imu_stream();
		}

		int64_t elapsed = k_uptime_get() - t0;

		if (elapsed > REPORT_DEADLINE_MS) {
			printk("[SCHED] report deadline miss (%lld ms)\n", (long long)elapsed);
		}

		k_msleep(REPORT_PERIOD_MS - MIN(elapsed, REPORT_PERIOD_MS));
	}
}

K_THREAD_DEFINE(rx_tid,     STACK_SIZE, rx_parse_thread, NULL, NULL, NULL, RX_PRIORITY,     0, 0);
K_THREAD_DEFINE(report_tid, STACK_SIZE, report_thread,   NULL, NULL, NULL, REPORT_PRIORITY, 0, 0);

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

	/* Ask Pixhawk to start streaming RAW_IMU immediately */
	request_imu_stream();

	return 0;
}

/*
 * ============================================================
 * HOW TO SPLIT INTO SUB-FILES
 * ============================================================
 *
 * src/
 *   main.c          — main(), K_THREAD_DEFINE, thread functions
 *   uart.h/.c       — rx_buf, uart_isr(), rx_get(), serial_write()
 *   mavlink_tx.h/.c — send_msg(), send_heartbeat(), request_imu_stream()
 *   mavlink_rx.h/.c — parse_byte(), on_message(), imu struct, px4_hb struct
 *
 * In CMakeLists.txt add:
 *   target_sources(app PRIVATE
 *     src/main.c
 *     src/uart.c
 *     src/mavlink_tx.c
 *     src/mavlink_rx.c
 *   )
 *
 * In uart.h, expose:
 *   extern const struct device *uart_dev;
 *   void uart_isr(const struct device *dev, void *ud);
 *   bool rx_get(uint8_t *b);
 *   void serial_write(const uint8_t *d, uint8_t n);
 *
 * In mavlink_rx.h, expose:
 *   extern struct { uint8_t type, autopilot, base_mode; } px4_hb;
 *   extern struct { int16_t ax,ay,az, gx,gy,gz, mx,my,mz; } imu;
 *   void parse_byte(uint8_t b);
 * ============================================================
 */

