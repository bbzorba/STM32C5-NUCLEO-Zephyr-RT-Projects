#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include "../inc/serial_com.h"
#include "../inc/mavlink_rx.h"
#include "../inc/mavlink_tx.h"

/* UART ring buffer ------------------------------------------- */

/* v1 (0xFE) and v2 (0xFD) parser state machine */

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
