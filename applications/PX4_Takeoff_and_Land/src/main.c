/*
 * PX4 Takeoff-and-Land via MAVLink  Zephyr RTOS
 *
 * Hardware:
 *   NUCLEO-C562RE USART1 (PB14=TX, PB15=RX) â†’ PX4 TELEM1  (57600 baud)
 *
 * Mission:  SET_MODE AUTO â†’ ARM â†’ TAKEOFF 10 m â†’ HOVER 3 s â†’ LAND
 *
 * Scheduling  Zephyr preemptive threads (Rate Monotonic):
 *
 *   Thread        Period    Priority   Purpose
 *   rx_parse      50 ms     5          Drain UART buffer, parse MAVLink
 *   mission       500 ms    7          Advance mission state machine
 *   heartbeat     1000 ms   9          Keep MAVLink link alive
 *
 * Lower priority number = higher scheduling priority in Zephyr.
 * Shorter period assigned higher priority (RMS optimal).
 */

/* ============================================================
 * Logic is now split into:
 *   serial_com.h/c   UART ring buffer + ISR
 *   mavlink_rx.h/c   MAVLink v1/v2 parser + shared RX state
 *   mavlink_tx.h/c   MAVLink frame builder (heartbeat, command_long)
 *   mission.h/c      mission state machine
 * ============================================================ */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include "../inc/serial_com.h"
#include "../inc/mavlink_rx.h"
#include "../inc/mavlink_tx.h"
#include "../inc/mission.h"

#define STACK_SIZE        1024
#define RX_PERIOD_MS        50
#define MISSION_PERIOD_MS  500
#define HB_PERIOD_MS      1000

/* Thread 1  RX parse (highest priority, shortest period)
 * Drains every byte from the ring buffer and feeds the MAVLink parser. */
static void rx_parse_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	uint8_t byte;

	while (true) {
		while (rx_get(&byte)) mav_parse_byte(byte);
		k_msleep(RX_PERIOD_MS);
	}
}

/* Thread 2  Mission (medium priority, medium period)
 * Calls mission_step() each period to advance the state machine. */
static void mission_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	while (true) {
		mission_step();
		k_msleep(MISSION_PERIOD_MS);
	}
}

/* Thread 3  Heartbeat (lowest priority, longest period)
 * Sends a MAVLink HEARTBEAT every second to keep the PX4 link alive. */
static void heartbeat_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	while (true) {
		send_heartbeat();
		printk("[HBEAT] sent\n");
		k_msleep(HB_PERIOD_MS);
	}
}

K_THREAD_DEFINE(rx_tid,  STACK_SIZE, rx_parse_thread,  NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(mis_tid, STACK_SIZE, mission_thread,   NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(hb_tid,  STACK_SIZE, heartbeat_thread, NULL, NULL, NULL, 9, 0, 0);

int main(void)
{
	uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart1));
	if (!device_is_ready(uart_dev)) {
		printk("ERROR: USART1 not ready  check boards/nucleo_c562re.overlay\n");
		return -1;
	}

	uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
	uart_irq_rx_enable(uart_dev);

	printk("PX4 MAVLink ready  USART1 at 57600 baud (PB14=TX, PB15=RX)\n");
	return 0;
}
