#include "../inc/serial_com.h"

/* Definitions of extern variables declared in serial_com.h */
uint8_t           rx_buf[RX_BUF];
volatile uint32_t rx_head;
uint32_t          rx_tail;
const struct device *uart_dev;

bool rx_get(uint8_t *b)
{
	if (rx_tail == rx_head) return false;
	*b = rx_buf[rx_tail++ & RX_MASK];
	return true;
}

void uart_isr(const struct device *dev, void *ud)
{
	ARG_UNUSED(ud);
	uint8_t c;

	while (uart_irq_update(dev) && uart_irq_rx_ready(dev))
		if (uart_fifo_read(dev, &c, 1) == 1)
			rx_buf[rx_head++ & RX_MASK] = c;
}

void serial_write(const uint8_t *d, uint8_t n)
{
	for (uint8_t i = 0; i < n; i++) uart_poll_out(uart_dev, d[i]);
}
