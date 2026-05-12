#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#define RX_BUF  256u
#define RX_MASK (RX_BUF - 1u)

extern uint8_t           rx_buf[RX_BUF];
extern volatile uint32_t rx_head;
extern          uint32_t rx_tail;

extern const struct device *uart_dev;

void uart_isr(const struct device *dev, void *ud);
bool rx_get(uint8_t *b);
void serial_write(const uint8_t *d, uint8_t n);

#endif /* SERIAL_COM_H */
