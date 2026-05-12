#ifndef MAVLINK_TX_H
#define MAVLINK_TX_H

#include <stdint.h>

void send_heartbeat(void);
void send_set_mode(uint32_t custom_mode);
void send_command_long(uint16_t cmd,
		       float p1, float p2, float p3,
		       float p4, float p5, float p6, float p7);

#endif /* MAVLINK_TX_H */
