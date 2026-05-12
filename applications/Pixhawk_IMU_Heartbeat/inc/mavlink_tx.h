#ifndef MAVLINK_TX_H
#define MAVLINK_TX_H

#include <zephyr/device.h>

void send_heartbeat(void);
void send_msg(uint8_t msgid, const uint8_t *p, uint8_t plen, uint8_t crc_extra);
void request_imu_stream(void);

#endif /* MAVLINK_TX_H */