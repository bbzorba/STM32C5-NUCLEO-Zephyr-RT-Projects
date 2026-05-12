#ifndef MAVLINK_RX_H
#define MAVLINK_RX_H

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

/* Pixhawk heartbeat fields (msgid=0) — written by mavlink_rx.c, read by main.c */
extern struct px4_hb_t {
	uint8_t type, autopilot, base_mode;
} px4_hb;

/* IMU raw data (msgid=27 RAW_IMU) — written by mavlink_rx.c, read by main.c */
extern struct imu_t {
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	int16_t mx, my, mz;
} imu;

void parse_byte(uint8_t b);
void on_message(uint32_t id, const uint8_t *p);

#endif /* MAVLINK_RX_H */
