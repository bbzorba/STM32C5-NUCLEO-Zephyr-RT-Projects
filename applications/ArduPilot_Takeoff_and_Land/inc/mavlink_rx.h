#ifndef MAVLINK_RX_H
#define MAVLINK_RX_H

#include <stdbool.h>
#include <stdint.h>

/* Shared state — written by mavlink_rx.c, read by mission.c */
extern volatile int32_t  g_rel_alt_mm;  /* GLOBAL_POSITION_INT relative alt (mm) */
extern volatile bool     g_ack_ready;   /* fresh COMMAND_ACK available            */
extern volatile uint16_t g_ack_cmd;     /* which command was ACK'd                */
extern volatile uint8_t  g_ack_result;  /* 0 = MAV_RESULT_ACCEPTED                */

extern struct ap_hb_t {
	uint8_t type, autopilot, base_mode;
} g_ap_hb;

void mav_parse_byte(uint8_t b);

#endif /* MAVLINK_RX_H */
