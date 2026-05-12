#include "../inc/mission.h"
#include "../inc/mavlink_tx.h"
#include "../inc/mavlink_rx.h"
#include <zephyr/kernel.h>

/* ArduPilot MAVLink command IDs */
#define CMD_DO_SET_MODE  176u   /* MAV_CMD_DO_SET_MODE            */
#define CMD_ARM_DISARM   400u   /* MAV_CMD_COMPONENT_ARM_DISARM   */
#define CMD_TAKEOFF       22u   /* MAV_CMD_NAV_TAKEOFF             */
#define CMD_LAND          21u   /* MAV_CMD_NAV_LAND                */

/* Mission parameters */
#define TAKEOFF_ALT_M    10     /* target altitude in metres       */
#define TAKEOFF_ALT_MM   9000   /* "reached altitude" threshold mm */
#define LANDED_ALT_MM    500    /* "landed" threshold mm           */
#define HOVER_MS         3000   /* hover duration before landing   */
#define MAX_CMD_RETRIES  3

static enum {
	M_IDLE,
	M_SET_MODE,       M_WAIT_MODE_ACK,
	M_ARM,            M_WAIT_ARM_ACK,
	M_TAKEOFF,        M_CLIMBING,
	M_HOVERING,
	M_LAND,           M_DESCENDING,
	M_DONE
} mission_state = M_IDLE;

static int64_t hover_start_ms;
static int64_t mode_wait_ms;

static const char *mav_result_str(uint8_t r)
{
	switch (r) {
	case 0: return "ACCEPTED";
	case 1: return "TEMPORARILY_REJECTED";
	case 2: return "DENIED";
	case 3: return "UNSUPPORTED";
	case 4: return "FAILED (prearm checks)";
	case 5: return "IN_PROGRESS";
	default: return "UNKNOWN";
	}
}

void mission_step(void)
{
	switch (mission_state) {

	case M_IDLE:
		printk("[MISSION] Starting — setting GUIDED mode\n");
		mission_state = M_SET_MODE;
		break;

	/* ── Step 1: Set ArduCopter to GUIDED mode ─────────── */
	case M_SET_MODE:
		/* SET_MODE (msgid=11): ArduPilot does NOT send COMMAND_ACK for this.
		 * base_mode=1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), custom_mode=4 (GUIDED) */
		send_set_mode(4);
		printk("[MISSION] Sent: SET_MODE GUIDED (ArduCopter custom_mode=4)\n");
		mode_wait_ms  = k_uptime_get();
		mission_state = M_WAIT_MODE_ACK;
		break;

	case M_WAIT_MODE_ACK:
		/* No COMMAND_ACK expected — just wait 500 ms for ArduPilot to switch mode */
		if ((k_uptime_get() - mode_wait_ms) >= 500) {
			printk("[MISSION] Mode delay done — proceeding to ARM\n");
			mission_state = M_ARM;
		}
		break;

	/* ── Step 2: Arm motors ──────────────────────────────── */
	case M_ARM:
		g_ack_ready = false;
		/* MAV_CMD_COMPONENT_ARM_DISARM: param1=1 (ARM) */
		send_command_long(CMD_ARM_DISARM, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		printk("[MISSION] Sent: ARM\n");
		mission_state = M_WAIT_ARM_ACK;
		break;

	case M_WAIT_ARM_ACK: {
		static int arm_retries;

		if (g_ack_ready && g_ack_cmd == CMD_ARM_DISARM) {
			g_ack_ready = false;
			if (g_ack_result == 0) {
				printk("[MISSION] ARM ACK OK\n");
				arm_retries   = 0;
				mission_state = M_TAKEOFF;
			} else if (g_ack_result == 4) {
				printk("[MISSION] ARM FAILED — prearm checks not passing. Aborting.\n");
				arm_retries   = 0;
				mission_state = M_DONE;
			} else {
				printk("[MISSION] ARM result=%d (%s), retry %d/%d\n",
				       g_ack_result, mav_result_str(g_ack_result),
				       ++arm_retries, MAX_CMD_RETRIES);
				if (arm_retries >= MAX_CMD_RETRIES) {
					printk("[MISSION] ARM failed — aborting.\n");
					arm_retries   = 0;
					mission_state = M_DONE;
				} else {
					mission_state = M_ARM;
				}
			}
		}
		break;
	}

	/* ── Step 3: Take off to TAKEOFF_ALT_M metres ───────── */
	case M_TAKEOFF:
		g_ack_ready = false;
		/* MAV_CMD_NAV_TAKEOFF: param7 = target altitude (m) relative to home */
		send_command_long(CMD_TAKEOFF, 0.0f, 0.0f, 0.0f,
				  0.0f, 0.0f, 0.0f, (float)TAKEOFF_ALT_M);
		printk("[MISSION] Sent: TAKEOFF to %d m\n", TAKEOFF_ALT_M);
		mission_state = M_CLIMBING;
		break;

	case M_CLIMBING:
		printk("[MISSION] Climbing — rel_alt = %d mm\n", (int)g_rel_alt_mm);
		if (g_rel_alt_mm >= TAKEOFF_ALT_MM) {
			hover_start_ms = k_uptime_get();
			printk("[MISSION] Reached %d m — hovering for %d s\n",
			       TAKEOFF_ALT_M, HOVER_MS / 1000);
			mission_state = M_HOVERING;
		}
		break;

	/* ── Step 4: Hover ───────────────────────────────────── */
	case M_HOVERING:
		if ((k_uptime_get() - hover_start_ms) >= HOVER_MS) {
			printk("[MISSION] Hover complete — initiating landing\n");
			mission_state = M_LAND;
		}
		break;

	/* ── Step 5: Land ────────────────────────────────────── */
	case M_LAND:
		g_ack_ready = false;
		/* MAV_CMD_NAV_LAND: all params = 0 → land at current position */
		send_command_long(CMD_LAND, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		printk("[MISSION] Sent: LAND\n");
		mission_state = M_DESCENDING;
		break;

	case M_DESCENDING:
		printk("[MISSION] Descending — rel_alt = %d mm\n", (int)g_rel_alt_mm);
		if (g_rel_alt_mm <= LANDED_ALT_MM) {
			printk("[MISSION] Landed — mission complete!\n");
			mission_state = M_DONE;
		}
		break;

	case M_DONE:
		/* Nothing left to do */
		break;
	}
}
