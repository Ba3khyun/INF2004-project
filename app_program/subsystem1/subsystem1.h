/*
 *	subsystem1.h
 *	Subsystem 1 -- Wi-Fi Communication, Command and Telemetry (Buddy 1,
 *	Lai Jia Bao, 2500915).  Also still hosts the template's LED heartbeat
 *	example, because subsystem 4's example (IF-05) suspends that task; it
 *	goes when the subsystem 4 example is replaced.
 *
 *	This header is subsystem 1's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem1/subsystem1.h".
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

/* ---------------------------------------------------------------------
 *  Wi-Fi Telemetry (Buddy 1) -- public interface
 * --------------------------------------------------------------------- */
#define S1_PRI_WIFI		PRI_WIFI	/* 40: lowest application priority */

/* MQTT topics (robot/<id>/...).  Change WIFI_ROBOT_ID here (or pass
   -DWIFI_ROBOT_ID=...) if two robots share a broker. */
#ifndef WIFI_ROBOT_ID
#define WIFI_ROBOT_ID		"is06"
#endif
#define WIFI_TOPIC_STATE	"robot/" WIFI_ROBOT_ID "/state"	/* publish   */
#define WIFI_TOPIC_CMD		"robot/" WIFI_ROBOT_ID "/cmd"	/* subscribe */

/* IF-B1-01 telemetry_q: any task -> Wi-Fi Telemetry Task.
   Bounded message buffer, 16 x TQ_MSG_LEN (4 bytes, app_data.h).
   Producers call telemetry_post(); never block, drop-oldest when full. */
#define WIFI_TELEM_Q_DEPTH	16
IMPORT ID	telemetry_q;

/*
 * telemetry_post -- report the latest value of one telemetry field.
 *   field  TQ_FIELD_VEH_STATE  (Vehicle Controller)  APP_VS_*
 *          TQ_FIELD_SPEED      (Motion Control)      cm/s, -100..100 or APP_SPEED_NA
 *          TQ_FIELD_HUMP       (IMU Processing)      mm, 0..62 or APP_HUMP_MM_NA
 *          TQ_FIELD_OBSTACLE   (Obstacle Scanning)   cm, 0..100 or APP_OBST_CM_NA
 *   One producer task per field.  Non-blocking (TMO_POL only), no lock,
 *   callable from any task at any priority (not from an ISR).
 *   Returns E_OK   queued (the oldest message may have been dropped),
 *           E_PAR  invalid field or out-of-range value: nothing queued,
 *           E_QOVR queue still full after dropping the oldest: dropped.
 */
IMPORT ER	telemetry_post(UINT field, INT value);

/* ---------------------------------------------------------------------
 *  LED heartbeat (template example)
 * --------------------------------------------------------------------- */
#define S1_PRI_LED	12		/* priority band 12-13 (TEAM.md) */
#define S1_LED_PIN	19		/* GP19: LED + 330R resistor to GND */
#define S1_TICK_MS	250		/* LED toggles every tick */

/* IF-05: the LED task's ID, so subsystem 4 may pause/resume it */
IMPORT ID	tid_led;

/* Private to subsystem 1, shared between its own .c files */
IMPORT ID	cyc_led;
IMPORT void	led_task(INT stacd, void *exinf);
IMPORT void	led_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM1_H */
