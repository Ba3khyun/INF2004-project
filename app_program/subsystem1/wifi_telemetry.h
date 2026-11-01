/*
 *	wifi_telemetry.h
 *	Subsystem 1 (Buddy 1) -- PRIVATE: kernel/network glue of the Wi-Fi
 *	Telemetry Task.  Other subsystems use subsystem1.h only.
 */

#ifndef WIFI_TELEMETRY_H
#define WIFI_TELEMETRY_H

#include "subsystem1.h"
#include "wifi_sm.h"

/* telemetry_q residency statistics (enqueue -> dequeue by the Wi-Fi task) */
typedef struct {
	UW	received;		/* valid status messages dequeued */
	UW	invalid;		/* malformed messages discarded */
	UW	last_delay_ms;
	UW	max_delay_ms;
	UW	over_50ms;		/* NFR1.1 budget exceeded */
	UW	drained_max;		/* most messages taken in one poll */
	/* Time between two consecutive wifi_app_poll() calls: the radio
	   poll, any join/leave call and the sleep.  A message posted just
	   after a poll waits up to one gap, so gaps bound the queue delay. */
	UW	max_gap_us;
	UW	gaps_over_50ms;
} WIFI_QSTATS;

/* wifi_app_poll() calls since boot (no busy loop: ~50/s expected) */
IMPORT volatile UW	wifi_poll_count;

/* Per-field drop counters, written ONLY by that field's producer task
   (single writer, aligned 16-bit store: no lock needed), read by Wi-Fi. */
IMPORT volatile UH	wifi_tq_drops[TQ_FIELD_COUNT];

IMPORT ER	wifi_telemetry_init(void);
IMPORT ER	wifi_telemetry_start(void);

/* Drop-oldest, non-blocking post of one packed status message to any
   message buffer with telemetry_q's geometry (used by telemetry_post and
   by the on-target test with a private buffer). */
IMPORT ER	wifi_tq_post_to(ID mbf, CONST UB msg[TQ_MSG_LEN], volatile UH *drops);

/* 1.024 ms timestamp used inside status messages (us >> 10, wraps) */
IMPORT UH	wifi_stamp_now(void);

/* Read-only views for tests and logs */
IMPORT const WIFI_STATS  *wifi_stats(void);
IMPORT const WIFI_QSTATS *wifi_qstats(void);
IMPORT WIFI_STATE	 wifi_state(void);
IMPORT ID		 wifi_task_id;		/* task running wifi_app_poll */
IMPORT PRI		 wifi_task_pri;		/* its priority (expect 40) */

/* System-side operations (kernel objects) for an extra state machine
   instance in the on-target tests: routing to remote_cmd_q / flg_safety
   and the ESTOP read are the real ones. */
IMPORT bool	wifi_k_estop_latched(void);
IMPORT bool	wifi_k_route_cmd(const APP_CMD *c);
IMPORT void	wifi_k_raise_abort(void);

#endif /* WIFI_TELEMETRY_H */
