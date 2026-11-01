/*
 *	wifi_sm.c
 *	Subsystem 1 (Buddy 1) -- Wi-Fi / MQTT state machine (see wifi_sm.h and
 *	docs/buddy1_wifi_telemetry.md, statechart 6.1).
 *
 *	Persistent states hold until an event: DISCONNECTED, CONNECTING_WIFI,
 *	CONNECTING_MQTT, SHARE_TELEMETRY, RECONNECTING.
 *	Transient states run once and leave in the same step: TRANSMIT_PAYLOAD,
 *	PARSE_REMOTE_CMD, REMOTE_ABORT_REQUESTED.
 */

#include "wifi_sm.h"

#define ELAPSED(now, t)		((uint32_t)((now) - (t)))
#define REACHED(now, t)		((int32_t)((uint32_t)((now) - (t))) >= 0)

static const char *const state_names[WIFI_ST_COUNT] = {
	"DISCONNECTED", "CONNECTING_WIFI", "CONNECTING_MQTT", "SHARE_TELEMETRY",
	"TRANSMIT_PAYLOAD", "PARSE_REMOTE_CMD", "REMOTE_ABORT_REQUESTED",
	"RECONNECTING"
};

const char *wifi_sm_state_name(WIFI_STATE s)
{
	return ((unsigned)s < (unsigned)WIFI_ST_COUNT) ? state_names[s] : "?";
}

static bool is_transient(WIFI_STATE s)
{
	return s == WIFI_ST_TRANSMIT_PAYLOAD || s == WIFI_ST_PARSE_REMOTE_CMD ||
	       s == WIFI_ST_REMOTE_ABORT_REQUESTED;
}

/* ---- B1 fault shadow: only this module writes the B1-owned bits ------ */
static void set_b1_faults(WIFI_SM *sm, uint16_t faults)
{
	faults = (uint16_t)(faults & FLT_B1_OWNED);
	if (faults != sm->b1_faults) {
		sm->b1_faults = faults;
		sm->ops->faults_changed(faults);
	}
}

/* ---- Outbound backlog (bounded ring, drop-oldest) -------------------- */
static void backlog_push(WIFI_SM *sm, const uint8_t frame[TLM_FRAME_LEN])
{
	uint32_t	tail, i;

	if (sm->bl_count == WIFI_BACKLOG_DEPTH) {
		sm->bl_head = (uint8_t)((sm->bl_head + 1u) % WIFI_BACKLOG_DEPTH);
		sm->bl_count--;
		sm->stats.backlog_dropped++;
		set_b1_faults(sm, (uint16_t)(sm->b1_faults | FLT_TELEM_OVERFLOW));
		sm->ops->event(WIFI_EV_BACKLOG_DROP, sm->stats.backlog_dropped, 0u);
	}
	tail = (sm->bl_head + sm->bl_count) % WIFI_BACKLOG_DEPTH;
	for (i = 0u; i < TLM_FRAME_LEN; i++) {
		sm->bl[tail][i] = frame[i];
	}
	app_telem_set_backlog(sm->bl[tail]);
	sm->bl_count++;
	sm->stats.frames_backlogged++;
	if (sm->bl_count > sm->stats.backlog_max) {
		sm->stats.backlog_max = sm->bl_count;
	}
}

/* ---- Telemetry frame ------------------------------------------------- */
static bool build_frame(WIFI_SM *sm, uint32_t now, uint8_t out[TLM_FRAME_LEN],
			uint16_t *faults_sent)
{
	APP_TELEM	t;

	app_telem_default(&t);
	sm->ops->snapshot(&t);
	t.version   = (uint8_t)APP_PROTO_VERSION;
	t.faults    = (uint16_t)((t.faults & (uint16_t)~FLT_B1_OWNED) | sm->b1_faults);
	t.seq       = sm->tlm_seq;
	t.time_10ms = (uint16_t)(now / APP_TLM_TIME_UNIT_MS);
	t.retry     = sm->retry;
	t.backlog   = false;
	if (app_telem_encode(&t, out) != APP_OK) {
		sm->stats.encode_errors++;
		return false;
	}
	sm->tlm_seq = (uint8_t)(sm->tlm_seq + 1u);	/* wraps 255 -> 0 */
	sm->stats.frames_built++;
	*faults_sent = t.faults;
	return true;
}

/* true once per period; resynchronises after an overrun (no burst) */
static bool telemetry_due(WIFI_SM *sm, uint32_t now)
{
	if (!REACHED(now, sm->next_due)) {
		return false;
	}
	sm->next_due += WIFI_TLM_PERIOD_MS;
	if (REACHED(now, sm->next_due)) {
		sm->stats.tick_overruns++;
		sm->next_due = now + WIFI_TLM_PERIOD_MS;
	}
	return true;
}

/* Age of the last TCP progress.  'now' is sampled once at the start of a
   poll, but the MQTT layer stamps progress with its own sys_now() call
   later in the same poll, so progress may be slightly AFTER now (a tick or
   a preemption in between).  Treat that as age 0, never as a wrap-around
   of almost 2^32 ms, which would be a false SOCK_TIMEOUT. */
static uint32_t progress_age(WIFI_SM *sm, uint32_t now)
{
	int32_t	age = (int32_t)(uint32_t)(now - sm->ops->mqtt_progress_ms());

	return (age > 0) ? (uint32_t)age : 0u;
}

static bool liveness_expired(WIFI_SM *sm, uint32_t now)
{
	return sm->ops->mqtt_unacked() > 0u &&
	       progress_age(sm, now) >= WIFI_LIVENESS_MS;
}

/* 0 if the link is usable, else the FLT_* bit describing why not */
static uint16_t link_fault(WIFI_SM *sm, uint32_t now)
{
	if (sm->ops->link() != WIFI_LINK_UP) {
		return FLT_WIFI_LINK;
	}
	if (sm->ops->mqtt_state() != WIFI_MQTT_SUBSCRIBED) {
		return FLT_MQTT_LINK;
	}
	if (liveness_expired(sm, now)) {
		return FLT_SOCK_TIMEOUT;
	}
	return 0u;
}

/* ---- Entry actions ---------------------------------------------------- */
static void enter(WIFI_SM *sm, WIFI_STATE next, uint32_t now)
{
	WIFI_STATE	prev = sm->state;
	uint32_t	shift;

	sm->state = next;
	if (is_transient(next)) {
		return;		/* t_enter keeps the persistent state's time */
	}
	sm->t_enter = now;
	sm->ops->event(WIFI_EV_STATE, (uint32_t)prev, (uint32_t)next);

	switch (next) {
	case WIFI_ST_CONNECTING_WIFI:
		sm->join_issued = false;
		sm->stats.reconnect_attempts++;
		break;

	case WIFI_ST_CONNECTING_MQTT:
		sm->sub_issued = false;
		if (!sm->ops->mqtt_connect()) {
			sm->cause = FLT_MQTT_LINK;
			enter(sm, WIFI_ST_RECONNECTING, now);
		}
		break;

	case WIFI_ST_SHARE_TELEMETRY:
		set_b1_faults(sm, (uint16_t)(sm->b1_faults & (uint16_t)~FLT_B1_LINK));
		sm->retry = 0u;
		sm->next_due = now;		/* publish at once */
		break;

	case WIFI_ST_RECONNECTING:
		if (prev == WIFI_ST_SHARE_TELEMETRY || is_transient(prev)) {
			uint32_t detect = (sm->ops->mqtt_unacked() > 0u) ?
				progress_age(sm, now) : 0u;
			sm->stats.link_downs++;
			sm->stats.last_detect_ms = detect;
			if (detect > sm->stats.max_detect_ms) {
				sm->stats.max_detect_ms = detect;
			}
			sm->t_down = now;
			sm->recovering = true;
			sm->ops->event(WIFI_EV_LINK_DOWN, sm->cause, detect);
		}
		set_b1_faults(sm, (uint16_t)(sm->b1_faults | sm->cause));
		sm->ops->mqtt_disconnect();
		if ((sm->cause & FLT_WIFI_LINK) != 0u ||
		    sm->retry >= WIFI_LEAVE_AFTER_RETRIES) {
			sm->ops->wifi_leave();
		}
		shift = (sm->retry < 3u) ? sm->retry : 3u;
		sm->backoff_ms = WIFI_BACKOFF_BASE_MS << shift;
		if (sm->backoff_ms > WIFI_BACKOFF_MAX_MS) {
			sm->backoff_ms = WIFI_BACKOFF_MAX_MS;
		}
		if (sm->retry < APP_RETRY_MAX) {
			sm->retry++;
		}
		break;

	default:
		break;
	}
}

static void go_reconnecting(WIFI_SM *sm, uint16_t cause, uint32_t now)
{
	sm->cause = cause;
	enter(sm, WIFI_ST_RECONNECTING, now);
}

/* ---- Transient state bodies ------------------------------------------- */
static void do_transmit(WIFI_SM *sm, uint32_t now)
{
	uint8_t		frame[TLM_FRAME_LEN];
	uint16_t	faults_sent = 0u;
	uint32_t	n;
	int		r;

	if (!build_frame(sm, now, frame, &faults_sent)) {
		sm->state = WIFI_ST_SHARE_TELEMETRY;
		return;
	}
	r = sm->ops->mqtt_publish(frame, TLM_FRAME_LEN);
	if (r == WIFI_PUB_OK) {
		sm->stats.frames_published++;
		/* report-once faults were in a published frame: clear them */
		set_b1_faults(sm, (uint16_t)(sm->b1_faults &
				(uint16_t)~(faults_sent & FLT_B1_REPORT_ONCE)));
		if (sm->recovering) {
			sm->recovering = false;
			sm->stats.last_recovery_ms = ELAPSED(now, sm->t_down);
			if (sm->stats.last_recovery_ms > sm->stats.max_recovery_ms) {
				sm->stats.max_recovery_ms = sm->stats.last_recovery_ms;
			}
			sm->ops->event(WIFI_EV_RECOVERED, sm->stats.last_recovery_ms, 0u);
		}
	} else {
		backlog_push(sm, frame);
		if (r == WIFI_PUB_DISCONNECTED) {
			go_reconnecting(sm, FLT_MQTT_LINK, now);
			return;
		}
		sm->stats.publish_busy++;
		sm->state = WIFI_ST_SHARE_TELEMETRY;
		return;
	}

	/* Flush a bounded number of backlog frames, oldest first */
	for (n = 0u; n < WIFI_FLUSH_PER_TICK && sm->bl_count > 0u; n++) {
		if (sm->ops->mqtt_publish(sm->bl[sm->bl_head], TLM_FRAME_LEN) != WIFI_PUB_OK) {
			break;		/* stays queued; retried next period */
		}
		sm->bl_head = (uint8_t)((sm->bl_head + 1u) % WIFI_BACKLOG_DEPTH);
		sm->bl_count--;
		sm->stats.backlog_flushed++;
		sm->stats.frames_published++;
	}
	sm->state = WIFI_ST_SHARE_TELEMETRY;
}

static void do_parse(WIFI_SM *sm)
{
	WIFI_CMD_VERDICT	v;
	APP_CMD			c = { CMD_NONE, 0u };
	uint32_t		slot = sm->rx_head;
	uint16_t		fault;

	sm->rx_head = (uint8_t)((sm->rx_head + 1u) % WIFI_RX_SLOTS);
	sm->rx_count--;

	v = wifi_cmd_check(&sm->filter, sm->rx_buf[slot], sm->rx_len[slot],
			   sm->ops->estop_latched(), &c);
	sm->stats.cmd_verdicts[v]++;
	fault = wifi_cmd_verdict_fault(v);
	if (fault != 0u) {
		set_b1_faults(sm, (uint16_t)(sm->b1_faults | fault));
	}
	sm->ops->event(WIFI_EV_CMD, (uint32_t)v, ((uint32_t)c.type << 8) | c.seq);

	if (v != CMD_V_ACCEPT) {
		sm->state = sm->return_state;	/* ignored: nothing is routed */
		return;
	}
	if (c.type == CMD_ABORT) {
		sm->abort_cmd = c;
		sm->state = WIFI_ST_REMOTE_ABORT_REQUESTED;
		return;
	}
	/* pause, or resume with ESTOP clear: a request to the Vehicle Controller */
	if (!sm->ops->route_cmd(&c)) {
		sm->stats.route_fail++;
		sm->ops->event(WIFI_EV_ROUTE_FAIL, c.type, c.seq);
	}
	sm->state = sm->return_state;
}

static void do_abort(WIFI_SM *sm, uint32_t now)
{
	/* Safety flag first: the Safety Handler (priority 5) preempts at once
	   and latches ESTOP before the Vehicle Controller sees the request. */
	sm->ops->raise_abort();
	if (!sm->ops->route_cmd(&sm->abort_cmd)) {
		sm->stats.route_fail++;
		sm->ops->event(WIFI_EV_ROUTE_FAIL, sm->abort_cmd.type, sm->abort_cmd.seq);
	}
	sm->stats.aborts++;
	sm->ops->event(WIFI_EV_ABORT, sm->abort_cmd.seq, 0u);
	sm->next_due = now;	/* report the new state in the next frame at once */

	if (sm->return_state == WIFI_ST_RECONNECTING) {
		sm->state = WIFI_ST_RECONNECTING;	/* keep its backoff timer */
	} else {
		uint16_t cause = link_fault(sm, now);

		sm->state = WIFI_ST_SHARE_TELEMETRY;	/* abort handling completed */
		if (cause != 0u) {
			go_reconnecting(sm, cause, now);	/* link still unavailable */
		}
	}
}

/* ---- Public ------------------------------------------------------------ */
void wifi_sm_init(WIFI_SM *sm, const WIFI_OPS *ops, uint32_t now_ms)
{
	uint8_t		*p = (uint8_t *)sm;
	uint32_t	i;

	for (i = 0u; i < sizeof(*sm); i++) {
		p[i] = 0u;
	}
	sm->ops = ops;
	sm->state = WIFI_ST_DISCONNECTED;
	sm->return_state = WIFI_ST_DISCONNECTED;
	sm->t_enter = now_ms;
	sm->next_due = now_ms;
	wifi_cmd_filter_init(&sm->filter);
	/* no link yet: report it from the first frame on */
	set_b1_faults(sm, (uint16_t)(FLT_WIFI_LINK | FLT_MQTT_LINK));
}

void wifi_sm_on_command(WIFI_SM *sm, const uint8_t *data, uint32_t len)
{
	uint32_t	slot, i, keep;

	sm->stats.cmd_rx++;
	if (sm->rx_count == WIFI_RX_SLOTS) {
		sm->stats.cmd_rx_overflow++;	/* newest dropped: never blocks */
		return;
	}
	slot = (sm->rx_head + sm->rx_count) % WIFI_RX_SLOTS;
	keep = (len < WIFI_RX_MAX) ? len : WIFI_RX_MAX;
	for (i = 0u; i < keep; i++) {
		sm->rx_buf[slot][i] = data[i];
	}
	sm->rx_len[slot] = (uint8_t)((len < 255u) ? len : 255u);
	sm->rx_count++;
}

void wifi_sm_report_fault(WIFI_SM *sm, uint16_t bits)
{
	set_b1_faults(sm, (uint16_t)(sm->b1_faults | (bits & FLT_B1_REPORT_ONCE)));
}

void wifi_sm_step(WIFI_SM *sm, uint32_t now)
{
	const WIFI_OPS	*ops = sm->ops;
	uint32_t	guard;
	WIFI_STATE	before;
	WIFI_LINK	link;
	WIFI_MQTT	mq;

	for (guard = 0u; guard < WIFI_SM_MAX_TRANSITIONS; guard++) {
		before = sm->state;

		switch (sm->state) {
		case WIFI_ST_DISCONNECTED:
			if (ops->radio_ready()) {
				enter(sm, WIFI_ST_CONNECTING_WIFI, now);
			}
			break;

		case WIFI_ST_CONNECTING_WIFI:
			link = ops->link();
			if (link == WIFI_LINK_UP) {
				enter(sm, WIFI_ST_CONNECTING_MQTT, now);
			} else if (!sm->join_issued) {
				sm->join_issued = true;
				if (link != WIFI_LINK_JOINING && !ops->wifi_join()) {
					go_reconnecting(sm, FLT_WIFI_LINK, now);
				}
			} else if ((link == WIFI_LINK_FAIL &&
				    ELAPSED(now, sm->t_enter) >= WIFI_JOIN_FAIL_GRACE_MS) ||
				   ELAPSED(now, sm->t_enter) >= WIFI_JOIN_TIMEOUT_MS) {
				go_reconnecting(sm, FLT_WIFI_LINK, now);
			}
			break;

		case WIFI_ST_CONNECTING_MQTT:
			mq = ops->mqtt_state();
			if (ops->link() != WIFI_LINK_UP) {
				go_reconnecting(sm, FLT_WIFI_LINK, now);
			} else if (mq == WIFI_MQTT_SUBSCRIBED) {
				enter(sm, WIFI_ST_SHARE_TELEMETRY, now);
			} else if (mq == WIFI_MQTT_FAILED ||
				   ELAPSED(now, sm->t_enter) >= WIFI_MQTT_TIMEOUT_MS) {
				go_reconnecting(sm, FLT_MQTT_LINK, now);
			} else if (mq == WIFI_MQTT_CONNECTED && !sm->sub_issued) {
				sm->sub_issued = true;
				if (!ops->mqtt_subscribe()) {
					go_reconnecting(sm, FLT_MQTT_LINK, now);
				}
			}
			break;

		case WIFI_ST_SHARE_TELEMETRY: {
			uint16_t	cause;

			if (sm->rx_count > 0u) {
				/* commands first: an abort is never delayed by a
				   failure detected in the same poll */
				sm->return_state = WIFI_ST_SHARE_TELEMETRY;
				enter(sm, WIFI_ST_PARSE_REMOTE_CMD, now);
			} else if ((cause = link_fault(sm, now)) != 0u) {
				go_reconnecting(sm, cause, now);
			} else if (telemetry_due(sm, now)) {
				enter(sm, WIFI_ST_TRANSMIT_PAYLOAD, now);
			}
			break;
		}

		case WIFI_ST_TRANSMIT_PAYLOAD:
			do_transmit(sm, now);
			break;

		case WIFI_ST_PARSE_REMOTE_CMD:
			do_parse(sm);
			break;

		case WIFI_ST_REMOTE_ABORT_REQUESTED:
			do_abort(sm, now);
			break;

		case WIFI_ST_RECONNECTING:
			if (sm->rx_count > 0u) {
				/* a command that arrived just before the link failed */
				sm->return_state = WIFI_ST_RECONNECTING;
				enter(sm, WIFI_ST_PARSE_REMOTE_CMD, now);
			} else if (ELAPSED(now, sm->t_enter) >= sm->backoff_ms) {
				enter(sm, WIFI_ST_CONNECTING_WIFI, now);
			}
			break;

		default:
			sm->state = WIFI_ST_DISCONNECTED;	/* defensive */
			break;
		}

		if (sm->state == before && !is_transient(sm->state)) {
			break;		/* settled */
		}
	}

	/* While offline, keep sampling at 10 Hz into the bounded backlog */
	if (sm->state != WIFI_ST_SHARE_TELEMETRY && telemetry_due(sm, now)) {
		uint8_t		frame[TLM_FRAME_LEN];
		uint16_t	unused;

		if (build_frame(sm, now, frame, &unused)) {
			backlog_push(sm, frame);
		}
	}
}
