/*
 *	wifi_sm.h
 *	Subsystem 1 (Buddy 1) -- Wi-Fi / MQTT state machine (statechart 6.1).
 *
 *	Pure C: every action goes through a WIFI_OPS table, so the same code
 *	runs inside the Wi-Fi Telemetry Task on the Pico W and inside the PC
 *	unit tests with fake network/kernel operations.
 *
 *	wifi_sm_step() never blocks and never loops on the network: each call
 *	evaluates at most WIFI_SM_MAX_TRANSITIONS transitions and returns.
 *	It must be called from the task that owns the cyw43/lwIP stack.
 */

#ifndef WIFI_SM_H
#define WIFI_SM_H

#include "../app_data.h"
#include "wifi_cmd.h"

/* ---- States (8 -> 3 bits) ------------------------------------------- */
typedef enum {
	WIFI_ST_DISCONNECTED = 0,
	WIFI_ST_CONNECTING_WIFI,
	WIFI_ST_CONNECTING_MQTT,
	WIFI_ST_SHARE_TELEMETRY,
	WIFI_ST_TRANSMIT_PAYLOAD,	/* transient */
	WIFI_ST_PARSE_REMOTE_CMD,	/* transient */
	WIFI_ST_REMOTE_ABORT_REQUESTED,	/* transient */
	WIFI_ST_RECONNECTING,
	WIFI_ST_COUNT
} WIFI_STATE;
#define WIFI_STATE_BITS		3u
_Static_assert(APP_BITS_FOR(WIFI_ST_COUNT) == WIFI_STATE_BITS, "wifi state width");

/* ---- Timing (ms) ----------------------------------------------------- */
#define WIFI_TLM_PERIOD_MS	100u	/* 10 Hz (NFR1.1) */
#define WIFI_LIVENESS_MS	1500u	/* no TCP ack for this long -> link down (NFR1.3) */
#define WIFI_JOIN_TIMEOUT_MS	15000u	/* association + DHCP */
#define WIFI_JOIN_FAIL_GRACE_MS	1000u	/* ignore a stale FAIL right after join */
#define WIFI_MQTT_TIMEOUT_MS	5000u	/* TCP connect + CONNACK + SUBACK */
#define WIFI_BACKOFF_BASE_MS	500u	/* 500, 1000, 2000, 4000, 4000 ... */
#define WIFI_BACKOFF_MAX_MS	4000u
#define WIFI_LEAVE_AFTER_RETRIES 2u	/* force a fresh association after this */

/* ---- Buffers --------------------------------------------------------- */
#define WIFI_BACKLOG_DEPTH	16u	/* outbound frames kept during an outage */
#define WIFI_FLUSH_PER_TICK	3u	/* backlog frames sent per 100 ms when back */
#define WIFI_RX_SLOTS		4u	/* received command payloads waiting */
#define WIFI_RX_MAX		8u	/* bytes kept per payload (frame is 3) */
#define WIFI_SM_MAX_TRANSITIONS	16u

/* ---- Interfaces used by the state machine --------------------------- */
typedef enum {
	WIFI_LINK_DOWN = 0,
	WIFI_LINK_JOINING,
	WIFI_LINK_UP,
	WIFI_LINK_FAIL
} WIFI_LINK;

typedef enum {
	WIFI_MQTT_IDLE = 0,
	WIFI_MQTT_CONNECTING,
	WIFI_MQTT_CONNECTED,
	WIFI_MQTT_SUBSCRIBED,
	WIFI_MQTT_FAILED
} WIFI_MQTT;

#define WIFI_PUB_OK		0
#define WIFI_PUB_BUSY		(-1)	/* output buffer / request slots full */
#define WIFI_PUB_DISCONNECTED	(-2)

typedef enum {
	WIFI_EV_STATE = 0,	/* a = old state, b = new state (persistent only) */
	WIFI_EV_LINK_DOWN,	/* a = cause FLT_*, b = ms since last TCP ack */
	WIFI_EV_RECOVERED,	/* a = ms from RECONNECTING to first publish */
	WIFI_EV_CMD,		/* a = WIFI_CMD_VERDICT, b = type << 8 | seq */
	WIFI_EV_ROUTE_FAIL,	/* a = command type (remote_cmd_q full) */
	WIFI_EV_ABORT,		/* a = seq */
	WIFI_EV_BACKLOG_DROP	/* a = total frames dropped so far */
} WIFI_EVENT;

typedef struct {
	/* network: only ever called from the stack-owning task */
	bool		(*radio_ready)(void);
	WIFI_LINK	(*link)(void);
	bool		(*wifi_join)(void);
	void		(*wifi_leave)(void);
	bool		(*mqtt_connect)(void);
	WIFI_MQTT	(*mqtt_state)(void);
	bool		(*mqtt_subscribe)(void);
	void		(*mqtt_disconnect)(void);
	int		(*mqtt_publish)(const uint8_t *frame, uint32_t len);
	uint32_t	(*mqtt_unacked)(void);
	uint32_t	(*mqtt_progress_ms)(void);	/* last TCP progress */
	/* system: all non-blocking */
	bool		(*estop_latched)(void);
	bool		(*route_cmd)(const APP_CMD *c);	/* -> remote_cmd_q */
	void		(*raise_abort)(void);		/* -> flg_safety */
	void		(*faults_changed)(uint16_t b1_faults);	/* -> flg_fault */
	void		(*snapshot)(APP_TELEM *t);	/* latest vehicle fields + faults */
	void		(*event)(WIFI_EVENT ev, uint32_t a, uint32_t b);
} WIFI_OPS;

typedef struct {
	uint32_t	frames_built;
	uint32_t	frames_published;	/* live + backlog */
	uint32_t	frames_backlogged;
	uint32_t	backlog_flushed;
	uint32_t	backlog_dropped;	/* drop-oldest events */
	uint32_t	backlog_max;		/* high-water mark */
	uint32_t	publish_busy;
	uint32_t	encode_errors;
	uint32_t	tick_overruns;		/* telemetry period missed */
	uint32_t	link_downs;
	uint32_t	reconnect_attempts;
	uint32_t	last_detect_ms, max_detect_ms;
	uint32_t	last_recovery_ms, max_recovery_ms;
	uint32_t	cmd_rx, cmd_rx_overflow;
	uint32_t	cmd_verdicts[CMD_V_COUNT];
	uint32_t	route_fail;
	uint32_t	aborts;
} WIFI_STATS;

typedef struct {
	const WIFI_OPS	*ops;
	WIFI_STATE	state;
	WIFI_STATE	return_state;	/* where PARSE/ABORT go back to */
	uint32_t	t_enter;	/* entry time of the persistent state */
	uint32_t	backoff_ms;
	uint32_t	next_due;
	uint32_t	t_down;		/* when the link was lost */
	uint16_t	cause;		/* FLT_* that sent us to RECONNECTING */
	uint16_t	b1_faults;	/* shadow of the B1-owned bits of flg_fault */
	uint8_t		retry;		/* 0..APP_RETRY_MAX */
	uint8_t		tlm_seq;
	bool		join_issued;
	bool		sub_issued;
	bool		recovering;
	APP_CMD		abort_cmd;
	WIFI_CMD_FILTER	filter;
	/* received command payloads (filled by wifi_sm_on_command) */
	uint8_t		rx_buf[WIFI_RX_SLOTS][WIFI_RX_MAX];
	uint8_t		rx_len[WIFI_RX_SLOTS];
	uint8_t		rx_head, rx_count;
	/* outbound backlog ring (drop-oldest) */
	uint8_t		bl[WIFI_BACKLOG_DEPTH][TLM_FRAME_LEN];
	uint8_t		bl_head, bl_count;
	WIFI_STATS	stats;
} WIFI_SM;

void wifi_sm_init(WIFI_SM *sm, const WIFI_OPS *ops, uint32_t now_ms);
void wifi_sm_step(WIFI_SM *sm, uint32_t now_ms);

/* Hand one received MQTT payload to the state machine (same task only).
   len is the TOTAL payload length; bytes beyond WIFI_RX_MAX are dropped
   and the command is then rejected as BAD_LENGTH. */
void wifi_sm_on_command(WIFI_SM *sm, const uint8_t *data, uint32_t len);

/* Raise a B1 report-once fault (FLT_B1_REPORT_ONCE bits only) detected
   outside the state machine, e.g. a telemetry_q overflow (same task only). */
void wifi_sm_report_fault(WIFI_SM *sm, uint16_t bits);

const char *wifi_sm_state_name(WIFI_STATE s);

#endif /* WIFI_SM_H */
