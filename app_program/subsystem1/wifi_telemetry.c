/*
 *	wifi_telemetry.c
 *	Subsystem 1 (Buddy 1) -- the Wi-Fi Telemetry Task: kernel and network
 *	glue around the pure state machine in wifi_sm.c.
 *
 *	Where it runs
 *	  NET=lwip MQTT=1: inside cyw43_utk_task (priority 40, core 0), the one
 *	    task that owns the CYW43439, lwIP and MQTT (NO_SYS=1: no other task
 *	    may call them).  cyw43_utk_task calls wifi_app_poll() every
 *	    iteration (~10 ms with the 10 ms kernel tick).
 *	  any other build: in wifi_task (priority 40), created by
 *	    wifi_telemetry_start(), with a "no network" back end, so telemetry_q
 *	    is still drained and the queue/backlog/command logic still runs.
 *
 *	What it never does
 *	  write motion_cmd_q or motor PWM, clear ESTOP, wait on anything
 *	  (every kernel call is TMO_POL or non-blocking), hold a lock, or
 *	  disable interrupts.
 */

#include "wifi_telemetry.h"

#if TM_MQTT
#include "cyw43_utk.h"
#include "mqtt_utk.h"
#endif

EXPORT ID		telemetry_q;
EXPORT volatile UH	wifi_tq_drops[TQ_FIELD_COUNT];
EXPORT ID		wifi_task_id;
EXPORT PRI		wifi_task_pri;

#define WIFI_DRAIN_MAX		32	/* bound the work per poll */
#define WIFI_STATS_PERIOD_MS	10000	/* periodic console summary */
#define WIFI_TASK_PERIOD_MS	10	/* fallback task poll period */

LOCAL WIFI_SM		wifi_sm;
LOCAL WIFI_QSTATS	qstats;
LOCAL INT		latest[TQ_FIELD_COUNT];	/* latest value per field */
LOCAL UH		drops_seen[TQ_FIELD_COUNT];
LOCAL volatile BOOL	wifi_ready;		/* set once by wifi_telemetry_start */
LOCAL UW		next_stats_ms;
LOCAL UW		last_poll_us;
EXPORT volatile UW	wifi_poll_count;	/* written by the Wi-Fi task only */

/* ===================================================================
 *  Time
 * =================================================================== */
LOCAL UW now_ms(void)
{
	SYSTIM	t;

	tk_get_otm(&t);		/* same clock as lwIP's sys_now() */
	return t.lo;
}

EXPORT UH wifi_stamp_now(void)
{
	/* RP2040 free-running 1 MHz timer, read-only, safe from any core */
	return (UH)((in_w(TIMER_TIMERAWL) >> 10) & 0xFFFFu);
}

/* ===================================================================
 *  telemetry_q producers (called by OTHER tasks)
 * =================================================================== */
EXPORT ER wifi_tq_post_to(ID mbf, CONST UB msg[TQ_MSG_LEN], volatile UH *drops)
{
	UB	old[TQ_MSG_LEN];
	ER	er;

	er = tk_snd_mbf(mbf, msg, TQ_MSG_LEN, TMO_POL);
	if (er != E_TMOUT) {
		return er;
	}
	/* Full: drop the OLDEST message (any task may receive from a message
	   buffer; the kernel serialises it), then retry once.  Two kernel
	   calls, no lock, no waiting. */
	(void)tk_rcv_mbf(mbf, old, TMO_POL);
	(*drops)++;
	er = tk_snd_mbf(mbf, msg, TQ_MSG_LEN, TMO_POL);
	if (er == E_TMOUT) {
		(*drops)++;		/* another producer took the slot */
		return E_QOVR;
	}
	return er;
}

EXPORT ER telemetry_post(UINT field, INT value)
{
	UB	msg[TQ_MSG_LEN];

	if (app_tq_pack(msg, field, value, wifi_stamp_now()) != APP_OK) {
		return E_PAR;
	}
	return wifi_tq_post_to(telemetry_q, msg, &wifi_tq_drops[field]);
}

/* ===================================================================
 *  telemetry_q consumer (Wi-Fi task)
 * =================================================================== */
LOCAL void drain_telemetry_q(void)
{
	UB		msg[TQ_MSG_LEN];
	uint32_t	field;
	int32_t		value;
	uint16_t	stamp;
	UW		n = 0;
	INT		len;
	UH		age;
	UW		delay_ms;

	while (n < WIFI_DRAIN_MAX &&
	       (len = tk_rcv_mbf(telemetry_q, msg, TMO_POL)) > 0) {
		n++;
		if (len != (INT)TQ_MSG_LEN ||
		    app_tq_unpack(msg, &field, &value, &stamp) != APP_OK) {
			qstats.invalid++;
			continue;
		}
		latest[field] = (INT)value;
		age = (UH)(wifi_stamp_now() - stamp);		/* modulo 2^16 */
		delay_ms = ((UW)age * 1024u) / 1000u;
		qstats.received++;
		qstats.last_delay_ms = delay_ms;
		if (delay_ms > qstats.max_delay_ms) {
			qstats.max_delay_ms = delay_ms;
		}
		if (delay_ms > 50u) {
			qstats.over_50ms++;
		}
	}
	if (n > qstats.drained_max) {
		qstats.drained_max = n;
	}
}

LOCAL void check_overflow(void)
{
	UW	f;
	UH	d;

	for (f = 1; f < TQ_FIELD_COUNT; f++) {
		d = wifi_tq_drops[f];
		if (d != drops_seen[f]) {
			drops_seen[f] = d;
			wifi_sm_report_fault(&wifi_sm, FLT_TELEM_OVERFLOW);
		}
	}
}

/* ===================================================================
 *  System operations (kernel objects) -- all non-blocking
 * =================================================================== */
LOCAL UINT ref_flg(ID flg)
{
	T_RFLG	r;

	return (tk_ref_flg(flg, &r) == E_OK) ? r.flgptn : 0;
}

EXPORT bool wifi_k_estop_latched(void)
{
	return (ref_flg(flg_safety) & SAFETY_ESTOP_LATCHED) != 0;
}

EXPORT bool wifi_k_route_cmd(const APP_CMD *c)
{
	UB	msg[RCMD_MSG_LEN];

	if (app_rcmd_pack(msg, c->type, c->seq) != APP_OK) {
		return false;
	}
	return tk_snd_mbf(remote_cmd_q, msg, RCMD_MSG_LEN, TMO_POL) == E_OK;
}

EXPORT void wifi_k_raise_abort(void)
{
	CHECK(tk_set_flg(flg_safety, SAFETY_REQ_REMOTE_ABORT), "wifi abort flag");
}

/* Publish the B1-owned bits of the shared fault mask.  Only bits in
   FLT_B1_OWNED are touched; both calls are atomic in the kernel, so other
   owners' bits can never be lost (no read-modify-write here). */
LOCAL void k_faults_changed(uint16_t b1)
{
	UINT	set = (UINT)(b1 & FLT_B1_OWNED);
	UINT	clr = (UINT)(~b1 & FLT_B1_OWNED);

	if (set != 0) {
		(void)tk_set_flg(flg_fault, set);
	}
	if (clr != 0) {
		(void)tk_clr_flg(flg_fault, ~clr);	/* flgptn &= ~clr */
	}
}

LOCAL void k_snapshot(APP_TELEM *t)
{
	UINT	faults = ref_flg(flg_fault) & FLT_ALL & ~(UINT)FLT_ESTOP_LATCHED;

	if (wifi_k_estop_latched()) {
		faults |= FLT_ESTOP_LATCHED;	/* single source: Safety latch */
	}
	t->veh_state  = (uint8_t)latest[TQ_FIELD_VEH_STATE];
	t->speed_cm_s = (int16_t)latest[TQ_FIELD_SPEED];
	t->hump_mm    = (uint8_t)latest[TQ_FIELD_HUMP];
	t->obst_cm    = (uint8_t)latest[TQ_FIELD_OBSTACLE];
	t->faults     = (uint16_t)faults;
}

LOCAL void k_event(WIFI_EVENT ev, uint32_t a, uint32_t b)
{
	UW	t = now_ms();

	switch (ev) {
	case WIFI_EV_STATE:
		tm_printf((UB*)"wifi: t=%u %s -> %s\n", t,
			  wifi_sm_state_name((WIFI_STATE)a),
			  wifi_sm_state_name((WIFI_STATE)b));
		break;
	case WIFI_EV_LINK_DOWN:
		tm_printf((UB*)"wifi: t=%u LINK_DOWN cause=0x%x %u ms after last TCP ack\n",
			  t, a, b);
		break;
	case WIFI_EV_RECOVERED:
		tm_printf((UB*)"wifi: t=%u RECOVERED first publish %u ms after link loss\n",
			  t, a);
		break;
	case WIFI_EV_CMD:
		tm_printf((UB*)"wifi: t=%u cmd type=%u seq=%u -> %s\n", t,
			  (b >> 8) & 0xFFu, b & 0xFFu,
			  wifi_cmd_verdict_name((WIFI_CMD_VERDICT)a));
		break;
	case WIFI_EV_ROUTE_FAIL:
		tm_printf((UB*)"wifi: t=%u remote_cmd_q full, cmd type=%u seq=%u not routed\n",
			  t, a, b);
		break;
	case WIFI_EV_ABORT:
		tm_printf((UB*)"wifi: t=%u REMOTE ABORT seq=%u -> flg_safety + remote_cmd_q\n",
			  t, a);
		break;
	case WIFI_EV_BACKLOG_DROP:
		if (a == 1u || (a % 100u) == 0u) {	/* rate-limited */
			tm_printf((UB*)"wifi: t=%u backlog full, dropped oldest (total %u)\n",
				  t, a);
		}
		break;
	default:
		break;
	}
}

/* ===================================================================
 *  Network operations
 * =================================================================== */
#if TM_MQTT

LOCAL bool n_radio_ready(void)
{
	T_CYW43_UTK_STATUS	st;

	cyw43_utk_get_status(&st);
	return st.init_complete != 0 && st.ready != 0;
}

LOCAL WIFI_LINK n_link(void)
{
	switch (cyw43_utk_link()) {
	case CYW43_UTK_LINK_UP:		return WIFI_LINK_UP;
	case CYW43_UTK_LINK_JOINING:	return WIFI_LINK_JOINING;
	case CYW43_UTK_LINK_FAIL:	return WIFI_LINK_FAIL;
	default:			return WIFI_LINK_DOWN;
	}
}

LOCAL bool n_wifi_join(void)	{ return cyw43_utk_join() == 0; }
LOCAL void n_wifi_leave(void)	{ cyw43_utk_leave(); }
LOCAL bool n_mqtt_connect(void)	{ return mqtt_utk_connect() == MQTT_UTK_OK; }
LOCAL void n_mqtt_disconnect(void) { mqtt_utk_disconnect(); }
LOCAL bool n_mqtt_subscribe(void) { return mqtt_utk_subscribe(WIFI_TOPIC_CMD) == MQTT_UTK_OK; }
LOCAL uint32_t n_unacked(void)	{ return mqtt_utk_unacked(); }
LOCAL uint32_t n_progress(void)	{ return mqtt_utk_last_progress_ms(); }

LOCAL WIFI_MQTT n_mqtt_state(void)
{
	switch (mqtt_utk_state()) {
	case MQTT_UTK_ST_CONNECTING:	return WIFI_MQTT_CONNECTING;
	case MQTT_UTK_ST_CONNECTED:	return WIFI_MQTT_CONNECTED;
	case MQTT_UTK_ST_SUBSCRIBED:	return WIFI_MQTT_SUBSCRIBED;
	case MQTT_UTK_ST_FAILED:	return WIFI_MQTT_FAILED;
	default:			return WIFI_MQTT_IDLE;
	}
}

LOCAL int n_publish(const uint8_t *frame, uint32_t len)
{
	switch (mqtt_utk_publish(WIFI_TOPIC_STATE, frame, (uint16_t)len)) {
	case MQTT_UTK_OK:	return WIFI_PUB_OK;
	case MQTT_UTK_E_CONN:	return WIFI_PUB_DISCONNECTED;
	default:		return WIFI_PUB_BUSY;
	}
}

LOCAL BOOL topic_is_cmd(const char *topic)
{
	const char	*want = WIFI_TOPIC_CMD;

	while (*want != '\0' && *topic == *want) {
		want++;
		topic++;
	}
	return (*want == '\0' && *topic == '\0') ? TRUE : FALSE;
}

/* MQTT RX callback: runs inside cyw43_poll() in this same task */
LOCAL void n_on_rx(const char *topic, const uint8_t *data, uint16_t len,
		   uint32_t tot_len)
{
	(void)len;	/* data holds min(len, staging) bytes; tot_len decides */
	if (topic_is_cmd(topic)) {
		wifi_sm_on_command(&wifi_sm, data, tot_len);
	}
}

#else	/* no network back end in this build */

LOCAL bool n_radio_ready(void)	{ return false; }
LOCAL WIFI_LINK n_link(void)	{ return WIFI_LINK_DOWN; }
LOCAL bool n_wifi_join(void)	{ return false; }
LOCAL void n_wifi_leave(void)	{ }
LOCAL bool n_mqtt_connect(void)	{ return false; }
LOCAL void n_mqtt_disconnect(void) { }
LOCAL bool n_mqtt_subscribe(void) { return false; }
LOCAL uint32_t n_unacked(void)	{ return 0; }
LOCAL uint32_t n_progress(void)	{ return 0; }
LOCAL WIFI_MQTT n_mqtt_state(void) { return WIFI_MQTT_IDLE; }
LOCAL int n_publish(const uint8_t *frame, uint32_t len)
{
	(void)frame; (void)len;
	return WIFI_PUB_DISCONNECTED;
}

#endif

LOCAL const WIFI_OPS wifi_ops = {
	n_radio_ready, n_link, n_wifi_join, n_wifi_leave,
	n_mqtt_connect, n_mqtt_state, n_mqtt_subscribe, n_mqtt_disconnect,
	n_publish, n_unacked, n_progress,
	wifi_k_estop_latched, wifi_k_route_cmd, wifi_k_raise_abort,
	k_faults_changed, k_snapshot, k_event
};

/* ===================================================================
 *  The Wi-Fi Telemetry Task body
 * =================================================================== */
LOCAL void print_stats(UW now)
{
	const WIFI_STATS	*s = &wifi_sm.stats;

	tm_printf((UB*)"wifi: stats t=%u st=%s pub=%u built=%u bl=%u/%u(max %u)"
		  " bl_drop=%u busy=%u downs=%u det_max=%u rec_max=%u\n",
		  now, wifi_sm_state_name(wifi_sm.state), s->frames_published,
		  s->frames_built, (UW)wifi_sm.bl_count, (UW)WIFI_BACKLOG_DEPTH,
		  s->backlog_max, s->backlog_dropped, s->publish_busy,
		  s->link_downs, s->max_detect_ms, s->max_recovery_ms);
	tm_printf((UB*)"wifi: queue rx=%u bad=%u delay_last=%u max=%u >50ms=%u"
		  " polls=%u gap_max=%u us gaps>50ms=%u"
		  " drain_max=%u drops=%u/%u/%u/%u cmd=%u ok=%u\n",
		  qstats.received, qstats.invalid, qstats.last_delay_ms,
		  qstats.max_delay_ms, qstats.over_50ms, wifi_poll_count,
		  qstats.max_gap_us, qstats.gaps_over_50ms, qstats.drained_max,
		  (UW)wifi_tq_drops[1], (UW)wifi_tq_drops[2],
		  (UW)wifi_tq_drops[3], (UW)wifi_tq_drops[4],
		  s->cmd_rx, s->cmd_verdicts[CMD_V_ACCEPT]);
}

EXPORT void wifi_app_poll(void)
{
	UW	now;

	if (!wifi_ready) {
		return;		/* subsystem1 objects not created yet */
	}
	__asm__ volatile("dmb" ::: "memory");	/* see wifi_telemetry_start */

	if (wifi_task_id == 0) {	/* first call, in the owning task */
		T_RTSK	r;

		wifi_task_id = tk_get_tid();
		if (tk_ref_tsk(TSK_SELF, &r) == E_OK) {
			wifi_task_pri = r.tskpri;
		}
		tm_printf((UB*)"wifi: Wi-Fi Telemetry Task id=%d pri=%d topics %s / %s\n",
			  wifi_task_id, wifi_task_pri, WIFI_TOPIC_STATE, WIFI_TOPIC_CMD);
#if TM_MQTT
		mqtt_utk_set_rx_callback(n_on_rx);
#endif
	}

	{
		UW	us = in_w(TIMER_TIMERAWL);	/* 1 MHz, read only */
		UW	gap = us - last_poll_us;

		if (wifi_poll_count != 0 && gap > qstats.max_gap_us) {
			qstats.max_gap_us = gap;
		}
		if (wifi_poll_count != 0 && gap > 50000u) {
			qstats.gaps_over_50ms++;
		}
		last_poll_us = us;
		wifi_poll_count++;
	}

	now = now_ms();
	drain_telemetry_q();
	check_overflow();
	wifi_sm_step(&wifi_sm, now);

	if ((INT)(now - next_stats_ms) >= 0) {
		next_stats_ms = now + WIFI_STATS_PERIOD_MS;
		print_stats(now);
	}
}

#if !TM_MQTT
LOCAL void wifi_task(INT stacd, void *exinf)
{
	(void)stacd; (void)exinf;
	for (;;) {
		wifi_app_poll();
		tk_dly_tsk(WIFI_TASK_PERIOD_MS);
	}
}
#endif

/* ===================================================================
 *  Init / start (called from usermain via subsystem1_init/_start)
 * =================================================================== */
EXPORT ER wifi_telemetry_init(void)
{
	T_CMBF	cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = WIFI_TELEM_Q_DEPTH * APP_MBF_SLOT(TQ_MSG_LEN);	/* 128 B */
	cmbf.maxmsz = TQ_MSG_LEN;
	telemetry_q = tk_cre_mbf(&cmbf);
	if (telemetry_q < E_OK) {
		return telemetry_q;
	}

	latest[TQ_FIELD_NONE]      = 0;
	latest[TQ_FIELD_VEH_STATE] = APP_VS_INITIALIZING;
	latest[TQ_FIELD_SPEED]     = APP_SPEED_NA;
	latest[TQ_FIELD_HUMP]      = APP_HUMP_MM_NA;
	latest[TQ_FIELD_OBSTACLE]  = APP_OBST_CM_NA;

	wifi_sm_init(&wifi_sm, &wifi_ops, now_ms());
	next_stats_ms = now_ms() + WIFI_STATS_PERIOD_MS;
	return E_OK;
}

EXPORT ER wifi_telemetry_start(void)
{
	/* Publish "ready" only after every object above exists.  With MQTT=1
	   cyw43_utk_task may already be running on core 0 while usermain runs
	   on core 1, hence the barrier before the single store. */
	__asm__ volatile("dmb" ::: "memory");
	wifi_ready = TRUE;

#if !TM_MQTT
	{
		ID	tid = app_start_task(wifi_task, S1_PRI_WIFI, TP_PRC1, "wifi_task");
		return (tid < E_OK) ? tid : E_OK;
	}
#else
	return E_OK;	/* runs inside cyw43_utk_task */
#endif
}

EXPORT const WIFI_STATS *wifi_stats(void)	{ return &wifi_sm.stats; }
EXPORT const WIFI_QSTATS *wifi_qstats(void)	{ return &qstats; }
EXPORT WIFI_STATE wifi_state(void)		{ return wifi_sm.state; }
