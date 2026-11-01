/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem1/logs/ and record the result
 *	in testing/subsystem1/TEST_LOG.md.  Test IDs: S1-U01, S1-U02, ...
 *
 *	S1-U01..U02  LED heartbeat (template example)
 *	S1-U04..U22  Wi-Fi Telemetry (Buddy 1).  The routing tests (U12-U19)
 *	             drive a SECOND state machine instance with a fake network
 *	             but the REAL remote_cmd_q, flg_safety and ESTOP latch; this
 *	             test task plays the Safety Handler and Vehicle Controller,
 *	             which do not exist in the firmware yet.
 */

#include "subsystem1.h"
#include "wifi_telemetry.h"
#include "../test_report.h"
#if TM_MQTT
#include "cyw43_utk.h"		/* S1-U24: radio status (read only) */
#endif

#if APP_TEST

/* ---- fake network for the routing tests (always connected) ----------- */
LOCAL bool t_true(void)			{ return true; }
LOCAL void t_void(void)			{ }
LOCAL WIFI_LINK t_link(void)		{ return WIFI_LINK_UP; }
LOCAL WIFI_MQTT t_mqtt(void)		{ return WIFI_MQTT_SUBSCRIBED; }
LOCAL int t_pub(const uint8_t *f, uint32_t n) { (void)f; (void)n; return WIFI_PUB_OK; }
LOCAL uint32_t t_zero(void)		{ return 0; }
LOCAL void t_faults(uint16_t b1)	{ (void)b1; }	/* never touch the live mask */
LOCAL void t_snap(APP_TELEM *t)		{ (void)t; }
LOCAL void t_event(WIFI_EVENT ev, uint32_t a, uint32_t b)
{
	if (ev == WIFI_EV_CMD) {
		tm_printf((UB*)"        test-sm: cmd type=%u seq=%u -> %s\n",
			  (b >> 8) & 0xFFu, b & 0xFFu,
			  wifi_cmd_verdict_name((WIFI_CMD_VERDICT)a));
	}
}

LOCAL const WIFI_OPS test_ops = {
	t_true, t_link, t_true, t_void, t_true, t_mqtt, t_true, t_void,
	t_pub, t_zero, t_zero,
	wifi_k_estop_latched, wifi_k_route_cmd, wifi_k_raise_abort,	/* REAL */
	t_faults, t_snap, t_event
};

LOCAL WIFI_SM	tsm;
LOCAL UW	tnow;

LOCAL void tsm_cmd(UB type, UB seq)
{
	APP_CMD	c;
	UB	f[CMD_FRAME_LEN];

	c.type = type;
	c.seq = seq;
	(void)app_cmd_encode(&c, f);
	wifi_sm_on_command(&tsm, f, CMD_FRAME_LEN);
	tnow += 10;
	wifi_sm_step(&tsm, tnow);
}

LOCAL void tsm_raw(CONST UB *p, UW len)
{
	wifi_sm_on_command(&tsm, p, len);
	tnow += 10;
	wifi_sm_step(&tsm, tnow);
}

/* Vehicle Controller stand-in: what did remote_cmd_q receive? */
LOCAL INT rcmd_take(APP_CMD *c)
{
	UB	m[RCMD_MSG_LEN];
	INT	n = 0;

	while (tk_rcv_mbf(remote_cmd_q, m, TMO_POL) == (INT)RCMD_MSG_LEN) {
		if (app_rcmd_unpack(m, c) == APP_OK) {
			n++;
		}
	}
	return n;
}

/* Safety Handler stand-in: consume the remote-abort request bit */
LOCAL BOOL abort_req_take(void)
{
	UINT	ptn = 0;

	return tk_wai_flg(flg_safety, SAFETY_REQ_REMOTE_ABORT,
			  TWF_ORW | TWF_BITCLR, &ptn, TMO_POL) == E_OK;
}

/* ---- concurrent fault-bit stress (R1) --------------------------------- */
LOCAL volatile UW	stress_errors_helper;
LOCAL volatile BOOL	stress_done;

LOCAL UW toggle_bit(UINT bit, UW rounds)
{
	T_RFLG	r;
	UW	i, errors = 0;

	for (i = 0; i < rounds; i++) {
		tk_set_flg(flg_fault, bit);
		if (tk_ref_flg(flg_fault, &r) != E_OK || (r.flgptn & bit) == 0) errors++;
		tk_clr_flg(flg_fault, ~bit);
		if (tk_ref_flg(flg_fault, &r) != E_OK || (r.flgptn & bit) != 0) errors++;
	}
	return errors;
}

LOCAL void stress_helper(INT stacd, void *exinf)
{
	(void)stacd; (void)exinf;
	/* FLT_IMU_I2C: its owner (IMU, Buddy 4) is not in this build.  It was
	   FLT_MOTOR, which Buddy 2's motion_task now owns and may change. */
	stress_errors_helper = toggle_bit(FLT_IMU_I2C, 5000);
	stress_done = TRUE;
	tk_ext_tsk();
}

/* ---- control-loop jitter probe (test A, software proxy) ---------------
   A 10 ms cyclic handler wakes a task at Line Following's priority (15) on
   core 0, the core the Wi-Fi Telemetry Task runs on.  The task stamps each
   wake with the 1 MHz hardware timer.  Lateness of one wake shows up as a
   longer and then a shorter period, so max |period - 10 ms| is the jitter.
   A wake request that queues up (wupcnt > 0) is a missed cycle. */
#define JIT_PERIOD_US	10000u
#define JIT_SAMPLES	1000		/* 10 s */
LOCAL ID		jit_tid;
LOCAL volatile UW	jit_n, jit_min_dt, jit_max_dt, jit_missed;
LOCAL volatile BOOL	jit_done;

LOCAL void jit_cyc(void *exinf)
{
	(void)exinf;
	(void)tk_wup_tsk(jit_tid);
}

LOCAL void jit_task(INT stacd, void *exinf)
{
	T_RTSK	r;
	UW	prev, now, dt;

	(void)stacd; (void)exinf;
	jit_min_dt = 0xFFFFFFFFu;
	jit_max_dt = 0;
	tk_slp_tsk(TMO_FEVR);			/* first wake: baseline */
	prev = in_w(TIMER_TIMERAWL);
	while (jit_n < JIT_SAMPLES) {
		tk_slp_tsk(TMO_FEVR);
		now = in_w(TIMER_TIMERAWL);
		dt = now - prev;
		prev = now;
		if (dt < jit_min_dt) jit_min_dt = dt;
		if (dt > jit_max_dt) jit_max_dt = dt;
		if (tk_ref_tsk(TSK_SELF, &r) == E_OK && r.wupcnt > 0) {
			jit_missed += (UW)r.wupcnt;
			(void)tk_can_wup(TSK_SELF);
		}
		jit_n++;
	}
	jit_done = TRUE;
	tk_ext_tsk();
}

EXPORT void subsystem1_test(void)
{
	T_RCYC	rcyc;
	T_RTSK	rtsk;
	T_RMBF	rmbf;
	APP_CMD	c = { CMD_NONE, 0 };
	ER	er;
	INT	i, n;

	TEST_INIT();

	/* ================= LED heartbeat (template) ================== */
	er = tk_ref_cyc(cyc_led, &rcyc);
	TEST_CHECK("S1-U01", er == E_OK && (rcyc.cycstat & TCYC_STA),
		   "cyclic handler is running");
	{
		INT	waiting = 0;
		for (i = 0; i < 10; i++) {
			tk_dly_tsk(97);
			if (tk_ref_tsk(tid_led, &rtsk) == E_OK &&
			    (rtsk.tskstat & TTS_WAI)) {
				waiting++;
			}
		}
		TEST_CHECK("S1-U02", waiting >= 9, "LED task sleeps between ticks");
	}

	/* ================= Wi-Fi Telemetry (Buddy 1) ================= */
	/* Not "empty": Buddy 2's motion_task posts speed from boot, before the
	   Wi-Fi task's first poll (the radio firmware load takes ~1.3 s). */
	er = tk_ref_mbf(telemetry_q, &rmbf);
	tm_printf((UB*)"        telemetry_q free=%d of %d bytes\n", rmbf.frbufsz,
		  WIFI_TELEM_Q_DEPTH * APP_MBF_SLOT(TQ_MSG_LEN));
	TEST_CHECK("S1-U04", er == E_OK && rmbf.maxmsz == (INT)TQ_MSG_LEN &&
		   rmbf.frbufsz <= WIFI_TELEM_Q_DEPTH * APP_MBF_SLOT(TQ_MSG_LEN) &&
		   (rmbf.frbufsz % APP_MBF_SLOT(TQ_MSG_LEN)) == 0,
		   "telemetry_q: 4-byte messages, 128-byte buffer (16 deep)");

	for (i = 0; i < 100 && wifi_task_id == 0; i++) {
		tk_dly_tsk(100);	/* the radio may still be loading firmware */
	}
	tm_printf((UB*)"        wifi task id=%d pri=%d state=%s\n", wifi_task_id,
		  wifi_task_pri, wifi_sm_state_name(wifi_state()));
	TEST_CHECK("S1-U05", wifi_task_id > 0 && wifi_task_pri == S1_PRI_WIFI,
		   "Wi-Fi Telemetry Task runs at priority 40");

	{
		/* Count polls instead of sampling the task state: tk_dly_tsk wakes
		   this test on the same 10 ms tick that wakes the Wi-Fi task, so a
		   state sample always lands while it is working.  The loop sleeps
		   one tick per iteration (~50 polls/s); a busy loop would make
		   thousands per second. */
		UW	p0 = wifi_poll_count, p;

		tk_dly_tsk(2000);
		p = wifi_poll_count - p0;
		tm_printf((UB*)"        Wi-Fi task: %u polls in 2 s, longest gap %u us\n",
			  p, wifi_qstats()->max_gap_us);
		TEST_CHECK("S1-U06", p >= 60 && p <= 150,
			   "Wi-Fi task sleeps between polls: 30..75 polls/s (no busy loop)");
	}

	{
		static const UB	gold[TLM_FRAME_LEN] =
			{ 0x45, 0x34, 0x34, 0x12, 0xFB, 0x19, 0x28, 0xA5, 0x00 };
		APP_TELEM	t, d;
		UB		f[TLM_FRAME_LEN];
		BOOL		same = TRUE;

		app_telem_default(&t);
		t.veh_state = APP_VS_DRIVING; t.retry = 2; t.seq = 0x34;
		t.time_10ms = 0x1234; t.speed_cm_s = -5; t.hump_mm = 25;
		t.obst_cm = 40; t.faults = 0x0A5;
		er = app_telem_encode(&t, f);
		for (i = 0; i < (INT)TLM_FRAME_LEN; i++) {
			if (f[i] != gold[i]) same = FALSE;
		}
		TEST_CHECK("S1-U07", er == APP_OK && same &&
			   app_telem_decode(f, TLM_FRAME_LEN, &d) == APP_OK &&
			   d.speed_cm_s == -5 && d.time_10ms == 0x1234,
			   "telemetry golden frame identical to the PC encoder");
	}

	{
		UB	f[CMD_FRAME_LEN];
		APP_CMD	a = { CMD_ABORT, 7 };
		(void)app_cmd_encode(&a, f);
		TEST_CHECK("S1-U08", app_crc8((CONST UB*)"123456789", 9) == 0xF4 &&
			   f[0] == 0x05 && f[1] == 0x07 && f[2] == 0x54,
			   "command frame abort/7 = 05 07 54, CRC-8 check 0xF4");
	}

	{
		T_CMBF		cm = {0};
		ID		q;
		UB		m[TQ_MSG_LEN];
		volatile UH	drops = 0;
		uint32_t	fld;
		int32_t		val;
		uint16_t	st;
		BOOL		order = TRUE;

		cm.mbfatr = TA_TFIFO;
		cm.bufsz  = WIFI_TELEM_Q_DEPTH * APP_MBF_SLOT(TQ_MSG_LEN);
		cm.maxmsz = TQ_MSG_LEN;
		q = tk_cre_mbf(&cm);
		for (i = 0; i < 20; i++) {
			(void)app_tq_pack(m, TQ_FIELD_OBSTACLE, i, 0);
			er = wifi_tq_post_to(q, m, &drops);
			if (er != E_OK) order = FALSE;
		}
		for (n = 0; tk_rcv_mbf(q, m, TMO_POL) == (INT)TQ_MSG_LEN; n++) {
			if (app_tq_unpack(m, &fld, &val, &st) != APP_OK || val != n + 4) {
				order = FALSE;
			}
		}
		tk_del_mbf(q);
		TEST_CHECK("S1-U09", q > 0 && order && n == 16 && drops == 4,
			   "20 posts into 16 slots: oldest 4 dropped, newest 16 kept in order, never blocks");
	}

	TEST_CHECK("S1-U10", telemetry_post(TQ_FIELD_SPEED, 101) == E_PAR &&
		   telemetry_post(TQ_FIELD_HUMP, 64) == E_PAR &&
		   telemetry_post(TQ_FIELD_NONE, 0) == E_PAR &&
		   telemetry_post(5, 0) == E_PAR &&
		   telemetry_post(TQ_FIELD_VEH_STATE, -1) == E_PAR,
		   "telemetry_post rejects invalid fields and out-of-range values");

	{
		/* TQ_FIELD_OBSTACLE: its producer (Buddy 5) is not in this build.
		   TQ_FIELD_SPEED is not used here any more: Buddy 2's motion_task
		   now posts it every 100 ms, so this test would break the "one
		   producer per field" rule and miscount. */
		/* NFR1.1 is "<= 50 ms while connected".  The since-boot maximum
		   includes the radio firmware load, so only this window's
		   over-50 ms count is used.  Not connected -> measured, reported,
		   but not judged. */
		UW	rx0, ov0;
		UW	gp0 = wifi_qstats()->gaps_over_50ms;
		UW	sp0 = wifi_tq_drops[TQ_FIELD_SPEED];
		UW	got, ov, gp;
		BOOL	ok = TRUE, conn = TRUE;

		/* Join + DHCP + broker take 5-10 s after boot; wait up to 30 s */
		for (i = 0; i < 300 && wifi_state() != WIFI_ST_SHARE_TELEMETRY; i++) {
			tk_dly_tsk(100);
		}
		rx0 = wifi_qstats()->received;
		ov0 = wifi_qstats()->over_50ms;

		for (i = 0; i < 10; i++) {
			ok &= telemetry_post(TQ_FIELD_OBSTACLE, i * 10) == E_OK;
			conn &= wifi_state() == WIFI_ST_SHARE_TELEMETRY;
			tk_dly_tsk(20);
		}
		tk_dly_tsk(200);
		ok &= telemetry_post(TQ_FIELD_OBSTACLE, APP_OBST_CM_NA) == E_OK;
		tk_dly_tsk(100);
		conn &= wifi_state() == WIFI_ST_SHARE_TELEMETRY;
		got = wifi_qstats()->received - rx0;
		ov = wifi_qstats()->over_50ms - ov0;
		gp = wifi_qstats()->gaps_over_50ms - gp0;
		tm_printf((UB*)"        telemetry_q window: %u msgs dequeued (11 ours + motion speed),"
			  " %u over 50 ms, %u poll gaps over 50 ms, connected=%d, state=%s\n",
			  got, ov, gp, conn, wifi_sm_state_name(wifi_state()));
		if (conn) {
			TEST_CHECK("S1-U11", ok && got >= 11 && ov == 0 &&
				   wifi_tq_drops[TQ_FIELD_SPEED] == sp0,
				   "telemetry_q delay <= 50 ms while connected (motion_task also producing)");
		} else {
			tm_printf((UB*)"[TEST] S1-U11 NOT-RUN  needs SHARE_TELEMETRY (AP + broker);"
				  " queue path ok=%d\n", ok && got >= 11);
		}
	}

	/* ---- routing with the real kernel objects ---- */
	wifi_sm_init(&tsm, &test_ops, 0);
	tnow = 0;
	for (i = 0; i < 5; i++) {		/* -> SHARE_TELEMETRY */
		tnow += 10;
		wifi_sm_step(&tsm, tnow);
	}
	(void)rcmd_take(&c);
	(void)abort_req_take();

	tsm_cmd(CMD_ABORT, 1);
	n = rcmd_take(&c);
	TEST_CHECK("S1-U12", n == 1 && c.type == CMD_ABORT && abort_req_take() &&
		   tsm.state == WIFI_ST_SHARE_TELEMETRY,
		   "abort -> flg_safety REMOTE_ABORT + remote_cmd_q, back to SHARE_TELEMETRY");

	tsm_cmd(CMD_ABORT, 1);
	TEST_CHECK("S1-U13", rcmd_take(&c) == 0 && !abort_req_take(),
		   "duplicate abort: nothing posted, no flag");

	{
		UB	bad[2] = { 0x05, 0x02 };
		tsm_raw(bad, 2);
		TEST_CHECK("S1-U14", rcmd_take(&c) == 0 && !abort_req_take(),
			   "malformed (2-byte) payload: nothing posted");
	}

	{
		UB	unk[CMD_FRAME_LEN] = { 0x01, 0x03, 0 };
		unk[2] = app_crc8(unk, 2);
		tsm_raw(unk, CMD_FRAME_LEN);
		TEST_CHECK("S1-U15", rcmd_take(&c) == 0 && !abort_req_take(),
			   "unknown command (type 0, valid CRC): nothing posted");
	}

	tsm_cmd(CMD_PAUSE, 2);
	n = rcmd_take(&c);
	TEST_CHECK("S1-U16", n == 1 && c.type == CMD_PAUSE && !abort_req_take(),
		   "pause -> remote_cmd_q only");

	tk_set_flg(flg_safety, SAFETY_ESTOP_LATCHED);	/* Safety stand-in latches */
	tsm_cmd(CMD_RESUME, 3);
	TEST_CHECK("S1-U17", rcmd_take(&c) == 0,
		   "resume while ESTOP latched: nothing posted");

	tk_clr_flg(flg_safety, ~(UINT)SAFETY_ESTOP_LATCHED);	/* local reset */
	tsm_cmd(CMD_RESUME, 3);
	n = rcmd_take(&c);
	TEST_CHECK("S1-U18", n == 1 && c.type == CMD_RESUME,
		   "resume after the local ESTOP reset -> remote_cmd_q");

	tsm_cmd(CMD_RESUME, 1);
	TEST_CHECK("S1-U19", rcmd_take(&c) == 0,
		   "resume with an old sequence number: nothing posted");

	/* ---- bounded buffers and shared fault mask ---- */
	tm_printf((UB*)"        live backlog max=%u dropped=%u frames built=%u published=%u\n",
		  wifi_stats()->backlog_max, wifi_stats()->backlog_dropped,
		  wifi_stats()->frames_built, wifi_stats()->frames_published);
	TEST_CHECK("S1-U20", wifi_stats()->backlog_max <= WIFI_BACKLOG_DEPTH &&
		   wifi_stats()->frames_built > 0,
		   "outbound backlog never exceeds 16 frames");

	{
		UW	mine;
		ID	tid;

		stress_done = FALSE;
		tid = app_start_task(stress_helper, 2, TP_PRC2, "stress");
		mine = toggle_bit(FLT_LINE_LOST, 5000);
		for (i = 0; i < 100 && !stress_done; i++) {
			tk_dly_tsk(10);
		}
		tk_del_tsk(tid);
		tm_printf((UB*)"        fault-bit stress: errors main=%u helper=%u\n",
			  mine, stress_errors_helper);
		TEST_CHECK("S1-U21", tid > 0 && stress_done && mine == 0 &&
			   stress_errors_helper == 0,
			   "two tasks toggling different fault bits 5000x: no bit lost");
	}

	{
		T_RFLG	r;
		(void)tk_ref_flg(flg_fault, &r);
		TEST_CHECK("S1-U22", (r.flgptn & FLT_ESTOP_LATCHED) == 0 &&
			   (r.flgptn & ~(UINT)FLT_ALL) == 0,
			   "flg_fault holds only defined bits; B1 never stores ESTOP there");
	}

	/* ---- S1-U23 jitter probe while the Wi-Fi task is active ---- */
	{
		T_CCYC	cc = {0};
		ID	cyc;
		UW	att0 = wifi_stats()->reconnect_attempts;
		UW	pub0 = wifi_stats()->frames_built;
		UW	jit;

		jit_n = 0; jit_missed = 0; jit_done = FALSE;
		jit_tid = app_start_task(jit_task, PRI_LINE_FOLLOW, TP_PRC1, "jit_probe");
		cc.cycatr = TA_HLNG | TA_STA;
		cc.cychdr = (FP)jit_cyc;
		cc.cyctim = JIT_PERIOD_US / 1000u;
		cc.cycphs = JIT_PERIOD_US / 1000u;
		cyc = tk_cre_cyc(&cc);
		for (i = 0; i < 130 && !jit_done; i++) {
			tk_dly_tsk(100);
		}
		if (cyc > 0) {
			tk_stp_cyc(cyc);
			tk_del_cyc(cyc);
		}
		tk_dly_tsk(20);
		if (jit_tid > 0) {
			tk_del_tsk(jit_tid);
		}
		jit = (jit_max_dt > JIT_PERIOD_US) ? jit_max_dt - JIT_PERIOD_US : 0u;
		if (jit_min_dt < JIT_PERIOD_US && JIT_PERIOD_US - jit_min_dt > jit) {
			jit = JIT_PERIOD_US - jit_min_dt;
		}
		tm_printf((UB*)"        jitter probe core0 pri15: %u cycles, period min=%u max=%u us,"
			  " jitter=%u us, missed=%u; Wi-Fi during probe: state=%s,"
			  " reconnect attempts +%u, frames built +%u\n",
			  jit_n, jit_min_dt, jit_max_dt, jit, jit_missed,
			  wifi_sm_state_name(wifi_state()),
			  wifi_stats()->reconnect_attempts - att0,
			  wifi_stats()->frames_built - pub0);
		TEST_CHECK("S1-U23", jit_tid > 0 && cyc > 0 && jit_done &&
			   jit_n == JIT_SAMPLES && jit <= 5000u && jit_missed == 0,
			   "10 ms loop on the Wi-Fi core: jitter <= 5 ms, no missed cycle (10 s)");
	}

	/* ---- S1-U24 radio and link behaviour seen on target ---- */
	{
		const WIFI_STATS *s = wifi_stats();
#if TM_MQTT
		T_CYW43_UTK_STATUS	st;

		cyw43_utk_get_status(&st);
		tm_printf((UB*)"        radio: init=%d ready=%u link_status=%d ip=%u.%u.%u.%u"
			  " polls=%u\n", st.init_result, st.ready, st.link_status,
			  st.ip_addr & 0xFFu, (st.ip_addr >> 8) & 0xFFu,
			  (st.ip_addr >> 16) & 0xFFu, (st.ip_addr >> 24) & 0xFFu,
			  st.poll_count);
#endif
		tm_printf((UB*)"        wifi sm: state=%s attempts=%u link_downs=%u published=%u"
			  " built=%u backlog=%u/%u dropped=%u det_max=%u ms rec_max=%u ms\n",
			  wifi_sm_state_name(wifi_state()), s->reconnect_attempts,
			  s->link_downs, s->frames_published, s->frames_built,
			  s->backlog_max, (UW)WIFI_BACKLOG_DEPTH, s->backlog_dropped,
			  s->max_detect_ms, s->max_recovery_ms);
		TEST_CHECK("S1-U24", wifi_state() != WIFI_ST_DISCONNECTED &&
			   s->frames_built > 0 && s->backlog_max <= WIFI_BACKLOG_DEPTH,
			   "radio up, state machine running, 10 Hz frames built, backlog bounded");
	}

	/* ---- S1-U25 the 1 MHz hardware timer agrees with the kernel clock ----
	   Every microsecond measurement in these tests, and the radio driver's
	   delays, depend on it.  Found broken after a power cycle (run 3). */
	{
		UW	tick = in_w(WDOG_TICK);
		UW	t0, d[3], k;

		for (i = 0; i < 3; i++) {
			t0 = in_w(TIMER_TIMERAWL);
			tk_dly_tsk(100);
			d[i] = in_w(TIMER_TIMERAWL) - t0;
		}
		tm_printf((UB*)"        WATCHDOG TICK=%x (expect 20c); tk_dly_tsk(100) = %u %u %u us on TIMER\n",
			  tick, d[0], d[1], d[2]);
		k = 1;
		for (i = 0; i < 3; i++) {
			if (d[i] < 100000u || d[i] > 125000u) k = 0;	/* 100 ms + <= 1 tick + slack */
		}
		TEST_CHECK("S1-U25", k && (tick & (UW)WDOG_TICK_RUNNING) &&
			   (tick & 0x1FFu) == 12u,
			   "TIMER runs at 1 MHz: tk_dly_tsk(100) = 100..125 ms on it, tick generator = 12 cycles");
	}

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
