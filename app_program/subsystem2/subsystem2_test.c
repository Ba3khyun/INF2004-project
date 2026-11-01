/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 *
 *	S2-U01..U02  example Producer
 *	S2-U03..U12  Motion Control (Buddy 2).  These NEVER drive the motors:
 *	             every check either reads registers or verifies that the
 *	             PWM compare values stay at 0 while a move is refused.
 *	S2-M01..M02  Motion Control with PWM output (wheels turn).  Built only
 *	             when S2_TEST_MOTORS is 1.  LIFT THE WHEELS OFF THE GROUND
 *	             FIRST.  Each move runs at 20 % PWM for at most 4 s.
 *
 *	The test task plays the Safety Handler (sets/clears SAFETY_ESTOP_LATCHED
 *	in flg_safety), which does not exist in the firmware yet.
 */

#include "subsystem2.h"
#include "drv_motion.h"			/* encoder counts (read only) */
#include "../subsystem1/wifi_telemetry.h"	/* telemetry_q statistics (read only) */
#include "../test_report.h"

#if APP_TEST

#ifndef S2_TEST_MOTORS
#define S2_TEST_MOTORS	0		/* 1 = also run S2-M01/M02 (wheels turn) */
#endif

/* RP2040 PWM slices 4 (GP8/GP9, left) and 5 (GP10/GP11, right) */
#define T_PWM_SLICE(s)		(PWM_BASE + (s) * 0x14u)
#define T_PWM_CSR(s)		(T_PWM_SLICE(s) + PWM_CHx_CSR)
#define T_PWM_DIV(s)		(T_PWM_SLICE(s) + PWM_CHx_DIV)
#define T_PWM_CC(s)		(T_PWM_SLICE(s) + PWM_CHx_CC)
#define T_PWM_TOP(s)		(T_PWM_SLICE(s) + PWM_CHx_TOP)
#define T_PWM_TOP_20KHZ		6249u		/* 125 MHz / 20 kHz - 1 */
#define T_IO_PROC1_INTE0	(IO_BANK0_BASE + 0x130u)
#define T_EDGE_HIGH(pin)	(1u << (4u * (pin) + 3u))

/* All four motor compare values (A = low 16 bits, B = high 16 bits) */
LOCAL UW pwm_cc_any(void)
{
	return in_w(T_PWM_CC(4)) | in_w(T_PWM_CC(5));
}

/* Largest compare value seen while sampling for ms milliseconds */
LOCAL UW pwm_watch(INT ms)
{
	UW	seen = 0;
	INT	t;

	for (t = 0; t < ms; t += 10) {
		seen |= pwm_cc_any();
		tk_dly_tsk(10);
	}
	return seen | pwm_cc_any();
}

LOCAL BOOL fault_motor(void)
{
	T_RFLG	r;

	return tk_ref_flg(flg_fault, &r) == E_OK && (r.flgptn & FLT_MOTOR) != 0;
}

LOCAL void safety_latch(BOOL on)	/* Safety Handler stand-in */
{
	if (on) {
		tk_set_flg(flg_safety, SAFETY_ESTOP_LATCHED);
	} else {
		tk_clr_flg(flg_safety, ~(UINT)SAFETY_ESTOP_LATCHED);
	}
}

#if S2_TEST_MOTORS
LOCAL UW us_now(void)
{
	return in_w(TIMER_TIMERAWL);
}
#endif

EXPORT void subsystem2_test(void)
{
	T_RSEM	rsem;
	T_RMPF	rmpf;
	T_RMBF	rmbf;
	T_RTSK	rtsk;
	UW	seq;
	INT	v, bad = 0, i;

	TEST_INIT();

	/* ================= example Producer ================= */
	for (seq = 0; seq < 40; seq++) {
		v = s2_make_value(seq);
		if (v < 0 || v > 100) bad++;
	}
	TEST_CHECK("S2-U01", bad == 0, "sample values stay within 0..100");

	TEST_CHECK("S2-U02",
		   tk_ref_sem(sem_slots, &rsem) == E_OK &&
		   rsem.semcnt >= 0 && rsem.semcnt <= S2_POOL_COUNT &&
		   tk_ref_mpf(mpf_samples, &rmpf) == E_OK &&
		   rmpf.frbcnt <= S2_POOL_COUNT,
		   "free slots and free blocks within pool size");

	/* ================= Motion Control (Buddy 2) ================= */
	safety_latch(FALSE);
	(void)motion_cmd_post(MC_CMD_CLEAR);	/* known state: IDLE */
	tk_dly_tsk(50);

	TEST_CHECK("S2-U03", tk_ref_mbf(motion_cmd_q, &rmbf) == E_OK &&
		   rmbf.maxmsz == MC_CMD_LEN &&
		   rmbf.frbufsz == MC_CMD_Q_DEPTH * APP_MBF_SLOT(MC_CMD_LEN),
		   "motion_cmd_q: 1-byte commands, 4 deep, empty");

	{
		/* tk_dly_tsk wakes this test on the same tick that wakes
		   motion_task, so a sample taken at once always sees it working.
		   Wait 3 ms past the tick first (busy, on core 0: motion_task is
		   on core 1 and unaffected), then sample. */
		INT	waiting = 0, running = 0;
		PRI	pri = 0;
		BOOL	ok = tk_ref_tsk(tid_motion, &rtsk) == E_OK;
		UW	t0;

		pri = rtsk.tskpri;
		for (i = 0; i < 20; i++) {
			tk_dly_tsk(7);
			t0 = in_w(TIMER_TIMERAWL);
			while (in_w(TIMER_TIMERAWL) - t0 < 3000u) {
				;
			}
			if (tk_ref_tsk(tid_motion, &rtsk) == E_OK) {
				if (rtsk.tskstat & TTS_WAI) {
					waiting++;
				} else {
					running++;
				}
			}
		}
		tm_printf((UB*)"        motion_task id=%d pri=%d, 3 ms after a tick: waiting %d,"
			  " running/ready %d of 20\n", tid_motion, pri, waiting, running);
		TEST_CHECK("S2-U04", tid_motion > 0 && ok && pri == S2_PRI_MOTION && waiting >= 18,
			   "motion_task runs at priority 10 and sleeps between ticks");
	}

	TEST_CHECK("S2-U05", motion_cmd_post((UB)'Z') == E_PAR &&
		   motion_cmd_post(0) == E_PAR && motion_cmd_post((UB)'f') == E_PAR,
		   "motion_cmd_post rejects unknown command codes");

	{
		UW	c4 = in_w(T_PWM_CSR(4)), c5 = in_w(T_PWM_CSR(5));
		UW	t4 = in_w(T_PWM_TOP(4)), t5 = in_w(T_PWM_TOP(5));
		UW	d4 = in_w(T_PWM_DIV(4)), d5 = in_w(T_PWM_DIV(5));
		BOOL	fs = TRUE;

		for (i = 8; i <= 11; i++) {
			if ((in_w(GPIO_CTRL(i)) & 0x1Fu) != GPIO_CTRL_FUNCSEL_PWM) fs = FALSE;
		}
		tm_printf((UB*)"        PWM slice4 csr=%x div=%x top=%u, slice5 csr=%x div=%x top=%u\n",
			  c4, d4, t4, c5, d5, t5);
		TEST_CHECK("S2-U06", (c4 & 1u) && (c5 & 1u) && t4 == T_PWM_TOP_20KHZ &&
			   t5 == T_PWM_TOP_20KHZ && d4 == 0x10u && d5 == 0x10u && fs,
			   "motor PWM: GP8-11 on slices 4/5, enabled, divider 1, 20 kHz");
	}

	TEST_CHECK("S2-U07", pwm_watch(200) == 0u,
		   "idle: all four motor outputs at 0 (brake) for 200 ms");

	{
		BOOL	pins = TRUE;
		UW	inte = in_w(T_IO_PROC1_INTE0);

		for (i = 6; i <= 7; i++) {
			UW	pad = in_w(GPIO(i));
			if ((in_w(GPIO_CTRL(i)) & 0x1Fu) != GPIO_CTRL_FUNCSEL_SIO ||
			    (pad & GPIO_PUE) == 0u || (pad & GPIO_PDE) != 0u) {
				pins = FALSE;
			}
		}
		tm_printf((UB*)"        encoders: L=%u R=%u pulses, PROC1_INTE0=%x\n",
			  encoder_get_left_count(), encoder_get_right_count(), inte);
		TEST_CHECK("S2-U08", pins && (inte & T_EDGE_HIGH(6)) && (inte & T_EDGE_HIGH(7)),
			   "encoders GP6/GP7: SIO input, pull-up, rising-edge IRQ on core 1");
	}

	/* ---- Safety ESTOP latched: every move refused, PWM stays 0 ---- */
	safety_latch(TRUE);
	{
		BOOL	q = motion_cmd_post(MC_CMD_FORWARD) == E_OK;
		UW	seen = pwm_watch(150);

		q &= motion_cmd_post(MC_CMD_LEFT) == E_OK;
		seen |= pwm_watch(150);
		q &= motion_cmd_post(MC_CMD_RIGHT) == E_OK;
		seen |= pwm_watch(150);
		TEST_CHECK("S2-U09", q && seen == 0u && !fault_motor(),
			   "ESTOP latched: F/L/R refused, PWM stays 0, no FLT_MOTOR");
	}
	safety_latch(FALSE);

	/* ---- local stop X: moves refused until C ---- */
	{
		BOOL	q = motion_cmd_post(MC_CMD_STOP) == E_OK;
		UW	seen;

		tk_dly_tsk(30);
		q &= motion_cmd_post(MC_CMD_FORWARD) == E_OK;
		seen = pwm_watch(200);
		TEST_CHECK("S2-U10", q && seen == 0u && !fault_motor(),
			   "local stop X: forward refused, PWM 0, X is not a fault");
		q = motion_cmd_post(MC_CMD_CLEAR) == E_OK;
		tk_dly_tsk(30);
		TEST_CHECK("S2-U11", q && pwm_watch(100) == 0u && !fault_motor(),
			   "clear C: back to IDLE, motors still braked, FLT_MOTOR clear");
	}

	/* ---- control tick rate (10 ms) ---- */
	{
		UW	t0, d, worst = 0, sum = 0;
		UW	n0, m0, n, m;

		/* Evidence for the change to a cyclic tick: how long does a
		   relative 10 ms timeout really last in this kernel? */
		for (i = 0; i < 10; i++) {
			t0 = in_w(TIMER_TIMERAWL);
			tk_dly_tsk(10);
			d = in_w(TIMER_TIMERAWL) - t0;
			sum += d;
			if (d > worst) worst = d;
		}
		tm_printf((UB*)"        kernel: tk_dly_tsk(10) lasts avg %u us, max %u us\n",
			  sum / 10u, worst);

		n0 = motion_tick_count; m0 = motion_ticks_missed;
		t0 = in_w(TIMER_TIMERAWL);
		tk_dly_tsk(2000);
		d = in_w(TIMER_TIMERAWL) - t0;
		n = motion_tick_count - n0; m = motion_ticks_missed - m0;
		tm_printf((UB*)"        motion_task: %u ticks in %u us (%u per s), missed %u\n",
			  n, d, (UW)((n * 1000000ull) / d), m);
		TEST_CHECK("S2-U13", n * 10000ull >= (d / 100u) * 98u &&
			   n * 10000ull <= (d / 100u) * 102u && m == 0,
			   "motion_task control tick runs at 100 Hz, no missed tick");
	}

	/* ---- S2-M03 (diagnostic, motors idle): does a task spinning on core 0
	   starve motion_task on core 1?  Run 5: ESTOP was ignored for 300 ms
	   while the test spun here.  A spinning priority-1 task is not a design
	   we expect, but if this fails the Safety/Wi-Fi/driver code on core 0
	   must never busy-wait for long.  Needs no motor movement. ---- */
	{
		UW	t0, n0, n, m0 = motion_ticks_missed;
		/* tk_get_prc() returns processor number 1/2; TP_PRC1 is the
		   affinity bit used in task attributes, not the return value. */
		BOOL	core0 = (tk_get_prc() == 1);

		n0 = motion_tick_count;
		t0 = in_w(TIMER_TIMERAWL);
		if (core0) {
			while (in_w(TIMER_TIMERAWL) - t0 < 100000u) {
				;			/* spin 100 ms, no kernel call */
			}
		} else {
			tk_dly_tsk(100);
		}
		n = motion_tick_count - n0;
		tm_printf((UB*)"        motion ticks during a 100 ms %s on core %d: %u (expect ~10), missed %u\n",
			  core0 ? "spin" : "sleep", tk_get_prc() - 1, n, motion_ticks_missed - m0);
		TEST_CHECK("S2-M03", n >= 8u,
			   "motion_task keeps ticking (>= 8 of 10) while a priority-1 task spins on core 0");
	}

	/* ---- speed telemetry to Buddy 1 (IF-B1-01) ---- */
	{
		UW	rx0 = wifi_qstats()->received;
		UW	dr0 = wifi_tq_drops[TQ_FIELD_SPEED];
		UW	got;

		tk_dly_tsk(2000);
		got = wifi_qstats()->received - rx0;
		tm_printf((UB*)"        speed reports dequeued by the Wi-Fi task in 2 s: %u,"
			  " drops %u, queue delay max %u ms\n", got,
			  (UW)wifi_tq_drops[TQ_FIELD_SPEED] - dr0, wifi_qstats()->max_delay_ms);
		TEST_CHECK("S2-U12", got >= 18 && got <= 22 &&
			   wifi_tq_drops[TQ_FIELD_SPEED] == dr0,
			   "motion_task posts TQ_FIELD_SPEED at 10 Hz, none dropped");
	}

#if S2_TEST_MOTORS
	/* ================= with PWM output: WHEELS MUST BE LIFTED ========= */
	tm_printf((UB*)"        S2-M: motors will turn at 20 %% PWM\n");
	{
		UW	l0 = encoder_get_left_count(), r0 = encoder_get_right_count();
		UW	on = 0;
		INT	t;

		(void)motion_cmd_post(MC_CMD_FORWARD);
		tk_dly_tsk(50);
		on = pwm_cc_any();
		for (t = 0; t < 4500 && pwm_cc_any() != 0u; t += 10) {
			tk_dly_tsk(10);
		}
		tm_printf((UB*)"        forward: cc while moving=%x, stopped after ~%d ms,"
			  " pulses L=%u R=%u, FLT_MOTOR=%d\n", on, t,
			  encoder_get_left_count() - l0, encoder_get_right_count() - r0,
			  fault_motor());
		TEST_CHECK("S2-M01", on != 0u && pwm_cc_any() == 0u && !fault_motor() &&
			   encoder_get_left_count() - l0 >= 300u &&
			   encoder_get_right_count() - r0 >= 300u,
			   "forward 0.5 rev: PWM on, both encoders count, stops by itself");
		(void)motion_cmd_post(MC_CMD_CLEAR);	/* in case of a timeout */
		tk_dly_tsk(50);
	}
	{
		/* Observer SLEEPS (tk_dly_tsk): run 5 showed that a task spinning
		   at priority 1 on core 0 is not a safe way to watch the PWM, see
		   S2-M03.  tk_dly_tsk(1) lasts one extra kernel tick, so the
		   latency read here is the true value plus up to ~20 ms. */
		UW	t0, lat = 0xFFFFFFFFu, on, n0, tk0;
		UW	m0 = motion_ticks_missed;

		(void)motion_cmd_post(MC_CMD_FORWARD);
		tk_dly_tsk(100);
		on = pwm_cc_any();
		n0 = motion_tick_count;
		safety_latch(TRUE);
		t0 = us_now();
		while (us_now() - t0 < 200000u) {
			if (pwm_cc_any() == 0u) {
				lat = us_now() - t0;
				break;
			}
			tk_dly_tsk(1);
		}
		tk0 = motion_tick_count - n0;
		tm_printf((UB*)"        ESTOP -> PWM 0: %u us (cc before=%x, sleeping observer),"
			  " motion ticks meanwhile %u, missed %u\n",
			  lat, on, tk0, motion_ticks_missed - m0);
		TEST_CHECK("S2-M02", on != 0u && lat <= 50000u && pwm_watch(200) == 0u,
			   "ESTOP latched during a move: PWM 0 within 50 ms (observer resolution ~20 ms) and stays 0");
		safety_latch(FALSE);
		(void)motion_cmd_post(MC_CMD_CLEAR);
		tk_dly_tsk(50);
	}
#else
	tm_printf((UB*)"        S2-M01/M02 not built (S2_TEST_MOTORS=0): motors never driven\n");
#endif

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
