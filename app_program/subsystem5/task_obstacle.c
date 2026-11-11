/*
 *	task_obstacle.c
 *	Subsystem 5 -- the obstacle task: owns the ultrasonic sensor and the
 *	scanning servo, runs the obstacle state machine (see subsystem5.h).
 *
 *	APIs used: tk_dly_tsk, tk_set_flg, tk_clr_flg, tk_snd_mbf, tk_rcv_mbf
 *
 *	Only this task touches GP0/GP1/GP12 and PWM slice 6.  Other
 *	subsystems learn what it found through mbf_obstacle (IF-06) and
 *	flg_obstacle (IF-07), never by calling the drivers.
 */

#include "subsystem5.h"
#include "s5_config.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include "obstacle_profile.h"
#include "../subsystem1/subsystem1.h"	/* IF-B1-01: telemetry_post */

/* Stage 1 coarse sweep, logical degrees (0 = right, 90 = ahead).
   Team design (Tribunal 1): 45..135 to fit the sweep budget.  The brief's
   example is 30/60/90/120/150; any multiples of 5 work. */
LOCAL const UB coarse_angles[] = { 45U, 70U, 90U, 110U, 135U };
#define S5_COARSE_COUNT	(sizeof(coarse_angles) / sizeof(coarse_angles[0]))

#define US_PER_MS	1000U

EXPORT volatile UW	s5_ping_ok;
EXPORT volatile UW	s5_ping_fail;
EXPORT volatile UW	s5_profiles_done;

LOCAL UINT	fail_run;		/* failed pings in a row */
LOCAL BOOL	fault_active;		/* S5_FLG_FAULT_NO_ECHO is set */

/* Big structures live here, not on the 2 KB task stack */
LOCAL S5_SCAN		scan;
LOCAL S5_PROFILE	prof;

/* ------------------------------------------------------------------ */

/*
 * Count every ping and raise/clear the sensor-fault flag.
 */
LOCAL BOOL note_ping(US_STATUS st)
{
	BOOL	ok = (st == US_OK) || (st == US_OUT_OF_RANGE);

	if (ok) {
		s5_ping_ok++;
		fail_run = 0U;
		if (fault_active) {
			/* tk_clr_flg keeps the bits that are 1 in the pattern */
			(void)tk_clr_flg(flg_obstacle, ~S5_FLG_FAULT_NO_ECHO);
			fault_active = FALSE;
			tm_printf((UB*)"S5: sensor answering again\n");
		}
	} else {
		s5_ping_fail++;
		if (fail_run < S5_FAULT_PINGS) {
			fail_run++;
		}
		if ((!fault_active) && (fail_run >= S5_FAULT_PINGS)) {
			(void)tk_set_flg(flg_obstacle, S5_FLG_FAULT_NO_ECHO);
			fault_active = TRUE;
			tm_printf((UB*)"S5: FAULT %s -- check 5 V power, GND, "
				  "TRIG=GP%u, ECHO=GP%u (via divider)\n",
				  (st == US_STUCK) ? "echo stuck high" : "no echo",
				  S5_PIN_US_TRIG, S5_PIN_US_ECHO);
		}
	}
	return ok;
}

/*
 * IF-06: send one event word without ever blocking.  If the buffer is
 * full, throw away the oldest event so the newest one gets in.
 */
LOCAL void publish(UW state, UW action, BOOL noecho)
{
	UW	evt;
	UW	oldest;
	ER	er;

	evt = s5_evt_pack(state, prof.detected, action, prof.width_cm,
			  prof.closest_cm, prof.closest_angle, noecho);

	er = tk_snd_mbf(mbf_obstacle, &evt, (INT)sizeof(evt), TMO_POL);
	if (er == E_TMOUT) {
		(void)tk_rcv_mbf(mbf_obstacle, &oldest, TMO_POL);
		er = tk_snd_mbf(mbf_obstacle, &evt, (INT)sizeof(evt), TMO_POL);
	}
	CHECK(er, "S5 publish");
	tm_printf((UB*)"S5: event 0x%x  state=%s action=%s\n",
		  evt, s5_state_name(state), s5_action_name(action));

	/* IF-B1-01: obstacle width for telemetry (TQ_FIELD_OBSTACLE).  Only
	   once it is measured (DECIDING) or gone (MONITORING); the STOP event
	   during PROFILING has no width yet.  Never blocks. */
	if (state != S5_ST_PROFILING) {
		UW	w = prof.width_cm;

		(void)telemetry_post(TQ_FIELD_OBSTACLE,
				     prof.detected ? (INT)((w > APP_OBST_CM_MAX) ? APP_OBST_CM_MAX : w)
						   : (INT)APP_OBST_CM_NA);
	}
}

/*
 * Point the servo, take the median of S5_PINGS_PER_POINT pings and add
 * the reading to the scan.  Returns TRUE if it is part of the obstacle.
 */
LOCAL BOOL scan_point(UINT angle, UH hit_limit, BOOL *p_noecho)
{
	UH		cm;
	US_STATUS	st;
	BOOL		valid;

	servo_move(angle);
	st = us_ping_median(S5_PINGS_PER_POINT, &cm);
	valid = note_ping(st);
	if (!valid) {
		*p_noecho = TRUE;
	}
	(void)s5_scan_insert(&scan, angle, cm, valid);
	return (valid && (cm <= hit_limit));
}

/*
 * Stage 2 fine scan of one edge: step from the last coarse hit towards
 * the first coarse miss in S5_FINE_STEP_DEG steps, stop at the first miss.
 */
LOCAL void refine_edge(INT hit_angle, INT miss_angle, UH hit_limit, BOOL *p_noecho)
{
	INT	step = (miss_angle > hit_angle) ? (INT)S5_FINE_STEP_DEG
						: -(INT)S5_FINE_STEP_DEG;
	INT	a = hit_angle + step;
	BOOL	hit = TRUE;

	while (hit && (a != miss_angle)) {
		hit = scan_point((UINT)a, hit_limit, p_noecho);
		a += step;
	}
}

/*
 * PROFILING: Stage 1 coarse sweep + Stage 2 fine scan + Stage 3 profile.
 * 'ahead_cm' is the distance that triggered it; readings within
 * S5_DEPTH_TOL_CM of it count as the same obstacle.
 */
LOCAL BOOL run_profile(UH ahead_cm, UW *p_ms)
{
	UH	hit_limit = (UH)(ahead_cm + S5_DEPTH_TOL_CM);
	UW	t0 = us_now_us();
	BOOL	noecho = FALSE;
	UINT	i;
	INT	r_hit = -1;
	INT	r_miss = -1;
	INT	l_hit = -1;
	INT	l_miss = -1;

	s5_scan_clear(&scan);

	/* Stage 1: coarse sweep, right to left */
	for (i = 0U; i < S5_COARSE_COUNT; i++) {
		(void)scan_point(coarse_angles[i], hit_limit, &noecho);
	}
	s5_profile_build(&scan, hit_limit, &prof);

	/* Stage 2: fine scan, only where an edge lies between a hit and a
	   miss.  Note both edges BEFORE scanning: inserting points moves the
	   indices. */
	if (prof.detected) {
		if (prof.idx_right > 0U) {
			r_hit  = (INT)scan.pt[prof.idx_right].angle;
			r_miss = (INT)scan.pt[prof.idx_right - 1U].angle;
		}
		if ((prof.idx_left + 1U) < scan.n) {
			l_hit  = (INT)scan.pt[prof.idx_left].angle;
			l_miss = (INT)scan.pt[prof.idx_left + 1U].angle;
		}
		if (r_hit >= 0) {
			refine_edge(r_hit, r_miss, hit_limit, &noecho);
		}
		if (l_hit >= 0) {
			refine_edge(l_hit, l_miss, hit_limit, &noecho);
		}
		/* Stage 3 again, with the extra points */
		s5_profile_build(&scan, hit_limit, &prof);
	}

	servo_move(S5_CENTRE_DEG);	/* look ahead again */
	*p_ms = (us_now_us() - t0) / US_PER_MS;
	return noecho;
}

LOCAL void print_profile(UW ms)
{
	UINT	i;

	for (i = 0U; i < scan.n; i++) {
		tm_printf((UB*)"S5:   %u deg  %u cm%s\n",
			  scan.pt[i].angle, scan.pt[i].cm,
			  (scan.pt[i].valid == 0U) ? "  (no echo)"
			  : (((i >= prof.idx_right) && (i <= prof.idx_left)
			      && prof.detected) ? "  <- obstacle" : ""));
	}
	tm_printf((UB*)"S5: width=%u cm  closest=%u cm at %u deg  edges x=%d..%d cm\n",
		  prof.width_cm, prof.closest_cm, prof.closest_angle,
		  prof.x_left_cm, prof.x_right_cm);
	tm_printf((UB*)"S5: left  %s clear=%u cm shift=%d cm\n",
		  prof.left_open ? "open  " : "closed", prof.left_clear_cm,
		  prof.shift_left_cm);
	tm_printf((UB*)"S5: right %s clear=%u cm shift=%d cm\n",
		  prof.right_open ? "open  " : "closed", prof.right_clear_cm,
		  prof.shift_right_cm);
	tm_printf((UB*)"S5: decision %s  (profile took %u ms, %u points)\n",
		  s5_action_name(prof.action), ms, scan.n);
}

/* Sleep for what is left of one monitoring period */
LOCAL void wait_rest_of_period(UW t0)
{
	UW	used_ms = (us_now_us() - t0) / US_PER_MS;

	if (used_ms < S5_MONITOR_PERIOD_MS) {
		tk_dly_tsk((RELTIM)(S5_MONITOR_PERIOD_MS - used_ms));
	}
}

/* ------------------------------------------------------------------ */

EXPORT void obstacle_task(INT stacd, void *exinf)
{
	UINT		state = S5_ST_MONITORING;
	UINT		confirm = 0U;
	UINT		clear = 0U;
	UINT		pings = 0U;
	UH		cm = 0U;
	UH		ahead_cm = 0U;
	US_STATUS	st;
	UW		t0;
	UW		ms = 0U;
	UW		bypass_start = 0U;
	BOOL		noecho = FALSE;
	BOOL		hold_sent = FALSE;	/* STOP sent, CONTINUE not yet */

	(void)stacd; (void)exinf;

	/* This task owns the hardware, so it sets it up */
	CHECK(servo_init(), "S5 servo_init");
	CHECK(us_init(), "S5 us_init");
	tm_printf((UB*)"S5: obstacle task ready, obstacle if < %u cm\n",
		  S5_OBSTACLE_CM);

	while (1) {
		t0 = us_now_us();

		switch (state) {
		case S5_ST_MONITORING:
			/* One forward ping per period */
			st = us_ping(&cm);
			(void)note_ping(st);

			if ((S5_PRINT_EVERY != 0U) && ((++pings % S5_PRINT_EVERY) == 0U)) {
				if (st == US_OK) {
					tm_printf((UB*)"S5: ahead %u cm\n", cm);
				} else if (st == US_OUT_OF_RANGE) {
					tm_printf((UB*)"S5: ahead > %u cm\n", S5_US_RANGE_MAX_CM);
				} else {
					tm_printf((UB*)"S5: ahead ? (no reading)\n");
				}
			}

			/* Back here after a bypass timeout with the car still
			   held: release it once the path is clear (the obstacle
			   may have moved into the 25..35 cm hysteresis band and
			   so never be confirmed again). */
			if (hold_sent) {
				if ((st == US_OUT_OF_RANGE) || ((st == US_OK) && (cm >= S5_CLEAR_CM))) {
					clear++;
				} else {
					clear = 0U;
				}
				if (clear >= S5_CLEAR_PINGS) {
					clear = 0U;
					tm_printf((UB*)"S5: path clear\n");
					prof = (S5_PROFILE){0};
					publish(S5_ST_MONITORING, S5_ACT_CONTINUE, fault_active);
					hold_sent = FALSE;
				}
			}

			/* Two close readings in a row = obstacle confirmed */
			if ((st == US_OK) && (cm < S5_OBSTACLE_CM)) {
				confirm++;
			} else {
				confirm = 0U;
			}
			if (confirm >= S5_CONFIRM_PINGS) {
				confirm = 0U;
				ahead_cm = cm;
				hold_sent = TRUE;	/* the STOP below holds the car */
				(void)tk_set_flg(flg_obstacle, S5_FLG_OBSTACLE_STOP);
				prof = (S5_PROFILE){0};
				prof.detected = TRUE;
				prof.closest_cm = cm;
				prof.closest_angle = (UB)S5_CENTRE_DEG;
				tm_printf((UB*)"S5: obstacle at %u cm -> STOP, profiling\n", cm);
				publish(S5_ST_PROFILING, S5_ACT_STOP, FALSE);
				state = S5_ST_PROFILING;
			} else {
				wait_rest_of_period(t0);
			}
			break;

		case S5_ST_PROFILING:
			noecho = run_profile(ahead_cm, &ms);
			state = S5_ST_DECIDING;
			break;

		case S5_ST_DECIDING:
			s5_profile_plan(&prof);
			print_profile(ms);
			publish(S5_ST_DECIDING, prof.action, noecho || fault_active);
			s5_profiles_done++;
			/* The decision now tells the Vehicle Controller what to
			   do, so our stop request is no longer needed. */
			(void)tk_clr_flg(flg_obstacle, ~S5_FLG_OBSTACLE_STOP);

			if (prof.action == S5_ACT_CONTINUE) {
				hold_sent = FALSE;	/* CONTINUE just sent */
				state = S5_ST_MONITORING;
			} else {
				clear = 0U;
				bypass_start = us_now_us();
				state = S5_ST_BYPASSING;
			}
			break;

		case S5_ST_BYPASSING:
			/* BENCH STAND-IN.  On the car the Vehicle Controller
			   drives the bypass (Buddy 2) and line recovery uses
			   Buddy 3's LINE_FOUND flag.  Here we just wait until
			   the path ahead is clear (move the obstacle away) or
			   the timeout passes.  If it is still there, MONITORING
			   sees it again and re-profiles: "reverse and retry". */
			st = us_ping(&cm);
			(void)note_ping(st);
			if ((st == US_OUT_OF_RANGE) || ((st == US_OK) && (cm >= S5_CLEAR_CM))) {
				clear++;
			} else {
				clear = 0U;
			}
			if (clear >= S5_CLEAR_PINGS) {
				tm_printf((UB*)"S5: path clear, back to monitoring\n");
				/* Tell the Vehicle Controller it may drive again
				   (IF-06: state MONITORING, no obstacle, CONTINUE).
				   Not sent on the timeout below: the obstacle may
				   still be there, and MONITORING re-checks it. */
				prof = (S5_PROFILE){0};
				publish(S5_ST_MONITORING, S5_ACT_CONTINUE, fault_active);
				hold_sent = FALSE;
				clear = 0U;
				state = S5_ST_MONITORING;
			} else if (((us_now_us() - bypass_start) / US_PER_MS) >= S5_BYPASS_TMO_MS) {
				tm_printf((UB*)"S5: bypass timeout, back to monitoring\n");
				clear = 0U;	/* MONITORING counts clear pings afresh */
				state = S5_ST_MONITORING;
			} else {
				wait_rest_of_period(t0);
			}
			break;

		default:
			/* Reserved states are not entered yet */
			state = S5_ST_MONITORING;
			break;
		}
	}
}
