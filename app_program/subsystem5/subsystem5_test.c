/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	Record results in testing/subsystem5/TEST_LOG.md.  IDs: S5-U01, ...
 *
 *	U01-U12 check the maths with made-up sweeps: no hardware needed.
 *	U13-U15 check the running system (timer, task, sensor).  They only
 *	READ counters: the sensor itself belongs to obstacle_task.
 */

#include "subsystem5.h"
#include "s5_config.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include "obstacle_profile.h"
#include "../test_report.h"

#if APP_TEST

#define T_FAR		((UH)S5_US_RANGE_MAX_CM)	/* "nothing there" */
#define T_N		5U
/* The kernel ticks every 10 ms and tk_dly_tsk(n) adds one tick, so a
   200 ms sleep lasts 200..210 ms.  Too short a sleep makes this test
   fail on a correct timer (20 ms really lasted ~27.5 ms on the bench). */
#define T_TIMER_MS	200U
#define T_TIMER_MIN_US	195000U
#define T_TIMER_MAX_US	215000U
#define T_LIVE_MS	1000U
#define T_MIN_PINGS	10U	/* 1 s at 50 ms per ping = ~20 */

LOCAL const UB	t_angles[T_N] = { 45U, 70U, 90U, 110U, 135U };
LOCAL S5_SCAN	t_scan;
LOCAL S5_PROFILE t_prof;

/* Build a 5-point sweep at 45/70/90/110/135 deg, then profile + plan it */
LOCAL void t_run(UH c45, UH c70, UH c90, UH c110, UH c135, UH hit_limit)
{
	const UH	cm[T_N] = { c45, c70, c90, c110, c135 };
	UINT		i;

	s5_scan_clear(&t_scan);
	for (i = 0U; i < T_N; i++) {
		(void)s5_scan_insert(&t_scan, t_angles[i], cm[i], TRUE);
	}
	s5_profile_build(&t_scan, hit_limit, &t_prof);
	s5_profile_plan(&t_prof);
}

EXPORT void subsystem5_test(void)
{
	UW	evt;
	UW	t0;
	UW	dt;
	UW	ok0;
	UW	all0;

	TEST_INIT();

	/* --- IF-06 packing ---------------------------------------------- */
	evt = s5_evt_pack(S5_ST_DECIDING, TRUE, S5_ACT_TURN_RIGHT, 23U, 41U, 110U, FALSE);
	TEST_CHECK("S5-U01",
		   (s5_evt_state(evt) == S5_ST_DECIDING) && (s5_evt_det(evt) == 1U)
		   && (s5_evt_action(evt) == S5_ACT_TURN_RIGHT) && (s5_evt_width(evt) == 23U)
		   && (s5_evt_closest(evt) == 41U) && (s5_evt_angle(evt) == 110U)
		   && (s5_evt_noecho(evt) == 0U),
		   "event word: every field round-trips");

	evt = s5_evt_pack(S5_ST_REACQUIRING, TRUE, S5_ACT_REVERSE, 500U, 9999U, 255U, TRUE);
	TEST_CHECK("S5-U02",
		   (s5_evt_state(evt) == S5_ST_REACQUIRING) && (s5_evt_action(evt) == S5_ACT_REVERSE)
		   && (s5_evt_width(evt) == 127U) && (s5_evt_closest(evt) == 511U)
		   && (s5_evt_angle(evt) == 180U) && (s5_evt_noecho(evt) == 1U)
		   && (s5_evt_pack(0U, FALSE, 0U, 0U, 0U, 0U, FALSE) == 0U),
		   "event word: max values kept, too-large values saturate, zero is 0");

	/* --- geometry ---------------------------------------------------- */
	TEST_CHECK("S5-U03",
		   (s5_lateral_cm(100U, 0U) == 100) && (s5_lateral_cm(100U, 90U) == 0)
		   && (s5_lateral_cm(100U, 180U) == -100) && (s5_lateral_cm(100U, 60U) == 50)
		   && (s5_lateral_cm(30U, 135U) == -21),
		   "lateral x = d*cos(angle), + = right");

	TEST_CHECK("S5-U04",
		   (servo_pulse_us(90U) == ((S5_SERVO_MIN_US + S5_SERVO_MAX_US) / 2U))
		   && ((servo_pulse_us(0U) + servo_pulse_us(180U)) == (S5_SERVO_MIN_US + S5_SERVO_MAX_US))
		   && (servo_pulse_us(250U) == servo_pulse_us(180U)),
		   "servo pulse: 90 deg = centre, ends symmetric, clamps above 180");

	/* --- Stage 3/4 with made-up sweeps --------------------------------
	   Car half width 8 + margin 5 = 13 cm must clear each edge. */

	/* Centred box: x = -10..+10, equal detours -> tie goes left */
	t_run(T_FAR, 30U, 28U, 30U, T_FAR, 43U);
	TEST_CHECK("S5-U05",
		   t_prof.detected && (t_prof.width_cm == 20U) && (t_prof.closest_cm == 28U)
		   && (t_prof.closest_angle == 90U) && t_prof.left_open && t_prof.right_open
		   && (t_prof.action == S5_ACT_TURN_LEFT),
		   "centred obstacle: width 20 cm, closest 28 cm @90, TURN_LEFT");

	/* Obstacle sits left of centre (x = -8..0): passing right is shorter */
	t_run(T_FAR, T_FAR, 24U, 25U, T_FAR, 39U);
	TEST_CHECK("S5-U06",
		   t_prof.detected && (t_prof.shift_right_cm < t_prof.shift_left_cm)
		   && (t_prof.action == S5_ACT_TURN_RIGHT),
		   "offset obstacle: picks the smaller detour (TURN_RIGHT)");

	/* Wall across the whole sweep */
	t_run(30U, 30U, 30U, 30U, 30U, 45U);
	TEST_CHECK("S5-U07",
		   t_prof.detected && (!t_prof.left_open) && (!t_prof.right_open)
		   && (t_prof.action == S5_ACT_REVERSE),
		   "blocked both sides: REVERSE");

	/* Nothing within range */
	t_run(T_FAR, T_FAR, T_FAR, T_FAR, T_FAR, 40U);
	TEST_CHECK("S5-U08", (!t_prof.detected) && (t_prof.action == S5_ACT_CONTINUE),
		   "empty sweep: CONTINUE");

	/* Something at 45 deg, 35 cm: x = +24 cm, outside the car's path */
	t_run(35U, T_FAR, T_FAR, T_FAR, T_FAR, 40U);
	TEST_CHECK("S5-U09", (!t_prof.detected) && (t_prof.action == S5_ACT_CONTINUE),
		   "object beside the path: CONTINUE");

	/* 5 cm ahead: too close to turn */
	t_run(T_FAR, T_FAR, 5U, T_FAR, T_FAR, 20U);
	TEST_CHECK("S5-U10", t_prof.detected && (t_prof.action == S5_ACT_REVERSE),
		   "too close: REVERSE");

	/* Left detour is shorter but something sits 38 cm away on the left
	   (< S5_PASS_CLEAR_CM): must go right instead */
	t_run(T_FAR, 21U, 20U, 38U, T_FAR, 35U);
	TEST_CHECK("S5-U11",
		   t_prof.detected && (t_prof.shift_left_cm < t_prof.shift_right_cm)
		   && (t_prof.left_clear_cm == 38U) && (t_prof.action == S5_ACT_TURN_RIGHT),
		   "short side not clear: TURN_RIGHT");

	/* Insert keeps angles sorted and replaces a repeated angle */
	s5_scan_clear(&t_scan);
	(void)s5_scan_insert(&t_scan, 90U, 50U, TRUE);
	(void)s5_scan_insert(&t_scan, 45U, 60U, TRUE);
	(void)s5_scan_insert(&t_scan, 135U, 70U, TRUE);
	(void)s5_scan_insert(&t_scan, 90U, 55U, TRUE);
	TEST_CHECK("S5-U12",
		   (t_scan.n == 3U) && (t_scan.pt[0].angle == 45U) && (t_scan.pt[1].angle == 90U)
		   && (t_scan.pt[1].cm == 55U) && (t_scan.pt[2].angle == 135U),
		   "scan insert: sorted by angle, repeat replaces");

	/* --- the running system ------------------------------------------ */
	t0 = us_now_us();
	tk_dly_tsk(T_TIMER_MS);
	dt = us_now_us() - t0;
	tm_printf((UB*)"[TEST] S5 info: %u us timer counts in %u ms\n", dt, T_TIMER_MS);
	TEST_CHECK("S5-U13", (dt >= T_TIMER_MIN_US) && (dt <= T_TIMER_MAX_US),
		   "hardware timer runs at 1 MHz");

	ok0  = s5_ping_ok;
	all0 = s5_ping_ok + s5_ping_fail;
	tk_dly_tsk(T_LIVE_MS);		/* let obstacle_task work */
	tm_printf((UB*)"[TEST] S5 info: %u pings, %u answered in %u ms\n",
		  (s5_ping_ok + s5_ping_fail) - all0, s5_ping_ok - ok0, T_LIVE_MS);
	TEST_CHECK("S5-U14",
		   (mbf_obstacle > 0) && (flg_obstacle > 0)
		   && (((s5_ping_ok + s5_ping_fail) - all0) >= T_MIN_PINGS),
		   "obstacle task running and pinging (IF-06/07 objects exist)");
	TEST_CHECK("S5-U15", (s5_ping_ok - ok0) >= T_MIN_PINGS,
		   "sensor answers (fails if not wired/powered)");

	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
