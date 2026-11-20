/*
 *	app_main.c
 *	START HERE.  usermain() below is your program's entry point.
 *
 *	How micro T-Kernel starts your code
 *	-----------------------------------
 *	  reset -> kernel initialises itself -> creates the "initial task"
 *	        -> initial task calls usermain()
 *
 *	usermain() runs inside that initial task at priority 1 (the highest).
 *	On a single core nothing you create runs until usermain() blocks, but
 *	on the dual-core build the OTHER core may start a new task at once.
 *	So usermain() works in two phases:
 *	  1. every subsystem creates its kernel objects  (subsystemN_init)
 *	  2. every subsystem starts its tasks             (subsystemN_start)
 *	and then puts the initial task to sleep forever.
 *	If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *	The example system: one subsystem per folder, one task each
 *	------------------------------------------------------------
 *	  subsystem1/  LED       blinks GP19, paced by a cyclic handler
 *	  subsystem2/  Producer  makes a "sample" every 300 ms     (core 1)
 *	  subsystem3/  Consumer  collects samples into statistics  (core 0)
 *	  subsystem4/  Monitor   prints reports, pauses/resumes the LED
 *	  subsystem5/  (empty template for the fifth student)
 *
 *	How they talk to each other (the interfaces, see TEAM.md section 5):
 *	  IF-01  S2 -> S3  mailbox mbx_samples      (owned by S3)
 *	  IF-02  S3 -> S2  semaphore sem_slots + memory pool mpf_samples
 *	                   (owned by S2: S3 hands slots and blocks back)
 *	  IF-03  S3 -> S4  message buffer mbf_summary + event flag flg_events
 *	                   (owned by S4)
 *	  IF-04  S4 -> S3  reads app_stats under mutex mtx_stats (owned by S3)
 *	  IF-05  S4 -> S1  suspends/resumes the LED task tid_led (owned by S1)
 *	The subsystem that RECEIVES or OWNS the data creates the object and
 *	publishes its ID in its header; the other side just uses it.
 *
 *	micro T-Kernel APIs used, and where to find them:
 *	  tasks       tk_cre_tsk tk_sta_tsk tk_slp_tsk tk_wup_tsk tk_can_wup
 *	              tk_dly_tsk tk_sus_tsk tk_rsm_tsk tk_chg_pri tk_ref_tsk
 *	              tk_get_tid tk_get_prc                         app_main.c, S1-S4
 *	  semaphore   tk_cre_sem tk_wai_sem tk_sig_sem tk_ref_sem       S2, S3, S4
 *	  mem. pool   tk_cre_mpf tk_get_mpf tk_rel_mpf tk_ref_mpf       S2, S3, S4
 *	  mailbox     tk_cre_mbx tk_snd_mbx tk_rcv_mbx                  S2, S3
 *	  mutex       tk_cre_mtx tk_loc_mtx tk_unl_mtx                  S3, S4
 *	  msg buffer  tk_cre_mbf tk_snd_mbf tk_rcv_mbf                  S3, S4
 *	  event flag  tk_cre_flg tk_set_flg tk_wai_flg                  S3, S4
 *	  cyclic      tk_cre_cyc tk_ref_cyc                             S1
 *	  alarm       tk_cre_alm tk_sta_alm tk_stp_alm                  S4
 *	  time        tk_get_tim                                        S2, S4
 *	  system      tk_ref_ver                                        app_main.c
 *
 *	Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"
#include "subsystem4/subsystem4.h"
#include "vehicle_controller/vehicle_controller.h"
#include "safety_handler/safety_handler.h"

#if APP_TEST
#include "safety_handler/tests/safety_test_suite.h"
#include "subsystem2/motion_control.h"
#endif

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
	T_CTSK	ctsk = {0};
	ID	tid;
	ER	er;

	ctsk.task    = entry;		/* function the task runs */
	ctsk.itskpri = pri;		/* initial priority (1 = highest) */
	ctsk.stksz   = TASK_STACK_SIZE;	/* kernel allocates the stack */
	ctsk.tskatr  = TA_HLNG		/* written in a high-level language (C) */
		     | TA_RNG3;		/* runs at user protection level */
#if TK_SUPPORT_SMP
	if (prc != 0) {
		ctsk.tskatr |= TA_ASSPRC;	/* honour the affinity below */
		ctsk.assprc  = prc;		/* which core(s) it may run on */
	}
#else
	(void)prc;			/* single-core build: no affinity */
#endif

	/* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
	tid = tk_cre_tsk(&ctsk);
	if (tid < E_OK) {
		CHECK(tid, name);
		return tid;
	}

	/* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
	er = tk_sta_tsk(tid, 0);
	if (er < E_OK) {
		tm_printf((UB*)"ERROR tk_sta_tsk for %s: %d\n", name, er);
		return (ID)er;
	}
	return tid;
}

/* Stop boot before starting any subsystem task if object creation fails.
 * Kept local to app_main.c; other uses of CHECK() remain unchanged.
 */
static void app_require_init(const char *name, ER er)
{
    if (er >= E_OK) {
        return;
    }

    tm_printf((UB *)
        "FATAL INIT: %s failed (err=%d). "
        "Skipping subsystem start phase.\n",
        name, (INT)er);

    /* Remain asleep; never proceed to starting application tasks. */
    for (;;) {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

#if APP_TEST
/* After the automated tests, keep the input-only diagnostics visible.
 * All values come from existing read-only IMU/encoder interfaces; do not
 * open device handles, acquire locks, consume event queues, or post motors.
 * The LSM303D cannot reliably detect a perfectly level, stationary lift.
 */
static void b4_live_imu_monitor(void)
{
    UW previous_samples = s4_imu_good_sample_count();
    UW previous_hump_start = s4_imu_hump_start_count();
    UW previous_hump_done = s4_imu_hump_done_word();
    UW previous_pickup = s4_imu_pickup_event_count();
    UW interval = 0U;
    UW previous_distance = motion_get_distance_mm();
    UINT stale_warned = 0U;

    tm_printf((UB*)"[B4-LIVE] START after tests; original Buddy3 ADC active, live sensors, pickup motor inhibit active\n");
    tm_printf((UB*)"[B4-LIVE] Observe pitch/roll and accel; pickup reason=1 low-g, 2 overturn, 3 upward motion heuristic\n");
    tm_printf((UB*)"[B4-LIVE] Upward-motion detection is experimental; perfectly smooth level lifting cannot be guaranteed\n");
    tm_printf((UB*)"[B4-LIVE] Hump heights need encoder travel; stationary tilts can produce INVALID 0mm readings\n");

    for (;;) {
        UW snap, samples, sample_delta, age, pickup, reason;
        UW hump_start, hump_done, distance, distance_delta;
        INT pitch_x10, roll_x10;
        s4_imu_state_t state;
        uint16_t ev;

        (void)tk_dly_tsk(2000);
        interval++;
        snap = s4_imu_get_snapshot();
        samples = s4_imu_good_sample_count();
        sample_delta = samples - previous_samples;
        previous_samples = samples;
        age = s4_imu_good_sample_age_ms();
        state = s4_imu_state();
        pitch_x10 = ((INT)s4_snap_pitch(snap) - 128) * 5; /* OB8 0.5deg/LSB */
        roll_x10 = ((INT)s4_snap_roll(snap) - 128) * 5;
        pickup = s4_imu_pickup_event_count();
        reason = s4_imu_pickup_reason();
        hump_start = s4_imu_hump_start_count();
        hump_done = s4_imu_hump_done_word();
        distance = motion_get_distance_mm();
        distance_delta = distance - previous_distance;
        previous_distance = distance;

        tm_printf((UB*)"[B4-LIVE] t=%us samples_2s=%u age_ms=%u state=%u valid=%u pitch_x10=%d roll_x10=%d accel_mg=%u dist_delta_mm=%u lift_peak_up_mg=%u lift_run=%u lift_max_run=%u pickup=%u reason=%u\n",
                  (UINT)(interval * 2U), (UINT)sample_delta, (UINT)age,
                  (UINT)state, (UINT)s4_snap_valid(snap),
                  pitch_x10, roll_x10, (UINT)s4_stat_accel_mg(),
                  (UINT)distance_delta, (UINT)s4_imu_lift_peak_up_mg(),
                  (UINT)s4_imu_lift_up_run(),
                  (UINT)s4_imu_lift_max_run(), (UINT)pickup, (UINT)reason);

        tm_printf((UB*)"[B4-ENC] total pulses left_GP7=%u right_GP6=%u\n",
                  (UINT)encoder_get_left_count(), (UINT)encoder_get_right_count());

        if (hump_start != previous_hump_start) {
            tm_printf((UB*)"[B4-HUMP-LIVE] START count=%u travel_2s=%u mm\n",
                      (UINT)hump_start, (UINT)distance_delta);
            previous_hump_start = hump_start;
        }
        if (hump_done != previous_hump_done) {
            ev = (uint16_t)(hump_done & 0xffffU);
            tm_printf((UB*)"[B4-HUMP-LIVE] DONE count=%u height_mm=%u valid=%u travel_2s=%u mm\n",
                      (UINT)(hump_done >> 16), (UINT)s4_evt_height_mm(ev),
                      (UINT)s4_evt_valid(ev), (UINT)distance_delta);
            previous_hump_done = hump_done;
        }
        if (pickup != previous_pickup) {
            tm_printf((UB*)"[B4-LIVE] PICKUP_DETECTED count=%u reason=%u; drive inhibited until reboot\n",
                      (UINT)pickup, (UINT)reason);
            previous_pickup = pickup;
        }
        if ((age > 200U || sample_delta < 140U) && pickup == 0U) {
            if (!stale_warned) {
                tm_printf((UB*)"[B4-LIVE] IMU_STALL suspected: stage=%u stage_age_ms=%u task_id=%d; motor forwarding remains disabled\n",
                          (UINT)s4_diag_stage(), (UINT)s4_diag_stage_age_ms(),
                          (INT)s4_diag_task_id());
                stale_warned = 1U;
            }
        } else {
            stale_warned = 0U;
        }
        if (pickup != 0U) {
            tm_printf((UB*)"[B4-LIVE] RESULT=DETECTED. Terminal IMU fault is expected; restart before further measurements.\n");
            break;
        }
    }
}
#endif /* APP_TEST */

EXPORT INT usermain(void)
{
	T_RVER	ver;

	/* tk_ref_ver: ask the kernel who it is */
	tk_ref_ver(&ver);
	tm_printf((UB*)"\n=== micro T-Kernel on Raspberry Pi Pico W"
		  " (spec 0x%04x, product 0x%04x) ===\n", ver.spver, ver.prver);
#if TK_SUPPORT_SMP
	tm_printf((UB*)"dual-core (SMP) build, usermain on core %d\n",
		  tk_get_prc() - 1);	/* tk_get_prc() counts from 1 */
#else
	tm_printf((UB*)"single-core build\n");
#endif


	/* Phase 1: create all kernel objects.
	* Do not start application tasks if any initialization fails.
	*/
	app_require_init("app_shared_init", app_shared_init());

	app_require_init("subsystem1_init", subsystem1_init());
	app_require_init("subsystem2_init", subsystem2_init());
	app_require_init("subsystem3_init", subsystem3_init());
	app_require_init("subsystem4_init", subsystem4_init());
	app_require_init("subsystem5_init", subsystem5_init());

	app_require_init("safety_handler_init", safety_handler_init());
	app_require_init("vehicle_controller_init", vehicle_controller_init());

	/* Phase 2: every subsystem starts its tasks */
	CHECK(subsystem1_start(), "subsystem1_start");
	CHECK(subsystem2_start(), "subsystem2_start");
	CHECK(subsystem3_start(), "subsystem3_start");
	CHECK(subsystem4_start(), "subsystem4_start");
	CHECK(subsystem5_start(), "subsystem5_start");

	CHECK(safety_handler_start(), "safety_handler_start");
	CHECK(vehicle_controller_start(), "vehicle_controller_start");

	tm_printf((UB*)"all subsystems started\n\n");

#if APP_TEST
    tm_printf((UB*)"[TEST] === test run start ===\n");

    subsystem1_test();
    subsystem2_test();
    subsystem3_test();
    subsystem4_test();
    subsystem5_test();

    /* Buddy 4 combined validation */
    (void)safety_test_suite_run();

    tm_printf((UB*)"[TEST] === test run end ===\n");
    /* Read-only, motor-disconnected live IMU check, running after the unit
     * tests so the motion/IMU timing probes are unaffected by serial output.
     * This is NOT a physical motor stop or hump calibration test.
     * The user may leave it running and inspect live pickup/tilt evidence.
     */
    b4_live_imu_monitor();
#endif

	/* Sleep forever.  Returning from usermain() would shut down. */
	tk_slp_tsk(TMO_FEVR);
	return 0;
}
