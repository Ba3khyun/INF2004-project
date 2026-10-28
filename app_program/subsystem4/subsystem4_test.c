/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem4/TEST_LOG.md and the
 *	INT- result in testing/integration/TEST_LOG.md.
 *
 *	BEFORE RUNNING: the board must lie still on the bench (not moving, not
 *	being tapped) for the first ~6 s after power-up.  S4-U12..S4-U16 depend
 *	on it.  Do not call imu_drv_* from here: the IMU task owns I2C0.
 */

#include "subsystem4.h"
#include "imu_lsm303.h"
#include "imu_math.h"
#include "../test_report.h"

#if APP_TEST

#define WAIT_STEP_MS        50
#define CAL_WAIT_MS         4500    /* 2 s calibration + start-up + margin  */
#define MEASURE_MS          3000

static ER wait_for_monitoring(INT limit_ms)
{
	INT waited = 0;
	while (s4_imu_state() != S4_ST_MONITORING) {
		if (waited >= limit_ms) {
			return E_TMOUT;
		}
		tk_dly_tsk(WAIT_STEP_MS);
		waited += WAIT_STEP_MS;
	}
	return E_OK;
}

static UW drain_events(void)
{
	UW n = 0U;
	uint16_t w;
	while (tk_rcv_mbf(mbf_imu_evt, &w, TMO_POL) >= E_OK) {
		n++;
	}
	return n;
}

EXPORT void subsystem4_test(void)
{
	T_RMBF	rmbf;
	T_RFLG	rflg;
	T_CFLG	cflg = {0};
	ID	flg_test;
	UW	before;
	UW	accepted;
	UW	mag_ovf_before;
	UW	snap;
	uint16_t msg;
	INT	i;

	TEST_INIT();

	/* ---- original checks, kept for the example Subsystem 3 link ---- */
	TEST_CHECK("S4-U01",
		   tk_ref_mbf(mbf_summary, &rmbf) == E_OK &&
		   rmbf.maxmsz == S4_SUMMARY_MAX,
		   "message buffer max line = S4_SUMMARY_MAX");

	/* ---- sensor and start-up ---- */
	TEST_CHECK("S4-U10", imu_drv_variant() == IMU_VAR_LSM303DLHC,
		   "chip identified as LSM303DLHC (mag ID 48 34 33)");

	TEST_CHECK("S4-U12", wait_for_monitoring(CAL_WAIT_MS) == E_OK,
		   "calibration finishes and task reaches MONITORING within 4.5 s");

	TEST_CHECK("S4-U13", s4_stat_accel_mg() >= 900U && s4_stat_accel_mg() <= 1100U,
		   "stationary |a| between 0.9 g and 1.1 g (accel range/scale correct)");

	TEST_CHECK("S4-U14", s4_stat_mag_mgauss() >= 150U && s4_stat_mag_mgauss() <= 1200U,
		   "magnetic field magnitude plausible (0.15 - 1.2 gauss)");

	snap = s4_imu_get_snapshot();
	TEST_CHECK("S4-U15", s4_snap_valid(snap) == 1U && s4_snap_state(snap) == S4_ST_MONITORING,
		   "snapshot word is valid and reports MONITORING");

	tk_ref_flg(flg_faults, &rflg);
	TEST_CHECK("S4-U15b", (rflg.flgptn & (FLT_IMU_I2C | FLT_IMU_CALIB | FLT_IMU_RANGE)) == 0U,
		   "no IMU fault bits set after start-up");

	/* ---- timing (NFR4.1 / NFR4.2) ---- */
	mag_ovf_before = s4_stat_mag_overflow();
	s4_stat_reset();
	tk_dly_tsk(MEASURE_MS);
	TEST_CHECK("S4-U16a", s4_stat_cycles() >= 280U && s4_stat_cycles() <= 320U,
		   "about 300 cycles in 3 s (10 ms period)");
	TEST_CHECK("S4-U16b", s4_stat_cycle_us_max() < 5000U,
		   "worst cycle execution time < 5 ms (NFR4.2)");
	TEST_CHECK("S4-U16c", s4_stat_period_us_max() < 20000U,
		   "worst gap between cycles < 20 ms (no missed tick)");
	TEST_CHECK("S4-U16d", s4_stat_mag_overflow() == mag_ovf_before,
		   "no magnetometer overflow while stationary");
	tm_printf((UB*)"[INFO] S4 timing: cycles=%u cycle_max=%uus period_max=%uus i2c_errors=%u\n",
		  (UINT)s4_stat_cycles(), (UINT)s4_stat_cycle_us_max(),
		  (UINT)s4_stat_period_us_max(), (UINT)imu_drv_error_count());

	/* ---- imu_evt_q depth: proves S4_EVT_BUFSZ really holds 8 messages ---- */
	(void)drain_events();
	accepted = 0U;
	for (i = 0; i < 16; i++) {
		msg = s4_evt_pack(S4_EVT_SHOCK_MINOR, 0U, 0U, (uint32_t)i);
		if (tk_snd_mbf(mbf_imu_evt, &msg, (INT)sizeof(msg), TMO_POL) >= E_OK) {
			accepted++;
		}
	}
	TEST_CHECK("S4-U17", accepted == (UW)S4_EVT_QUEUE_DEPTH,
		   "imu_evt_q holds exactly 8 messages (kernel overhead as assumed)");
	tm_printf((UB*)"[INFO] S4-U17 accepted %u of 16 messages\n", (UINT)accepted);
	(void)drain_events();

	/* ---- packed formats behave the same on the Pico as on the PC ---- */
	msg = s4_evt_pack(S4_EVT_HUMP_DONE, 42U, 1U, 17U);
	TEST_CHECK("S4-U18a",
		   s4_evt_type(msg) == S4_EVT_HUMP_DONE && s4_evt_height_mm(msg) == 42U &&
		   s4_evt_valid(msg) == 1U && s4_evt_seq(msg) == 17U,
		   "imu_evt word packs and unpacks correctly on target");
	snap = s4_snap_pack(0xA5U, 0x80U, 0x7FU, S4_ST_HUMP_TRACKING, 1U, 9U);
	TEST_CHECK("S4-U18b",
		   s4_snap_heading(snap) == 0xA5U && s4_snap_pitch(snap) == 0x80U &&
		   s4_snap_roll(snap) == 0x7FU && s4_snap_state(snap) == S4_ST_HUMP_TRACKING &&
		   s4_snap_valid(snap) == 1U && s4_snap_seq(snap) == 9U,
		   "snapshot word packs and unpacks correctly on target");
	TEST_CHECK("S4-U18c",
		   imu_deg_to_bam8(90.0f) == 64U && imu_deg_to_ob8(10.0f) == 148U &&
		   imu_bam8_diff(5U, 250U) == 11,
		   "BAM8 / offset-binary maths correct on target (soft-float)");

	/* ---- event flag semantics used for the shared fault mask ---- */
	cflg.flgatr = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_test = tk_cre_flg(&cflg);
	if (flg_test >= E_OK) {
		(void)tk_set_flg(flg_test, FLT_IMU_I2C | FLT_NO_ECHO);
		(void)tk_clr_flg(flg_test, (UINT)(~FLT_IMU_I2C));   /* clear ONLY the IMU bit */
		tk_ref_flg(flg_test, &rflg);
		TEST_CHECK("S4-U19", rflg.flgptn == FLT_NO_ECHO,
			   "tk_set_flg ORs and tk_clr_flg(~bit) clears only that bit");
		(void)tk_del_flg(flg_test);
	} else {
		TEST_CHECK("S4-U19", 0, "could not create the test event flag");
	}

	/* ---- no events lost so far ---- */
	TEST_CHECK("S4-U20", s4_stat_events_dropped() == 0U,
		   "no imu events dropped because the queue was full");

	/* ---- INT-02 (example IF-03): kept until Subsystem 3 is replaced ---- */
	before = s4_lines_received;
	tk_dly_tsk(3500);
	TEST_CHECK("INT-02", s4_lines_received >= before + 2,
		   "summary lines flow from subsystem 3 to subsystem 4");

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
