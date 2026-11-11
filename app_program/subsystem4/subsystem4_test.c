/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem4/TEST_LOG.md and the
 *	INT- result in testing/integration/TEST_LOG.md.
 *
 *	S4-U01, INT-02  template Monitor (IF-03)
 *	S4-U03..U09     interim Vehicle Controller + Safety Handler
 *	                (task_vehicle.c).  Normal builds start these tasks in
 *	                subsystem4_start(); test builds do not, because the
 *	                S1/S2 tests play those roles.  So this test starts them
 *	                itself, AFTER the S1 and S2 tests are done, then plays
 *	                Wi-Fi (remote_cmd_q, SAFETY_REQ_REMOTE_ABORT) and
 *	                Obstacle (mbf_obstacle, flg_obstacle) and checks the
 *	                controller's state and line gate.  The tasks keep
 *	                running afterwards, and the last check leaves the car
 *	                stopped (remote ABORT is sticky until reset).
 */

#include "subsystem4.h"
#include "../subsystem5/subsystem5.h"	/* IF-06/07: mbf_obstacle, flg_obstacle */
#include "../subsystem2/subsystem2.h"	/* MC_CMD_DRIVE_FORWARD */
#include "../test_report.h"

#if APP_TEST

#define S4T_SETTLE_MS	100	/* > 2 vehicle_task periods (10..20 ms each) */

LOCAL ER s4t_remote(UINT type, UINT seq)
{
	UB	m[RCMD_MSG_LEN];

	if (app_rcmd_pack(m, type, seq) != APP_OK) {
		return E_PAR;
	}
	return tk_snd_mbf(remote_cmd_q, m, RCMD_MSG_LEN, TMO_POL);
}

LOCAL ER s4t_obstacle(UW state, BOOL det, UW action)
{
	UW	evt = s5_evt_pack(state, det, action, 20U, 15U, 90U, FALSE);

	return tk_snd_mbf(mbf_obstacle, &evt, (INT)sizeof(evt), TMO_POL);
}

LOCAL UINT s4t_safety_ptn(void)
{
	T_RFLG	r;

	return (tk_ref_flg(flg_safety, &r) == E_OK) ? r.flgptn : 0xFFFFFFFFU;
}

/* Line gate is closed: a line-following post is refused with E_OBJ */
LOCAL BOOL s4t_gate_closed(void)
{
	return !vc_line_gate_is_open() &&
	       vc_line_cmd_post(MC_CMD_DRIVE_FORWARD) == E_OBJ;
}

LOCAL void s4t_report(const char *what)
{
	tm_printf((UB*)"        %s: vc state=%u gate=%s flg_safety=0x%x\n",
		  what, vc_vehicle_state(),
		  vc_line_gate_is_open() ? "open" : "closed", s4t_safety_ptn());
}

EXPORT void subsystem4_test(void)
{
	T_RMBF	rmbf;
	UW	before;
	ID	tid_s, tid_v;
	UB	drain[RCMD_MSG_LEN];
	UW	evt;

	TEST_INIT();

	/* S4-U01: the message buffer accepts the documented line length */
	TEST_CHECK("S4-U01",
		   tk_ref_mbf(mbf_summary, &rmbf) == E_OK &&
		   rmbf.maxmsz == S4_SUMMARY_MAX,
		   "message buffer max line = S4_SUMMARY_MAX");

	/* INT-02 (integration, IF-03): summary lines from subsystem 3 reach
	   the monitor.  One line per 5 samples = one per 1.5 s. */
	before = s4_lines_received;
	tk_dly_tsk(3500);
	TEST_CHECK("INT-02", s4_lines_received >= before + 2,
		   "summary lines flow from subsystem 3 to subsystem 4");

	/* ============ interim Vehicle Controller + Safety Handler ============ */
	/* Known start: no ESTOP, no requests, no stale commands or events */
	(void)tk_clr_flg(flg_safety, 0U);
	(void)tk_clr_flg(flg_obstacle, ~S5_FLG_OBSTACLE_STOP);
	while (tk_rcv_mbf(remote_cmd_q, drain, TMO_POL) > 0) {
	}
	while (tk_rcv_mbf(mbf_obstacle, &evt, TMO_POL) > 0) {
	}

	tid_s = app_start_task(safety_task, S4_PRI_SAFETY, 0, "safety_task");
	tid_v = app_start_task(vehicle_task, S4_PRI_VEHICLE, 0, "vehicle_task");
	tk_dly_tsk(S4T_SETTLE_MS);
	s4t_report("started");
	TEST_CHECK("S4-U03",
		   tid_s > 0 && tid_v > 0 &&
		   vc_vehicle_state() == APP_VS_DRIVING && vc_line_gate_is_open(),
		   "safety + vehicle tasks start; no hold -> DRIVING, line gate open");

	/* IF-B1-02 PAUSE / RESUME (test plays the Wi-Fi task) */
	(void)s4t_remote(CMD_PAUSE, 1U);
	tk_dly_tsk(S4T_SETTLE_MS);
	s4t_report("after PAUSE");
	TEST_CHECK("S4-U04",
		   vc_vehicle_state() == APP_VS_PAUSED && s4t_gate_closed(),
		   "remote PAUSE -> PAUSED, line gate closed (post refused E_OBJ)");

	(void)s4t_remote(CMD_RESUME, 2U);
	tk_dly_tsk(S4T_SETTLE_MS);
	s4t_report("after RESUME");
	TEST_CHECK("S4-U05",
		   vc_vehicle_state() == APP_VS_DRIVING && vc_line_gate_is_open(),
		   "remote RESUME -> DRIVING, line gate open again");

	/* IF-06 obstacle events (test plays subsystem 5) */
	(void)s4t_obstacle(S5_ST_PROFILING, TRUE, S5_ACT_STOP);
	tk_dly_tsk(S4T_SETTLE_MS);
	{
		BOOL held = (vc_vehicle_state() == APP_VS_OBSTACLE_BYPASS) &&
			    s4t_gate_closed();

		(void)s4t_obstacle(S5_ST_DECIDING, TRUE, S5_ACT_TURN_LEFT);
		tk_dly_tsk(S4T_SETTLE_MS);
		held = held && (vc_vehicle_state() == APP_VS_OBSTACLE_BYPASS);
		(void)s4t_obstacle(S5_ST_MONITORING, FALSE, S5_ACT_CONTINUE);
		tk_dly_tsk(S4T_SETTLE_MS);
		s4t_report("after obstacle STOP/TURN_LEFT/CONTINUE");
		TEST_CHECK("S4-U06",
			   held && vc_vehicle_state() == APP_VS_DRIVING &&
			   vc_line_gate_is_open(),
			   "obstacle STOP and TURN hold the car; CONTINUE releases it");
	}

	/* IF-07 stop bit alone holds, and releases as soon as it clears
	   (it must never be latched by the controller) */
	(void)tk_set_flg(flg_obstacle, S5_FLG_OBSTACLE_STOP);
	tk_dly_tsk(S4T_SETTLE_MS);
	{
		BOOL held = (vc_vehicle_state() == APP_VS_OBSTACLE_BYPASS) &&
			    s4t_gate_closed();

		(void)tk_clr_flg(flg_obstacle, ~S5_FLG_OBSTACLE_STOP);
		tk_dly_tsk(S4T_SETTLE_MS);
		s4t_report("after flg_obstacle STOP set/clear");
		TEST_CHECK("S4-U07",
			   held && vc_vehicle_state() == APP_VS_DRIVING &&
			   vc_line_gate_is_open(),
			   "S5_FLG_OBSTACLE_STOP holds while set, released when cleared");
	}

	/* IF-B1-03 remote abort request -> Safety latches ESTOP */
	(void)tk_set_flg(flg_safety, SAFETY_REQ_REMOTE_ABORT);
	tk_dly_tsk(S4T_SETTLE_MS);
	{
		UINT ptn = s4t_safety_ptn();

		s4t_report("after SAFETY_REQ_REMOTE_ABORT");
		TEST_CHECK("S4-U08",
			   (ptn & SAFETY_ESTOP_LATCHED) != 0U &&
			   (ptn & SAFETY_REQ_REMOTE_ABORT) == 0U &&
			   vc_vehicle_state() == APP_VS_ERROR && s4t_gate_closed(),
			   "abort request: Safety clears it, latches ESTOP; VC -> ERROR, gate closed");
	}

	/* Remote ABORT message is sticky in the controller: even after the
	   latch is reset (test plays the local reset) and a RESUME arrives,
	   the car stays stopped.  Leaves the car stopped until reset. */
	(void)s4t_remote(CMD_ABORT, 3U);
	tk_dly_tsk(S4T_SETTLE_MS);
	(void)tk_clr_flg(flg_safety, ~(UINT)SAFETY_ESTOP_LATCHED);
	(void)s4t_remote(CMD_RESUME, 4U);
	tk_dly_tsk(S4T_SETTLE_MS);
	s4t_report("after ABORT, latch reset, RESUME");
	TEST_CHECK("S4-U09",
		   vc_vehicle_state() == APP_VS_ERROR && s4t_gate_closed(),
		   "remote ABORT is sticky: latch reset + RESUME do not restart the car");

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
