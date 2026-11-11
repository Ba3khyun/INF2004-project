/*
 *	task_vehicle.c
 *	Subsystem 4 -- INTERIM Vehicle Controller and Safety Handler
 *	(Buddy 4 role).  See subsystem4.h for the interface.
 *
 *	Until Buddy 4's own code lands, these two tasks connect the parts that
 *	already exist:
 *	  Wi-Fi (B1)      remote_cmd_q, SAFETY_REQ_REMOTE_ABORT
 *	  Motion (B2)     motion_cmd_q (MC_CMD_HALT), SAFETY_ESTOP_LATCHED
 *	  Line (B3)       vc_line_cmd_post() = the line gate
 *	  Obstacle (B5)   mbf_obstacle, flg_obstacle
 *	  Telemetry (B1)  TQ_FIELD_VEH_STATE
 *
 *	APIs used: tk_wai_flg (TWF_BITCLR), tk_set_flg, tk_ref_flg,
 *	           tk_rcv_mbf, tk_loc_mtx / tk_unl_mtx, tk_dly_tsk
 */

#include "subsystem4.h"
#include "../subsystem1/subsystem1.h"	/* IF-B1-01: telemetry_post */
#include "../subsystem2/subsystem2.h"	/* motion_cmd_post, MC_CMD_HALT */
#include "../subsystem5/subsystem5.h"	/* IF-06 / IF-07 */

/* Requests that latch the emergency stop.  SAFETY_REQ_OBSTACLE_STOP is
   not one of them: an obstacle is a normal, recoverable stop handled by
   the Vehicle Controller through IF-06/IF-07. */
#define SAFETY_LATCH_REQS	(SAFETY_REQ_REMOTE_ABORT | SAFETY_REQ_COLLISION)

EXPORT ID	mtx_vc_gate;

/* Line gate.  Written only by vehicle_task, under mtx_vc_gate. */
LOCAL volatile BOOL	gate_open = TRUE;
LOCAL BOOL		halt_pending = FALSE;	/* vehicle_task only */

/* Last vehicle state decided by vehicle_task (read by the tests) */
LOCAL volatile UINT	vc_state = APP_VS_INITIALIZING;

EXPORT BOOL vc_line_gate_is_open(void)
{
	return gate_open;
}

EXPORT UINT vc_vehicle_state(void)
{
	return vc_state;
}

/* ------------------------------------------------------------------ */
/*  IF-B4-01: line following -> Motion Control, through the gate      */
/* ------------------------------------------------------------------ */

EXPORT ER vc_line_cmd_post(UB cmd)
{
	ER	er;

	er = tk_loc_mtx(mtx_vc_gate, TMO_FEVR);
	if (er < E_OK) {
		return er;		/* no gate object: do not drive */
	}
	er = gate_open ? motion_cmd_post(cmd) : E_OBJ;
	(void)tk_unl_mtx(mtx_vc_gate);
	return er;
}

/* Close (or keep closed) the gate and make sure Motion Control halts.
   The halt is posted under the same mutex, so any line command queued
   before it is overridden and none can be queued after it. */
LOCAL void gate_close_and_halt(void)
{
	if (tk_loc_mtx(mtx_vc_gate, TMO_FEVR) < E_OK) {
		return;
	}
	if (gate_open) {
		gate_open = FALSE;
		halt_pending = TRUE;
	}
	if (halt_pending && motion_cmd_post(MC_CMD_HALT) == E_OK) {
		halt_pending = FALSE;	/* else retried next period */
	}
	(void)tk_unl_mtx(mtx_vc_gate);
}

LOCAL void gate_reopen(void)
{
	if (tk_loc_mtx(mtx_vc_gate, TMO_FEVR) < E_OK) {
		return;
	}
	gate_open = TRUE;
	halt_pending = FALSE;
	(void)tk_unl_mtx(mtx_vc_gate);
}

/* IF-B4-02 (tests only): in test builds vehicle_task is not running, so
   a test that needs Motion Control to itself closes the gate (and halts
   any line-following move) and reopens it afterwards. */
EXPORT void vc_line_gate_set(BOOL open)
{
	if (open) {
		gate_reopen();
	} else {
		gate_close_and_halt();
	}
}

/* ------------------------------------------------------------------ */
/*  Safety Handler                                                    */
/* ------------------------------------------------------------------ */

EXPORT void safety_task(INT stacd, void *exinf)
{
	UINT	ptn;
	ER	er;

	(void)stacd; (void)exinf;

	tm_printf((UB*)"safety: interim Safety Handler ready\n");
	while (1) {
		ptn = 0U;
		er = tk_wai_flg(flg_safety, SAFETY_LATCH_REQS,
				TWF_ORW | TWF_BITCLR, &ptn, TMO_FEVR);
		if (er < E_OK) {
			CHECK(er, "safety tk_wai_flg");
			tk_dly_tsk(100);
			continue;
		}
		CHECK(tk_set_flg(flg_safety, SAFETY_ESTOP_LATCHED),
		      "safety latch ESTOP");
		tm_printf((UB*)"safety: ESTOP LATCHED (%s%s) -- reset the board to clear\n",
			  ((ptn & SAFETY_REQ_REMOTE_ABORT) != 0U) ? "remote abort " : "",
			  ((ptn & SAFETY_REQ_COLLISION) != 0U) ? "collision" : "");
	}
}

/* ------------------------------------------------------------------ */
/*  Vehicle Controller                                                */
/* ------------------------------------------------------------------ */

LOCAL UW vc_now_ms(void)
{
	SYSTIM	t;

	(void)tk_get_otm(&t);
	return t.lo;
}

LOCAL UINT ref_ptn(ID flgid)
{
	T_RFLG	r;

	if (tk_ref_flg(flgid, &r) < E_OK) {
		return 0xFFFFFFFFU;	/* cannot tell: treat every bit as set */
	}
	return r.flgptn;
}

LOCAL const char *vs_name(UINT vs)
{
	switch (vs) {
	case APP_VS_DRIVING:		return "DRIVING";
	case APP_VS_OBSTACLE_BYPASS:	return "OBSTACLE (stopped)";
	case APP_VS_PAUSED:		return "PAUSED";
	case APP_VS_ERROR:		return "ERROR (ESTOP)";
	default:			return "?";
	}
}

EXPORT void vehicle_task(INT stacd, void *exinf)
{
	BOOL	paused = FALSE;		/* remote PAUSE, cleared by RESUME */
	BOOL	aborted = FALSE;	/* remote ABORT: sticky until reset */
	BOOL	evt_hold = FALSE;	/* last IF-06 event was not CONTINUE */
	BOOL	obstacle_hold;		/* evt_hold or IF-07 stop bit, this period */
	BOOL	estop;
	BOOL	allowed;
	UINT	vs;
	UINT	last_vs = APP_VS_COUNT;	/* "none yet": forces first report */
	UW	last_report = 0U;
	UW	now;
	UW	evt;
	UB	rmsg[RCMD_MSG_LEN];
	APP_CMD	c;
	INT	len;

	(void)stacd; (void)exinf;

	tm_printf((UB*)"vc: interim Vehicle Controller ready (line gate open)\n");
	while (1) {
		/* Remote requests from the Wi-Fi task (IF-B1-02).  An ABORT has
		   already latched ESTOP through Safety; it is also sticky here
		   (RESUME does not undo it), so nothing restarts even if the
		   latch were cleared. */
		while ((len = tk_rcv_mbf(remote_cmd_q, rmsg, TMO_POL)) == (INT)RCMD_MSG_LEN) {
			if (app_rcmd_unpack(rmsg, &c) != APP_OK) {
				continue;
			}
			switch (c.type) {
			case CMD_ABORT:
				aborted = TRUE;
				break;
			case CMD_PAUSE:
				paused = TRUE;
				break;
			case CMD_RESUME:
				paused = FALSE;
				break;
			default:
				break;
			}
			tm_printf((UB*)"vc: remote %s (seq %u)\n",
				  (c.type == CMD_ABORT) ? "ABORT" :
				  (c.type == CMD_PAUSE) ? "PAUSE" : "RESUME", c.seq);
		}
		if (len != E_TMOUT) {
			CHECK(len, "vc tk_rcv_mbf remote_cmd_q");
		}

		/* Obstacle events (IF-06): STOP holds the car, CONTINUE frees
		   it.  TURN_LEFT/TURN_RIGHT/REVERSE keep it stopped: subsystem 5
		   sends CONTINUE once the path ahead is clear again. */
		while ((len = tk_rcv_mbf(mbf_obstacle, &evt, TMO_POL)) == (INT)S5_EVT_MSG_SIZE) {
			BOOL	hold = (s5_evt_action(evt) != S5_ACT_CONTINUE);

			if (hold != evt_hold) {
				tm_printf((UB*)"vc: obstacle %s (action %u, %u cm)\n",
					  hold ? "-> stop and wait" : "clear -> resume",
					  s5_evt_action(evt), s5_evt_closest(evt));
			}
			evt_hold = hold;
		}
		if (len != E_TMOUT) {
			CHECK(len, "vc tk_rcv_mbf mbf_obstacle");
		}
		/* IF-07: the stop bit is set before the STOP event is sent;
		   honour it even if that event was lost.  Recomputed every
		   period (never latched into evt_hold): S5 clears the bit just
		   after it sends its decision, and a CONTINUE read in between
		   must not leave the car held for ever. */
		obstacle_hold = evt_hold ||
				((ref_ptn(flg_obstacle) & S5_FLG_OBSTACLE_STOP) != 0U);

		estop = (ref_ptn(flg_safety) & SAFETY_ESTOP_LATCHED) != 0U;

		if (estop || aborted) {
			vs = APP_VS_ERROR;
		} else if (paused) {
			vs = APP_VS_PAUSED;
		} else if (obstacle_hold) {
			vs = APP_VS_OBSTACLE_BYPASS;
		} else {
			vs = APP_VS_DRIVING;
		}

		allowed = (vs == APP_VS_DRIVING);
		vc_state = vs;
		if (!allowed) {
			gate_close_and_halt();	/* also retries a pending halt */
		} else if (!gate_open) {
			gate_reopen();
		}

		now = vc_now_ms();
		if (vs != last_vs || (UW)(now - last_report) >= S4_VC_STATE_REPORT_MS) {
			if (vs != last_vs) {
				tm_printf((UB*)"vc: state %s\n", vs_name(vs));
			}
			(void)telemetry_post(TQ_FIELD_VEH_STATE, (INT)vs);
			last_vs = vs;
			last_report = now;
		}

		tk_dly_tsk(S4_VC_PERIOD_MS);
	}
}
