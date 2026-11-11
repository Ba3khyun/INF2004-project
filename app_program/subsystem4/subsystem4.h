/*
 *	subsystem4.h
 *	Subsystem 4 -- Monitor (reporting).  Student ID: <student ID>.
 *
 *	Public interface of subsystem 4.  Other subsystems include it as
 *	"../subsystem4/subsystem4.h".
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

#define S4_PRI_MONITOR		18	/* priority band 18-19 (TEAM.md) */
#define S4_ALARM_MS		2000	/* report anyway if nothing for 2 s */
#define S4_LED_TOGGLE_REPORTS	6	/* pause/resume the LED every N reports */

/* IF-03: send one text line (at most S4_SUMMARY_MAX bytes, no '\0'
   needed) to mbf_summary, then set FLG_BATCH in flg_events. */
#define S4_SUMMARY_MAX		48
#define FLG_BATCH		0x0002	/* set by subsystem 3 */
#define FLG_ALARM		0x0001	/* private: set by our alarm handler */

IMPORT ID	mbf_summary;
IMPORT ID	flg_events;

/* ---------------------------------------------------------------------
 *  INTERIM Vehicle Controller + Safety Handler (Buddy 4 role)
 *  task_vehicle.c.  Ties subsystems 1, 2, 3 and 5 together until Buddy 4's
 *  own code lands; Buddy 4 replaces task_vehicle.c and keeps the names.
 *
 *  safety_task   PRI_SAFETY (5).  Waits on flg_safety for
 *                SAFETY_REQ_REMOTE_ABORT (B1) or SAFETY_REQ_COLLISION (B4
 *                IMU), clears the request (TWF_BITCLR) and latches
 *                SAFETY_ESTOP_LATCHED.  Motion Control brakes on the latch
 *                within one 10 ms tick.  The latch is cleared only by a
 *                local reset (power cycle / RUN button).
 *  vehicle_task  PRI_VEHICLE_CTRL (12).  Every S4_VC_PERIOD_MS:
 *                  remote_cmd_q  (IF-B1-02) ABORT / PAUSE / RESUME
 *                  mbf_obstacle  (IF-06)    STOP ... CONTINUE
 *                  flg_obstacle  (IF-07)    S5_FLG_OBSTACLE_STOP
 *                  flg_safety               SAFETY_ESTOP_LATCHED
 *                decides whether line following may drive the motors
 *                (the "line gate"), halts Motion Control when it closes,
 *                and reports TQ_FIELD_VEH_STATE (IF-B1-01).
 *                Obstacle handling is stop-and-wait: the car stays
 *                stopped until subsystem 5 reports the path clear
 *                (action CONTINUE).  No bypass manoeuvre is driven.
 *
 *  IF-B4-01  S3 -> VC: vc_line_cmd_post(cmd).  Line following posts its
 *            motion commands through the gate instead of straight to
 *            motion_cmd_q.  Returns E_OK when posted, E_OBJ while the
 *            gate is closed (nothing posted), or motion_cmd_post()'s
 *            error.  The gate check and the post are done under
 *            mtx_vc_gate, so no line command can slip in after the
 *            controller's halt.  The gate starts OPEN, and in test builds
 *            (APP_TEST) the two tasks are not started: the on-target
 *            tests play the Safety/VC roles themselves.
 * --------------------------------------------------------------------- */
#define S4_PRI_SAFETY		PRI_SAFETY		/* 5 */
#define S4_PRI_VEHICLE		PRI_VEHICLE_CTRL	/* 12 */
#define S4_VC_PERIOD_MS		10	/* = one kernel tick */
#define S4_VC_STATE_REPORT_MS	1000	/* re-send vehicle state at least this often */

IMPORT ID	mtx_vc_gate;
IMPORT ER	vc_line_cmd_post(UB cmd);

/* IF-B4-02 (on-target tests only, vehicle_task not running): close the
   line gate and halt (FALSE) or reopen it (TRUE).  The S2 tests close it
   so line following cannot post commands while they check Motion. */
IMPORT void	vc_line_gate_set(BOOL open);
IMPORT BOOL	vc_line_gate_is_open(void);	/* tests: gate state */
IMPORT UINT	vc_vehicle_state(void);		/* tests: last APP_VS_* decided */

/* Private to subsystem 4 */
IMPORT void	vehicle_task(INT stacd, void *exinf);
IMPORT void	safety_task(INT stacd, void *exinf);
IMPORT ID	alm_monitor;
IMPORT UW	s4_lines_received;	/* for the tests */
IMPORT void	monitor_task(INT stacd, void *exinf);
IMPORT void	monitor_alarm_handler(void *exinf);

#endif /* SUBSYSTEM4_H */
