/*
 *	subsystem5.c
 *	Subsystem 5 -- Obstacle scanning: creates its objects and starts its
 *	task.  The task itself is in task_obstacle.c.
 *
 *	APIs shown: tk_cre_mbf, tk_cre_flg
 *
 *	Files in this folder
 *	  subsystem5.h        public interface (IF-06, IF-07, pins, packing)
 *	  s5_config.h         every tuning number
 *	  drv_ultrasonic.c/h  HC-SR04 driver (trigger, echo timing, median)
 *	  drv_servo.c/h       50 Hz PWM servo driver
 *	  obstacle_profile.c/h  Stage 3 profile + Stage 4 plan (pure maths)
 *	  task_obstacle.c     the state machine task that ties it together
 *	  subsystem5_test.c   on-target tests (make TEST=1)
 */

#include "subsystem5.h"

EXPORT ID	mbf_obstacle;	/* IF-06 */
EXPORT ID	flg_obstacle;	/* IF-07 */

/* Message buffer bytes: each message costs a 4-byte length header plus
   the 4-byte word (see kernel/tkernel/messagebuf.c) */
#define S5_MBF_HEADER_SIZE	4U
#define S5_MBF_BUF_SIZE		(S5_EVT_DEPTH * (S5_MBF_HEADER_SIZE + S5_EVT_MSG_SIZE))

EXPORT UW s5_evt_pack(UW state, BOOL det, UW action, UW width_cm,
		      UW closest_cm, UW angle_deg, BOOL noecho)
{
	UW	evt = 0U;

	/* Saturate instead of wrapping: 130 cm wide must not read as 2 cm */
	if (width_cm > S5_EVT_MAX(S5_EVT_WIDTH_BITS)) {
		width_cm = S5_EVT_MAX(S5_EVT_WIDTH_BITS);
	}
	if (closest_cm > S5_EVT_MAX(S5_EVT_DIST_BITS)) {
		closest_cm = S5_EVT_MAX(S5_EVT_DIST_BITS);
	}
	if (angle_deg > S5_ANGLE_MAX_DEG) {
		angle_deg = S5_ANGLE_MAX_DEG;
	}

	evt |= (state  & S5_EVT_MAX(S5_EVT_STATE_BITS)) << S5_EVT_STATE_SHIFT;
	evt |= (det ? 1UL : 0UL)                       << S5_EVT_DET_SHIFT;
	evt |= (action & S5_EVT_MAX(S5_EVT_ACT_BITS))   << S5_EVT_ACT_SHIFT;
	evt |= width_cm                                << S5_EVT_WIDTH_SHIFT;
	evt |= closest_cm                              << S5_EVT_DIST_SHIFT;
	evt |= angle_deg                               << S5_EVT_ANGLE_SHIFT;
	evt |= (noecho ? 1UL : 0UL)                    << S5_EVT_NOECHO_SHIFT;
	return evt;
}

EXPORT ER subsystem5_init(void)
{
	T_CMBF	cmbf = {0};
	T_CFLG	cflg = {0};

	/* IF-06: obstacle events, one 32-bit word each */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = S5_MBF_BUF_SIZE;
	cmbf.maxmsz = S5_EVT_MSG_SIZE;
	mbf_obstacle = tk_cre_mbf(&cmbf);
	if (mbf_obstacle < E_OK) {
		return mbf_obstacle;
	}

	/* IF-07: OBSTACLE_STOP / FAULT_NO_ECHO bits.  TA_WMUL: the Safety
	   handler and the Vehicle Controller may both wait on it. */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0U;
	flg_obstacle = tk_cre_flg(&cflg);
	return (flg_obstacle < E_OK) ? flg_obstacle : E_OK;
}

EXPORT ER subsystem5_start(void)
{
	/* Any core: the busy-wait while timing an echo then only ever holds
	   up lower-priority work on whichever core it runs on */
	ID tid = app_start_task(obstacle_task, S5_PRI_MAIN, 0, "obstacle_task");
	return (tid < E_OK) ? tid : E_OK;
}
