/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion Control (Buddy 2), plus the example Producer
 *	that subsystems 3 and 4 still depend on.  Student ID: <student ID>.
 *
 *	Public interface of subsystem 2.  Other subsystems include it as
 *	"../subsystem2/subsystem2.h".
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

/* ---------------------------------------------------------------------
 *  Motion Control (Buddy 2)
 *
 *  motion_task (task_motion.c) owns PWM slices 4 and 5 (GP8-GP11) and
 *  the encoder inputs GP6/GP7 with IO_IRQ_BANK0.  It runs at PRI_MOTION
 *  on core 1 with a 10 ms control tick.
 *
 *  Input   motion_cmd_q   message buffer, MC_CMD_LEN bytes per command.
 *                         Writer: Vehicle Controller (B4); until it exists,
 *                         any task (e.g. tests) via motion_cmd_post().
 *          flg_safety     SAFETY_ESTOP_LATCHED, read with tk_ref_flg every
 *                         tick and before every start: brakes and aborts
 *                         the current move, rejects new moves while set.
 *  Output  flg_fault      FLT_MOTOR, set on movement or encoder timeout,
 *                         cleared by MC_CMD_CLEAR.
 *          telemetry_q    TQ_FIELD_SPEED every 100 ms (telemetry_post).
 * --------------------------------------------------------------------- */
#define S2_PRI_MOTION		PRI_MOTION	/* 10, IS-06 task table */
#define MC_TICK_MS		10		/* control period */

/* One command = one byte; the codes are the original serial-test keys */
#define MC_CMD_LEN		1
#define MC_CMD_Q_DEPTH		4
#define MC_CMD_FORWARD		((UB)'F')	/* forward 0.50 revolutions */
#define MC_CMD_LEFT		((UB)'L')	/* pivot left 0.25 revolutions */
#define MC_CMD_RIGHT		((UB)'R')	/* pivot right 0.25 wheel revolutions */
#define MC_CMD_STOP		((UB)'X')	/* brake and latch local stop */
#define MC_CMD_CLEAR		((UB)'C')	/* clear local stop and FLT_MOTOR */
#define MC_CMD_DRIVE_FORWARD	((UB)'f')	/* continuous forward until another command */
#define MC_CMD_DRIVE_RIGHT	((UB)'r')	/* continuous right pivot until another command */
#define MC_CMD_DRIVE_LEFT	((UB)'l')	/* continuous left pivot until another command */
#define MC_CMD_STEER_RIGHT	((UB)'q')	/* gentle forward arc to the right */
#define MC_CMD_STEER_LEFT	((UB)'a')	/* gentle forward arc to the left */
#define MC_CMD_HALT		((UB)'H')	/* brake without latching emergency stop */

IMPORT ID	motion_cmd_q;
IMPORT ID	tid_motion;	/* motion_task's ID (read by the on-target tests) */

/* The 10 ms control tick comes from a cyclic handler (cyc_motion) that
   wakes motion_task.  A relative timeout cannot give it: the kernel adds
   one tick to every timeout (kernel/tkernel/timer.c), so a 10 ms wait
   lasts ~20 ms.  Counters written by motion_task only, read by tests. */
IMPORT ID		cyc_motion;
IMPORT volatile UW	motion_tick_count;	/* control ticks run */
IMPORT volatile UW	motion_ticks_missed;	/* wake-ups that queued up */
IMPORT void		motion_cyclic_handler(void *exinf);

/* Queue one command.  Continuous drive commands run until replaced or halted.
   Never blocks; returns E_OK, E_PAR (unknown code) or E_TMOUT (queue full). */
IMPORT ER	motion_cmd_post(UB cmd);
/* TRUE only when the motion task is idle and no command is queued. */
IMPORT BOOL	motion_is_idle(void);

/* Private to subsystem 2 */
IMPORT void	motion_task(INT stacd, void *exinf);

/* ---------------------------------------------------------------------
 *  Example Producer (IF-01/IF-02).  Remove together with the
 *  subsystem 3 and 4 examples, which use SAMPLE_MSG, sem_slots and
 *  mpf_samples.
 * --------------------------------------------------------------------- */
#define S2_PRI_PRODUCER		14	/* priority band 14-15 (TEAM.md) */
#define S2_PERIOD_MS		300	/* one sample every 300 ms */

/* ---------------------------------------------------------------------
 *  IF-01 data format: one sample, sent to subsystem 3's mailbox.
 *  A mailbox message MUST start with a T_MSG header: the kernel links
 *  messages together through it, so the data itself is never copied.
 * --------------------------------------------------------------------- */
typedef struct {
	T_MSG	hdr;		/* used by the kernel, do not touch */
	UW	seq;		/* sample sequence number */
	UW	time_ms;	/* system time when the sample was taken */
	INT	value;		/* the "measurement", 0..100 */
	INT	prc;		/* processor (core) that produced it: 1 or 2 */
} SAMPLE_MSG;

#define S2_POOL_COUNT	4	/* at most 4 samples in flight */

/* IF-02: the receiver of a sample gives its memory block back to
   mpf_samples (tk_rel_mpf) and then a slot back to sem_slots
   (tk_sig_sem, count 1).  Nothing else may use these two objects. */
IMPORT ID	sem_slots;
IMPORT ID	mpf_samples;

/* Private to subsystem 2 */
IMPORT void	producer_task(INT stacd, void *exinf);
IMPORT INT	s2_make_value(UW seq);

#endif /* SUBSYSTEM2_H */
