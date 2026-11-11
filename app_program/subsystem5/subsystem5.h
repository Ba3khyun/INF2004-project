/*
 *	subsystem5.h
 *	Subsystem 5 -- Adaptive ultrasonic scanning and obstacle profiling.
 *	Hardware: HC-SR04 ultrasonic sensor mounted on a hobby servo.
 *	Student ID: 2500364 (Buddy 5).
 *
 *	This header is subsystem 5's PUBLIC interface: what the Vehicle
 *	Controller (Buddy 4), the Safety handler (Buddy 4/5) and telemetry
 *	(Buddy 1) may use.  Everything else stays LOCAL in the .c files.
 *
 *	What subsystem 5 does (one task, obstacle_task, see task_obstacle.c):
 *
 *	  MONITORING_AHEAD   servo at 90 deg (straight ahead), one ping every
 *	                     50 ms.  Two pings in a row closer than
 *	                     S5_OBSTACLE_CM = obstacle confirmed.
 *	        |
 *	        v  sets S5_FLG_OBSTACLE_STOP, sends event (action STOP)
 *	  PROFILING          Stage 1 coarse sweep: 45/70/90/110/135 deg.
 *	                     Stage 2 fine scan: 5 deg steps, only between the
 *	                     last "hit" and first "miss" at each edge.
 *	        |
 *	        v
 *	  DECIDING_DIRECTION Stage 3 profile: width, closest point, left and
 *	                     right clearance.  Stage 4 plan: CONTINUE,
 *	                     TURN_LEFT, TURN_RIGHT or REVERSE.  Sends event,
 *	                     clears S5_FLG_OBSTACLE_STOP.
 *	        |
 *	        v
 *	  BYPASSING_OBSTACLE The Vehicle Controller drives the bypass.  Until
 *	                     that exists this state is a bench stand-in: it
 *	                     waits for the path ahead to clear (or a timeout)
 *	                     and goes back to MONITORING_AHEAD.
 *
 *	  SEARCHING_FOR_LINE / RECOVERY_REWIND / REACQUIRING_LINE (Stage 5)
 *	  have codes reserved below.  They need Buddy 2's motion API and
 *	  Buddy 3's LINE_FOUND flag, which do not exist yet.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

#define S5_PRI_MAIN		20	/* priority band 20-21 (TEAM.md) */

/* ---------------------------------------------------------------------
 *  Pins (claimed in TEAM.md section 2)
 *  HC-SR04: VCC = 5 V (VBUS), GND = GND.
 *  ECHO is a 5 V signal: put a 1 kOhm (series) / 2 kOhm (to GND) divider
 *  in front of GP0, or the RP2040 pin can be damaged (assumption A5.1).
 *  GP0/GP1 are UART0 when built with CRASH_REPORT=1 or CONSOLE=uart.
 * --------------------------------------------------------------------- */
#define S5_PIN_US_ECHO		0U	/* GP0  <- HC-SR04 ECHO (via divider) */
#define S5_PIN_US_TRIG		1U	/* GP1  -> HC-SR04 TRIG (3.3 V is OK) */
#define S5_PIN_SERVO		12U	/* GP12 -> servo signal, PWM slice 6 A */

#if !BOARD_UART0_FREE
#warning "Subsystem 5: GP0/GP1 are used by UART0 in this build; the ultrasonic sensor will not work"
#endif

/* ---------------------------------------------------------------------
 *  State codes (3 bits, Statechart 6.5)
 * --------------------------------------------------------------------- */
#define S5_ST_MONITORING	0U	/* MONITORING_AHEAD */
#define S5_ST_PROFILING		1U	/* PROFILING */
#define S5_ST_DECIDING		2U	/* DECIDING_DIRECTION */
#define S5_ST_BYPASSING		3U	/* BYPASSING_OBSTACLE */
#define S5_ST_SEARCHING		4U	/* SEARCHING_FOR_LINE   (reserved) */
#define S5_ST_REWIND		5U	/* RECOVERY_REWIND      (reserved) */
#define S5_ST_REACQUIRING	6U	/* REACQUIRING_LINE     (reserved) */

/* Recommended action (3 bits, Stage 4 of the brief) */
#define S5_ACT_CONTINUE		0U	/* nothing in the car's path */
#define S5_ACT_STOP		1U	/* obstacle confirmed, profiling now */
#define S5_ACT_TURN_LEFT	2U	/* bypass on the left */
#define S5_ACT_TURN_RIGHT	3U	/* bypass on the right */
#define S5_ACT_REVERSE		4U	/* blocked or too close: back off, retry */

/* ---------------------------------------------------------------------
 *  IF-06  S5 -> Vehicle Controller: message buffer mbf_obstacle
 *
 *  Each message is ONE 32-bit word, packed with explicit shifts and
 *  masks.  No C bit-fields: their layout is up to the compiler, and the
 *  team agreed (Tribunal 1, minimum-bit change) to use masks/shifts.
 *
 *   bit 31  30......23  22......14  13.....7  6....4   3   2....0
 *      +---+-----------+-----------+---------+-------+---+-------+
 *      |NE | angle deg | closest cm| width cm| action|det| state |
 *      +---+-----------+-----------+---------+-------+---+-------+
 *        1       8           9          7        3     1     3    = 32
 *
 *   field    range        meaning
 *   state    0..6         S5_ST_xxx when the event was sent
 *   det      0/1          1 = obstacle in the car's path
 *   action   0..4         S5_ACT_xxx, what subsystem 5 recommends
 *   width    0..127 cm    estimated obstacle width (saturates at 127)
 *   closest  0..511 cm    nearest obstacle point (saturates at 511)
 *   angle    0..180 deg   direction of the nearest point:
 *                         0 = right, 90 = straight ahead, 180 = left
 *   NE       0/1          1 = some pings got no echo (data is weaker)
 *
 *  Two events per obstacle: (1) state PROFILING + action STOP as soon as
 *  it is confirmed; (2) state DECIDING + the recommended action.
 *  The sender never blocks.  If the buffer is full the OLDEST event is
 *  dropped, so the receiver always sees the newest decision.
 *  Receive with:  UW evt;  tk_rcv_mbf(mbf_obstacle, &evt, tmout);
 * --------------------------------------------------------------------- */
#define S5_EVT_MSG_SIZE		4U	/* bytes per message */
#define S5_EVT_DEPTH		4U	/* messages the buffer can hold */

#define S5_EVT_STATE_SHIFT	0U
#define S5_EVT_STATE_BITS	3U
#define S5_EVT_DET_SHIFT	3U
#define S5_EVT_DET_BITS		1U
#define S5_EVT_ACT_SHIFT	4U
#define S5_EVT_ACT_BITS		3U
#define S5_EVT_WIDTH_SHIFT	7U
#define S5_EVT_WIDTH_BITS	7U
#define S5_EVT_DIST_SHIFT	14U
#define S5_EVT_DIST_BITS	9U
#define S5_EVT_ANGLE_SHIFT	23U
#define S5_EVT_ANGLE_BITS	8U
#define S5_EVT_NOECHO_SHIFT	31U
#define S5_EVT_NOECHO_BITS	1U

#define S5_EVT_MAX(bits)	((UW)((1UL << (bits)) - 1UL))
#define S5_ANGLE_MAX_DEG	180U

/* Compile-time proof that the fields touch end to end and fill 32 bits */
_Static_assert(sizeof(UW) == S5_EVT_MSG_SIZE, "UW must be 32 bits");
_Static_assert(S5_EVT_DET_SHIFT    == (S5_EVT_STATE_SHIFT + S5_EVT_STATE_BITS), "det overlaps");
_Static_assert(S5_EVT_ACT_SHIFT    == (S5_EVT_DET_SHIFT   + S5_EVT_DET_BITS),   "action overlaps");
_Static_assert(S5_EVT_WIDTH_SHIFT  == (S5_EVT_ACT_SHIFT   + S5_EVT_ACT_BITS),   "width overlaps");
_Static_assert(S5_EVT_DIST_SHIFT   == (S5_EVT_WIDTH_SHIFT + S5_EVT_WIDTH_BITS), "closest overlaps");
_Static_assert(S5_EVT_ANGLE_SHIFT  == (S5_EVT_DIST_SHIFT  + S5_EVT_DIST_BITS),  "angle overlaps");
_Static_assert(S5_EVT_NOECHO_SHIFT == (S5_EVT_ANGLE_SHIFT + S5_EVT_ANGLE_BITS), "noecho overlaps");
_Static_assert((S5_EVT_NOECHO_SHIFT + S5_EVT_NOECHO_BITS) == 32U, "event must be exactly 32 bits");
_Static_assert(S5_ANGLE_MAX_DEG <= S5_EVT_MAX(S5_EVT_ANGLE_BITS), "angle field too small");

/* Build an event word.  Values that do not fit are saturated, not wrapped. */
IMPORT UW s5_evt_pack(UW state, BOOL det, UW action, UW width_cm,
		      UW closest_cm, UW angle_deg, BOOL noecho);

/* Read one field back out of an event word */
Inline UW s5_evt_field(UW evt, UW shift, UW bits)
{
	return ((evt >> shift) & S5_EVT_MAX(bits));
}
Inline UW s5_evt_state(UW evt)   { return s5_evt_field(evt, S5_EVT_STATE_SHIFT,  S5_EVT_STATE_BITS); }
Inline UW s5_evt_det(UW evt)     { return s5_evt_field(evt, S5_EVT_DET_SHIFT,    S5_EVT_DET_BITS); }
Inline UW s5_evt_action(UW evt)  { return s5_evt_field(evt, S5_EVT_ACT_SHIFT,    S5_EVT_ACT_BITS); }
Inline UW s5_evt_width(UW evt)   { return s5_evt_field(evt, S5_EVT_WIDTH_SHIFT,  S5_EVT_WIDTH_BITS); }
Inline UW s5_evt_closest(UW evt) { return s5_evt_field(evt, S5_EVT_DIST_SHIFT,   S5_EVT_DIST_BITS); }
Inline UW s5_evt_angle(UW evt)   { return s5_evt_field(evt, S5_EVT_ANGLE_SHIFT,  S5_EVT_ANGLE_BITS); }
Inline UW s5_evt_noecho(UW evt)  { return s5_evt_field(evt, S5_EVT_NOECHO_SHIFT, S5_EVT_NOECHO_BITS); }

/* ---------------------------------------------------------------------
 *  IF-07  S5 -> Safety handler / Vehicle Controller: event flag
 *  flg_obstacle.  Only subsystem 5 sets or clears these bits, and only
 *  with tk_set_flg / tk_clr_flg, which are atomic (no "mask |= bit"
 *  race, Tribunal 1 decision).  Other subsystems wait on or refer to it.
 * --------------------------------------------------------------------- */
#define S5_FLG_OBSTACLE_STOP	0x00000001U	/* set: obstacle confirmed
						   ahead, stop the car.
						   Cleared by S5 once the
						   decision event is sent. */
#define S5_FLG_FAULT_NO_ECHO	0x00000002U	/* set: sensor silent for
						   S5_FAULT_PINGS pings in a
						   row.  Cleared by S5 on the
						   next good ping. */

IMPORT ID	mbf_obstacle;	/* IF-06 */
IMPORT ID	flg_obstacle;	/* IF-07 */

/* ---------------------------------------------------------------------
 *  Private to subsystem 5, shared between its own .c files
 * --------------------------------------------------------------------- */
IMPORT volatile UW	s5_ping_ok;		/* pings that got an answer */
IMPORT volatile UW	s5_ping_fail;		/* pings with no/stuck echo */
IMPORT volatile UW	s5_profiles_done;	/* completed profile+decision */
IMPORT void		obstacle_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM5_H */
