/*
 *	s5_config.h
 *	Subsystem 5 -- every tuning number in one place.  PRIVATE to S5.
 *
 *	Start with these values on the bench, then change them from your
 *	measurements (TEST_LOG.md S5-M01..M06) and record why.
 */

#ifndef S5_CONFIG_H
#define S5_CONFIG_H

/* ---------------------------------------------------------------------
 *  Ultrasonic (HC-SR04)
 *  Sound travels about 343 m/s, so 1 cm there AND back takes ~58 us.
 * --------------------------------------------------------------------- */
#define S5_US_RANGE_MAX_CM	200U	/* further = "nothing there".  Also
					   caps the busy-wait at ~11.6 ms */
#define S5_US_US_PER_CM		58U	/* echo microseconds per cm */
#define S5_US_TRIG_PULSE_US	10U	/* HC-SR04 needs >= 10 us on TRIG */
#define S5_US_RISE_TMO_US	3000U	/* ECHO must go high within this
					   after TRIG, else: no sensor */
#define S5_US_IDLE_TMO_MS	60U	/* wait this long for ECHO to go low
					   before triggering, else: stuck */
#define S5_PING_GAP_MS		25U	/* between pings, so old echoes from
					   far walls die out first */
#define S5_PINGS_PER_POINT	3U	/* median of N pings per scan angle
					   (1..5).  1 = fastest, 3 = steadier */

/* ---------------------------------------------------------------------
 *  Servo (SG90-class hobby servo, 50 Hz)
 * --------------------------------------------------------------------- */
#define S5_SERVO_MIN_US		500U	/* pulse width at 0 deg   (datasheet) */
#define S5_SERVO_MAX_US		2500U	/* pulse width at 180 deg (datasheet) */
#define S5_SERVO_MS_PER_DEG	2U	/* slew time; SG90 ~1.7 ms/deg at 5 V */
#define S5_SERVO_SETTLE_MS	20U	/* extra wait for the horn to settle */
#define S5_SERVO_MIRROR		0	/* 0: servo 0 deg points to the car's
					   RIGHT.  1: it points LEFT (flip) */

/* ---------------------------------------------------------------------
 *  Monitoring ahead
 * --------------------------------------------------------------------- */
#define S5_MONITOR_PERIOD_MS	50U	/* one forward ping every 50 ms */
#define S5_OBSTACLE_CM		25U	/* closer than this = obstacle */
#define S5_CONFIRM_PINGS	2U	/* in a row, filters single glitches */
#define S5_CLEAR_CM		(S5_OBSTACLE_CM + 10U)	/* hysteresis */
#define S5_CLEAR_PINGS		3U	/* in a row = path is clear again */
#define S5_FAULT_PINGS		5U	/* failed pings in a row = fault */
#define S5_PRINT_EVERY		10U	/* print distance every N pings
					   (10 x 50 ms = 2 lines/s), 0 = off */

/* ---------------------------------------------------------------------
 *  Profiling (coarse angles are listed in task_obstacle.c)
 * --------------------------------------------------------------------- */
#define S5_CENTRE_DEG		90U	/* straight ahead */
#define S5_FINE_STEP_DEG	5U	/* fine-scan resolution */
#define S5_DEPTH_TOL_CM		15U	/* a reading is part of the obstacle
					   if <= first distance + this */
#define S5_WIDTH_CORR_CM	0	/* subtract for beam spread; set from
					   the S5-M05 width test */

/* ---------------------------------------------------------------------
 *  Avoidance planning (signed: used in lateral +/- maths)
 * --------------------------------------------------------------------- */
#define S5_HALF_CAR_CM		((W)8)	/* half the car's width */
#define S5_MARGIN_CM		((W)5)	/* extra gap beside the obstacle */
#define S5_PASS_CLEAR_CM	40U	/* a side is passable if nothing is
					   closer than this beyond the edge */
#define S5_TOO_CLOSE_CM		8U	/* closer: reverse, do not turn */
#define S5_BYPASS_TMO_MS	5000U	/* bench stand-in: give up waiting */

#endif /* S5_CONFIG_H */
