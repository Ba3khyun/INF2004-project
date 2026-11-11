/*
 *	obstacle_profile.h
 *	Subsystem 5 -- obstacle profile maths and avoidance planning.
 *	PRIVATE to S5.  No hardware access, so it is unit-tested directly
 *	in subsystem5_test.c with made-up scans.
 *
 *	Geometry (seen from above, sensor at the origin):
 *
 *	          180 deg (left)        90 deg (ahead)       0 deg (right)
 *	   x < 0  <--------------------------+--------------------------> x > 0
 *
 *	   A reading of d cm at angle a is at lateral position
 *	       x = d * cos(a)        (cm, + = right of the car's centreline)
 *	   The obstacle's width is (rightmost x) - (leftmost x) of its points.
 */

#ifndef OBSTACLE_PROFILE_H
#define OBSTACLE_PROFILE_H

#include "subsystem5.h"

#define S5_SCAN_MAX_POINTS	24U	/* coarse (5) + fine (<= 2 x 4) fits */

/* One reading of the sweep */
typedef struct {
	UH	cm;		/* distance (S5_US_RANGE_MAX_CM = nothing) */
	UB	angle;		/* logical degrees, 0 = right .. 180 = left */
	UB	valid;		/* TRUE if the sensor answered */
} S5_SCAN_POINT;

/* A whole sweep, kept sorted by angle (index 0 = rightmost) */
typedef struct {
	S5_SCAN_POINT	pt[S5_SCAN_MAX_POINTS];
	UINT		n;
} S5_SCAN;

/* The result of Stage 3 (profile) and Stage 4 (plan) */
typedef struct {
	BOOL	detected;	/* obstacle in the car's path */
	UINT	idx_right;	/* obstacle = pt[idx_right .. idx_left] */
	UINT	idx_left;
	W	x_right_cm;	/* lateral position of its right edge */
	W	x_left_cm;	/* lateral position of its left edge */
	UH	width_cm;
	UH	closest_cm;
	UB	closest_angle;
	BOOL	right_open;	/* free space seen right of the obstacle */
	BOOL	left_open;	/* free space seen left of the obstacle */
	UH	right_clear_cm;	/* nearest thing beyond the right edge */
	UH	left_clear_cm;	/* nearest thing beyond the left edge */
	W	shift_right_cm;	/* sideways move needed to pass right */
	W	shift_left_cm;	/* sideways move needed to pass left */
	UB	action;		/* S5_ACT_xxx */
} S5_PROFILE;

/* Lateral position x = cm * cos(angle), integer only (no FPU on M0+) */
IMPORT W	s5_lateral_cm(UH cm, UINT angle_deg);

IMPORT void	s5_scan_clear(S5_SCAN *scan);

/* Add a reading, keeping the scan sorted; replaces a reading at the same
   angle.  FALSE if the scan is full. */
IMPORT BOOL	s5_scan_insert(S5_SCAN *scan, UINT angle_deg, UH cm, BOOL valid);

/* Is this reading part of the obstacle? */
IMPORT BOOL	s5_is_hit(const S5_SCAN_POINT *p, UH hit_limit_cm);

/* Stage 3: find the obstacle nearest straight ahead and measure it */
IMPORT void	s5_profile_build(const S5_SCAN *scan, UH hit_limit_cm,
				 S5_PROFILE *prof);

/* Stage 4: fill prof->action (and shifts) from a built profile */
IMPORT void	s5_profile_plan(S5_PROFILE *prof);

IMPORT const char *s5_action_name(UINT action);
IMPORT const char *s5_state_name(UINT state);

#endif /* OBSTACLE_PROFILE_H */
