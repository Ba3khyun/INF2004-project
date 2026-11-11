/*
 *	obstacle_profile.c
 *	Subsystem 5 -- Stage 3 (profile) and Stage 4 (avoidance planning).
 *
 *	Pure calculations on a finished sweep: no pins, no waiting, no kernel
 *	calls.  That keeps them fast and lets subsystem5_test.c check them
 *	with made-up sweeps.
 *
 *	All maths is integer: the RP2040's Cortex-M0+ has no floating-point
 *	unit, and every scan angle is a multiple of 5 deg, so a 19-entry
 *	cosine table is exact enough (error < 0.5 %).
 */

#include "obstacle_profile.h"
#include "s5_config.h"

#define COS_STEP_DEG	5U
#define COS_SCALE	1000

/* cos(a) * 1000 for a = 0, 5, 10, ... 90 deg */
LOCAL const UH cos_1000[] = {
	1000U, 996U, 985U, 966U, 940U, 906U, 866U, 819U, 766U, 707U,
	643U, 574U, 500U, 423U, 342U, 259U, 174U, 87U, 0U
};

EXPORT W s5_lateral_cm(UH cm, UINT angle_deg)
{
	UINT	a = (angle_deg > S5_ANGLE_MAX_DEG) ? S5_ANGLE_MAX_DEG : angle_deg;
	W	c;

	/* cos(180 - a) = -cos(a): only 0..90 is stored */
	if (a <= S5_CENTRE_DEG) {
		c = (W)cos_1000[(a + (COS_STEP_DEG / 2U)) / COS_STEP_DEG];
	} else {
		c = -(W)cos_1000[((S5_ANGLE_MAX_DEG - a) + (COS_STEP_DEG / 2U))
				 / COS_STEP_DEG];
	}
	return (((W)cm * c) / COS_SCALE);
}

EXPORT void s5_scan_clear(S5_SCAN *scan)
{
	scan->n = 0U;
}

EXPORT BOOL s5_scan_insert(S5_SCAN *scan, UINT angle_deg, UH cm, BOOL valid)
{
	UINT	i;
	UINT	j;

	/* Same angle measured again: replace it */
	for (i = 0U; i < scan->n; i++) {
		if (scan->pt[i].angle == (UB)angle_deg) {
			scan->pt[i].cm    = cm;
			scan->pt[i].valid = (UB)valid;
			return TRUE;
		}
	}
	if (scan->n >= S5_SCAN_MAX_POINTS) {
		return FALSE;
	}

	/* Find the slot that keeps the angles ascending, shift the rest up */
	i = 0U;
	while ((i < scan->n) && (scan->pt[i].angle < (UB)angle_deg)) {
		i++;
	}
	for (j = scan->n; j > i; j--) {
		scan->pt[j] = scan->pt[j - 1U];
	}
	scan->pt[i].angle = (UB)angle_deg;
	scan->pt[i].cm    = cm;
	scan->pt[i].valid = (UB)valid;
	scan->n++;
	return TRUE;
}

EXPORT BOOL s5_is_hit(const S5_SCAN_POINT *p, UH hit_limit_cm)
{
	return ((p->valid != 0U) && (p->cm <= hit_limit_cm));
}

/*
 * Free space beside the obstacle: the nearest valid reading in
 * pt[first .. last].  No valid reading there means we cannot see past
 * the edge (it reaches the end of the sweep), so that side is closed.
 */
LOCAL void side_clearance(const S5_SCAN *scan, INT first, INT last,
			  BOOL *p_open, UH *p_clear)
{
	INT	i;

	*p_open  = FALSE;
	*p_clear = 0U;
	for (i = first; i <= last; i++) {
		if (scan->pt[i].valid != 0U) {
			if ((!*p_open) || (scan->pt[i].cm < *p_clear)) {
				*p_clear = scan->pt[i].cm;
			}
			*p_open = TRUE;
		}
	}
}

EXPORT void s5_profile_build(const S5_SCAN *scan, UH hit_limit_cm,
			     S5_PROFILE *prof)
{
	UINT	i;
	UINT	seed = 0U;
	UINT	off;
	UINT	best_off = S5_ANGLE_MAX_DEG + 1U;
	BOOL	found = FALSE;
	W	x;
	W	width;

	*prof = (S5_PROFILE){0};
	prof->action = S5_ACT_CONTINUE;

	/* 1. Seed: the hit nearest to straight ahead.  That is the obstacle
	      that stopped us; other objects further round are ignored. */
	for (i = 0U; i < scan->n; i++) {
		if (s5_is_hit(&scan->pt[i], hit_limit_cm)) {
			off = (scan->pt[i].angle > S5_CENTRE_DEG)
				? (scan->pt[i].angle - S5_CENTRE_DEG)
				: (S5_CENTRE_DEG - scan->pt[i].angle);
			if (off < best_off) {
				best_off = off;
				seed = i;
				found = TRUE;
			}
		}
	}
	if (!found) {
		return;			/* nothing there: CONTINUE */
	}
	prof->detected = TRUE;

	/* 2. Grow to both sides while the neighbours are hits too */
	prof->idx_right = seed;
	while ((prof->idx_right > 0U)
	       && s5_is_hit(&scan->pt[prof->idx_right - 1U], hit_limit_cm)) {
		prof->idx_right--;
	}
	prof->idx_left = seed;
	while (((prof->idx_left + 1U) < scan->n)
	       && s5_is_hit(&scan->pt[prof->idx_left + 1U], hit_limit_cm)) {
		prof->idx_left++;
	}

	/* 3. Closest point and lateral extent of the obstacle */
	prof->closest_cm    = scan->pt[seed].cm;
	prof->closest_angle = scan->pt[seed].angle;
	prof->x_right_cm    = s5_lateral_cm(scan->pt[seed].cm, scan->pt[seed].angle);
	prof->x_left_cm     = prof->x_right_cm;
	for (i = prof->idx_right; i <= prof->idx_left; i++) {
		x = s5_lateral_cm(scan->pt[i].cm, scan->pt[i].angle);
		if (x > prof->x_right_cm) {
			prof->x_right_cm = x;
		}
		if (x < prof->x_left_cm) {
			prof->x_left_cm = x;
		}
		if (scan->pt[i].cm < prof->closest_cm) {
			prof->closest_cm    = scan->pt[i].cm;
			prof->closest_angle = scan->pt[i].angle;
		}
	}

	/* 4. Width, minus the beam-spread correction (calibrate: S5-M05) */
	width = (prof->x_right_cm - prof->x_left_cm) - (W)S5_WIDTH_CORR_CM;
	prof->width_cm = (width > 0) ? (UH)width : 0U;

	/* 5. Clearance beyond each edge (index 0 side = right) */
	side_clearance(scan, 0, (INT)prof->idx_right - 1,
		       &prof->right_open, &prof->right_clear_cm);
	side_clearance(scan, (INT)prof->idx_left + 1, (INT)scan->n - 1,
		       &prof->left_open, &prof->left_clear_cm);
}

EXPORT void s5_profile_plan(S5_PROFILE *prof)
{
	/* Car centre must end up this far beyond the obstacle's edge */
	W	need = S5_HALF_CAR_CM + S5_MARGIN_CM;
	BOOL	left_ok;
	BOOL	right_ok;

	if (!prof->detected) {
		prof->action = S5_ACT_CONTINUE;
		return;
	}

	/* Sideways distance to move so the car just clears each edge.
	   Example: left edge at x = -10 -> move 13 - (-10) = 23 cm left. */
	prof->shift_left_cm  = need - prof->x_left_cm;
	prof->shift_right_cm = need + prof->x_right_cm;

	/* Entirely beside our path (needs no shift on one side)? Ignore it. */
	if ((prof->shift_left_cm <= 0) || (prof->shift_right_cm <= 0)) {
		prof->detected = FALSE;
		prof->action = S5_ACT_CONTINUE;
		return;
	}

	/* Too close to turn without clipping it: back off and re-profile */
	if (prof->closest_cm < S5_TOO_CLOSE_CM) {
		prof->action = S5_ACT_REVERSE;
		return;
	}

	left_ok  = prof->left_open  && (prof->left_clear_cm  >= S5_PASS_CLEAR_CM);
	right_ok = prof->right_open && (prof->right_clear_cm >= S5_PASS_CLEAR_CM);

	if (left_ok && right_ok) {
		/* Both possible: the smaller detour keeps us nearest the line
		   (requirement 8: minimise deviation).  Tie -> left. */
		prof->action = (prof->shift_left_cm <= prof->shift_right_cm)
				? S5_ACT_TURN_LEFT : S5_ACT_TURN_RIGHT;
	} else if (left_ok) {
		prof->action = S5_ACT_TURN_LEFT;
	} else if (right_ok) {
		prof->action = S5_ACT_TURN_RIGHT;
	} else {
		prof->action = S5_ACT_REVERSE;	/* boxed in: reverse, retry */
	}
}

EXPORT const char *s5_action_name(UINT action)
{
	switch (action) {
	case S5_ACT_CONTINUE:	return "CONTINUE";
	case S5_ACT_STOP:	return "STOP";
	case S5_ACT_TURN_LEFT:	return "TURN_LEFT";
	case S5_ACT_TURN_RIGHT:	return "TURN_RIGHT";
	case S5_ACT_REVERSE:	return "REVERSE";
	default:		return "?";
	}
}

EXPORT const char *s5_state_name(UINT state)
{
	switch (state) {
	case S5_ST_MONITORING:	return "MONITORING_AHEAD";
	case S5_ST_PROFILING:	return "PROFILING";
	case S5_ST_DECIDING:	return "DECIDING_DIRECTION";
	case S5_ST_BYPASSING:	return "BYPASSING_OBSTACLE";
	case S5_ST_SEARCHING:	return "SEARCHING_FOR_LINE";
	case S5_ST_REWIND:	return "RECOVERY_REWIND";
	case S5_ST_REACQUIRING:	return "REACQUIRING_LINE";
	default:		return "?";
	}
}
