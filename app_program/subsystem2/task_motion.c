/*
 *	task_motion.c
 *	Subsystem 2 -- Motion Control task (Buddy 2).
 *
 *	Ported from buddy2_motion_control/buddy2_motion_control.c.  The
 *	state machine (IDLE / FORWARD / LEFT / RIGHT / EMERGENCY_STOP), the
 *	targets, the PWM duty and both timeouts are unchanged.  What changed:
 *	  original                          here
 *	  main() superloop                  motion_task, PRI_MOTION, core 1
 *	  getchar_timeout_us() F/L/R/X/C    motion_cmd_q (same letters)
 *	  time_us_64() + sleep_ms(1)        10 ms cyclic handler cyc_motion
 *	                                    wakes the task; tk_get_otm()
 *	  printf                            tm_printf (no %f: milli-revs)
 *	  stop when USB monitor unplugged   dropped (commands no longer come
 *	                                    from the monitor)
 *	  --                                SAFETY_ESTOP_LATCHED, FLT_MOTOR,
 *	                                    TQ_FIELD_SPEED (team interfaces)
 *
 *	This task is the ONLY caller of the motion_control motor/encoder API.
 *
 *	APIs shown: tk_rcv_mbf (with timeout), tk_ref_flg, tk_set_flg,
 *	            tk_clr_flg, tk_get_otm
 */

#include "subsystem2.h"
#include "../app_debug.h"
#include "motion_control.h"
#include "motion_feedback_math.h"
#include "../subsystem1/subsystem1.h" /* IF-B1-01: telemetry_post */

/* ========================
 * Configuration
 * ======================== */
#define MOVE_TIMEOUT_MS 4000u
#define ENCODER_TIMEOUT_MS 250u

#define FORWARD_REVOLUTIONS 0.50f
#define TURN_REVOLUTIONS 0.25f

#define MOVE_PWM 20.0f
#define TURN_PWM 15.0f
#define STEER_RIGHT_INNER_PWM 12.0f

/* SAFETY GATE: speed requests are computed, stored and validated, but DO NOT
 * alter motor PWM while live ESTOP and command arbitration are unverified.
 * Do not enable this gate just to get tests passing. */
#define B2_ENABLE_LIVE_SPEED_CAP_PWM 0


/* Speed telemetry (TQ_FIELD_SPEED, cm/s).  WHEEL_DIAMETER_CM is a
   placeholder: measure the wheel and correct it. */
#define TELEM_PERIOD_MS 100u
#define WHEEL_DIAMETER_CM 6.2f // changed from 6.5f to 6.2f to match the actual wheel diameter

/* ========================
 * Motion states
 * ======================== */
typedef enum
{
	MOTION_IDLE,
	MOTION_FORWARD,
	MOTION_ARC_RIGHT,
	MOTION_ARC_LEFT,
	MOTION_LEFT,
	MOTION_RIGHT,
	MOTION_EMERGENCY_STOP
} MotionState;

/* All state below is private to motion_task: no locking needed. */
LOCAL volatile MotionState motion_state = MOTION_IDLE;
LOCAL BOOL continuous_motion = FALSE;

LOCAL BOOL left_done = FALSE;
LOCAL BOOL right_done = FALSE;

LOCAL UW start_left = 0u;
LOCAL UW start_right = 0u;

LOCAL UW previous_left = 0u;
LOCAL UW previous_right = 0u;

LOCAL UW start_time = 0u; /* ms, tk_get_otm */
LOCAL UW last_left_pulse = 0u;
LOCAL UW last_right_pulse = 0u;

LOCAL float target_revolutions = 0.0f;

LOCAL UW tlm_time = 0u;
LOCAL UW tlm_left = 0u;
LOCAL UW tlm_right = 0u;

/* Cumulative encoder distance for Buddy 4's IMU */
LOCAL volatile UW g_motion_distance_mm = 0U;
LOCAL float g_motion_distance_total = 0.0f;
LOCAL UW g_distance_prev_left = 0U;
LOCAL UW g_distance_prev_right = 0U;

/* Motion task is the sole writer of these four aligned published words.
 * The double-read of even sequence creates a consistent cross-core snapshot.
 * The signs are inferred from commanded motion state (single A-channel IRQ).
 */
LOCAL volatile UW g_wheel_feedback_seq = 0u;
LOCAL volatile INT g_wheel_left_mm = 0;
LOCAL volatile INT g_wheel_right_mm = 0;
LOCAL volatile UW g_wheel_sample_ms = 0u;
LOCAL volatile UW g_wheel_direction_inferred = 0u;
LOCAL float g_wheel_left_total_mm = 0.0f;
LOCAL float g_wheel_right_total_mm = 0.0f;
/* Speed cap setter never calls motor routines: only motor task may apply it. */
LOCAL volatile UW g_requested_speed_cap_pct = MC_SPEED_CAP_NORMAL;

EXPORT ER motion_speed_cap_request(UW percent)
{
    if (percent > 100u) return E_PAR;
    __atomic_store_n(&g_requested_speed_cap_pct, percent, __ATOMIC_RELEASE);
    return E_OK;
}
EXPORT UW motion_speed_cap_requested(void)
{
    return __atomic_load_n(&g_requested_speed_cap_pct, __ATOMIC_ACQUIRE);
}
EXPORT ER motion_get_wheel_feedback(motion_wheel_feedback_t *out)
{
    UW i, a, b;
    if (!out) return E_PAR;
    for (i = 0u; i < 6u; ++i) {
        a = __atomic_load_n(&g_wheel_feedback_seq, __ATOMIC_SEQ_CST);
        if ((a == 0u) || (a & 1u)) continue;
        out->left_signed_mm = __atomic_load_n(&g_wheel_left_mm, __ATOMIC_SEQ_CST);
        out->right_signed_mm = __atomic_load_n(&g_wheel_right_mm, __ATOMIC_SEQ_CST);
        out->sample_ms = __atomic_load_n(&g_wheel_sample_ms, __ATOMIC_SEQ_CST);
        out->direction_inferred = __atomic_load_n(&g_wheel_direction_inferred, __ATOMIC_SEQ_CST);
        b = __atomic_load_n(&g_wheel_feedback_seq, __ATOMIC_SEQ_CST);
        if ((a == b) && ((b & 1u) == 0u)) {
            out->sequence = b >> 1;
            return E_OK;
        }
    }
    return E_TMOUT;
}
LOCAL INT round_signed_mm(float x)
{
    return (INT)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}
LOCAL void publish_wheel_feedback(UW stamp, UW inferred)
{
    UW next = __atomic_load_n(&g_wheel_feedback_seq, __ATOMIC_RELAXED) + 1u;
    __atomic_store_n(&g_wheel_feedback_seq, next, __ATOMIC_SEQ_CST); /* odd */
    __atomic_store_n(&g_wheel_left_mm, round_signed_mm(g_wheel_left_total_mm), __ATOMIC_SEQ_CST);
    __atomic_store_n(&g_wheel_right_mm, round_signed_mm(g_wheel_right_total_mm), __ATOMIC_SEQ_CST);
    __atomic_store_n(&g_wheel_sample_ms, stamp, __ATOMIC_SEQ_CST);
    __atomic_store_n(&g_wheel_direction_inferred, inferred, __ATOMIC_SEQ_CST);
    __atomic_store_n(&g_wheel_feedback_seq, next + 1u, __ATOMIC_SEQ_CST); /* even */
}


/* Read-only interface for Subsystem 4 */
EXPORT UW motion_get_distance_mm(void)
{
    return g_motion_distance_mm;
}

/* Monotonic milliseconds; differences are wrap-safe as UW. */
LOCAL UW now_ms(void)
{
	SYSTIM t;

	tk_get_otm(&t);
	return t.lo;
}

/*
 * Provisional encoder-distance bridge.
 * Motion task is the only writer.
 * IMU task reads the aligned 32-bit published distance.
 *
 * Assumes wheel diameter of 6.5 cm and the encoder
 * pulses/revolution configured in motion_control.c.
 */
LOCAL void update_encoder_distance(void)
{
    UW left_now;
    UW right_now;
    UW left_delta;
    UW right_delta;
    float left_mm;
    float right_mm;

    left_now = encoder_get_left_count();
    right_now = encoder_get_right_count();

    left_delta = left_now - g_distance_prev_left;
    right_delta = right_now - g_distance_prev_right;

    g_distance_prev_left = left_now;
    g_distance_prev_right = right_now;

    left_mm = encoder_left_distance_cm(left_delta, WHEEL_DIAMETER_CM) * 10.0f;
    right_mm = encoder_right_distance_cm(right_delta, WHEEL_DIAMETER_CM) * 10.0f;

    /* Cumulative forward-path distance remains backward compatible with IMU. */
    if (motion_state == MOTION_FORWARD || motion_state == MOTION_ARC_LEFT ||
        motion_state == MOTION_ARC_RIGHT) {
        g_motion_distance_total += (left_mm + right_mm) * 0.5f;
        g_motion_distance_mm = (UW)(g_motion_distance_total + 0.5f);
        g_wheel_left_total_mm += left_mm;
        g_wheel_right_total_mm += right_mm;
        publish_wheel_feedback(now_ms(), 1u);
    } else if (motion_state == MOTION_LEFT) {
        /* Model left pivot: negative left wheel, positive right wheel. */
        g_wheel_left_total_mm -= left_mm;
        g_wheel_right_total_mm += right_mm;
        publish_wheel_feedback(now_ms(), 1u);
    } else if (motion_state == MOTION_RIGHT) {
        g_wheel_left_total_mm += left_mm;
        g_wheel_right_total_mm -= right_mm;
        publish_wheel_feedback(now_ms(), 1u);
    } else {
        /* Stationary: publish heartbeat without inventing wheel direction. */
        publish_wheel_feedback(now_ms(), 0u);
    }
}


LOCAL INT milli_rev(float rev)
{
	return (INT)(rev * 1000.0f + 0.5f);
}

LOCAL BOOL is_moving(void)
{
	return motion_state == MOTION_FORWARD ||
		   motion_state == MOTION_ARC_RIGHT ||
		   motion_state == MOTION_ARC_LEFT ||
		   motion_state == MOTION_LEFT ||
		   motion_state == MOTION_RIGHT;
}

/* ========================
 * Emergency stop (local latch)
 * ======================== */

/* fault: TRUE for a motor/turn timeout, which is reported as FLT_MOTOR */
LOCAL void emergency_stop(const char *reason, BOOL fault)
{
	/* Stop immediately and latch emergency state. */
	motors_brake();
	motion_state = MOTION_EMERGENCY_STOP;
	continuous_motion = FALSE;

	left_done = TRUE;
	right_done = TRUE;

	if (fault)
	{
		CHECK(tk_set_flg(flg_fault, FLT_MOTOR), "tk_set_flg FLT_MOTOR");
	}

	tm_printf((UB *)"\nmotion: EMERGENCY STOP: %s\n", reason);
	tm_printf((UB *)"motion: motors stopped. Send C to clear.\n");
}

LOCAL void clear_emergency_stop(void)
{
	if (motion_state != MOTION_EMERGENCY_STOP)
	{
		tm_printf((UB *)"motion: no emergency stop to clear.\n");
		return;
	}

	motors_brake();
	motion_state = MOTION_IDLE;
	continuous_motion = FALSE;
	CHECK(tk_clr_flg(flg_fault, ~(UINT)FLT_MOTOR), "tk_clr_flg FLT_MOTOR");

	tm_printf((UB *)"\nmotion: emergency stop cleared.\n");
	tm_printf((UB *)"motion: motion commands can be accepted again.\n");
}

/* ========================
 * Safety ESTOP (owned by the Safety Handler, read only here)
 * ======================== */

LOCAL BOOL safety_estop_latched(void)
{
	T_RFLG rflg;

	if (tk_ref_flg(flg_safety, &rflg) < E_OK)
	{
		return TRUE; /* cannot tell: fail safe */
	}
	return (rflg.flgptn & SAFETY_ESTOP_LATCHED) != 0u;
}

LOCAL void check_safety(void)
{
	if (is_moving() && safety_estop_latched())
	{
		motors_brake();
		motion_state = MOTION_IDLE;
		continuous_motion = FALSE;
		left_done = TRUE;
		right_done = TRUE;
		tm_printf((UB *)"\nmotion: aborted, safety ESTOP latched.\n");
	}
}

/* ========================
 * Start movement
 * ======================== */

/* Only the motion-owning task may translate cap requests into physical PWM.
 * With B2_ENABLE_LIVE_SPEED_CAP_PWM=0 this returns Buddy 2's original PWM. */
LOCAL float capped_pwm(float base)
{
#if B2_ENABLE_LIVE_SPEED_CAP_PWM
    return b2_cap_pwm(base, motion_speed_cap_requested());
#else
    return base;
#endif
}
#if B2_ENABLE_LIVE_SPEED_CAP_PWM
LOCAL void apply_active_speed_limit(void)
{
    float drive = capped_pwm(MOVE_PWM);
    float steer = capped_pwm(STEER_RIGHT_INNER_PWM);
    float pivot = capped_pwm(TURN_PWM);
    if (!continuous_motion || !is_moving() || safety_estop_latched()) return;
    switch (motion_state) {
    case MOTION_FORWARD: motor1_forward(drive); motor2_reverse(drive); break;
    case MOTION_ARC_LEFT: motor1_forward(steer); motor2_reverse(drive); break;
    case MOTION_ARC_RIGHT: motor1_forward(drive); motor2_reverse(steer); break;
    case MOTION_LEFT: motors_left(pivot); break;
    case MOTION_RIGHT: motors_right(pivot); break;
    default: break;
    }
}
#endif

LOCAL void start_motion(MotionState command, BOOL continuous)
{
	if (safety_estop_latched())
	{
		tm_printf((UB *)"motion: movement rejected: safety ESTOP latched.\n");
		return;
	}

	if (motion_state == MOTION_EMERGENCY_STOP)
	{
		tm_printf((UB *)"motion: movement rejected: emergency stop active.\n");
		return;
	}

	if (motion_state != MOTION_IDLE && !continuous_motion)
	{
		tm_printf((UB *)"motion: movement rejected: robot is busy.\n");
		return;
	}

	motors_brake();

	start_left = encoder_get_left_count();
	start_right = encoder_get_right_count();

	previous_left = start_left;
	previous_right = start_right;

	start_time = now_ms();
	last_left_pulse = start_time;
	last_right_pulse = start_time;

	left_done = FALSE;
	right_done = FALSE;
	continuous_motion = continuous;

	motion_state = command;

	if (command == MOTION_FORWARD ||
		command == MOTION_ARC_RIGHT ||
		command == MOTION_ARC_LEFT)
	{
		target_revolutions = continuous ? 0.0f : FORWARD_REVOLUTIONS;
		if (command == MOTION_ARC_RIGHT)
		{
			tm_printf((UB *)"\nmotion: starting gentle right arc...\n");
			motor1_forward(capped_pwm(MOVE_PWM));
			motor2_reverse(capped_pwm(STEER_RIGHT_INNER_PWM));
		}
		else if (command == MOTION_ARC_LEFT)
		{
			tm_printf((UB *)"\nmotion: starting gentle left arc...\n");
			motor1_forward(capped_pwm(STEER_RIGHT_INNER_PWM));
			motor2_reverse(capped_pwm(MOVE_PWM));
		}
		else
		{
			tm_printf((UB *)"\nmotion: starting %sFORWARD movement...\n",
					  continuous ? "continuous " : "");
			motor1_forward(capped_pwm(MOVE_PWM));
			motor2_reverse(capped_pwm(MOVE_PWM));
		}
	}
	else if (command == MOTION_LEFT)
	{
		target_revolutions = continuous ? 0.0f : TURN_REVOLUTIONS;
		tm_printf((UB *)"\nmotion: starting %sLEFT turn...\n",
				  continuous ? "continuous " : "");
		motors_left(capped_pwm(TURN_PWM));
	}
	else if (command == MOTION_RIGHT)
	{
		target_revolutions = continuous ? 0.0f : TURN_REVOLUTIONS;
		tm_printf((UB *)"\nmotion: starting %sRIGHT turn...\n",
				  continuous ? "continuous " : "");
		motors_right(capped_pwm(TURN_PWM));
	}
}

/* ========================
 * Motion update (every MC_TICK_MS)
 * ======================== */

LOCAL void update_motion(UW now)
{
	if (!is_moving())
	{
		return;
	}

	UW current_left = encoder_get_left_count();
	UW current_right = encoder_get_right_count();

	UW left_pulses = current_left - start_left;
	UW right_pulses = current_right - start_right;

	if (current_left != previous_left)
	{
		last_left_pulse = now;
	}

	if (current_right != previous_right)
	{
		last_right_pulse = now;
	}

	float left_revolutions = encoder_left_revolutions(left_pulses);
	float right_revolutions = encoder_right_revolutions(right_pulses);

	/* Discrete moves stop each wheel at its target; continuous moves do not. */
	if (!continuous_motion && !left_done && left_revolutions >= target_revolutions)
	{
		motor1_forward(0.0f);
		left_done = TRUE;
	}

	if (!continuous_motion && !right_done && right_revolutions >= target_revolutions)
	{
		motor2_reverse(0.0f);
		right_done = TRUE;
	}

	/* Both wheels reached the target. */
	if (!continuous_motion && left_done && right_done)
	{
		motors_brake();
		motion_state = MOTION_IDLE;

		tm_printf((UB *)"\nmotion: movement completed.\n");
		tm_printf((UB *)"motion: left  revolutions: %d/1000\n",
				  milli_rev(left_revolutions));
		tm_printf((UB *)"motion: right revolutions: %d/1000\n",
				  milli_rev(right_revolutions));
		tm_printf((UB *)"motion: motors stopped.\n");
		return;
	}

	/* Prevent movement from running indefinitely. */
	if (!continuous_motion && now - start_time >= MOVE_TIMEOUT_MS)
	{
		emergency_stop("Movement timeout", TRUE);
		return;
	}

	/* Stop when a moving wheel loses encoder pulses. */
	if ((!left_done && now - last_left_pulse >= ENCODER_TIMEOUT_MS) ||
		(!right_done && now - last_right_pulse >= ENCODER_TIMEOUT_MS))
	{
		emergency_stop("Encoder timeout", TRUE);
		return;
	}

	previous_left = current_left;
	previous_right = current_right;
}

/* ========================
 * Speed telemetry (IF-B1-01)
 * ======================== */

LOCAL void post_speed(UW now)
{
	UW dt = now - tlm_time;
	UW l, r;
	INT speed = 0;

	if (dt < TELEM_PERIOD_MS)
	{
		return;
	}
	l = encoder_get_left_count();
	r = encoder_get_right_count();

	/* Linear speed only while driving forward; a pivot is 0 cm/s. */
	if (motion_state == MOTION_FORWARD ||
		motion_state == MOTION_ARC_RIGHT ||
		motion_state == MOTION_ARC_LEFT)
	{
		float cm = (encoder_left_distance_cm(l - tlm_left, WHEEL_DIAMETER_CM) +
					encoder_right_distance_cm(r - tlm_right, WHEEL_DIAMETER_CM)) /
				   2.0f;
		speed = (INT)((cm * 1000.0f) / (float)dt + 0.5f);
		if (speed > APP_SPEED_MAX)
		{
			speed = APP_SPEED_MAX;
		}
	}
	(void)telemetry_post(TQ_FIELD_SPEED, speed); /* never blocks */

	tlm_time = now;
	tlm_left = l;
	tlm_right = r;
}

/* ========================
 * Commands
 * ======================== */

LOCAL void process_command(UB cmd)
{
	switch (cmd)
	{
	case MC_CMD_STOP:
		emergency_stop("Stop command", FALSE);
		break;
	case MC_CMD_CLEAR:
		clear_emergency_stop();
		break;
	case MC_CMD_FORWARD:
		start_motion(MOTION_FORWARD, FALSE);
		break;
	case MC_CMD_LEFT:
		start_motion(MOTION_LEFT, FALSE);
		break;
	case MC_CMD_RIGHT:
		start_motion(MOTION_RIGHT, FALSE);
		break;
	case MC_CMD_DRIVE_FORWARD:
		start_motion(MOTION_FORWARD, TRUE);
		break;
	case MC_CMD_DRIVE_RIGHT:
		start_motion(MOTION_RIGHT, TRUE);
		break;
	case MC_CMD_DRIVE_LEFT:
		start_motion(MOTION_LEFT, TRUE);
		break;
	case MC_CMD_STEER_RIGHT:
		start_motion(MOTION_ARC_RIGHT, TRUE);
		break;
	case MC_CMD_STEER_LEFT:
		start_motion(MOTION_ARC_LEFT, TRUE);
		break;
	case MC_CMD_HALT:
		if (motion_state != MOTION_EMERGENCY_STOP)
		{
			motors_brake();
			motion_state = MOTION_IDLE;
			continuous_motion = FALSE;
			left_done = TRUE;
			right_done = TRUE;
			tm_printf((UB *)"\nmotion: halted by line-follow controller.\n");
		}
		break;
	default:
		break; /* motion_cmd_post() rejects these */
	}
}

/* ========================
 * Task
 * ======================== */

EXPORT volatile UW motion_tick_count;
EXPORT volatile UW motion_ticks_missed;

EXPORT BOOL motion_is_idle(void)
{
	T_RMBF status;

	if (motion_state != MOTION_IDLE ||
		tk_ref_mbf(motion_cmd_q, &status) != E_OK)
	{
		return FALSE;
	}

	return status.frbufsz >= (SZ)(MC_CMD_Q_DEPTH * APP_MBF_SLOT(MC_CMD_LEN));
}

/* cyc_motion, every MC_TICK_MS (subsystem2.c) */
EXPORT void motion_cyclic_handler(void *exinf)
{
	(void)exinf;
	(void)tk_wup_tsk(tid_motion);
}

EXPORT void motion_task(INT stacd, void *exinf)
{
	UB cmd;
	INT len;
	UW now;
	ER er;
	T_RTSK self;

	(void)stacd;
	(void)exinf;

	/* Hardware init happens here so it runs on this task's core: the
	   encoder interrupt is enabled on the core that calls it. */
	er = motors_init();
	if (er >= E_OK)
	{
		er = encoders_init();
	}
	if (er < E_OK)
	{
		motors_brake();
		CHECK(er, "motion_task hardware init");
		tk_ext_tsk();
	}
	
	g_distance_prev_left = encoder_get_left_count();
	g_distance_prev_right = encoder_get_right_count();
	g_motion_distance_mm = 0U;
	g_motion_distance_total = 0.0f;
    g_wheel_left_total_mm = 0.0f;
    g_wheel_right_total_mm = 0.0f;
    __atomic_store_n(&g_requested_speed_cap_pct, MC_SPEED_CAP_NORMAL, __ATOMIC_RELEASE);
    publish_wheel_feedback(now_ms(), 0u);


	tm_printf((UB *)"motion: Buddy 2 Motion Control ready (motion_cmd_q)\n");
	tm_printf((UB *)"motion:   F = forward, L = left, R = right,"
					" X = stop, C = clear stop\n");

	tlm_time = now_ms();

	/* One iteration per 10 ms tick.  The previous loop waited with
	   tk_rcv_mbf(..., next_update - now); the kernel adds one tick to
	   every timeout, so that "10 ms" tick really ran every ~20 ms. */
	while (1)
	{
		(void)tk_slp_tsk(TMO_FEVR); /* woken by cyc_motion */
		if (tk_ref_tsk(TSK_SELF, &self) == E_OK && self.wupcnt > 0)
		{
			motion_ticks_missed += (UW)self.wupcnt; /* overrun */
			(void)tk_can_wup(TSK_SELF);
		}

		/* Update cumulative distance from the previous control period. */
		update_encoder_distance();

		/* Check safety before processing queued movement commands. */
		check_safety();

		/* Commands first (at most the queue depth), never waiting */
		while ((len = tk_rcv_mbf(motion_cmd_q, &cmd, TMO_POL)) == MC_CMD_LEN)
		{

#if APP_DEBUG_B2_COMMAND
			/* Optional queue trace: NOT needed for normal motion execution. */
		tm_printf((UB *)"[BUDDY 2] Received command: %c\r\n", cmd);
#endif

			process_command(cmd);
			check_safety();
		}
		if (len != E_TMOUT)
		{
			CHECK(len, "tk_rcv_mbf motion_cmd_q");
		}

		now = now_ms();
		check_safety();
#if B2_ENABLE_LIVE_SPEED_CAP_PWM
        /* Reapply latest bounded request at the existing 10ms motion tick.
         * Only for continuous moves; never reenergize completed discrete wheels. */
        apply_active_speed_limit();
#endif
		update_motion(now);
		post_speed(now);
		motion_tick_count++;
	}
}
