#include "vehicle_controller.h"
#include "../app_debug.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem2/subsystem2.h"  /* motor commands, wheel feedback and PWM scale */
#define VC_CORE_USE_TEAM_STATE 1
#include "vehicle_controller_core.h"
#include "vehicle_controller_motion_adapter.h"
#include "../subsystem2/motion_feedback_math.h"
#include "../subsystem1/subsystem1.h"  /* telemetry_post + command encoding */
#include "../subsystem5/subsystem5.h"  /* obstacle message/flag definitions */
#include "b4_input_bridge.h"
#include "hump_drive_config.h"

#define VC_TASK_PERIOD_MS 50
#define VC_MAX_IMU_AGE_MS 200u
/* Existing B1/B5 self-tests consume their own queues. Don't steal test
 * messages in APP_TEST builds; exercise the pure bridge model instead. */
#define VC_CONSUME_REAL_INPUTS (!APP_TEST)
#define VC_DEBUG_PRINT APP_DEBUG_VC
/* MISSING GEOMETRY: the real wheel-centre separation must be measured
 * before interpreting live encoder yaw. 0 disables angle conversion. */
#define VC_WHEEL_TRACK_MM 0.0f
#define VC_ENABLE_TRACK_DISTANCE_YAW 0
/* Buddy 2's motion task is the only code that applies this PWM scale. */
#define VC_SEND_SPEED_SCALE_REQUEST 1

/* Powered handoff: Buddy 2 latches its own EMERGENCY_STOP (FLT_MOTOR) on an
 * encoder or movement timeout, and only MC_CMD_CLEAR releases it.  The VC
 * holds the car, waits, clears and retries a bounded number of times, then
 * stays stopped (ERROR) until reboot.  The count resets after a quiet run. */
#define VC_MOTOR_RETRY_WAIT_MS   1000u
#define VC_MOTOR_RETRY_MAX       3u
#define VC_MOTOR_RETRY_RESET_MS  10000u
/* Powered handoff: an IMU I2C fault (e.g. a glitch when the motors start)
 * is retried: the VC asks Buddy 4's IMU task to reinitialise and
 * recalibrate.  The car stays halted (IMU not ready) until it is back.
 * A pickup/overturn end-of-demo is never retried. */
#define VC_IMU_REINIT_PERIOD_MS  2000u


static const vma_codes_t vc5a_codes = {
    MC_CMD_DRIVE_FORWARD, MC_CMD_STEER_LEFT, MC_CMD_STEER_RIGHT,
    MC_CMD_DRIVE_LEFT, MC_CMD_DRIVE_RIGHT, MC_CMD_HALT
};

static vc_core_t vc;
static b4_input_bridge_t g_bridge;
static UW g_last_pickup_event;
static volatile UW vc_live_state = APP_VS_INITIALIZING;
static volatile UW vc_live_output = VC_OUTPUT_HALT;
static uint32_t vc_candidate_command = MC_CMD_HALT;
static uint32_t vc_safe_command = MC_CMD_HALT;
static uint32_t vc_adapter_blockers = VMA_BLOCK_B5_MISSING |
    VMA_BLOCK_STOP_UNVERIFIED | VMA_BLOCK_HANDOFF_PENDING;
/* VC task is the only writer. Readers use 32-bit atomic loads. */
static uint32_t vc_line_updates = 0u;
static uint32_t vc_speed_scale_request = MC_SPEED_SCALE_NORMAL;
static uint32_t vc_wheel_feedback_seq = 0u;
static int32_t vc_wheel_left_mm = 0;
static int32_t vc_wheel_right_mm = 0;
static uint32_t vc_wheel_feedback_age = UINT32_MAX;

/* Set by the VC task. UINT32_MAX means no line intent was ever received. */
static uint32_t vc_line_age_ms = UINT32_MAX;
/* Single-producer (Buddy 3), single-consumer (VC): latest-wins 32-bit mailbox.
 * bits 0..7 = intent; bit 8 = physical line visible; bits 9..31 = sequence.
 * 32-bit atomic load/store avoid torn reads on the SMP kernel. */
static uint32_t vc_line_word;
static uint32_t vc_line_sequence;

ER vehicle_controller_line_motion_post(UB command, INT line_visible)
{
    vc_line_intent_t intent;
    switch (command) {
    case MC_CMD_DRIVE_FORWARD: intent = VC_INTENT_FORWARD; break;
    case MC_CMD_STEER_LEFT:    intent = VC_INTENT_STEER_LEFT; break;
    case MC_CMD_STEER_RIGHT:   intent = VC_INTENT_STEER_RIGHT; break;
    case MC_CMD_DRIVE_LEFT:   intent = VC_INTENT_PIVOT_LEFT; break;
    case MC_CMD_DRIVE_RIGHT:  intent = VC_INTENT_PIVOT_RIGHT; break;
    case MC_CMD_HALT:         intent = VC_INTENT_STOP; break;
    default: return E_PAR;
    }
    /* Only Buddy 3 calls this: single writer to sequence and mailbox. */
    uint32_t seq = (++vc_line_sequence) & 0x7FFFFFu;
    uint32_t word = (seq << 9) | ((line_visible != 0) ? 0x100u : 0u) | (uint32_t)intent;
    __atomic_store_n(&vc_line_word, word, __ATOMIC_RELEASE);
    return E_OK;
}

UW vehicle_controller_state(void) { return vc_live_state; }
UW vehicle_controller_output(void) { return vc_live_output; }
UW vehicle_controller_candidate_motion_command(void)
{ return __atomic_load_n(&vc_candidate_command, __ATOMIC_ACQUIRE); }
UW vehicle_controller_safe_motion_command(void)
{ return __atomic_load_n(&vc_safe_command, __ATOMIC_ACQUIRE); }
UW vehicle_controller_motion_blockers(void)
{ return __atomic_load_n(&vc_adapter_blockers, __ATOMIC_ACQUIRE); }
UW vehicle_controller_line_update_count(void)
{ return __atomic_load_n(&vc_line_updates, __ATOMIC_ACQUIRE); }
/* Number of Buddy 3 publications, modulo the 23-bit mailbox sequence range.
 * A publication count can advance even if the VC task skips intermediate
 * messages: the mailbox deliberately implements latest-wins semantics. */
UW vehicle_controller_line_published_count(void)
{
    return (__atomic_load_n(&vc_line_word, __ATOMIC_ACQUIRE) >> 9);
}
/* Last intent age computed on the VC task's 50 ms cycle; UINT32_MAX means
 * no valid input has arrived. It is NOT an independent sensor health check. */
UW vehicle_controller_line_age_ms(void)
{ return __atomic_load_n(&vc_line_age_ms, __ATOMIC_ACQUIRE); }

UW vehicle_controller_speed_scale_requested(void)
{ return __atomic_load_n(&vc_speed_scale_request, __ATOMIC_ACQUIRE); }
UW vehicle_controller_wheel_feedback_seq(void)
{ return __atomic_load_n(&vc_wheel_feedback_seq, __ATOMIC_ACQUIRE); }
UW vehicle_controller_obstacle_hold(void) { return g_bridge.obstacle_hold; }
UW vehicle_controller_obstacle_action(void) { return g_bridge.obstacle_action; }
UW vehicle_controller_remote_consumed(void) { return g_bridge.remote_count; }
UW vehicle_controller_obstacle_consumed(void) { return g_bridge.obstacle_count; }
UW vehicle_controller_pickup_terminal(void) { return g_bridge.pickup_terminal; }

static const char *vc_state_name(UW s)
{
    switch (s) {
    case APP_VS_INITIALIZING:     return "INITIALIZING";
    case APP_VS_DRIVING:          return "DRIVING";
    case APP_VS_HUMP_CROSSING:    return "HUMP_CROSSING";
    case APP_VS_OBSTACLE_BYPASS:  return "OBSTACLE";
    case APP_VS_LINE_SEARCH:      return "LINE_SEARCH";
    case APP_VS_PAUSED:           return "PAUSED";
    case APP_VS_ERROR:            return "ERROR";
    case APP_VS_MISSION_COMPLETE: return "MISSION_COMPLETE";
    default:                      return "?";
    }
}

/* One line, printed only when the state or a hold reason changes, so the
 * console always says why the car is (not) moving. */
static void vc_print_status(UW state, UW why)
{
    tm_printf((UB*)"[VC] state=%s%s%s%s%s%s%s%s%s\n", vc_state_name(state),
              why ? "  hold:" : "",
              (why & 0x01u) ? " IMU-not-ready" : "",
              (why & 0x02u) ? " safety-request" : "",
              (why & 0x04u) ? " ESTOP" : "",
              (why & 0x08u) ? " obstacle" : "",
              (why & 0x10u) ? " obstacle-sensor-fault" : "",
              (why & 0x20u) ? " remote-pause/abort" : "",
              (why & 0x40u) ? " motor-fault" : "");
}

ER vehicle_controller_init(void)
{
    vc_core_init(&vc);
    b4_input_bridge_init(&g_bridge);
    g_last_pickup_event = s4_imu_pickup_event_count();
    vc_live_state = APP_VS_INITIALIZING;
    vc_live_output = VC_OUTPUT_HALT;
    __atomic_store_n(&vc_line_word, 0u, __ATOMIC_RELEASE);
    vc_line_sequence = 0u;
    __atomic_store_n(&vc_candidate_command, MC_CMD_HALT, __ATOMIC_RELEASE);
    __atomic_store_n(&vc_safe_command, MC_CMD_HALT, __ATOMIC_RELEASE);
    __atomic_store_n(&vc_adapter_blockers,
        VMA_BLOCK_B5_MISSING | VMA_BLOCK_STOP_UNVERIFIED |
        VMA_BLOCK_HANDOFF_PENDING, __ATOMIC_RELEASE);
    __atomic_store_n(&vc_line_updates, 0u, __ATOMIC_RELEASE);
    __atomic_store_n(&vc_line_age_ms, UINT32_MAX, __ATOMIC_RELEASE);
    __atomic_store_n(&vc_speed_scale_request, MC_SPEED_SCALE_NORMAL, __ATOMIC_RELEASE);
    __atomic_store_n(&vc_wheel_feedback_seq, 0u, __ATOMIC_RELEASE);
    return E_OK;
}

static void vehicle_controller_task(INT stacd, void *exinf)
{
    SYSTIM otm;
    T_RFLG safety;
#if VC_DEBUG_PRINT
    UW last_log_ms = 0U;
    UW previous_state = APP_VS_COUNT;
#endif
    UW previous_imu_state = S4_ST_COUNT;
#if !APP_TEST
    UW last_state_telemetry_ms = 0U;
    UW last_sent_state = APP_VS_COUNT;
#endif
#if VC_POWERED_HANDOFF
    UB last_posted = 0u;
    INT motor_fault_seen = 0;   /* FLT_MOTOR currently set */
    INT motor_failed = 0;       /* retries used up: stay stopped */
    UINT motor_retries = 0u;
    UW motor_fault_ms = 0u;     /* fault seen / last clear sent */
    UW motor_clear_ms = 0u;
    UW imu_reinit_ms = 0u;
    INT imu_reinit_pending = 0;
#endif
    INT vc_motor_hold = 0;      /* motor fault holds the car (ERROR) */
    UW last_status_state = APP_VS_COUNT;
    UW last_status_why = 0xFFFFFFFFu;
    INT was_ready = 0;
    uint32_t last_line_word = 0u;
    /* Independent hump event taps: no imu_evt_q receives here. */
    UW last_hump_start_count = s4_imu_hump_start_count();
    UW last_hump_done_word = s4_imu_hump_done_word();
    (void)stacd;
    (void)exinf;

    for (;;) {
        UW snap = s4_imu_get_snapshot();
        s4_imu_state_t state = s4_snap_state(snap);
        INT valid = (INT)s4_snap_valid(snap);
        INT safe_to_read = (tk_ref_flg(flg_safety, &safety) == E_OK);
        INT time_ok = (tk_get_otm(&otm) == E_OK);
        UW now = time_ok ? (UW)otm.lo : 0U;
        /* SHOCK_DETECTED is a 200 ms observation of a minor bump (e.g. a
         * hump landing), not a sensor fault: keep driving through it.
         * Collisions/pickups arrive separately via flg_safety. */
        INT monitor_ready = valid &&
            s4_imu_good_sample_age_ms() <= VC_MAX_IMU_AGE_MS &&
            (state == S4_ST_MONITORING || state == S4_ST_HUMP_TRACKING ||
             state == S4_ST_SHOCK_DETECTED);
        vc_output_t output;

        if (monitor_ready && !was_ready) {
            vc_core_imu(&vc, VC_IMU_READY, 0U, 0U);
        } else if (!monitor_ready && was_ready && state != S4_ST_FAULT) {
            /* Snapshot invalidated: fail closed until a fresh ready snapshot. */
            vc_core_imu(&vc, VC_IMU_FAULT, 0U, 0U);
        }
        if (state == S4_ST_FAULT && previous_imu_state != S4_ST_FAULT) {
            vc_core_imu(&vc, VC_IMU_FAULT, 0U, 0U);
        }
        {
            UW starts = s4_imu_hump_start_count();
            UW done = s4_imu_hump_done_word();
            INT got_start = (starts != last_hump_start_count);
            INT got_done = (done != 0U && done != last_hump_done_word);
            if (!monitor_ready) {
                /* A stuck I2C task must not deliver old hump events. */
                last_hump_start_count = starts;
                last_hump_done_word = done;
                got_start = 0;
                got_done = 0;
            }
            if (got_start) {
                vc_core_imu(&vc, VC_IMU_HUMP_START, 0U, 0U);
                last_hump_start_count = starts;
#if VC_DEBUG_PRINT
                tm_printf((UB*)"[VC-TERRAIN] HUMP_START observed (read-only)\n");
#endif
            } else if (monitor_ready && state == S4_ST_HUMP_TRACKING &&
                       previous_imu_state != S4_ST_HUMP_TRACKING) {
                /* State-transition fallback when the tap is not yet seen. */
                vc_core_imu(&vc, VC_IMU_HUMP_START, 0U, 0U);
            }
            if (got_done) {
                uint16_t evt = (uint16_t)(done & 0xFFFFU);
                uint8_t valid = (uint8_t)s4_evt_valid(evt);
                uint8_t height = (uint8_t)s4_evt_height_mm(evt);
                if (s4_evt_type(evt) == S4_EVT_HUMP_DONE) {
                    vc_core_imu(&vc, VC_IMU_HUMP_DONE, valid, height);
#if !APP_TEST
                    /* Report valid height only; 63 is telemetry N/A. */
                    (void)telemetry_post(TQ_FIELD_HUMP,
                        valid && height <= APP_HUMP_MM_MAX ? (INT)height :
                        (INT)APP_HUMP_MM_NA);
#endif
#if VC_DEBUG_PRINT
                    tm_printf((UB*)"[VC-TERRAIN] HUMP_DONE valid=%u peak=%u mm (NO MOTOR COMMAND)\n",
                              (UINT)valid, (UINT)height);
#endif
                }
                last_hump_done_word = done;
            } else if (monitor_ready && state == S4_ST_MONITORING &&
                       previous_imu_state == S4_ST_HUMP_TRACKING) {
                /* Fallback: don't claim a valid height without HUMP_DONE. */
                vc_core_imu(&vc, VC_IMU_HUMP_DONE, 0U, 0U);
            }
        }
#if VC_POWERED_HANDOFF
        if (state == S4_ST_FAULT && s4_imu_pickup_event_count() == 0u && time_ok) {
            if (!imu_reinit_pending ||
                (UW)(now - imu_reinit_ms) >= VC_IMU_REINIT_PERIOD_MS) {
                if (s4_imu_request_reinit() == E_OK) {
                    if (!imu_reinit_pending)
                        tm_printf((UB*)"[VC] IMU fault: requesting reinit (car halted)\n");
                    imu_reinit_pending = 1;
                    imu_reinit_ms = now;
                }
            }
        } else if (state != S4_ST_FAULT) {
            imu_reinit_pending = 0;
        }
#endif
        was_ready = monitor_ready;
        previous_imu_state = (UW)state;

        /* End of demo: irreversible software terminal state until reboot.
         * Physical ESTOP assertion remains behind the team's separate gate. */
        {
            UW pickup = s4_imu_pickup_event_count();
            if (pickup != g_last_pickup_event && pickup != 0u) {
                b4_input_bridge_pickup(&g_bridge);
                vc_core_complete(&vc);
                g_last_pickup_event = pickup;
            }
        }
#if VC_CONSUME_REAL_INPUTS
        /* B1 accepted commands, bounded to 4 dequeues per 50 ms cycle. */
        {
            unsigned i;
            for (i = 0u; i < REMOTE_CMD_Q_DEPTH; i++) {
                UB msg[RCMD_MSG_LEN];
                APP_CMD cmd;
                INT n = tk_rcv_mbf(remote_cmd_q, msg, TMO_POL);
                if (n == E_TMOUT) break;
                if (n == (INT)RCMD_MSG_LEN && app_rcmd_unpack(msg, &cmd) == APP_OK)
                    b4_input_bridge_remote(&g_bridge, cmd.type);
                else if (n < E_OK) break;
            }
        }
        /* Buddy 5's stop flag can clear before bypass. Preserve an
         * independent hold until a later clear event and fresh line. */
        {
            unsigned i;
            T_RFLG obstacle_flags;
            ER er = tk_ref_flg(flg_obstacle, &obstacle_flags);
            if (er == E_OK) {
                b4_input_bridge_obstacle_flag(&g_bridge,
                    (obstacle_flags.flgptn & S5_FLG_OBSTACLE_STOP) != 0u,
                    (obstacle_flags.flgptn & S5_FLG_FAULT_NO_ECHO) != 0u);
            } else b4_input_bridge_obstacle_flag(&g_bridge, 1, 1);
            for (i = 0u; i < S5_EVT_DEPTH; i++) {
                UW word = 0u;
                INT n = tk_rcv_mbf(mbf_obstacle, &word, TMO_POL);
                if (n == E_TMOUT) break;
                if (n == (INT)S5_EVT_MSG_SIZE) {
                    b4_input_bridge_obstacle(&g_bridge, s5_evt_state(word),
                                             s5_evt_det(word), s5_evt_action(word));
                } else if (n < E_OK) break;
            }
        }
#endif
        /* A stale line cannot release an obstacle hold. */
        b4_input_bridge_refresh(&g_bridge,
            vc.line_received && vc.line_visible && time_ok &&
            ((UW)(now - vc.last_line_ms) <= VC_LINE_TIMEOUT_MS));
        if (g_bridge.paused) vc_core_pause(&vc);
        else vc_core_resume(&vc); /* doesn't clear any safety latch */

#if VC_POWERED_HANDOFF
        /* Buddy 2 motor fault (encoder/movement timeout): hold, wait,
         * clear, retry; give up after VC_MOTOR_RETRY_MAX attempts. */
        {
            T_RFLG ff;
            INT fault = (tk_ref_flg(flg_fault, &ff) == E_OK) &&
                        ((ff.flgptn & FLT_MOTOR) != 0u);
            if (fault) {
                if (!motor_fault_seen) {
                    motor_fault_seen = 1;
                    motor_fault_ms = now;
                    tm_printf((UB*)"[VC] Buddy 2 motor fault (check wheels/encoders)\n");
                } else if (!motor_failed &&
                           (UW)(now - motor_fault_ms) >= VC_MOTOR_RETRY_WAIT_MS) {
                    if (motor_retries >= VC_MOTOR_RETRY_MAX) {
                        motor_failed = 1;
                        tm_printf((UB*)"[VC] motor fault persists after %u retries: "
                                  "stopped until reboot\n", (UINT)motor_retries);
                    } else if (motion_cmd_post(MC_CMD_CLEAR) == E_OK) {
                        motor_retries++;
                        motor_fault_ms = now;
                        motor_clear_ms = now;
                        tm_printf((UB*)"[VC] clearing motor fault, retry %u/%u\n",
                                  (UINT)motor_retries, (UINT)VC_MOTOR_RETRY_MAX);
                    }
                }
            } else {
                motor_fault_seen = 0;
                if (motor_retries != 0u && !motor_failed &&
                    (UW)(now - motor_clear_ms) >= VC_MOTOR_RETRY_RESET_MS)
                    motor_retries = 0u;
            }
            vc_motor_hold = motor_fault_seen || motor_failed;
        }
#endif
        if (!safe_to_read || !time_ok) {
            vc_core_set_safety(&vc, 1, 1); /* fail closed on RTOS error */
        } else {
            UW req = SAFETY_REQ_COLLISION | SAFETY_REQ_OBSTACLE_STOP |
                     SAFETY_REQ_REMOTE_ABORT;
            vc_core_set_safety(&vc,
                              ((safety.flgptn & req) != 0U) ||
                              b4_input_bridge_halt(&g_bridge) ||
                              vc_motor_hold,
                              (safety.flgptn & SAFETY_ESTOP_LATCHED) != 0U);
        }

        /* Consume latest intention (never consume subsystem 4 event queue). */
        {
            uint32_t line_word = __atomic_load_n(&vc_line_word, __ATOMIC_ACQUIRE);
            if (line_word != last_line_word) {
                vc_line_intent_t line_intent =
                    (vc_line_intent_t)(line_word & 0xFFu);
                if (line_word != 0u && line_intent <= VC_INTENT_STOP) {
                    vc_core_line(&vc, line_intent, now);
                    vc_core_line_visible(&vc, (line_word & 0x100u) != 0u);
                }
                last_line_word = line_word;
                /* Single VC-task writer; avoid libatomic __atomic_fetch_add_4
                 * (unavailable in our Cortex-M0+ link configuration). */
                uint32_t n = __atomic_load_n(&vc_line_updates, __ATOMIC_RELAXED);
                __atomic_store_n(&vc_line_updates, n + 1u, __ATOMIC_RELEASE);
            }
        }
        __atomic_store_n(&vc_line_age_ms,
            vc.line_received && time_ok ? (uint32_t)(now - vc.last_line_ms)
                                       : UINT32_MAX,
            __ATOMIC_RELEASE);
        vc.travel_mm = motion_get_distance_mm();
        output = vc_core_step(&vc, now);
#if !APP_TEST
        if (time_ok && (last_sent_state != (UW)vc.state ||
                       (UW)(now - last_state_telemetry_ms) >= 1000u)) {
            (void)telemetry_post(TQ_FIELD_VEH_STATE, (INT)vc.state);
            last_sent_state = (UW)vc.state;
            last_state_telemetry_ms = now;
        }
#endif
        /* New Buddy 2 interface: read signed wheel progress only.  The
         * inferred sign is based on commanded motion, not quadrature B.
         * No turn angle is trusted until wheel track has been measured. */
        {
            motion_wheel_feedback_t wheel;
            if (motion_get_wheel_feedback(&wheel) == E_OK) {
                __atomic_store_n(&vc_wheel_feedback_seq, wheel.sequence, __ATOMIC_RELEASE);
                vc_wheel_left_mm = wheel.left_signed_mm;
                vc_wheel_right_mm = wheel.right_signed_mm;
                vc_wheel_feedback_age = (UW)(now - wheel.sample_ms);
#if VC_ENABLE_TRACK_DISTANCE_YAW
                /* Float expression guarded by a physically measured constant. */
                (void)b2_estimated_turn_deg((float)wheel.left_signed_mm,
                    (float)wheel.right_signed_mm, VC_WHEEL_TRACK_MM);
#endif
            }
        }
        /* 125% PWM scale on a detected hump; 0 on invalid safety/IMU state. */
        {
            UW scale = MC_SPEED_SCALE_NORMAL;
            if (!monitor_ready || !safe_to_read || !time_ok ||
                b4_input_bridge_halt(&g_bridge) ||
                (safety.flgptn & (SAFETY_REQ_COLLISION |
                  SAFETY_REQ_OBSTACLE_STOP | SAFETY_REQ_REMOTE_ABORT |
                  SAFETY_ESTOP_LATCHED)) != 0U) scale = MC_SPEED_SCALE_STOP;
            else if (vc.hump_active) scale = MC_SPEED_SCALE_HUMP;
#if VC_POWERED_HANDOFF
            if (output == VC_OUTPUT_HALT) scale = MC_SPEED_SCALE_STOP;
#endif
            __atomic_store_n(&vc_speed_scale_request, scale, __ATOMIC_RELEASE);
#if VC_SEND_SPEED_SCALE_REQUEST
            (void)motion_speed_scale_request(scale);
#endif
        }
        vc_live_state = (UW)vc.state;
        /* Publish the decision in both test and powered builds. */
        vc_live_output = (UW)output;
        {
            vma_inputs_t in = {0};
            vma_result_t p;
            UW flags = safe_to_read ? safety.flgptn : 0u;
            in.safety_read_ok = safe_to_read;
            in.imu_healthy = monitor_ready;
            in.safety_request_active = b4_input_bridge_halt(&g_bridge) || !safe_to_read ||
                ((flags & (SAFETY_REQ_COLLISION |
                    SAFETY_REQ_OBSTACLE_STOP |
                    SAFETY_REQ_REMOTE_ABORT)) != 0u);
            in.estop_latched = !safe_to_read ||
                ((flags & SAFETY_ESTOP_LATCHED) != 0u);
            in.obstacle_source_verified = VC5A_B5_SOURCE_VERIFIED;
            in.live_estop_verified = VC5A_LIVE_STOP_VERIFIED;
            in.motor_handoff_approved = VC5A_MOTOR_HANDOFF_APPROVED;
            p = vma_preview(output, &in, &vc5a_codes);
            __atomic_store_n(&vc_candidate_command,
                             p.candidate_command, __ATOMIC_RELEASE);
#if VC_POWERED_HANDOFF
            p.safe_command = p.model_ready ? p.candidate_command : MC_CMD_HALT;
#endif
            __atomic_store_n(&vc_safe_command,
                             p.safe_command, __ATOMIC_RELEASE);
            __atomic_store_n(&vc_adapter_blockers,
                             p.blockers, __ATOMIC_RELEASE);
#if VC_POWERED_HANDOFF
            /* The VC is the only motion_cmd_q writer.  Post on change, and
             * re-post a drive command that Buddy 2 is no longer executing
             * (it went idle with nothing queued).  motion_is_idle() is FALSE
             * while ESTOP is latched, so a refused move is not spammed. */
            UB command = p.safe_command;
            if (command != MC_CMD_HALT && command == last_posted &&
                motion_is_idle())
                last_posted = 0u;
            if (command != last_posted && motion_cmd_post(command) == E_OK)
                last_posted = command;
#endif
        }
        {
            UW why = 0u;
            UW flags = safe_to_read ? safety.flgptn : 0u;
            if (!monitor_ready) why |= 0x01u;
            if (!safe_to_read || (flags & (SAFETY_REQ_COLLISION |
                SAFETY_REQ_OBSTACLE_STOP | SAFETY_REQ_REMOTE_ABORT)) != 0u)
                why |= 0x02u;
            if ((flags & SAFETY_ESTOP_LATCHED) != 0u) why |= 0x04u;
            if (g_bridge.obstacle_hold) why |= 0x08u;
            if (g_bridge.obstacle_sensor_fault) why |= 0x10u;
            if (g_bridge.paused || g_bridge.remote_abort_latched) why |= 0x20u;
            if (vc_motor_hold) why |= 0x40u;
            if ((UW)vc.state != last_status_state || why != last_status_why) {
                vc_print_status((UW)vc.state, why);
                last_status_state = (UW)vc.state;
                last_status_why = why;
            }
        }

#if VC_DEBUG_PRINT
        if (vc_live_state != previous_state ||
            (time_ok && (UW)(now - last_log_ms) >= 5000U)) {
            tm_printf((UB*)"[VC] state=%u line=%u intent=%u output=%u powered=%u\n",
                      vc_live_state, (UW)vc.line_visible, (UW)vc.intent, vc_live_output, (UINT)VC_POWERED_HANDOFF);
            tm_printf((UB*)"[VC-B2] wheel_seq=%u L=%d R=%d age=%ums scale_req=%u%%\n",
                      (UINT)__atomic_load_n(&vc_wheel_feedback_seq, __ATOMIC_ACQUIRE),
                      (INT)vc_wheel_left_mm, (INT)vc_wheel_right_mm,
                      (UINT)vc_wheel_feedback_age,
                      (UINT)__atomic_load_n(&vc_speed_scale_request, __ATOMIC_ACQUIRE));
            previous_state = vc_live_state;
            last_log_ms = now;
        }
#endif
        tk_dly_tsk(VC_TASK_PERIOD_MS);
    }
}

ER vehicle_controller_start(void)
{
    ID tid = app_start_task(vehicle_controller_task, PRI_VEHICLE_CTRL,
                            0, "vehicle_controller_task");
    return (tid < E_OK) ? (ER)tid : E_OK;
}
