/* Buddy 4 Vehicle Controller - Phase 3: passive RTOS line-intent observer.
 * Reads IMU and safety; receives Buddy 3 line intents. Sends NO motor commands.
 * Does NOT consume mbf_imu_evt or remote_cmd_q; avoids disrupting S4 tests.
 */
#include "vehicle_controller.h"
#include "../app_debug.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem2/subsystem2.h"  /* MC_CMD_* codes only */
#define VC_CORE_USE_TEAM_STATE 1
#include "vehicle_controller_core.h"
#include "vehicle_controller_motion_adapter.h"
#include "../subsystem2/motion_feedback_math.h"

#define VC_TASK_PERIOD_MS 50
#define VC_DEBUG_PRINT APP_DEBUG_VC
/* MISSING GEOMETRY: the real wheel-centre separation must be measured
 * before interpreting live encoder yaw. 0 disables angle conversion. */
#define VC_WHEEL_TRACK_MM 0.0f
#define VC_ENABLE_TRACK_DISTANCE_YAW 0
/* This request-only handoff does NOT send a motor command or change PWM:
 * Buddy 2's live PWM application gate stays OFF. */
#define VC_SEND_SPEED_CAP_REQUEST 1


/* Phase 5A compile-time integration gates. Buddy 5's live obstacle source,
 * the active ESTOP path and Buddy 2 command arbitration are NOT verified.
 * No code in this module submits commands to Buddy 2, regardless of gates. */
#define VC5A_B5_SOURCE_VERIFIED 0
#define VC5A_LIVE_STOP_VERIFIED 0
#define VC5A_MOTOR_HANDOFF_APPROVED 0

static const vma_codes_t vc5a_codes = {
    MC_CMD_DRIVE_FORWARD, MC_CMD_STEER_LEFT, MC_CMD_STEER_RIGHT,
    MC_CMD_DRIVE_LEFT, MC_CMD_DRIVE_RIGHT, MC_CMD_HALT
};

static vc_core_t vc;
static volatile UW vc_live_state = APP_VS_INITIALIZING;
static volatile UW vc_live_output = VC_OUTPUT_HALT;
static uint32_t vc_candidate_command = MC_CMD_HALT;
static uint32_t vc_safe_command = MC_CMD_HALT;
static uint32_t vc_adapter_blockers = VMA_BLOCK_B5_MISSING |
    VMA_BLOCK_STOP_UNVERIFIED | VMA_BLOCK_HANDOFF_PENDING;
/* VC task is the only writer. Readers use 32-bit atomic loads. */
static uint32_t vc_line_updates = 0u;
static uint32_t vc_speed_cap_request = MC_SPEED_CAP_NORMAL;
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

UW vehicle_controller_speed_cap_requested(void)
{ return __atomic_load_n(&vc_speed_cap_request, __ATOMIC_ACQUIRE); }
UW vehicle_controller_wheel_feedback_seq(void)
{ return __atomic_load_n(&vc_wheel_feedback_seq, __ATOMIC_ACQUIRE); }

ER vehicle_controller_init(void)
{
    vc_core_init(&vc);
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
    __atomic_store_n(&vc_speed_cap_request, MC_SPEED_CAP_NORMAL, __ATOMIC_RELEASE);
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
        INT monitor_ready = valid &&
            (state == S4_ST_MONITORING || state == S4_ST_HUMP_TRACKING);
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
        was_ready = monitor_ready;
        previous_imu_state = (UW)state;

        if (!safe_to_read || !time_ok) {
            vc_core_set_safety(&vc, 1, 1); /* fail closed on RTOS error */
        } else {
            UW req = SAFETY_REQ_COLLISION | SAFETY_REQ_OBSTACLE_STOP |
                     SAFETY_REQ_REMOTE_ABORT;
            vc_core_set_safety(&vc, (safety.flgptn & req) != 0U,
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
        output = vc_core_step(&vc, now);
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
        /* 45% hump request; 0 on invalid IMU/safety read; 100 otherwise.
         * Never controls motor PWM with Buddy 2's default compile gate. */
        {
            UW cap = MC_SPEED_CAP_NORMAL;
            if (!monitor_ready || !safe_to_read || !time_ok ||
                (safety.flgptn & (SAFETY_REQ_COLLISION |
                  SAFETY_REQ_OBSTACLE_STOP | SAFETY_REQ_REMOTE_ABORT |
                  SAFETY_ESTOP_LATCHED)) != 0u) cap = 0u;
            else if (vc.hump_active) cap = MC_SPEED_CAP_HUMP;
            __atomic_store_n(&vc_speed_cap_request, cap, __ATOMIC_RELEASE);
#if VC_SEND_SPEED_CAP_REQUEST
            (void)motion_speed_cap_request(cap); /* request only, no PWM writes */
#endif
        }
        vc_live_state = (UW)vc.state;
        /* Passive-only: output is a diagnostic, never sent to motor. */
        vc_live_output = (UW)output;
        {
            vma_inputs_t in = {0};
            vma_result_t p;
            UW flags = safe_to_read ? safety.flgptn : 0u;
            in.safety_read_ok = safe_to_read;
            in.imu_healthy = monitor_ready;
            in.safety_request_active = !safe_to_read ||
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
            __atomic_store_n(&vc_safe_command,
                             p.safe_command, __ATOMIC_RELEASE);
            __atomic_store_n(&vc_adapter_blockers,
                             p.blockers, __ATOMIC_RELEASE);
            /* p.candidate_command and p.safe_command are NEVER posted. */
        }

#if VC_DEBUG_PRINT
        if (vc_live_state != previous_state ||
            (time_ok && (UW)(now - last_log_ms) >= 5000U)) {
            tm_printf((UB*)"[VC] state=%u line=%u intent=%u output=%u (PASSIVE; no motor commands)\n",
                      vc_live_state, (UW)vc.line_visible, (UW)vc.intent, vc_live_output);
            tm_printf((UB*)"[VC-B2] wheel_seq=%u L=%d R=%d age=%ums cap_req=%u%% (PWM CAP OFF)\n",
                      (UINT)__atomic_load_n(&vc_wheel_feedback_seq, __ATOMIC_ACQUIRE),
                      (INT)vc_wheel_left_mm, (INT)vc_wheel_right_mm,
                      (UINT)vc_wheel_feedback_age,
                      (UINT)__atomic_load_n(&vc_speed_cap_request, __ATOMIC_ACQUIRE));
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
