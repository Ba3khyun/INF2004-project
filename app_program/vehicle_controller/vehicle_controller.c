/* Buddy 4 Vehicle Controller - Phase 3: passive RTOS line-intent observer.
 * Reads IMU and safety; receives Buddy 3 line intents. Sends NO motor commands.
 * Does NOT consume mbf_imu_evt or remote_cmd_q; avoids disrupting S4 tests.
 */
#include "vehicle_controller.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem2/subsystem2.h"  /* MC_CMD_* codes only */
#define VC_CORE_USE_TEAM_STATE 1
#include "vehicle_controller_core.h"

#define VC_TASK_PERIOD_MS 50
#define VC_DEBUG_PRINT 1

static vc_core_t vc;
static volatile UW vc_live_state = APP_VS_INITIALIZING;
static volatile UW vc_live_output = VC_OUTPUT_HALT;
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

ER vehicle_controller_init(void)
{
    vc_core_init(&vc);
    vc_live_state = APP_VS_INITIALIZING;
    vc_live_output = VC_OUTPUT_HALT;
    __atomic_store_n(&vc_line_word, 0u, __ATOMIC_RELEASE);
    vc_line_sequence = 0u;
    return E_OK;
}

static void vehicle_controller_task(INT stacd, void *exinf)
{
    SYSTIM otm;
    T_RFLG safety;
    UW last_log_ms = 0U;
    UW previous_state = APP_VS_COUNT;
    UW previous_imu_state = S4_ST_COUNT;
    INT was_ready = 0;
    uint32_t last_line_word = 0u;
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
        if (monitor_ready && state == S4_ST_HUMP_TRACKING &&
            previous_imu_state != S4_ST_HUMP_TRACKING) {
            vc_core_imu(&vc, VC_IMU_HUMP_START, 0U, 0U);
        }
        if (monitor_ready && state == S4_ST_MONITORING &&
            previous_imu_state == S4_ST_HUMP_TRACKING) {
            /* Snapshot does NOT carry measured hump height or validity.
             * Event queue integration comes in a later phase. */
            vc_core_imu(&vc, VC_IMU_HUMP_DONE, 0U, 0U);
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
            }
        }
        output = vc_core_step(&vc, now);
        vc_live_state = (UW)vc.state;
        /* Passive-only: output is a diagnostic, never sent to motor. */
        vc_live_output = (UW)output;

#if VC_DEBUG_PRINT
        if (vc_live_state != previous_state ||
            (time_ok && (UW)(now - last_log_ms) >= 5000U)) {
            tm_printf((UB*)"[VC] state=%u line=%u intent=%u output=%u (PASSIVE; no motor commands)\n",
                      vc_live_state, (UW)vc.line_visible, (UW)vc.intent, vc_live_output);
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
