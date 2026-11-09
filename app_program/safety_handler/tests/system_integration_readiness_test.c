/* Buddy 4 consolidated integration readiness diagnostics.
 * All probes are READ-ONLY. No motor commands, queue receives, flag writes,
 * safety resets, sensor simulation into live tasks, or fault injections.
 * Passing does NOT authorize power to motors or autonomous operation.
 */
#include "../../app.h"
#include "../../app_debug.h"
#include "../../subsystem2/subsystem2.h"
#include "../../subsystem4/subsystem4.h"
#include "../../vehicle_controller/vehicle_controller.h"
#define VC_CORE_USE_TEAM_STATE 1
#include "../../vehicle_controller/vehicle_controller_motion_adapter.h"
#include "../safety_handler.h"
#include "system_integration_readiness_test.h"

#define SYS_CHECK(what, condition) do { \
    if (!(condition)) { \
        tm_printf((UB*)"[B4-SYS] FAIL: %s\n", what); \
        ++failures; \
    } \
} while (0)

static void print_blocker_names(UW b)
{
    if (b & VMA_BLOCK_SAFETY_READ) tm_printf((UB*)"[B4-SYS] gate: safety flag read failed\n");
    if (b & VMA_BLOCK_IMU) tm_printf((UB*)"[B4-SYS] gate: IMU not ready\n");
    if (b & VMA_BLOCK_SAFETY_REQ) tm_printf((UB*)"[B4-SYS] gate: safety request pending\n");
    if (b & VMA_BLOCK_ESTOP) tm_printf((UB*)"[B4-SYS] gate: shared ESTOP latched\n");
    if (b & VMA_BLOCK_B5_MISSING) tm_printf((UB*)"[B4-SYS] gate: Buddy 5 obstacle source unverified\n");
    if (b & VMA_BLOCK_STOP_UNVERIFIED) tm_printf((UB*)"[B4-SYS] gate: active ESTOP unverified\n");
    if (b & VMA_BLOCK_HANDOFF_PENDING) tm_printf((UB*)"[B4-SYS] gate: Buddy 2 motor handoff unapproved\n");
    if (b & VMA_BLOCK_BAD_OUTPUT) tm_printf((UB*)"[B4-SYS] gate: invalid VC output\n");
    if (b & VMA_BLOCK_BAD_INPUT) tm_printf((UB*)"[B4-SYS] gate: invalid VC adapter input\n");
}

/* Only read task state; never wake, suspend, resume or reset it. */
static void print_imu_task_state(const char *when)
{
    T_RTSK ts = {0};
    ID tid = s4_diag_task_id();
    ER er = (tid > 0) ? tk_ref_tsk(tid, &ts) : E_ID;
    tm_printf((UB*)"[B4-IMU-TASK] %s id=%d ref=%d state=0x%x pri=%d "
                     "wait=0x%x wid=%d sus=%d stage=%u age=%ums "
                     "wakes=%u other_wait_errors=%u\n",
              when, (INT)tid, (INT)er,
              (UINT)((er == E_OK) ? ts.tskstat : 0U),
              (INT)((er == E_OK) ? ts.tskpri : 0),
              (UINT)((er == E_OK) ? ts.tskwait : 0U),
              (INT)((er == E_OK) ? ts.wid : 0),
              (INT)((er == E_OK) ? ts.suscnt : 0),
              (UINT)s4_diag_stage(), (UINT)s4_diag_stage_age_ms(),
              (UINT)s4_diag_wakeups(), (UINT)s4_diag_other_wait_errors());
              tm_printf((UB*)
                "[B4-FLAG-ID] flg_imu=%d flg_events=%d "
                "flg_safety=%d flg_fault=%d task_id=%d\n",
                (INT)flg_imu,
                (INT)flg_events,
                (INT)flg_safety,
                (INT)flg_fault,
                (INT)tid);
}

int system_integration_readiness_test_run(void)
{
    const UW missing_gates = VMA_BLOCK_B5_MISSING |
        VMA_BLOCK_STOP_UNVERIFIED | VMA_BLOCK_HANDOFF_PENDING;
    UW pub0, pub1, obs0, obs1, obs_after_settle, imu0, imu1, tick0, tick1, miss0, miss1;
    UW blockers, snap, age, initial_safe, final_safe;
    UW cb0, cb1, tickerr0, tickerr1, waits0, waits1;
    T_RCYC imu_cyclic;
    T_RFLG imu_flag;
    ER cyc_er, imu_flag_er;
    T_RFLG flags;
    T_RMBF motor_q;
    ER er;
    int failures = 0;

    /* Check shared objects without consuming messages or clearing bits. */
    er = tk_ref_flg(flg_safety, &flags);
    SYS_CHECK("shared safety flag readable", er == E_OK);
    if (er == E_OK) {
        tm_printf((UB*)"[B4-SYS] live safety flags=0x%x observer_latch=%u reasons=0x%x\n",
                  (UINT)flags.flgptn, (UINT)safety_handler_would_latch(),
                  (UINT)safety_handler_reasons());
    }
    er = tk_ref_mbf(motion_cmd_q, &motor_q);
    SYS_CHECK("Buddy 2 command queue is accessible (read only)", er == E_OK);
    er = tk_ref_mbf(remote_cmd_q, &motor_q);
    SYS_CHECK("Buddy 1 remote command queue is accessible (read only)", er == E_OK);
    /* Existence of remote_cmd_q is NOT proof that Wi-Fi is online or that
     * VC actually consumes remote commands. It currently does NOT. */

    pub0 = vehicle_controller_line_published_count();
    obs0 = vehicle_controller_line_update_count();
    imu0 = s4_stat_cycles();
    cb0 = s4_stat_handler_ticks();
    tickerr0 = s4_stat_tick_flag_errors();
    waits0 = s4_stat_wait_timeouts();
    tick0 = motion_tick_count;
    miss0 = motion_ticks_missed;
    initial_safe = vehicle_controller_safe_motion_command();
#if APP_DEBUG_TEST_DETAIL
    print_imu_task_state("BEFORE");
#endif
    /* Sampling window: lets IMU (10ms), Buddy 2 (10ms) and VC (50ms) run. */
    er = tk_dly_tsk(600);
    SYS_CHECK("600ms probe delay completed", er == E_OK);
    pub1 = vehicle_controller_line_published_count();
    obs1 = vehicle_controller_line_update_count();
    imu1 = s4_stat_cycles();
    cb1 = s4_stat_handler_ticks();
    tickerr1 = s4_stat_tick_flag_errors();
    waits1 = s4_stat_wait_timeouts();
    cyc_er = tk_ref_cyc(cyc_imu, &imu_cyclic);
    imu_flag_er = tk_ref_flg(flg_imu, &imu_flag);
    if (APP_DEBUG_TEST_DETAIL ||
        (UW)(imu1 - imu0) < 40u || (UW)(imu1 - imu0) > 85u) {
        tm_printf((UB*)"[B4-IMU-DIAG] 600ms handler=%u task=%u flag_errors=%u wait_timeouts=%u\n",
                  (UINT)(cb1 - cb0), (UINT)(imu1 - imu0),
                  (UINT)(tickerr1 - tickerr0), (UINT)(waits1 - waits0));
        tm_printf((UB*)"[B4-IMU-DIAG] cyclic er=%d active=%d left_ms=%u; flag er=%d pattern=0x%x waiter=%d\n",
                  (INT)cyc_er, (INT)((cyc_er == E_OK) && ((imu_cyclic.cycstat & TCYC_STA) != 0U)),
                  (UINT)((cyc_er == E_OK) ? imu_cyclic.lfttim : 0U), (INT)imu_flag_er,
                  (UINT)((imu_flag_er == E_OK) ? imu_flag.flgptn : 0U),
                  (INT)((imu_flag_er == E_OK) ? imu_flag.wtsk : 0));
        print_imu_task_state("AFTER");
    }
    tick1 = motion_tick_count;
    miss1 = motion_ticks_missed;
    final_safe = vehicle_controller_safe_motion_command();
    blockers = vehicle_controller_motion_blockers();
    age = vehicle_controller_line_age_ms();
    snap = s4_imu_get_snapshot();

    tm_printf((UB*)"[B4-SYS] Buddy3 PUBLISHED seq=%u->%u delta=%u; VC CONSUMED=%u->%u delta=%u\n",
              (UINT)pub0, (UINT)pub1, (UINT)((pub1 - pub0) & 0x7FFFFFu),
              (UINT)obs0, (UINT)obs1, (UINT)(obs1 - obs0));
    tm_printf((UB*)"[B4-SYS] line age=%u ms (%s), VC state=%u intent preview=%c safe=%c\n",
              (UINT)age, age == 0xFFFFFFFFu ? "NEVER RECEIVED" :
              (age > 500u ? "STALE" : "FRESH"),
              (UINT)vehicle_controller_state(),
              (INT)vehicle_controller_candidate_motion_command(),
              (INT)final_safe);

    if (((pub1 - pub0) & 0x7FFFFFu) == 0u) {
        tm_printf((UB*)"[B4-SYS] LIVE LINE: NOT OBSERVED in 600ms; Buddy 3 task/sensors require team check\n");
    } else if (obs1 == obs0 && ((pub1 - pub0) & 0x7FFFFFu) >= 2u) {
        /* Late posts near the sampling boundary could still be waiting for
         * the VC's next 50ms tick. Allow 120ms to settle before failing. */
        (void)tk_dly_tsk(120);
        obs_after_settle = vehicle_controller_line_update_count();
        tm_printf((UB*)"[B4-SYS] VC settle: consumed=%u (+%u after 120ms)\n",
                  (UINT)obs_after_settle, (UINT)(obs_after_settle - obs1));
        SYS_CHECK("Buddy3 publications reach VC task after settling",
                  obs_after_settle > obs0);
    } else {
        tm_printf((UB*)"[B4-SYS] LIVE LINE: publications observed (not proof of correct sensor classification)\n");
    }
    SYS_CHECK("IMU 100Hz task advanced during sample window",
              (UW)(imu1 - imu0) >= 40u && (UW)(imu1 - imu0) <= 85u);
    SYS_CHECK("Buddy2 100Hz motion tick advanced during sample window",
              (UW)(tick1 - tick0) >= 40u && (UW)(tick1 - tick0) <= 85u);
    SYS_CHECK("Buddy2 no missed ticks in sampling window", miss1 == miss0);
    SYS_CHECK("IMU snapshot valid and monitoring",
              s4_snap_valid(snap) &&
              (s4_snap_state(snap) == S4_ST_MONITORING ||
               s4_snap_state(snap) == S4_ST_HUMP_TRACKING));
    tm_printf((UB*)"[B4-SYS] 600ms: IMU +%u cycles; motion +%u ticks, missed +%u\n",
              (UINT)(imu1 - imu0), (UINT)(tick1 - tick0), (UINT)(miss1 - miss0));

    /* This is an integration build with Buddy 5, active ESTOP and motor
     * arbitration unverified. All three gates MUST remain asserted. */
    SYS_CHECK("Buddy5/live-ESTOP/Buddy2 handoff safety gates stay blocked",
              (blockers & missing_gates) == missing_gates);
    SYS_CHECK("motor output initially HALT", initial_safe == MC_CMD_HALT);
    SYS_CHECK("motor output remains HALT", final_safe == MC_CMD_HALT);
    tm_printf((UB*)"[B4-SYS] blockers=0x%x; gate decode:\n", (UINT)blockers);
    print_blocker_names(blockers);
    tm_printf((UB*)"[B4-SYS] READINESS=HOLD: Buddy5 + live ESTOP + Buddy2 arbitration not verified\n");
    tm_printf((UB*)"[B4-SYS] Buddy1 Wi-Fi transport and remote-command consumption NOT verified\n");
    tm_printf((UB*)"[B4-SYS] No queue receives, motor posts, or shared safety writes performed\n");
    return failures;
}
