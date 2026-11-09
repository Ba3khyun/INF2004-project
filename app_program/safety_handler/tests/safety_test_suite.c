#include "../../app.h"
#include "../../app_debug.h"
#include "safety_test_suite.h"
#include "vehicle_safety_test.h"
#include "vehicle_motion_integration_test.h"
#include "system_integration_readiness_test.h"
#include "terrain_turn_integration_test.h"
#include "motion_bridge_integration_test.h"
#include "safety_core_test.h"
#include "safety_diagnostics.h"
#include "safety_policy_test.h"
#include "safety_bridge_test.h"
#include "safety_handoff_test.h"
#include "../../subsystem4/subsystem4.h"
#include "../../vehicle_controller/vehicle_controller.h"

/* This suite is read-only to all kernel safety flags and motion outputs. */
static int result(const char *name, int failures)
{
    tm_printf((UB*)"[B4-ALL] %-20s %s (failed=%d)\n",
              name, failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}

static int live_readonly_health(void)
{
    T_RFLG f;
    UW initial, final, snap;
    UW cb0, cb1, errors0, errors1, timeout0, timeout1;
    T_RCYC cyc;
    T_RFLG tick_flag;
    ER cyc_er, flag_er;
    ER er;
    int failures = 0;

    er = tk_ref_flg(flg_safety, &f);
    if (er < E_OK) {
        tm_printf((UB*)"[B4-ALL] cannot read safety flags: er=%d\n", (INT)er);
        failures++;
    } else {
        tm_printf((UB*)"[B4-ALL] safety flag read OK (0x%x) -- NO WRITES\n",
                  (UINT)f.flgptn);
    }

    initial = s4_stat_cycles();
    cb0 = s4_stat_handler_ticks();
    errors0 = s4_stat_tick_flag_errors();
    timeout0 = s4_stat_wait_timeouts();
    (void)tk_dly_tsk(500);
    final = s4_stat_cycles();
    cb1 = s4_stat_handler_ticks();
    errors1 = s4_stat_tick_flag_errors();
    timeout1 = s4_stat_wait_timeouts();
    cyc_er = tk_ref_cyc(cyc_imu, &cyc);
    flag_er = tk_ref_flg(flg_imu, &tick_flag);
    /* Always show stall diagnostics on failure, even in quiet mode. */
    if (APP_DEBUG_TEST_DETAIL ||
        (UW)(final - initial) < 35u || (UW)(final - initial) > 65u) {
        tm_printf((UB*)"[B4-IMU-DIAG] 500ms handler=%u task=%u flag_errors=%u wait_timeouts=%u\n",
                  (UINT)(cb1 - cb0), (UINT)(final - initial),
                  (UINT)(errors1 - errors0), (UINT)(timeout1 - timeout0));
        tm_printf((UB*)"[B4-IMU-DIAG] cyclic er=%d active=%d left_ms=%u; flag er=%d pattern=0x%x waiter=%d\n",
                  (INT)cyc_er, (INT)((cyc_er == E_OK) && ((cyc.cycstat & TCYC_STA) != 0U)),
                  (UINT)((cyc_er == E_OK) ? cyc.lfttim : 0U), (INT)flag_er,
                  (UINT)((flag_er == E_OK) ? tick_flag.flgptn : 0U),
                  (INT)((flag_er == E_OK) ? tick_flag.wtsk : 0));
    }
    snap = s4_imu_get_snapshot();
    tm_printf((UB*)"[B4-ALL] imu cycles before=%u after=%u delta=%u state=%u valid=%u\n",
              (UINT)initial, (UINT)final, (UINT)(final - initial),
              (UINT)s4_snap_state(snap), (UINT)s4_snap_valid(snap));
    /* Guard against the previously observed cycles=0 / stuck calibration. */
    if ((UW)(final - initial) < 35u || (UW)(final - initial) > 65u) failures++;
    if (!s4_snap_valid(snap) ||
        (s4_snap_state(snap) != S4_ST_MONITORING &&
         s4_snap_state(snap) != S4_ST_HUMP_TRACKING)) failures++;

    tm_printf((UB*)"[B4-ALL] VC observed state=%u output=%u (PASSIVE expected)\n",
              (UINT)vehicle_controller_state(), (UINT)vehicle_controller_output());
    /* Do NOT require LINE_SEARCH: physical sensor may or may not see line.
     * Do NOT require FORWARD: the firmware must not move motors here. */
    return failures;
}

int safety_test_suite_run(void)
{
    int failed_groups = 0;
    tm_printf((UB*)"\n[B4-ALL] ===== ONE-SHOT VALIDATION START =====\n");
    failed_groups += result("SAFETY CORE", safety_validation_run());
    /* Existing Phase 4B diagnostic prints observer state and IMU delta. */
    failed_groups += result("SAFETY DIAGNOSTICS", safety_phase4b_diagnostics());
    failed_groups += result("SAFETY RESET POLICY", safety_phase4c_test_run());
    failed_groups += result("ESTOP BRIDGE MODEL", safety_phase4d_test_run());
    failed_groups += result("REQUEST HANDOFF MODEL", safety_phase4e_test_run());
    failed_groups += result("VC + SAFETY MODELS", safety_phase4f_checks_run());
    failed_groups += result("LIVE IMU HEALTH", live_readonly_health());
    failed_groups += result("MOTION MAPPING", vehicle_motion_integration_test_run());
    failed_groups += result("LIVE INTEGRATION", system_integration_readiness_test_run());
    failed_groups += result("HUMP + TURN SOFTWARE", terrain_turn_integration_test_run());
    failed_groups += result("B2 TURN + SPEED API", motion_bridge_integration_test_run());
    tm_printf((UB*)"[B4-ALL] FINAL: %s (%d / 11 groups failed; MOTOR DISABLED)\n",
              failed_groups ? "FAIL" : "PASS", failed_groups);
    tm_printf((UB*)"[B4-ALL] READINESS=HOLD until live safety / missing Buddy5 / Buddy2 approval.\n");
    tm_printf((UB*)"[B4-ALL] Model PASS does NOT certify physical ESTOP or motion.\n");
    tm_printf((UB*)"[B4-ALL] ===== ONE-SHOT VALIDATION END =====\n\n");
    return failed_groups;
}
