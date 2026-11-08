#include "safety_diagnostics.h"
#include "../safety_core.h"
#include "../safety_handler.h"
#include "../../subsystem4/subsystem4.h"

/* Simulate an eventual active ESTOP owner without writing real flags. */
static int policy_selftest(void)
{
    safety_core_t s;
    unsigned int simulated_estop = 0u;
    int failed = 0;
#define ASSERT_TRUE(x) do { if (!(x)) ++failed; } while (0)
    safety_core_init(&s);
    ASSERT_TRUE(s.latched == 0u);
    safety_core_observe(&s, SC_REQ_COLLISION);
    simulated_estop = s.latched;
    ASSERT_TRUE(simulated_estop == 1u);
    safety_core_observe(&s, 0u);
    ASSERT_TRUE(s.latched == 1u); /* clearing request must NOT release */
    ASSERT_TRUE(safety_core_try_reset(&s, 0u, 0) == 0);
    ASSERT_TRUE(s.latched == 1u);
    ASSERT_TRUE(safety_core_try_reset(&s, SC_REQ_COLLISION, 1) == 0);
    ASSERT_TRUE(s.latched == 1u);
    ASSERT_TRUE(safety_core_try_reset(&s, 0u, 1) == 1);
    ASSERT_TRUE(s.latched == 0u);
    safety_core_observe(&s, SC_REQ_REMOTE_ABORT | SC_REQ_OBSTACLE_STOP);
    ASSERT_TRUE(s.latched == 1u);
    ASSERT_TRUE(s.pending_reasons == (SC_REQ_REMOTE_ABORT | SC_REQ_OBSTACLE_STOP));
#undef ASSERT_TRUE
    return failed;
}

/* Run at the end of APP_TEST so the COM monitor will capture these prints.
 * Read-only diagnostics: never start/stop motors, never set/clear flags,
 * never reset IMU statistics or request a reinitialization.
 */
int safety_phase4b_diagnostics(void)
{
    UW before = s4_stat_cycles();
    UW snap_before = s4_imu_get_snapshot();
    s4_imu_state_t state_before = s4_imu_state();
    T_RFLG current_flags;
    ER flag_er = tk_ref_flg(flg_safety, &current_flags);
    int failed = policy_selftest();

    tm_printf((UB*)"[P4B] policy simulation %s failures=%d (NO LIVE ESTOP WRITES)\n",
              failed == 0 ? "PASS" : "FAIL", failed);
    tm_printf((UB*)"[P4B] observer latch=%u reasons=0x%x flag_read=%d live_flag=0x%x\n",
              (UINT)safety_handler_would_latch(),
              (UINT)safety_handler_reasons(), (INT)flag_er,
              (UINT)((flag_er == E_OK) ? current_flags.flgptn : 0u));
    tm_printf((UB*)"[P4B] IMU BEFORE state=%u snap_state=%u valid=%u cycles=%u\n",
              (UINT)state_before, (UINT)s4_snap_state(snap_before),
              (UINT)s4_snap_valid(snap_before), (UINT)before);
    (void)tk_dly_tsk(500);
    {
        UW after = s4_stat_cycles();
        UW snap_after = s4_imu_get_snapshot();
        tm_printf((UB*)"[P4B] IMU AFTER state=%u snap_state=%u valid=%u cycles=%u delta=%u\n",
                  (UINT)s4_imu_state(), (UINT)s4_snap_state(snap_after),
                  (UINT)s4_snap_valid(snap_after), (UINT)after,
                  (UINT)(after - before));
        if (after == before) {
            tm_printf((UB*)"[P4B] IMU WARNING: no recorded cycles in 500 ms; inspect tick/task state\n");
        }
    }
    return failed;
}
