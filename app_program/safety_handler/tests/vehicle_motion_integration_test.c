/* Phase 5A: real Buddy 2 command codes, pure VC model, read-only live probes.
 * NO motion_cmd_post(), tk_set_flg(), tk_clr_flg(), or motor writes.
 */
#include "../../app.h"
#include "../../subsystem2/subsystem2.h"
#include "../../subsystem4/subsystem4.h"
#include "../../vehicle_controller/vehicle_controller.h"
#define VC_CORE_USE_TEAM_STATE 1
#include "../../vehicle_controller/vehicle_controller_motion_adapter.h"
#include "vehicle_motion_integration_test.h"

#define V5A_CHECK(name, expr) do { \
    if (!(expr)) { \
        tm_printf((UB*)"[B4-5A] FAIL: %s\n", name); \
        failures++; \
    } \
} while (0)

int vehicle_motion_integration_test_run(void)
{
    static const vma_codes_t codes = {
        MC_CMD_DRIVE_FORWARD, MC_CMD_STEER_LEFT, MC_CMD_STEER_RIGHT,
        MC_CMD_DRIVE_LEFT, MC_CMD_DRIVE_RIGHT, MC_CMD_HALT
    };
    static const vc_output_t outs[] = {
        VC_OUTPUT_FORWARD, VC_OUTPUT_STEER_LEFT, VC_OUTPUT_STEER_RIGHT,
        VC_OUTPUT_PIVOT_LEFT, VC_OUTPUT_PIVOT_RIGHT
    };
    static const UB expected[] = {
        MC_CMD_DRIVE_FORWARD, MC_CMD_STEER_LEFT, MC_CMD_STEER_RIGHT,
        MC_CMD_DRIVE_LEFT, MC_CMD_DRIVE_RIGHT
    };
    vma_inputs_t good = {1, 1, 0, 0, 1, 1, 1};
    vma_inputs_t i;
    vma_result_t r;
    vc_core_t vc;
    T_RFLG sf;
    ER er;
    UW old_updates, new_updates, snap;
    int failures = 0;
    unsigned k;

    /* Match the real Buddy 2 API, not duplicated character literals. */
    for (k = 0u; k < sizeof(outs) / sizeof(outs[0]); ++k) {
        r = vma_preview(outs[k], &good, &codes);
        V5A_CHECK("Buddy2 code mapping", r.candidate_command == expected[k]);
        V5A_CHECK("mapping readiness is hypothetical", r.model_ready == 1u);
        V5A_CHECK("preview never releases motor command", r.safe_command == MC_CMD_HALT);
    }
    r = vma_preview(VC_OUTPUT_HALT, &good, &codes);
    V5A_CHECK("HALT maps to HALT", r.candidate_command == MC_CMD_HALT && !r.model_ready);
    r = vma_preview((vc_output_t)99, &good, &codes);
    V5A_CHECK("unknown output rejected", (r.blockers & VMA_BLOCK_BAD_OUTPUT) != 0u &&
              r.candidate_command == MC_CMD_HALT);
    r = vma_preview(VC_OUTPUT_FORWARD, 0, &codes);
    V5A_CHECK("invalid preview inputs rejected", (r.blockers & VMA_BLOCK_BAD_INPUT) != 0u);

    i = good; i.safety_read_ok = 0;
    r = vma_preview(VC_OUTPUT_FORWARD, &i, &codes);
    V5A_CHECK("unreadable safety flags inhibit", (r.blockers & VMA_BLOCK_SAFETY_READ) &&
              !r.model_ready && r.safe_command == MC_CMD_HALT);
    i = good; i.imu_healthy = 0;
    r = vma_preview(VC_OUTPUT_FORWARD, &i, &codes);
    V5A_CHECK("invalid IMU inhibits", (r.blockers & VMA_BLOCK_IMU) && !r.model_ready);
    i = good; i.safety_request_active = 1;
    r = vma_preview(VC_OUTPUT_FORWARD, &i, &codes);
    V5A_CHECK("collision/abort/obstacle inhibits", (r.blockers & VMA_BLOCK_SAFETY_REQ) && !r.model_ready);
    i = good; i.estop_latched = 1;
    r = vma_preview(VC_OUTPUT_FORWARD, &i, &codes);
    V5A_CHECK("ESTOP inhibits", (r.blockers & VMA_BLOCK_ESTOP) && !r.model_ready);
    i = good; i.obstacle_source_verified = 0;
    r = vma_preview(VC_OUTPUT_FORWARD, &i, &codes);
    V5A_CHECK("missing Buddy5 inhibits", (r.blockers & VMA_BLOCK_B5_MISSING) && !r.model_ready);
    i = good; i.live_estop_verified = 0;
    r = vma_preview(VC_OUTPUT_FORWARD, &i, &codes);
    V5A_CHECK("unverified live ESTOP inhibits", (r.blockers & VMA_BLOCK_STOP_UNVERIFIED) && !r.model_ready);
    i = good; i.motor_handoff_approved = 0;
    r = vma_preview(VC_OUTPUT_FORWARD, &i, &codes);
    V5A_CHECK("unreviewed Buddy2 arbitration inhibits", (r.blockers & VMA_BLOCK_HANDOFF_PENDING) && !r.model_ready);
    i = good; i.obstacle_source_verified = 0; i.live_estop_verified = 0;
    i.motor_handoff_approved = 0;
    r = vma_preview(VC_OUTPUT_FORWARD, &i, &codes);
    V5A_CHECK("missing inputs combine", (r.blockers &
               (VMA_BLOCK_B5_MISSING | VMA_BLOCK_STOP_UNVERIFIED |
                VMA_BLOCK_HANDOFF_PENDING)) ==
               (VMA_BLOCK_B5_MISSING | VMA_BLOCK_STOP_UNVERIFIED |
                VMA_BLOCK_HANDOFF_PENDING));
    V5A_CHECK("missing inputs still HALT", r.safe_command == MC_CMD_HALT);

    /* Simulate Buddy 3 and Buddy 1 in an ISOLATED core (not live queues). */
    vc_core_init(&vc);
    vc_core_line(&vc, VC_INTENT_FORWARD, 100u);
    V5A_CHECK("unready IMU prevents forward", vc_core_step(&vc, 101u) == VC_OUTPUT_HALT);
    vc_core_imu(&vc, VC_IMU_READY, 0, 0);
    V5A_CHECK("Buddy3 line allows proposal", vc_core_step(&vc, 102u) == VC_OUTPUT_FORWARD);
    vc_core_pause(&vc); /* simulated Buddy1 pause */
    V5A_CHECK("remote pause model HALT", vc_core_step(&vc, 103u) == VC_OUTPUT_HALT);
    vc_core_resume(&vc); /* simulated Buddy1 resume, not safety reset */
    vc_core_set_safety(&vc, 1, 0); /* simulated abort */
    V5A_CHECK("remote abort overrides resume", vc_core_step(&vc, 104u) == VC_OUTPUT_HALT);
    vc_core_set_safety(&vc, 0, 1);
    V5A_CHECK("ESTOP persists in core model", vc_core_step(&vc, 105u) == VC_OUTPUT_HALT);
    vc_core_set_safety(&vc, 0, 0); /* simulated authorized reset only */
    vc_core_line_visible(&vc, 0);
    V5A_CHECK("line missing HALT", vc_core_step(&vc, 106u) == VC_OUTPUT_HALT);
    vc_core_line_visible(&vc, 1);
    V5A_CHECK("stale line intent HALT", vc_core_step(&vc, 602u) == VC_OUTPUT_HALT);

    /* Read-only live checks: no writing to VC mailbox, flags or motor queue. */
    er = tk_ref_flg(flg_safety, &sf);
    V5A_CHECK("live safety flag readable", er == E_OK);
    snap = s4_imu_get_snapshot();
    tm_printf((UB*)"[B4-5A] IMU state=%u valid=%u; safety_read=%d\n",
              (UINT)s4_snap_state(snap), (UINT)s4_snap_valid(snap), (INT)er);
    old_updates = vehicle_controller_line_update_count();
    (void)tk_dly_tsk(200);
    new_updates = vehicle_controller_line_update_count();
    tm_printf((UB*)"[B4-5A] Buddy3->VC updates=%u (+%u in 200ms), candidate=%c safe=%c blockers=0x%x\n",
              (UINT)new_updates, (UINT)(new_updates - old_updates),
              (INT)vehicle_controller_candidate_motion_command(),
              (INT)vehicle_controller_safe_motion_command(),
              (UINT)vehicle_controller_motion_blockers());
    if (new_updates == old_updates) {
        tm_printf((UB*)"[B4-5A] NOTE: no new physical Buddy3 line updates observed; hardware path unverified\n");
    }
    V5A_CHECK("live VC adapter returns HALT", vehicle_controller_safe_motion_command() == MC_CMD_HALT);
    V5A_CHECK("unavailable producers are blocked", (vehicle_controller_motion_blockers() &
              (VMA_BLOCK_B5_MISSING | VMA_BLOCK_STOP_UNVERIFIED |
               VMA_BLOCK_HANDOFF_PENDING)) ==
              (VMA_BLOCK_B5_MISSING | VMA_BLOCK_STOP_UNVERIFIED |
               VMA_BLOCK_HANDOFF_PENDING));
    tm_printf((UB*)"[B4-5A] no motor posts, no shared safety writes; Buddy1/5 LIVE paths NOT validated\n");
    return failures;
}
