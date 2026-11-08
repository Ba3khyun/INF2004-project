/* Phase 4F extra model tests. All objects are local to this function.
 * Includes line-found path without a physical track, state priority,
 * timeout wrap-around, and ESTOP producer/consumer contract limitations.
 */
#include "vehicle_safety_test.h"
#include "../safety_estop_bridge.h"
#include "../safety_policy.h"
#define VC_CORE_USE_TEAM_STATE 1
/* The team's vc_state_t normally comes from app.h in firmware builds.
 * For host compilation a minimal state mapping is defined below. */
#ifdef PHASE4F_HOST
 typedef enum { VC_INITIALIZING=0, VC_DRIVING=1, VC_HUMP_CROSSING=2,
   VC_OBSTACLE_BYPASS=3, VC_LINE_SEARCH=4, VC_PAUSED=5, VC_ERROR=6,
   VC_MISSION_COMPLETE=7 } vc_state_t;
#else
 #include "../../app.h"
#endif
#include "../../vehicle_controller/vehicle_controller_core.h"

#define B4_CHECK(c) do { if (!(c)) failures++; } while (0)
int safety_phase4f_checks_run(void)
{
    int failures = 0;
    vc_core_t v;
    seb_state_t b;
    p4c_policy_t p;
    p4c_decision_t d;
    uint32_t flags;

    /* VC: startup must be inhibited before IMU-ready. */
    vc_core_init(&v);
    vc_core_line(&v, VC_INTENT_FORWARD, 100u);
    B4_CHECK(vc_core_step(&v, 101u) == VC_OUTPUT_HALT && v.state == VC_INITIALIZING);

    /* Simulates a line-found sensor reading. No actual sensor or motors. */
    vc_core_imu(&v, VC_IMU_READY, 0, 0);
    B4_CHECK(vc_core_step(&v, 102u) == VC_OUTPUT_FORWARD && v.state == VC_DRIVING);
    vc_core_line(&v, VC_INTENT_STEER_LEFT, 110u);
    B4_CHECK(vc_core_step(&v, 111u) == VC_OUTPUT_STEER_LEFT);
    vc_core_line(&v, VC_INTENT_STEER_RIGHT, 112u);
    B4_CHECK(vc_core_step(&v, 113u) == VC_OUTPUT_STEER_RIGHT);
    vc_core_line(&v, VC_INTENT_PIVOT_LEFT, 114u);
    B4_CHECK(vc_core_step(&v, 115u) == VC_OUTPUT_PIVOT_LEFT);
    vc_core_line(&v, VC_INTENT_PIVOT_RIGHT, 116u);
    B4_CHECK(vc_core_step(&v, 117u) == VC_OUTPUT_PIVOT_RIGHT);
    vc_core_line_visible(&v, 0);
    B4_CHECK(vc_core_step(&v, 118u) == VC_OUTPUT_HALT && v.state == VC_LINE_SEARCH);
    vc_core_line_visible(&v, 1);
    vc_core_line(&v, VC_INTENT_FORWARD, 120u);
    B4_CHECK(vc_core_step(&v, 620u) == VC_OUTPUT_FORWARD); /* 500ms is still fresh */
    B4_CHECK(vc_core_step(&v, 621u) == VC_OUTPUT_HALT && v.state == VC_LINE_SEARCH);
    vc_core_line(&v, VC_INTENT_FORWARD, 700u);
    vc_core_imu(&v, VC_IMU_HUMP_START, 0, 0);
    B4_CHECK(vc_core_step(&v, 701u) == VC_OUTPUT_HALT && v.state == VC_HUMP_CROSSING);
    vc_core_imu(&v, VC_IMU_HUMP_DONE, 1, 34);
    B4_CHECK(vc_core_step(&v, 702u) == VC_OUTPUT_FORWARD && v.last_hump_mm == 34);
    vc_core_pause(&v);
    B4_CHECK(vc_core_step(&v, 703u) == VC_OUTPUT_HALT && v.state == VC_PAUSED);
    vc_core_set_safety(&v, 1, 0);
    B4_CHECK(vc_core_step(&v, 704u) == VC_OUTPUT_HALT && v.state == VC_ERROR);
    vc_core_set_safety(&v, 0, 1);
    vc_core_resume(&v);
    B4_CHECK(vc_core_step(&v, 705u) == VC_OUTPUT_HALT && v.state == VC_ERROR);
    vc_core_set_safety(&v, 0, 0);
    vc_core_imu(&v, VC_IMU_FAULT, 0, 0);
    B4_CHECK(vc_core_step(&v, 706u) == VC_OUTPUT_HALT && v.state == VC_ERROR);
    vc_core_imu(&v, VC_IMU_READY, 0, 0);
    B4_CHECK(vc_core_step(&v, 707u) == VC_OUTPUT_FORWARD && v.state == VC_DRIVING);
    vc_core_complete(&v);
    B4_CHECK(vc_core_step(&v, 708u) == VC_OUTPUT_HALT && v.state == VC_MISSION_COMPLETE);

    /* Wrap-around-safe intent age is handled by unsigned subtraction. */
    vc_core_init(&v);
    vc_core_imu(&v, VC_IMU_READY, 0, 0);
    vc_core_line(&v, VC_INTENT_FORWARD, 0xFFFFFFF0u);
    B4_CHECK(vc_core_step(&v, 0x0000000Au) == VC_OUTPUT_FORWARD);

    /* ESTOP bridge must preserve unrelated bits, latch, and fail closed. */
    seb_init(&b);
    flags = SEB_REMOTE_ABORT | (1u << 14);
    B4_CHECK(seb_step(&b, flags, 1) == 1);
    B4_CHECK(b.reasons == SEB_REMOTE_ABORT);
    flags |= SEB_ESTOP; /* only simulated write */
    B4_CHECK((flags & (1u << 14)) != 0u);
    flags &= ~(SEB_REMOTE_ABORT | SEB_ESTOP); /* simulate erroneous clear */
    B4_CHECK(seb_step(&b, flags, 1) == 1 && b.sticky);
    seb_init(&b);
    B4_CHECK(seb_step(&b, 0u, 0) == 1 && b.read_failed);
    B4_CHECK(seb_step(&b, 0u, 1) == 1);

    /* Proposed local reset: absent authorization, busy motors, bad sensors,
     * active requests, or flag-read errors must prevent release. */
    p4c_policy_init(&p);
    d = p4c_policy_step(&p, P4C_COLLISION, 1, 0, 1, 1);
    B4_CHECK(d.propose_assert_estop && d.proposed_motor_inhibit);
    d = p4c_policy_step(&p, 0, 1, 0, 1, 1);
    B4_CHECK(!d.propose_allow_release && p.latched);
    d = p4c_policy_step(&p, 0, 1, 1, 0, 1);
    B4_CHECK(!d.propose_allow_release && p.latched);
    d = p4c_policy_step(&p, 0, 1, 1, 1, 0);
    B4_CHECK(!d.propose_allow_release && p.latched);
    d = p4c_policy_step(&p, 0, 0, 1, 1, 1);
    B4_CHECK(!d.propose_allow_release && p.latched);
    d = p4c_policy_step(&p, 0, 1, 1, 1, 1);
    B4_CHECK(d.propose_allow_release && !p.latched);
    return failures;
}
