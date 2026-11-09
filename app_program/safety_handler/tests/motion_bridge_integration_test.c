/* Pure-model + live read-only Buddy2 encoder bridge checks; no motor posts. */
#include "../../app.h"
#include "../../subsystem2/subsystem2.h"
#include "../../subsystem2/motion_feedback_math.h"
#include "../../vehicle_controller/vehicle_controller.h"
#include "motion_bridge_integration_test.h"
#define B4_CHECK_B2(x) do { if (!(x)) failures++; } while (0)
static float absf_b2(float x) { return x < 0.0f ? -x : x; }
int motion_bridge_integration_test_run(void)
{
    int failures = 0;
    motion_wheel_feedback_t before = {0}, after = {0};
    ER e0 = motion_get_wheel_feedback(&before);
    ER e1;
    UW last = motion_speed_cap_requested();
    /* Do not change cap in this test; VC is its active writer. */
    B4_CHECK_B2(last <= 100u);
    B4_CHECK_B2(motion_speed_cap_request(101u) == E_PAR);
    B4_CHECK_B2(motion_get_wheel_feedback(0) == E_PAR);
    B4_CHECK_B2(absf_b2(b2_cap_pwm(20.0f, 45u) - 9.0f) < 0.01f);
    B4_CHECK_B2(absf_b2(b2_cap_pwm(15.0f, 40u) - 6.0f) < 0.01f);
    B4_CHECK_B2(b2_cap_pwm(20.0f, 101u) == 0.0f);
    B4_CHECK_B2(absf_b2(b2_estimated_turn_deg(-94.2478f, 94.2478f, 120.0f) - 90.0f) < 1.0f);
    B4_CHECK_B2(absf_b2(b2_estimated_turn_deg(94.2478f, -94.2478f, 120.0f) + 90.0f) < 1.0f);
    B4_CHECK_B2(b2_estimated_turn_deg(-10.f, 10.f, 0.0f) == 0.0f);
    (void)tk_dly_tsk(150);
    e1 = motion_get_wheel_feedback(&after);
    B4_CHECK_B2(e0 == E_OK && e1 == E_OK);
    if (e0 == E_OK && e1 == E_OK) B4_CHECK_B2((UW)(after.sequence - before.sequence) >= 7u);
    tm_printf((UB*)"[B4-B2] wheel_seq %u->%u delta=%u L=%dmm R=%dmm sign=command-inferred\n",
              (UINT)before.sequence, (UINT)after.sequence,
              (UINT)(after.sequence-before.sequence),
              (INT)after.left_signed_mm, (INT)after.right_signed_mm);
    tm_printf((UB*)"[B4-B2] speed cap request=%u%% VC=%u%%; PWM APPLY OFF; no motor posts\n",
              (UINT)motion_speed_cap_requested(),
              (UINT)vehicle_controller_speed_cap_requested());
    return failures;
}
