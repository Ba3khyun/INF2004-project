#include "safety_policy_test.h"
#include "../safety_policy.h"

int safety_phase4c_test_run(void)
{
    int failures = 0;
    p4c_policy_t p;
    p4c_decision_t d;
#define VERIFY(cond) do { if (!(cond)) ++failures; } while (0)
    p4c_policy_init(&p);
    VERIFY(p.latched == 0 && p.evidence == 0);
    d = p4c_policy_step(&p, 0, 1, 0, 1, 1);
    VERIFY(d.propose_assert_estop == 0 && d.proposed_motor_inhibit == 0);

    d = p4c_policy_step(&p, P4C_COLLISION, 1, 0, 1, 1);
    VERIFY(d.propose_assert_estop == 1 && d.proposed_motor_inhibit == 1);
    VERIFY(p.evidence == P4C_COLLISION);
    d = p4c_policy_step(&p, P4C_OBSTACLE_STOP, 1, 0, 1, 1);
    VERIFY(p.evidence == (P4C_COLLISION | P4C_OBSTACLE_STOP));
    d = p4c_policy_step(&p, P4C_REMOTE_ABORT, 1, 0, 1, 1);
    VERIFY(p.evidence == P4C_REQUEST_MASK);

    d = p4c_policy_step(&p, 0, 1, 0, 1, 1);
    VERIFY(d.propose_assert_estop == 1 && d.propose_allow_release == 0);
    d = p4c_policy_step(&p, P4C_COLLISION, 1, 1, 1, 1);
    VERIFY(d.propose_allow_release == 0 && p.latched == 1);
    d = p4c_policy_step(&p, 0, 1, 1, 0, 1);
    VERIFY(d.propose_allow_release == 0 && p.latched == 1);
    d = p4c_policy_step(&p, 0, 1, 1, 1, 0);
    VERIFY(d.propose_allow_release == 0 && p.latched == 1);
    d = p4c_policy_step(&p, 0, 1, 1, 1, 1);
    VERIFY(d.propose_allow_release == 1 && p.latched == 0 && p.evidence == 0);

    d = p4c_policy_step(&p, P4C_REMOTE_ABORT, 1, 0, 1, 1);
    VERIFY(p.latched == 1 && d.propose_assert_estop == 1);
    d = p4c_policy_step(&p, 0, 0, 1, 1, 1);
    VERIFY(p.latched == 1 && p.input_read_failed == 1);
    VERIFY(d.propose_allow_release == 0 && d.proposed_motor_inhibit == 1);
    d = p4c_policy_step(&p, 0, 1, 0, 1, 1);
    VERIFY(p.input_read_failed == 1 && p.latched == 1);
    d = p4c_policy_step(&p, 0, 1, 1, 1, 1);
    VERIFY(p.input_read_failed == 0 && d.propose_allow_release == 1);

    d = p4c_policy_step(&p, 0, 1, 0, 1, 0);
    VERIFY(d.proposed_motor_inhibit == 1 && d.propose_allow_release == 0);
    d = p4c_policy_step(&p, 0x80000000u, 1, 0, 1, 1);
    VERIFY(d.propose_assert_estop == 0 && p.evidence == 0);
    d = p4c_policy_step(&p, P4C_OBSTACLE_STOP, 1, 0, 1, 1);
    VERIFY(d.propose_assert_estop == 1 && p.evidence == P4C_OBSTACLE_STOP);
#undef VERIFY
    return failures;
}
