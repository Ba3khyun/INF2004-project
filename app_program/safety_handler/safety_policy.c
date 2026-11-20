#include "safety_policy.h"

void p4c_policy_init(p4c_policy_t *p)
{
    if (!p) return;
    p->evidence = 0u;
    p->latched = 0u;
    p->input_read_failed = 0u;
}

p4c_decision_t p4c_policy_step(p4c_policy_t *p, uint32_t requests, int flag_read_ok,
                               int authorized_local_reset, int motion_idle,
                               int sensor_health_ok)
{
    p4c_decision_t d = {0u, 0u, 1u};
    uint32_t active = requests & P4C_REQUEST_MASK;
    if (!p) return d;

    if (!flag_read_ok) {
        p->input_read_failed = 1u;
        p->latched = 1u;
    } else {
        p->evidence |= active;
        if (active) p->latched = 1u;
        /* A failed read remains a sticky reason until explicitly reset. */
    }

    /* Reset is a proposed policy only; it does NOT clear any kernel flags.
     * In particular a remote RESUME does not constitute local authorization. */
    if (flag_read_ok && active == 0u && authorized_local_reset &&
        motion_idle && sensor_health_ok) {
        p->latched = 0u;
        p->evidence = 0u;
        p->input_read_failed = 0u;
        d.propose_allow_release = 1u;
    }

    d.propose_assert_estop = p->latched;
    d.proposed_motor_inhibit = (uint8_t)(p->latched || !flag_read_ok ||
                                          !sensor_health_ok);
    return d;
}
