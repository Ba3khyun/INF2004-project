#include "safety_bridge_test.h"
#include "../safety_estop_bridge.h"

int safety_phase4d_test_run(void)
{
    int failures = 0;
    seb_state_t s;
    seb_init(&s);
    if (seb_step(&s, 0u, 1)) failures++;
    if (!seb_step(&s, SEB_COLLISION, 1)) failures++;
    if (!seb_step(&s, 0u, 1)) failures++;
    if (s.reasons != SEB_COLLISION) failures++;
    seb_init(&s);
    if (!seb_step(&s, SEB_OBSTACLE, 1)) failures++;
    seb_init(&s);
    if (!seb_step(&s, SEB_REMOTE_ABORT, 1)) failures++;
    seb_init(&s);
    if (!seb_step(&s, 0u, 0)) failures++;
    if (!seb_step(&s, 0u, 1) || !s.read_failed) failures++;
    seb_init(&s);
    if (!seb_step(&s, SEB_ESTOP, 1)) failures++;
    return failures;
}
