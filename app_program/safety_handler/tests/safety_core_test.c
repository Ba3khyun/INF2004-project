#include "safety_core_test.h"
#include "../safety_core.h"

/* Never touch the live g_safety singleton: test a local instance only. */
int safety_validation_run(void)
{
    safety_core_t s;
    int failures = 0;
#define VERIFY(x) do { if (!(x)) failures++; } while (0)

    safety_core_init(&s);
    VERIFY(s.latched == 0u);
    VERIFY(s.pending_reasons == 0u);

    safety_core_observe(&s, 0u);
    VERIFY(s.latched == 0u);

    safety_core_observe(&s, SC_REQ_COLLISION);
    VERIFY(s.latched == 1u);
    VERIFY(s.pending_reasons == SC_REQ_COLLISION);

    safety_core_observe(&s, SC_REQ_OBSTACLE_STOP);
    VERIFY(s.pending_reasons == (SC_REQ_COLLISION | SC_REQ_OBSTACLE_STOP));

    safety_core_observe(&s, SC_REQ_REMOTE_ABORT);
    VERIFY(s.pending_reasons == SC_ALL_REQUESTS);

    safety_core_observe(&s, 0u);
    VERIFY(s.latched == 1u);
    VERIFY(s.pending_reasons == SC_ALL_REQUESTS);

    VERIFY(safety_core_try_reset(&s, 0u, 0) == 0);
    VERIFY(s.latched == 1u);
    VERIFY(safety_core_try_reset(&s, SC_REQ_REMOTE_ABORT, 1) == 0);
    VERIFY(s.latched == 1u);

    /* Unknown bits must not trigger this logical safety core. */
    safety_core_init(&s);
    safety_core_observe(&s, (1u << 8));
    VERIFY(s.latched == 0u);

    safety_core_observe(&s, SC_REQ_REMOTE_ABORT);
    VERIFY(s.latched == 1u);
    VERIFY(s.pending_reasons == SC_REQ_REMOTE_ABORT);
    VERIFY(safety_core_try_reset(&s, 0u, 1) == 1);
    VERIFY(s.latched == 0u);
    VERIFY(s.pending_reasons == 0u);

    /* A new request after reset must relatch. */
    safety_core_observe(&s, SC_REQ_OBSTACLE_STOP);
    VERIFY(s.latched == 1u);
    VERIFY(s.pending_reasons == SC_REQ_OBSTACLE_STOP);

#undef VERIFY
    return failures;
}
