#include "../safety_handler/safety_core.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    safety_core_t s;
    safety_core_init(&s);
    assert(!s.latched && !s.pending_reasons);
    safety_core_observe(&s, 0u);
    assert(!s.latched);
    safety_core_observe(&s, SC_REQ_COLLISION);
    assert(s.latched && s.pending_reasons == SC_REQ_COLLISION);
    safety_core_observe(&s, SC_REQ_REMOTE_ABORT | SC_REQ_OBSTACLE_STOP);
    assert(s.latched && s.pending_reasons == SC_ALL_REQUESTS);
    safety_core_observe(&s, 0u);
    assert(s.latched); /* never automatically clears */
    assert(!safety_core_try_reset(&s, 0u, 0));
    assert(!safety_core_try_reset(&s, SC_REQ_COLLISION, 1));
    assert(s.latched);
    assert(safety_core_try_reset(&s, 0u, 1));
    assert(!s.latched && !s.pending_reasons);
    safety_core_observe(&s, SC_REQ_OBSTACLE_STOP);
    assert(s.latched && s.pending_reasons == SC_REQ_OBSTACLE_STOP);
    puts("Safety core: all 11 checks passed");
    return 0;
}
