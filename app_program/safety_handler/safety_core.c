#include "safety_core.h"

void safety_core_init(safety_core_t *s)
{
    if (s == 0) return;
    s->pending_reasons = 0u;
    s->latched = 0u;
}

void safety_core_observe(safety_core_t *s, uint32_t request_bits)
{
    uint32_t reasons;
    if (s == 0) return;
    reasons = request_bits & SC_ALL_REQUESTS;
    if (reasons != 0u) {
        s->pending_reasons |= reasons;
        s->latched = 1u;
    }
}

int safety_core_try_reset(safety_core_t *s, uint32_t live_requests,
                          int local_reset_authorized)
{
    if (s == 0 || !local_reset_authorized ||
        (live_requests & SC_ALL_REQUESTS) != 0u) {
        return 0;
    }
    s->latched = 0u;
    s->pending_reasons = 0u;
    return 1;
}
