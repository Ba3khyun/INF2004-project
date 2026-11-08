#include "safety_estop_bridge.h"
void seb_init(seb_state_t *s)
{
    if (!s) return;
    s->reasons = 0;
    s->sticky = 0;
    s->read_failed = 0;
}
int seb_step(seb_state_t *s, uint32_t observed_flags, int read_ok)
{
    uint32_t active;
    if (!s) return 1; /* Fail closed if state is unavailable. */
    if (!read_ok) {
        s->read_failed = 1;
        s->sticky = 1;
    } else {
        active = observed_flags & SEB_REQ_MASK;
        s->reasons |= active;
        if (active || (observed_flags & SEB_ESTOP)) s->sticky = 1;
    }
    /* Reassert on every tick: also repairs unexpected flag clearing. */
    return s->sticky ? 1 : 0;
}
