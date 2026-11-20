#ifndef SAFETY_CORE_H
#define SAFETY_CORE_H

#include <stdint.h>

/* Pure safety decision logic. The caller supplies the shared flag mask. */
#define SC_REQ_COLLISION       (1u << 0)
#define SC_REQ_OBSTACLE_STOP   (1u << 1)
#define SC_REQ_REMOTE_ABORT    (1u << 2)
#define SC_ALL_REQUESTS (SC_REQ_COLLISION | SC_REQ_OBSTACLE_STOP | SC_REQ_REMOTE_ABORT)

typedef struct {
    uint32_t pending_reasons;  /* sticky evidence until explicit reset */
    uint8_t latched;           /* never auto clears */
} safety_core_t;

void safety_core_init(safety_core_t *s);
void safety_core_observe(safety_core_t *s, uint32_t request_bits);
/* Explicit reset allowed only if the current live requests have cleared.
 * This is a logic-only check, NOT authorization to release physical motors. */
int safety_core_try_reset(safety_core_t *s, uint32_t live_requests,
                          int local_reset_authorized);

#endif
