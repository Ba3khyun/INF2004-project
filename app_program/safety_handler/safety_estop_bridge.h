#ifndef SAFETY_ESTOP_BRIDGE_H
#define SAFETY_ESTOP_BRIDGE_H
#include <stdint.h>

/* Pure decision bridge: no kernel calls, no motor commands, no release path. */
#define SEB_COLLISION    (1u << 0)
#define SEB_OBSTACLE     (1u << 1)
#define SEB_REMOTE_ABORT (1u << 2)
#define SEB_REQ_MASK (SEB_COLLISION | SEB_OBSTACLE | SEB_REMOTE_ABORT)
#define SEB_ESTOP       (1u << 8)

typedef struct {
    uint32_t reasons;
    uint8_t sticky;
    uint8_t read_failed;
} seb_state_t;

void seb_init(seb_state_t *s);
/* Returns 1 when the caller should assert the ESTOP bit.
 * Sticky for whole boot; reset is intentionally NOT exposed. */
int seb_step(seb_state_t *s, uint32_t observed_flags, int read_ok);
#endif
