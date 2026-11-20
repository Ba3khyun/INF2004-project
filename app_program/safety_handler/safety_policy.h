#ifndef SAFETY_PHASE4C_POLICY_H
#define SAFETY_PHASE4C_POLICY_H
#include <stdint.h>

/* Offline policy model, never wired to tk_set_flg/tk_clr_flg/motion_cmd_post.
 * These outputs are proposals for future review with Buddy 2 & Buddy 5. */
#define P4C_COLLISION     (1u << 0)
#define P4C_OBSTACLE_STOP (1u << 1)
#define P4C_REMOTE_ABORT  (1u << 2)
#define P4C_REQUEST_MASK (P4C_COLLISION | P4C_OBSTACLE_STOP | P4C_REMOTE_ABORT)

typedef struct {
    uint32_t evidence;
    uint8_t latched;
    uint8_t input_read_failed;
} p4c_policy_t;

typedef struct {
    uint8_t propose_assert_estop;
    uint8_t propose_allow_release;
    uint8_t proposed_motor_inhibit;
} p4c_decision_t;

void p4c_policy_init(p4c_policy_t *p);
p4c_decision_t p4c_policy_step(p4c_policy_t *p, uint32_t requests, int flag_read_ok,
                               int authorized_local_reset, int motion_idle,
                               int sensor_health_ok);
#endif
