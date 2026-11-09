#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "../app.h"

/* Phase 3: passive line-intent receiver. No motor commands issued. */
ER vehicle_controller_init(void);
ER vehicle_controller_start(void);

/* Buddy 3 only: publish its existing MC_CMD_* line-following intention.
 * NO motor actuation and NO motion_cmd_q writes. E_PAR on unknown command. */
ER vehicle_controller_line_motion_post(UB command, INT line_visible);

/* Read-only status for diagnostics; updated by VC task. */
UW vehicle_controller_state(void);    /* APP_VS_* */
UW vehicle_controller_output(void);   /* vc_output_t, NEVER actuated */
/* Phase 5A preview only, never transmitted to Buddy 2. */
UW vehicle_controller_candidate_motion_command(void);
UW vehicle_controller_safe_motion_command(void); /* always MC_CMD_HALT */
UW vehicle_controller_motion_blockers(void); /* VMA_BLOCK_* */
UW vehicle_controller_line_update_count(void); /* VC task consumed updates */
UW vehicle_controller_line_published_count(void); /* Buddy 3 posts, 23-bit modulo */
UW vehicle_controller_line_age_ms(void);
UW vehicle_controller_speed_cap_requested(void); /* cap sent, no PWM application */
UW vehicle_controller_wheel_feedback_seq(void); /* Buddy 2's 100Hz read-only sample */ /* ms since accepted intent; UINT32_MAX = none */

#endif
