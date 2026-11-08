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

#endif
