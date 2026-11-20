#ifndef HUMP_DRIVE_CONFIG_H
#define HUMP_DRIVE_CONFIG_H

/* Team integration settings.
 *
 * Normal builds: the Vehicle Controller is the ONLY writer of Buddy 2's
 * motion_cmd_q (powered handoff), and the Safety Handler asserts the live
 * shared ESTOP (safety_handler.c).  Buddy 3 publishes line intents to the
 * VC; Buddy 1 and Buddy 5 feed it remote commands and obstacle events.
 *
 * Test builds (APP_TEST): the on-target self-tests drive motion_cmd_q and
 * flg_safety themselves, so the VC stays passive (preview only) and these
 * gates read 0, exactly as the readiness tests expect. */
#if APP_TEST
#define VC5A_B5_SOURCE_VERIFIED 0
#define VC5A_LIVE_STOP_VERIFIED 0
#define VC5A_MOTOR_HANDOFF_APPROVED 0
#else
#define VC5A_B5_SOURCE_VERIFIED 1	/* S5 IF-06/07 consumed by the VC */
#define VC5A_LIVE_STOP_VERIFIED 1	/* Safety Handler asserts SAFETY_ESTOP_LATCHED */
#define VC5A_MOTOR_HANDOFF_APPROVED 1	/* VC is the sole motion_cmd_q writer */
#endif

#if !APP_TEST && VC5A_B5_SOURCE_VERIFIED && VC5A_LIVE_STOP_VERIFIED && VC5A_MOTOR_HANDOFF_APPROVED
#define VC_POWERED_HANDOFF 1
#else
#define VC_POWERED_HANDOFF 0
#endif
#endif
