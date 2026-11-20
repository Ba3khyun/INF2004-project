#ifndef HUMP_DRIVE_CONFIG_H
#define HUMP_DRIVE_CONFIG_H

/* Team integration settings. Set each after its physical verification. */
#define VC5A_B5_SOURCE_VERIFIED 0
#define VC5A_LIVE_STOP_VERIFIED 0
#define VC5A_MOTOR_HANDOFF_APPROVED 0
#if !APP_TEST && VC5A_B5_SOURCE_VERIFIED && VC5A_LIVE_STOP_VERIFIED && VC5A_MOTOR_HANDOFF_APPROVED
#define VC_POWERED_HANDOFF 1
#else
#define VC_POWERED_HANDOFF 0
#endif
#endif
