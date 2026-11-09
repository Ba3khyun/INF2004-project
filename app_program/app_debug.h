/* Application diagnostic controls — v6 integration cleanup.
 * Change APP_DEBUG_VERBOSE to 1 for periodic serial status messages.
 * Individual modules can be enabled independently below.
 * IMPORTANT: debug flags do NOT change safety/motor gates or APP_TEST.
 * Fault/error messages and one-shot PASS/FAIL summaries stay visible.
 */
#ifndef APP_DEBUG_H
#define APP_DEBUG_H

#ifndef APP_DEBUG_VERBOSE
#define APP_DEBUG_VERBOSE 0
#endif

#ifndef APP_DEBUG_IMU
#define APP_DEBUG_IMU APP_DEBUG_VERBOSE
#endif
#ifndef APP_DEBUG_VC
#define APP_DEBUG_VC APP_DEBUG_VERBOSE
#endif
#ifndef APP_DEBUG_SAFETY
#define APP_DEBUG_SAFETY APP_DEBUG_VERBOSE
#endif
#ifndef APP_DEBUG_S3_SENSOR
#define APP_DEBUG_S3_SENSOR APP_DEBUG_VERBOSE
#endif
#ifndef APP_DEBUG_S3_STEERING
#define APP_DEBUG_S3_STEERING APP_DEBUG_VERBOSE
#endif
#ifndef APP_DEBUG_B2_COMMAND
#define APP_DEBUG_B2_COMMAND APP_DEBUG_VERBOSE
#endif
#ifndef APP_DEBUG_TEST_DETAIL
#define APP_DEBUG_TEST_DETAIL APP_DEBUG_VERBOSE
#endif
/* A separate explicit opt-in: adds physical I2C reads and console traffic. */
#ifndef APP_DEBUG_IMU_EXTRA_I2C
#define APP_DEBUG_IMU_EXTRA_I2C 0
#endif

#endif /* APP_DEBUG_H */
