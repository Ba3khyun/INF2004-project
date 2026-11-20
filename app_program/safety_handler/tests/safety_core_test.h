#ifndef SAFETY_VALIDATION_H
#define SAFETY_VALIDATION_H
/* Independent software-only test. Returns number of failed assertions.
 * No RTOS calls, no shared flags, no motor interaction. */
int safety_validation_run(void);
#endif
