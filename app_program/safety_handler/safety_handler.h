#ifndef SAFETY_HANDLER_H
#define SAFETY_HANDLER_H

#include "../app.h"

/* Phase 4A: passive RTOS observer. Does not set/clear shared flags or motor commands. */
ER safety_handler_init(void);
ER safety_handler_start(void);
UW safety_handler_would_latch(void);
UW safety_handler_reasons(void);

#endif
