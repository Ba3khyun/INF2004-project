/*
 *	wifi_cmd.h
 *	Subsystem 1 (Buddy 1) -- remote command validation.  Pure C, no kernel
 *	calls, so it is unit-tested on a PC (testing/subsystem1/host/).
 *
 *	A command is ACCEPTED only if every check passes, in this order:
 *	  1. length == CMD_FRAME_LEN            else CMD_V_BAD_LENGTH
 *	  2. CRC-8 matches                      else CMD_V_BAD_CRC
 *	  3. version == APP_PROTO_VERSION       else CMD_V_BAD_VERSION
 *	  4. reserved bits are zero             else CMD_V_BAD_RESERVED
 *	  5. command type is 1..3               else CMD_V_UNKNOWN
 *	  6. sequence rules (below)             else CMD_V_DUPLICATE / CMD_V_STALE
 *	  7. RESUME only if ESTOP is clear      else CMD_V_ESTOP_LATCHED
 *
 *	Sequence rules (8-bit serial-number arithmetic, d = seq - last):
 *	  d == 0          duplicate              -> ignored (any command)
 *	  d in 1..127     newer                  -> accepted
 *	  d in 128..255   older / out of window  -> ABORT and PAUSE still
 *	                  accepted (fail-safe direction) and re-baseline the
 *	                  window; RESUME rejected (CMD_V_STALE)
 *	  no baseline yet (after boot): ABORT/PAUSE accepted and set the
 *	                  baseline; RESUME rejected (CMD_V_STALE), so a
 *	                  retained or replayed "resume" can never be the first
 *	                  command a freshly booted robot obeys.
 *	A rejected command never changes the filter state.
 */

#ifndef WIFI_CMD_H
#define WIFI_CMD_H

#include "../app_data.h"

typedef enum {
	CMD_V_ACCEPT = 0,
	CMD_V_BAD_LENGTH,
	CMD_V_BAD_CRC,
	CMD_V_BAD_VERSION,
	CMD_V_BAD_RESERVED,
	CMD_V_UNKNOWN,
	CMD_V_DUPLICATE,
	CMD_V_STALE,
	CMD_V_ESTOP_LATCHED,
	CMD_V_COUNT
} WIFI_CMD_VERDICT;

typedef struct {
	bool	have_seq;	/* a baseline sequence exists */
	uint8_t	last_seq;	/* sequence of the last accepted command */
} WIFI_CMD_FILTER;

void wifi_cmd_filter_init(WIFI_CMD_FILTER *f);

/* Validate one received payload.  On CMD_V_ACCEPT *out holds the command.
   *out is also filled (best effort) for sequence/ESTOP rejections so the
   caller can log which command was refused. */
WIFI_CMD_VERDICT wifi_cmd_check(WIFI_CMD_FILTER *f, const uint8_t *buf,
				uint32_t len, bool estop_latched, APP_CMD *out);

/* Fault bit (FLT_*) a verdict raises, or 0 if it raises none */
uint16_t wifi_cmd_verdict_fault(WIFI_CMD_VERDICT v);

const char *wifi_cmd_verdict_name(WIFI_CMD_VERDICT v);

#endif /* WIFI_CMD_H */
