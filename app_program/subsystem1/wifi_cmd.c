/*
 *	wifi_cmd.c
 *	Subsystem 1 (Buddy 1) -- remote command validation (see wifi_cmd.h).
 */

#include "wifi_cmd.h"

void wifi_cmd_filter_init(WIFI_CMD_FILTER *f)
{
	f->have_seq = false;
	f->last_seq = 0u;
}

WIFI_CMD_VERDICT wifi_cmd_check(WIFI_CMD_FILTER *f, const uint8_t *buf,
				uint32_t len, bool estop_latched, APP_CMD *out)
{
	APP_CMD	c;
	uint8_t	d;

	switch (app_cmd_decode(buf, len, &c)) {
	case APP_OK:		break;
	case APP_E_LEN:		return CMD_V_BAD_LENGTH;
	case APP_E_CRC:		return CMD_V_BAD_CRC;
	case APP_E_VERSION:	return CMD_V_BAD_VERSION;
	case APP_E_RESERVED:	return CMD_V_BAD_RESERVED;
	default:		return CMD_V_UNKNOWN;
	}
	*out = c;

	if (f->have_seq) {
		d = (uint8_t)(c.seq - f->last_seq);	/* modulo 256 */
		if (d == 0u) {
			return CMD_V_DUPLICATE;
		}
		if (d >= 128u && c.type == CMD_RESUME) {
			return CMD_V_STALE;
		}
	} else if (c.type == CMD_RESUME) {
		return CMD_V_STALE;		/* no baseline after boot */
	}

	if (c.type == CMD_RESUME && estop_latched) {
		return CMD_V_ESTOP_LATCHED;	/* filter unchanged: may be resent */
	}

	f->have_seq = true;
	f->last_seq = c.seq;
	return CMD_V_ACCEPT;
}

uint16_t wifi_cmd_verdict_fault(WIFI_CMD_VERDICT v)
{
	switch (v) {
	case CMD_V_BAD_LENGTH:
	case CMD_V_BAD_CRC:
	case CMD_V_UNKNOWN:
		return FLT_CMD_MALFORMED;
	case CMD_V_BAD_VERSION:
	case CMD_V_BAD_RESERVED:
		return FLT_PROTO_ERROR;
	default:
		return 0u;	/* accepted, duplicate, stale, ESTOP: no fault */
	}
}

const char *wifi_cmd_verdict_name(WIFI_CMD_VERDICT v)
{
	static const char *const names[CMD_V_COUNT] = {
		"ACCEPT", "BAD_LENGTH", "BAD_CRC", "BAD_VERSION",
		"BAD_RESERVED", "UNKNOWN", "DUPLICATE", "STALE",
		"ESTOP_LATCHED"
	};
	return ((unsigned)v < (unsigned)CMD_V_COUNT) ? names[v] : "?";
}
