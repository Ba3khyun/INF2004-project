/*
 *	test_wifi_host.c
 *	PC unit tests for Buddy 1's pure-C modules:
 *	  app_program/app_data.h            frame / message codecs
 *	  app_program/subsystem1/wifi_cmd.c command validation
 *	  app_program/subsystem1/wifi_sm.c  Wi-Fi/MQTT state machine
 *	The network, kernel objects and clock are faked, so these tests prove
 *	the LOGIC only.  Timing on the Pico W and real Wi-Fi behaviour are
 *	covered by the on-target and manual tests in ../TEST_LOG.md.
 *
 *	Build and run: ./run_host_tests.sh   (gcc, -Wall -Wextra -Werror, ASan/UBSan)
 *	Output format matches app_program/test_report.h:  [TEST] <ID> PASS|FAIL
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "app_data.h"
#include "wifi_cmd.h"
#include "wifi_sm.h"

/* ------------------------------------------------------------------ */
static int g_pass, g_fail;

#define CHECK(id, cond, desc)						\
	do {								\
		if (cond) { g_pass++; printf("[TEST] %s PASS  %s\n", id, desc); } \
		else { g_fail++; printf("[TEST] %s FAIL  %s   (%s:%d)\n",	\
					id, desc, __FILE__, __LINE__); }	\
	} while (0)

/* ===================================================================
 *  Fake system (network + kernel objects + clock)
 * =================================================================== */
#define MAXPUB	4096

static struct {
	uint32_t	now;
	bool		radio;
	WIFI_LINK	link;
	WIFI_MQTT	mqtt;
	bool		join_ok, connect_ok, sub_ok, route_ok;
	bool		auto_connect;	/* connect -> CONNECTED -> SUBSCRIBED at once */
	bool		ack;		/* TCP acks publishes immediately */
	int		pub_result;
	uint32_t	unacked, progress;
	uint32_t	skew;		/* publish stamps progress this far AFTER now */
	bool		estop;
	int		joins, leaves, connects, disconnects, subs, aborts;
	int		npub;
	uint8_t		pub[MAXPUB][TLM_FRAME_LEN];
	int		nrouted;
	APP_CMD		routed[64];
	uint16_t	b1_faults;
	APP_TELEM	snap;
	int		ops_calls;	/* to bound the work per step */
	int		link_down_events;
	uint32_t	last_detect;
} F;

static bool f_radio(void)		{ F.ops_calls++; return F.radio; }
static WIFI_LINK f_link(void)		{ F.ops_calls++; return F.link; }
static bool f_join(void)
{
	F.ops_calls++; F.joins++;
	if (F.join_ok) { F.link = WIFI_LINK_UP; }
	return F.join_ok;
}
static void f_leave(void)		{ F.ops_calls++; F.leaves++; F.link = WIFI_LINK_DOWN; }
static bool f_connect(void)
{
	F.ops_calls++; F.connects++;
	if (!F.connect_ok) { return false; }
	F.mqtt = F.auto_connect ? WIFI_MQTT_CONNECTED : WIFI_MQTT_CONNECTING;
	F.unacked = 0u;
	return true;
}
static WIFI_MQTT f_mqtt(void)		{ F.ops_calls++; return F.mqtt; }
static bool f_sub(void)
{
	F.ops_calls++; F.subs++;
	if (F.sub_ok && F.auto_connect) { F.mqtt = WIFI_MQTT_SUBSCRIBED; }
	return F.sub_ok;
}
static void f_disc(void)		{ F.ops_calls++; F.disconnects++; F.mqtt = WIFI_MQTT_IDLE; F.unacked = 0u; }
static int f_pub(const uint8_t *frame, uint32_t len)
{
	F.ops_calls++;
	if (len != TLM_FRAME_LEN) { return WIFI_PUB_BUSY; }
	if (F.pub_result != WIFI_PUB_OK) { return F.pub_result; }
	if (F.npub < MAXPUB) { memcpy(F.pub[F.npub], frame, TLM_FRAME_LEN); }
	F.npub++;
	if (F.ack) { F.progress = F.now; }
	else { if (F.unacked == 0u) { F.progress = F.now + F.skew; } F.unacked++; }
	return WIFI_PUB_OK;
}
static uint32_t f_unacked(void)		{ F.ops_calls++; return F.unacked; }
static uint32_t f_progress(void)	{ F.ops_calls++; return F.progress; }
static bool f_estop(void)		{ F.ops_calls++; return F.estop; }
static bool f_route(const APP_CMD *c)
{
	F.ops_calls++;
	if (!F.route_ok) { return false; }
	if (F.nrouted < 64) { F.routed[F.nrouted] = *c; }
	F.nrouted++;
	return true;
}
static void f_abort(void)		{ F.ops_calls++; F.aborts++; }
static void f_faults(uint16_t b1)	{ F.ops_calls++; F.b1_faults = b1; }
static void f_snap(APP_TELEM *t)
{
	F.ops_calls++;
	t->veh_state = F.snap.veh_state; t->speed_cm_s = F.snap.speed_cm_s;
	t->hump_mm = F.snap.hump_mm; t->obst_cm = F.snap.obst_cm;
	t->faults = F.snap.faults;
}
static void f_event(WIFI_EVENT ev, uint32_t a, uint32_t b)
{
	(void)a;
	if (ev == WIFI_EV_LINK_DOWN) { F.link_down_events++; F.last_detect = b; }
}

static const WIFI_OPS fake_ops = {
	f_radio, f_link, f_join, f_leave, f_connect, f_mqtt, f_sub, f_disc,
	f_pub, f_unacked, f_progress, f_estop, f_route, f_abort, f_faults,
	f_snap, f_event
};

static WIFI_SM sm;

static void fake_reset(void)
{
	memset(&F, 0, sizeof(F));
	F.now = 1000u;
	F.radio = true; F.link = WIFI_LINK_DOWN; F.mqtt = WIFI_MQTT_IDLE;
	F.join_ok = F.connect_ok = F.sub_ok = F.route_ok = true;
	F.auto_connect = true; F.ack = true; F.pub_result = WIFI_PUB_OK;
	app_telem_default(&F.snap);
	F.snap.veh_state = APP_VS_DRIVING; F.snap.speed_cm_s = 30;
	F.snap.hump_mm = 12; F.snap.obst_cm = APP_OBST_CM_NA;
	wifi_sm_init(&sm, &fake_ops, F.now);
}

/* advance the clock in 10 ms poll steps (the target's poll period) */
static void run_ms(uint32_t ms)
{
	uint32_t	t;

	for (t = 0u; t < ms; t += 10u) {
		F.now += 10u;
		wifi_sm_step(&sm, F.now);
	}
}

static void bring_up(void)
{
	fake_reset();
	run_ms(50u);
}

static void cmd_frame(uint8_t out[CMD_FRAME_LEN], uint8_t type, uint8_t seq)
{
	APP_CMD	c = { type, seq };
	(void)app_cmd_encode(&c, out);
}

static void send_cmd(uint8_t type, uint8_t seq)
{
	uint8_t	f[CMD_FRAME_LEN];
	cmd_frame(f, type, seq);
	wifi_sm_on_command(&sm, f, CMD_FRAME_LEN);
	F.now += 10u;
	wifi_sm_step(&sm, F.now);
}

static void send_raw(const uint8_t *p, uint32_t len)
{
	wifi_sm_on_command(&sm, p, len);
	F.now += 10u;
	wifi_sm_step(&sm, F.now);
}

/* ===================================================================
 *  E. Binary frame correctness
 * =================================================================== */
static bool telem_equal(const APP_TELEM *a, const APP_TELEM *b)
{
	return a->version == b->version && a->veh_state == b->veh_state &&
	       a->retry == b->retry && a->seq == b->seq &&
	       a->time_10ms == b->time_10ms && a->speed_cm_s == b->speed_cm_s &&
	       a->hump_mm == b->hump_mm && a->obst_cm == b->obst_cm &&
	       a->faults == b->faults && a->backlog == b->backlog;
}

static bool roundtrip(const APP_TELEM *t)
{
	uint8_t		f[TLM_FRAME_LEN];
	APP_TELEM	d;

	return app_telem_encode(t, f) == APP_OK &&
	       app_telem_decode(f, TLM_FRAME_LEN, &d) == APP_OK && telem_equal(t, &d);
}

static void test_frames(void)
{
	APP_TELEM	t, d;
	uint8_t		f[TLM_FRAME_LEN], g[TLM_FRAME_LEN];
	int		ok, i, bit;
	int32_t		v;

	/* H01 typical */
	app_telem_default(&t);
	t.veh_state = APP_VS_HUMP_CROSSING; t.retry = 1; t.seq = 77;
	t.time_10ms = 4321; t.speed_cm_s = 27; t.hump_mm = 18; t.obst_cm = 33;
	t.faults = FLT_WIFI_LINK | FLT_COLLISION; t.backlog = false;
	CHECK("S1-H01", roundtrip(&t), "telemetry round trip, typical values");

	/* H02 minimum values */
	app_telem_default(&t);
	t.veh_state = 0; t.retry = 0; t.seq = 0; t.time_10ms = 0;
	t.speed_cm_s = APP_SPEED_MIN; t.hump_mm = 0; t.obst_cm = 0; t.faults = 0;
	CHECK("S1-H02", roundtrip(&t), "telemetry round trip, minimum values");

	/* H03 maximum values */
	t.veh_state = APP_VS_COUNT - 1u; t.retry = APP_RETRY_MAX; t.seq = 255;
	t.time_10ms = 0xFFFFu; t.speed_cm_s = APP_SPEED_MAX;
	t.hump_mm = APP_HUMP_MM_MAX; t.obst_cm = APP_OBST_CM_MAX;
	t.faults = FLT_ALL; t.backlog = true;
	CHECK("S1-H03", roundtrip(&t), "telemetry round trip, maximum values");

	/* H04 "not available" sentinels */
	app_telem_default(&t);
	CHECK("S1-H04", roundtrip(&t) && t.speed_cm_s == APP_SPEED_NA &&
	      t.hump_mm == APP_HUMP_MM_NA && t.obst_cm == APP_OBST_CM_NA,
	      "telemetry round trip, n/a sentinels");

	/* H05 invalid values rejected before encoding, buffer untouched */
	ok = 1;
	{
		static const int16_t bad_speed[] = { 101, -101, -127, 127 };
		for (i = 0; i < 4; i++) {
			app_telem_default(&t); t.speed_cm_s = bad_speed[i];
			memset(f, 0xA5, sizeof(f));
			ok &= app_telem_encode(&t, f) == APP_E_RANGE && f[0] == 0xA5;
		}
		app_telem_default(&t); t.hump_mm = 64;  ok &= app_telem_encode(&t, f) == APP_E_RANGE;
		app_telem_default(&t); t.obst_cm = 101; ok &= app_telem_encode(&t, f) == APP_E_RANGE;
		app_telem_default(&t); t.obst_cm = 126; ok &= app_telem_encode(&t, f) == APP_E_RANGE;
		app_telem_default(&t); t.veh_state = 8; ok &= app_telem_encode(&t, f) == APP_E_RANGE;
		app_telem_default(&t); t.retry = 8;     ok &= app_telem_encode(&t, f) == APP_E_RANGE;
		app_telem_default(&t); t.faults = 0x1000u; ok &= app_telem_encode(&t, f) == APP_E_RANGE;
		app_telem_default(&t); t.version = 0;   ok &= app_telem_encode(&t, f) == APP_E_VERSION;
		app_telem_default(&t); t.version = 2;   ok &= app_telem_encode(&t, f) == APP_E_VERSION;
	}
	CHECK("S1-H05", ok, "out-of-range fields rejected before encoding");

	/* H06 signed speed two's complement */
	app_telem_default(&t); t.speed_cm_s = -1;
	ok = app_telem_encode(&t, f) == APP_OK && f[TLM_B4_SPEED] == 0xFFu;
	t.speed_cm_s = -100;
	ok &= app_telem_encode(&t, f) == APP_OK && f[TLM_B4_SPEED] == 0x9Cu &&
	      app_telem_decode(f, TLM_FRAME_LEN, &d) == APP_OK && d.speed_cm_s == -100;
	t.speed_cm_s = APP_SPEED_NA;
	ok &= app_telem_encode(&t, f) == APP_OK && f[TLM_B4_SPEED] == 0x80u;
	CHECK("S1-H06", ok, "signed speed: -1=0xFF, -100=0x9C, n/a=0x80");

	/* H07 golden vector: exact bytes, little endian */
	app_telem_default(&t);
	t.veh_state = APP_VS_DRIVING; t.retry = 2; t.seq = 0x34; t.time_10ms = 0x1234;
	t.speed_cm_s = -5; t.hump_mm = 25; t.obst_cm = 40; t.faults = 0x0A5; t.backlog = false;
	{
		static const uint8_t gold[TLM_FRAME_LEN] =
			{ 0x45, 0x34, 0x34, 0x12, 0xFB, 0x19, 0x28, 0xA5, 0x00 };
		ok = app_telem_encode(&t, f) == APP_OK && memcmp(f, gold, TLM_FRAME_LEN) == 0;
		printf("        golden frame:");
		for (i = 0; i < (int)TLM_FRAME_LEN; i++) printf(" %02X", f[i]);
		printf("\n");
	}
	CHECK("S1-H07", ok, "golden frame 45 34 34 12 FB 19 28 A5 00 (LE time/faults)");

	/* H08 every reserved bit rejected on decode */
	ok = 1;
	app_telem_default(&t);
	(void)app_telem_encode(&t, f);
	{
		static const struct { int byte; uint8_t mask; } rsvd[] = {
			{5, 0x80}, {6, 0x80}, {8, 0x10}, {8, 0x20}, {8, 0x40}, {8, 0x80} };
		for (i = 0; i < 6; i++) {
			memcpy(g, f, sizeof(g));
			g[rsvd[i].byte] |= rsvd[i].mask;
			ok &= app_telem_decode(g, TLM_FRAME_LEN, &d) == APP_E_RESERVED;
		}
	}
	CHECK("S1-H08", ok, "each of the 6 reserved bits set -> frame rejected");

	/* H09 protocol version */
	ok = 1;
	for (v = 0; v < 4; v++) {
		memcpy(g, f, sizeof(g));
		g[0] = (uint8_t)((g[0] & ~TLM_B0_VER_MASK) | (uint8_t)v);
		ok &= (app_telem_decode(g, TLM_FRAME_LEN, &d) == APP_OK) == (v == (int32_t)APP_PROTO_VERSION);
	}
	CHECK("S1-H09", ok, "only protocol version 1 decodes (0, 2, 3 rejected)");

	/* H10 invalid lengths */
	ok = app_telem_decode(f, 0, &d) == APP_E_LEN && app_telem_decode(f, 8, &d) == APP_E_LEN &&
	     app_telem_decode(f, 10, &d) == APP_E_LEN && app_telem_decode(NULL, 9, &d) == APP_E_LEN;
	CHECK("S1-H10", ok, "lengths 0, 8, 10 and NULL rejected");

	/* H12 out-of-range bytes on the wire */
	ok = 1;
	memcpy(g, f, sizeof(g)); g[TLM_B4_SPEED] = 101;  ok &= app_telem_decode(g, 9, &d) == APP_E_RANGE;
	memcpy(g, f, sizeof(g)); g[TLM_B4_SPEED] = 0x81; ok &= app_telem_decode(g, 9, &d) == APP_E_RANGE;
	memcpy(g, f, sizeof(g)); g[6] = 101;              ok &= app_telem_decode(g, 9, &d) == APP_E_RANGE;
	CHECK("S1-H12", ok, "decoder rejects speed 101/-127 and obstacle 101");

	/* H13 exhaustive consistency: encode accepts exactly what decode accepts */
	ok = 1;
	for (v = -128; v <= 127; v++) {
		app_telem_default(&t); t.speed_cm_s = (int16_t)v;
		ok &= (app_telem_encode(&t, f) == APP_OK) == app_speed_valid(v);
		if (app_speed_valid(v)) ok &= roundtrip(&t);
	}
	for (v = 0; v <= 255; v++) {
		app_telem_default(&t); t.obst_cm = (uint8_t)v;
		ok &= (app_telem_encode(&t, f) == APP_OK) == app_obst_valid((uint32_t)v);
		app_telem_default(&t); t.hump_mm = (uint8_t)v;
		ok &= (app_telem_encode(&t, f) == APP_OK) == (v <= 63);
	}
	for (bit = 0; bit < 16; bit++) {
		app_telem_default(&t); t.faults = (uint16_t)(1u << bit);
		ok &= (app_telem_encode(&t, f) == APP_OK) == (bit < (int)FLT_BITS);
		if (bit < (int)FLT_BITS) ok &= roundtrip(&t);
	}
	CHECK("S1-H13", ok, "exhaustive speed/obstacle/hump/fault-bit validity and round trip");

	/* H14 telemetry_q status message */
	{
		uint8_t		m[TQ_MSG_LEN];
		uint32_t	fld;
		int32_t		val;
		uint16_t	st;

		ok = app_tq_pack(m, TQ_FIELD_SPEED, -42, 0xBEEF) == APP_OK &&
		     app_tq_unpack(m, &fld, &val, &st) == APP_OK &&
		     fld == TQ_FIELD_SPEED && val == -42 && st == 0xBEEF &&
		     m[2] == 0xEF && m[3] == 0xBE;
		ok &= app_tq_pack(m, TQ_FIELD_OBSTACLE, 100, 1) == APP_OK &&
		      app_tq_unpack(m, &fld, &val, &st) == APP_OK && val == 100;
		ok &= app_tq_pack(m, TQ_FIELD_SPEED, 101, 0) == APP_E_RANGE;
		ok &= app_tq_pack(m, TQ_FIELD_VEH_STATE, 8, 0) == APP_E_RANGE;
		ok &= app_tq_pack(m, TQ_FIELD_HUMP, -1, 0) == APP_E_RANGE;
		ok &= app_tq_pack(m, TQ_FIELD_NONE, 0, 0) == APP_E_TYPE;
		ok &= app_tq_pack(m, 7, 0, 0) == APP_E_TYPE;
		(void)app_tq_pack(m, TQ_FIELD_HUMP, 5, 0); m[0] |= 0x08;
		ok &= app_tq_unpack(m, &fld, &val, &st) == APP_E_RESERVED;
	}
	CHECK("S1-H14", ok, "telemetry_q 4-byte message: round trip, ranges, reserved bits");

	/* H15 remote_cmd_q message */
	{
		uint8_t	m[RCMD_MSG_LEN];
		APP_CMD	c;
		ok = app_rcmd_pack(m, CMD_PAUSE, 9) == APP_OK && app_rcmd_unpack(m, &c) == APP_OK &&
		     c.type == CMD_PAUSE && c.seq == 9;
		ok &= app_rcmd_pack(m, CMD_NONE, 0) == APP_E_RANGE && app_rcmd_pack(m, 4, 0) == APP_E_RANGE;
		m[0] = 0x05; ok &= app_rcmd_unpack(m, &c) == APP_E_RANGE;
	}
	CHECK("S1-H15", ok, "remote_cmd_q 2-byte message: round trip and rejects");

	/* H16 CRC-8 check value */
	CHECK("S1-H16", app_crc8((const uint8_t *)"123456789", 9) == 0xF4u,
	      "CRC-8/SMBUS check value 0xF4");

	/* H17 command golden vectors */
	{
		uint8_t	c[CMD_FRAME_LEN];
		APP_CMD	d2;
		cmd_frame(c, CMD_ABORT, 7);
		ok = c[0] == 0x05 && c[1] == 0x07 && c[2] == app_crc8(c, 2);
		cmd_frame(c, CMD_PAUSE, 0);  ok &= c[0] == 0x09;
		cmd_frame(c, CMD_RESUME, 255); ok &= c[0] == 0x0D && c[1] == 0xFF;
		ok &= app_cmd_decode(c, 3, &d2) == APP_OK && d2.type == CMD_RESUME && d2.seq == 255;
		cmd_frame(c, CMD_ABORT, 7);
		printf("        golden abort seq 7: %02X %02X %02X\n", c[0], c[1], c[2]);
	}
	CHECK("S1-H17", ok, "command frames: abort=05, pause=09, resume=0D, CRC");
}

/* ===================================================================
 *  D. Remote command safety
 * =================================================================== */
static void test_commands(void)
{
	uint8_t	f[CMD_FRAME_LEN], big[20];
	int	r0, a0;

	/* H20 valid abort */
	bring_up();
	send_cmd(CMD_ABORT, 10);
	CHECK("S1-H20", F.aborts == 1 && F.nrouted == 1 && F.routed[0].type == CMD_ABORT &&
	      sm.state == WIFI_ST_SHARE_TELEMETRY,
	      "valid abort -> flg_safety + remote_cmd_q, back to SHARE_TELEMETRY");

	/* H21 duplicate abort */
	send_cmd(CMD_ABORT, 10);
	CHECK("S1-H21", F.aborts == 1 && F.nrouted == 1 &&
	      sm.stats.cmd_verdicts[CMD_V_DUPLICATE] == 1,
	      "duplicate abort (same seq) ignored, nothing re-posted");

	/* H22 malformed payloads */
	r0 = F.nrouted; a0 = F.aborts;
	cmd_frame(f, CMD_PAUSE, 11);
	memset(big, 0x5A, sizeof(big));
	memcpy(big, f, sizeof(f));
	send_raw(f, 2);				/* short (incomplete) */
	send_raw(big, 4);			/* valid frame + 1 byte */
	send_raw(big, sizeof(big));		/* oversize */
	f[2] ^= 0x01; send_raw(f, 3);		/* corrupted CRC */
	CHECK("S1-H22", F.nrouted == r0 && F.aborts == a0 &&
	      (F.b1_faults & FLT_CMD_MALFORMED) != 0 &&
	      sm.stats.cmd_verdicts[CMD_V_BAD_LENGTH] == 3 &&
	      sm.stats.cmd_verdicts[CMD_V_BAD_CRC] == 1,
	      "short/long/oversize/corrupted payloads ignored, FLT_CMD_MALFORMED set");

	/* H23 unknown command: type 0 with a valid CRC */
	f[0] = APP_PROTO_VERSION; f[1] = 12; f[2] = app_crc8(f, 2);
	send_raw(f, 3);
	CHECK("S1-H23", F.nrouted == r0 && sm.stats.cmd_verdicts[CMD_V_UNKNOWN] == 1,
	      "unknown command type ignored");

	/* H24 bad version / reserved bits -> protocol error */
	f[0] = (uint8_t)(2u | (CMD_PAUSE << 2)); f[1] = 13; f[2] = app_crc8(f, 2);
	send_raw(f, 3);
	f[0] = (uint8_t)(1u | (CMD_PAUSE << 2) | 0x10u); f[1] = 14; f[2] = app_crc8(f, 2);
	send_raw(f, 3);
	CHECK("S1-H24", F.nrouted == r0 && (F.b1_faults & FLT_PROTO_ERROR) != 0 &&
	      sm.stats.cmd_verdicts[CMD_V_BAD_VERSION] == 1 &&
	      sm.stats.cmd_verdicts[CMD_V_BAD_RESERVED] == 1,
	      "version 2 and reserved bit rejected, FLT_PROTO_ERROR set");

	/* H25 invalid sequence */
	bring_up();
	send_cmd(CMD_RESUME, 5);			/* first after boot */
	CHECK("S1-H25a", F.nrouted == 0 && sm.stats.cmd_verdicts[CMD_V_STALE] == 1,
	      "RESUME as first command after boot rejected (no baseline)");
	send_cmd(CMD_PAUSE, 100);
	send_cmd(CMD_RESUME, 90);			/* behind the window */
	CHECK("S1-H25b", F.nrouted == 1 && F.routed[0].type == CMD_PAUSE &&
	      sm.stats.cmd_verdicts[CMD_V_STALE] == 2,
	      "RESUME with an older sequence rejected");
	send_cmd(CMD_PAUSE, 50);			/* older, but fail-safe */
	CHECK("S1-H25c", F.nrouted == 2 && F.routed[1].type == CMD_PAUSE,
	      "older PAUSE still accepted (fail-safe direction)");
	send_cmd(CMD_ABORT, 255);
	send_cmd(CMD_PAUSE, 0);				/* wraps 255 -> 0: newer */
	CHECK("S1-H25d", F.nrouted == 4 && F.routed[3].seq == 0,
	      "sequence wrap 255 -> 0 treated as newer");

	/* H26 pause routed */
	bring_up();
	send_cmd(CMD_PAUSE, 1);
	CHECK("S1-H26", F.nrouted == 1 && F.routed[0].type == CMD_PAUSE && F.aborts == 0,
	      "pause -> remote_cmd_q only (no Safety flag)");

	/* H27 resume while ESTOP latched */
	F.estop = true;
	send_cmd(CMD_RESUME, 2);
	CHECK("S1-H27", F.nrouted == 1 && sm.stats.cmd_verdicts[CMD_V_ESTOP_LATCHED] == 1,
	      "resume rejected while local ESTOP is latched");

	/* H28 resume after authorised local reset (Safety clears the latch) */
	F.estop = false;
	send_cmd(CMD_RESUME, 2);			/* same frame resent */
	CHECK("S1-H28", F.nrouted == 2 && F.routed[1].type == CMD_RESUME,
	      "same resume accepted after the local reset");

	/* H29 Wi-Fi reconnect does not touch ESTOP and does not replay */
	F.estop = true;
	F.link = WIFI_LINK_DOWN;
	run_ms(6000u);
	F.estop = true;
	CHECK("S1-H29", sm.state == WIFI_ST_SHARE_TELEMETRY && F.nrouted == 2 && F.aborts == 0,
	      "reconnect with ESTOP latched: no command replayed, nothing routed");

	/* H30 abort and link loss in the same step -> not a dead end */
	bring_up();
	wifi_sm_on_command(&sm, (cmd_frame(f, CMD_ABORT, 3), f), 3);
	F.link = WIFI_LINK_DOWN;
	F.now += 10u; wifi_sm_step(&sm, F.now);
	CHECK("S1-H30", F.aborts == 1 && sm.state == WIFI_ST_RECONNECTING,
	      "abort processed even as the link drops, then RECONNECTING");

	/* H31 abort while RECONNECTING keeps the back-off timer */
	{
		uint32_t	t_enter = sm.t_enter;
		wifi_sm_on_command(&sm, (cmd_frame(f, CMD_ABORT, 4), f), 3);
		F.now += 10u; wifi_sm_step(&sm, F.now);
		CHECK("S1-H31", F.aborts == 2 && sm.state == WIFI_ST_RECONNECTING &&
		      sm.t_enter == t_enter, "abort in RECONNECTING returns to RECONNECTING");
	}

	/* H32 remote_cmd_q full: Safety flag still raised */
	bring_up();
	F.route_ok = false;
	send_cmd(CMD_ABORT, 1);
	CHECK("S1-H32", F.aborts == 1 && sm.stats.route_fail == 1,
	      "remote_cmd_q full: abort still reaches flg_safety");

	/* H33 report-once fault cleared after it was published */
	bring_up();
	send_raw(f, 1);
	{
		int	seen = 0, n0 = F.npub, i;
		APP_TELEM	d;
		run_ms(300u);
		for (i = n0; i < F.npub && i < MAXPUB; i++) {
			if (app_telem_decode(F.pub[i], 9, &d) == APP_OK &&
			    (d.faults & FLT_CMD_MALFORMED)) seen++;
		}
		CHECK("S1-H33", seen == 1 && (F.b1_faults & FLT_CMD_MALFORMED) == 0,
		      "FLT_CMD_MALFORMED reported in exactly one frame, then cleared");
	}

	/* H34 ESTOP bit in telemetry comes from the snapshot only */
	bring_up();
	F.snap.faults = FLT_ESTOP_LATCHED;
	run_ms(200u);
	{
		APP_TELEM	d;
		CHECK("S1-H34", app_telem_decode(F.pub[F.npub - 1], 9, &d) == APP_OK &&
		      (d.faults & FLT_ESTOP_LATCHED) && (F.b1_faults & FLT_ESTOP_LATCHED) == 0,
		      "ESTOP shown in telemetry, never written by B1");
	}
}

/* ===================================================================
 *  State machine, link loss, reconnect, telemetry queue/backlog
 * =================================================================== */
static void test_state_machine(void)
{
	int		n0, ok, trial;
	uint32_t	t0, worst = 0u;
	APP_TELEM	d;

	/* H40 start-up path and 10 Hz */
	fake_reset();
	ok = sm.state == WIFI_ST_DISCONNECTED;
	F.now += 10u; wifi_sm_step(&sm, F.now);
	ok &= sm.state == WIFI_ST_CONNECTING_WIFI && F.joins == 1;
	F.now += 10u; wifi_sm_step(&sm, F.now);
	ok &= sm.state == WIFI_ST_CONNECTING_MQTT && F.connects == 1 && F.subs == 1;
	F.now += 10u; wifi_sm_step(&sm, F.now);
	/* live frame + the one frame sampled while still offline (backlog) */
	ok &= sm.state == WIFI_ST_SHARE_TELEMETRY && F.npub == 2 && sm.bl_count == 0;
	n0 = F.npub;
	run_ms(10000u);
	CHECK("S1-H40", ok && F.npub - n0 >= 99 && F.npub - n0 <= 101 &&
	      (F.b1_faults & FLT_B1_LINK) == 0,
	      "DISCONNECTED->CONNECTING_WIFI->CONNECTING_MQTT->SHARE; 100 frames in 10 s");

	/* H41 link-loss detection by TCP progress, 10 trials at different phases */
	ok = 1;
	for (trial = 0; trial < 10; trial++) {
		bring_up();
		run_ms(1000u + (uint32_t)trial * 37u);
		F.ack = false;			/* AP switched off: nothing acked */
		t0 = F.now;
		while (sm.state == WIFI_ST_SHARE_TELEMETRY && F.now - t0 < 5000u) {
			F.now += 10u; wifi_sm_step(&sm, F.now);
		}
		if (F.now - t0 > worst) worst = F.now - t0;
		ok &= sm.state == WIFI_ST_RECONNECTING && (F.b1_faults & FLT_SOCK_TIMEOUT) &&
		      F.last_detect <= 2000u;
	}
	printf("        worst simulated detection: %u ms (poll 10 ms)\n", (unsigned)worst);
	CHECK("S1-H41", ok && worst <= 2000u, "silent TCP loss detected <= 2 s in 10/10 trials");

	/* H42 Wi-Fi association lost */
	bring_up();
	F.link = WIFI_LINK_DOWN;
	F.now += 10u; wifi_sm_step(&sm, F.now);
	CHECK("S1-H42", sm.state == WIFI_ST_RECONNECTING && (F.b1_faults & FLT_WIFI_LINK) &&
	      F.leaves == 1 && F.disconnects >= 1, "Wi-Fi loss -> RECONNECTING at once, leave + disconnect");

	/* H43 MQTT broker loss */
	bring_up();
	F.mqtt = WIFI_MQTT_FAILED;
	F.now += 10u; wifi_sm_step(&sm, F.now);
	CHECK("S1-H43", sm.state == WIFI_ST_RECONNECTING && (F.b1_faults & FLT_MQTT_LINK),
	      "MQTT loss -> RECONNECTING, FLT_MQTT_LINK");

	/* H44 bounded back-off, saturating retry, bounded work per step */
	bring_up();
	F.join_ok = false;
	F.link = WIFI_LINK_DOWN;
	{
		uint32_t	seen[10];
		int		k = 0, max_calls = 0;
		uint32_t	last_enter = sm.t_enter;

		while (k < 10 && F.now < 200000u) {
			F.ops_calls = 0;
			F.now += 10u; wifi_sm_step(&sm, F.now);
			if (F.ops_calls > max_calls) max_calls = F.ops_calls;
			/* each new entry into RECONNECTING restarts t_enter */
			if (sm.state == WIFI_ST_RECONNECTING && sm.t_enter != last_enter) {
				seen[k++] = sm.backoff_ms;
			}
			last_enter = sm.t_enter;
		}
		ok = k == 10 && seen[0] == 500u && seen[1] == 1000u && seen[2] == 2000u &&
		     seen[3] == 4000u && seen[9] == 4000u && sm.retry == APP_RETRY_MAX &&
		     max_calls <= 40;
		printf("        back-off ms: %u %u %u %u %u ... retry=%u, max ops/step=%d\n",
		       (unsigned)seen[0], (unsigned)seen[1], (unsigned)seen[2],
		       (unsigned)seen[3], (unsigned)seen[4], (unsigned)sm.retry, max_calls);
	}
	CHECK("S1-H44", ok, "back-off 0.5/1/2/4 s capped, retry saturates at 7, bounded work per step");

	/* H45 recovery without reset */
	F.join_ok = true;
	t0 = F.now;
	while (sm.state != WIFI_ST_SHARE_TELEMETRY && F.now - t0 < 10000u) {
		F.now += 10u; wifi_sm_step(&sm, F.now);
	}
	n0 = F.npub;
	run_ms(1000u);
	CHECK("S1-H45", sm.state == WIFI_ST_SHARE_TELEMETRY && sm.retry == 0 &&
	      (F.b1_faults & FLT_B1_LINK) == 0 && F.npub > n0,
	      "AP restored -> SHARE_TELEMETRY, faults cleared, publishing resumes");

	/* H46 30 s outage: backlog bounded, drop-oldest, flush on recovery */
	bring_up();
	run_ms(1000u);
	{
		uint32_t	flushed0 = sm.stats.backlog_flushed;
	F.link = WIFI_LINK_DOWN; F.join_ok = false;
	run_ms(30000u);
	ok = sm.bl_count == WIFI_BACKLOG_DEPTH && sm.stats.backlog_max == WIFI_BACKLOG_DEPTH &&
	     sm.stats.backlog_dropped > 250u && (F.b1_faults & FLT_TELEM_OVERFLOW);
	{
		/* the backlog holds the NEWEST 16 frames, consecutive sequence */
		uint32_t	i;
		uint8_t		first;
		(void)app_telem_decode(sm.bl[sm.bl_head], 9, &d);
		first = d.seq;
		for (i = 0u; i < WIFI_BACKLOG_DEPTH; i++) {
			ok &= app_telem_decode(sm.bl[(sm.bl_head + i) % WIFI_BACKLOG_DEPTH], 9, &d) == APP_OK &&
			      d.backlog && d.seq == (uint8_t)(first + i);
		}
		ok &= (uint8_t)(sm.tlm_seq - 1u) == (uint8_t)(first + WIFI_BACKLOG_DEPTH - 1u);
	}
	F.join_ok = true;
	run_ms(WIFI_BACKOFF_MAX_MS + 2000u);
	CHECK("S1-H46", ok && sm.bl_count == 0 &&
	      sm.stats.backlog_flushed - flushed0 == WIFI_BACKLOG_DEPTH &&
	      (F.b1_faults & FLT_TELEM_OVERFLOW) == 0,
	      "30 s outage: backlog stays at 16 (newest kept), flushed after recovery");
	}
	printf("        sizeof(WIFI_SM) = %u bytes (static, constant during outage)\n",
	       (unsigned)sizeof(WIFI_SM));

	/* H47 5 min at 10 Hz with a 30 s outage */
	fake_reset();			/* F.npub and the stats start at 0 */
	run_ms(120000u);
	F.link = WIFI_LINK_DOWN; F.join_ok = false;
	run_ms(30000u);
	F.join_ok = true;
	run_ms(150000u);
	printf("        5 min soak: published=%d built=%u dropped=%u flushed=%u recovery=%u ms\n",
	       F.npub, (unsigned)sm.stats.frames_built, (unsigned)sm.stats.backlog_dropped,
	       (unsigned)sm.stats.backlog_flushed, (unsigned)sm.stats.max_recovery_ms);
	CHECK("S1-H47", sm.stats.frames_built >= 2995u && sm.stats.frames_built <= 3005u &&
	      (uint32_t)F.npub + sm.stats.backlog_dropped == sm.stats.frames_built &&
	      sm.stats.backlog_max == WIFI_BACKLOG_DEPTH,
	      "5 min soak: every frame published or counted as dropped, backlog <= 16");

	/* H48 publish busy -> backlog, stay connected */
	bring_up();
	F.pub_result = WIFI_PUB_BUSY;
	run_ms(300u);
	ok = sm.state == WIFI_ST_SHARE_TELEMETRY && sm.bl_count >= 2 && sm.stats.publish_busy >= 2;
	F.pub_result = WIFI_PUB_OK;
	run_ms(300u);
	CHECK("S1-H48", ok && sm.bl_count == 0, "publish busy: frames kept in backlog, sent later");

	/* H49 join timeout and FAIL grace */
	fake_reset();
	F.join_ok = false;
	F.link = WIFI_LINK_JOINING;
	F.now += 10u; wifi_sm_step(&sm, F.now);
	ok = sm.state == WIFI_ST_CONNECTING_WIFI;
	run_ms(WIFI_JOIN_TIMEOUT_MS);
	CHECK("S1-H49", ok && sm.state == WIFI_ST_RECONNECTING && (F.b1_faults & FLT_WIFI_LINK),
	      "join that never completes times out after 15 s");

	/* H50 MQTT connect timeout */
	fake_reset();
	F.auto_connect = false;
	F.now += 10u; wifi_sm_step(&sm, F.now);	/* -> CONNECTING_WIFI, join */
	F.now += 10u; wifi_sm_step(&sm, F.now);	/* -> CONNECTING_MQTT */
	ok = sm.state == WIFI_ST_CONNECTING_MQTT;
	run_ms(WIFI_MQTT_TIMEOUT_MS);
	CHECK("S1-H50", ok && sm.state == WIFI_ST_RECONNECTING && (F.b1_faults & FLT_MQTT_LINK),
	      "broker that never answers times out after 5 s");

	/* H51 overrun: one late frame, no burst */
	bring_up();
	n0 = F.npub;
	F.now += 350u; wifi_sm_step(&sm, F.now);
	CHECK("S1-H51", F.npub - n0 == 1 && sm.stats.tick_overruns == 1,
	      "350 ms starvation -> 1 frame, not a burst of 3");

	/* H52 clock skew: on target, now_ms() is sampled at the start of
	   wifi_app_poll() and mqtt_utk_publish() stamps sys_now() later, so a
	   tick (or preemption) in between makes progress > now.  That must not
	   look like a 49-day-old ack (false SOCK_TIMEOUT). */
	bring_up();
	F.ack = false;
	F.skew = 10u;
	{
		uint32_t	t;
		int		downs0 = F.link_down_events;

		for (t = 0u; t < 3000u; t += 10u) {
			F.now += 10u;
			if (F.unacked != 0u && F.now - F.progress < 0x80000000u) {
				F.unacked = 0u;		/* TCP ack arrives in the next poll */
				F.progress = F.now;
			}
			wifi_sm_step(&sm, F.now);
		}
		CHECK("S1-H52", sm.state == WIFI_ST_SHARE_TELEMETRY &&
		      F.link_down_events == downs0 && sm.stats.max_detect_ms <= 2000u,
		      "progress stamped after 'now' (tick between samples): no false SOCK_TIMEOUT");
	}

	/* H11 telemetry sequence wraps 255 -> 0 */
	bring_up();
	run_ms(26000u);
	ok = 0;
	{
		int	i;
		for (i = 1; i < F.npub && i < MAXPUB; i++) {
			APP_TELEM	a, b;
			if (app_telem_decode(F.pub[i - 1], 9, &a) == APP_OK &&
			    app_telem_decode(F.pub[i], 9, &b) == APP_OK &&
			    a.seq == 255 && b.seq == 0) ok = 1;
		}
	}
	CHECK("S1-H11", ok, "telemetry sequence wraps 255 -> 0");
}

/* ===================================================================
 *  F. Message size and RAM comparison (measurements, printed)
 * =================================================================== */
static void test_sizes(void)
{
	char		json[256];
	int		jt_typ, jt_max, jc_typ, jc_max;
	unsigned	topic_s = (unsigned)strlen("robot/is06/state");
	unsigned	topic_c = (unsigned)strlen("robot/is06/cmd");

	/* Week 6 JSON payloads (Design Review 6.1) */
	jt_typ = snprintf(json, sizeof(json),
		"{\"seq\":%u,\"t_ms\":%u,\"state\":\"%s\",\"speed_cm_s\":%d,"
		"\"hump_mm\":%d,\"obs_w_mm\":%d,\"fault\":%u}",
		1234u, 123456u, "DRIVING", 27, 18, 330, 0u);
	jt_max = snprintf(json, sizeof(json),
		"{\"seq\":%u,\"t_ms\":%u,\"state\":\"%s\",\"speed_cm_s\":%d,"
		"\"hump_mm\":%d,\"obs_w_mm\":%d,\"fault\":%u}",
		4294967295u, 4294967295u, "MISSION_COMPLETE", -100, 62, 1000, 4095u);
	jc_typ = snprintf(json, sizeof(json), "{\"seq\":%u,\"cmd\":\"%s\"}", 12u, "abort");
	jc_max = snprintf(json, sizeof(json), "{\"seq\":%u,\"cmd\":\"%s\"}", 4294967295u, "resume");

	printf("        telemetry payload: JSON %d B typical / %d B worst  vs binary %u B\n",
	       jt_typ, jt_max, (unsigned)TLM_FRAME_LEN);
	printf("        command payload:   JSON %d B typical / %d B worst  vs binary %u B\n",
	       jc_typ, jc_max, (unsigned)CMD_FRAME_LEN);
	/* MQTT 3.1.1 PUBLISH QoS 0: 1 type + 1 remaining-length (<128) + 2 topic len + topic + payload */
	printf("        MQTT PUBLISH on the wire: JSON %u B / binary %u B per frame;"
	       " at 10 Hz %u B/s vs %u B/s\n",
	       (unsigned)(4u + topic_s + (unsigned)jt_typ), 4u + topic_s + TLM_FRAME_LEN,
	       10u * (4u + topic_s + (unsigned)jt_typ), 10u * (4u + topic_s + TLM_FRAME_LEN));
	printf("        MQTT command on the wire: JSON %u B / binary %u B\n",
	       4u + topic_c + (unsigned)jc_typ, 4u + topic_c + CMD_FRAME_LEN);
	printf("        telemetry_q element: 4 B packed (8 B in the kernel buffer) vs 32 B for"
	       " 8 x 32-bit fields (36 B in the buffer)\n");
	printf("        telemetry_q RAM (16 deep): %u B vs %u B;  backlog 16 x 9 = %u B vs 16 x %d = %u B JSON\n",
	       16u * 8u, 16u * 36u, 16u * TLM_FRAME_LEN, jt_max, 16u * (unsigned)jt_max);

	CHECK("S1-H60", jt_typ > (int)TLM_FRAME_LEN * 10 && jc_typ > (int)CMD_FRAME_LEN * 5,
	      "binary telemetry/command payloads are >10x / >5x smaller than JSON");
}

int main(void)
{
	test_frames();
	test_commands();
	test_state_machine();
	test_sizes();
	printf("[TEST] subsystem1-host SUMMARY %d passed, %d failed\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
