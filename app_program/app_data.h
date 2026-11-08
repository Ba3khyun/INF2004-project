/*
 *	app_data.h
 *	Central data-representation header shared by ALL buddies (IS-06).
 *	Changed only by team agreement (same rule as app.h).
 *
 *	What lives here
 *	  - valid ranges, field widths, masks and shifts for every compact
 *	    field that crosses a task or network boundary
 *	  - the binary MQTT telemetry frame   (robot/<id>/state, 9 bytes)
 *	  - the binary MQTT command frame     (robot/<id>/cmd,   3 bytes)
 *	  - the telemetry_q status message    (4 bytes)
 *	  - the remote_cmd_q message          (2 bytes)
 *	  - shared fault bits and Safety request bits (event-flag patterns)
 *	  - inline pack/unpack accessors that validate ranges
 *
 *	Rules (Tribunal 1 decisions, see docs/buddy1_wifi_telemetry.md)
 *	  - Network frames and queue messages are built ONLY with the
 *	    accessors below: explicit masks and shifts, byte by byte, little
 *	    endian.  Never cast a byte buffer to a struct pointer (RP2040 has
 *	    no unaligned access), never rely on C bit-field ordering.
 *	  - Shared fault bits are changed ONLY with tk_set_flg / tk_clr_flg on
 *	    the shared event flag, never with "mask |= bit" on a variable.
 *	  - Every value is range-checked before encoding.  Out-of-range values
 *	    are rejected (APP_E_RANGE), never silently truncated.
 *
 *	This header is pure C99/C11 (stdint/stdbool only) so the codecs can be
 *	unit-tested on a PC: testing/subsystem1/host/.
 */

#ifndef APP_DATA_H
#define APP_DATA_H

#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------
 *  Generic helpers
 * --------------------------------------------------------------------- */

/* Return codes of the pack/unpack accessors */
#define APP_OK			0
#define APP_E_LEN		(-1)	/* wrong frame length */
#define APP_E_VERSION		(-2)	/* unsupported protocol version */
#define APP_E_RESERVED		(-3)	/* a reserved bit is not zero */
#define APP_E_RANGE		(-4)	/* a field is outside its valid range */
#define APP_E_TYPE		(-5)	/* unknown command / field identifier */
#define APP_E_CRC		(-6)	/* integrity check failed */

/* Minimum number of bits that can hold n distinct codes (n <= 65536).
   Used by the _Static_asserts below to PROVE each width is minimal. */
#define APP_BITS_FOR(n)							\
	(((n) <= 2u) ? 1u : ((n) <= 4u) ? 2u : ((n) <= 8u) ? 3u :	\
	 ((n) <= 16u) ? 4u : ((n) <= 32u) ? 5u : ((n) <= 64u) ? 6u :	\
	 ((n) <= 128u) ? 7u : ((n) <= 256u) ? 8u : ((n) <= 512u) ? 9u :	\
	 ((n) <= 1024u) ? 10u : ((n) <= 2048u) ? 11u :			\
	 ((n) <= 4096u) ? 12u : ((n) <= 8192u) ? 13u :			\
	 ((n) <= 16384u) ? 14u : ((n) <= 32768u) ? 15u : 16u)

/* Mask of the low w bits */
#define APP_MASK(w)		((uint32_t)((1ul << (w)) - 1ul))

static inline void app_put_u16le(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static inline uint16_t app_get_u16le(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}

/* Two's-complement int8 <-> byte without implementation-defined casts */
static inline uint8_t app_i8_to_byte(int32_t v)
{
	return (uint8_t)((uint32_t)v & 0xFFu);
}

static inline int32_t app_byte_to_i8(uint8_t b)
{
	return (b >= 0x80u) ? ((int32_t)b - 256) : (int32_t)b;
}

/* ---------------------------------------------------------------------
 *  Protocol version (telemetry and command frames)
 *  2 bits: 0 = invalid (an all-zero/erased frame is rejected), 1..3 usable.
 * --------------------------------------------------------------------- */
#define APP_PROTO_VERSION	1u
#define APP_PROTO_VER_BITS	2u
#define APP_PROTO_VER_CODES	4u	/* 0 invalid + versions 1..3 */
_Static_assert(APP_BITS_FOR(APP_PROTO_VER_CODES) == APP_PROTO_VER_BITS,
	       "protocol version width is not minimal");
_Static_assert(APP_PROTO_VERSION >= 1u &&
	       APP_PROTO_VERSION <= APP_MASK(APP_PROTO_VER_BITS),
	       "protocol version does not fit");

/* ---------------------------------------------------------------------
 *  Field definitions (range, width, units).  Owner = producer task.
 * --------------------------------------------------------------------- */

/* Vehicle (mission) state -- owner: Vehicle Controller (Buddy 4).
   8 states of the system statechart -> 3 bits, all codes used. */
#define APP_VS_INITIALIZING	0u
#define APP_VS_DRIVING		1u	/* following line / barcode / instruction */
#define APP_VS_HUMP_CROSSING	2u
#define APP_VS_OBSTACLE_BYPASS	3u
#define APP_VS_LINE_SEARCH	4u
#define APP_VS_PAUSED		5u
#define APP_VS_ERROR		6u	/* safe stopped, ESTOP latched */
#define APP_VS_MISSION_COMPLETE	7u
#define APP_VS_COUNT		8u
#define APP_VS_BITS		3u
_Static_assert(APP_BITS_FOR(APP_VS_COUNT) == APP_VS_BITS, "vehicle state width");

/* Speed -- owner: Motion Control (Buddy 2).  Signed cm/s, 1 cm/s per LSB.
   Valid -100..+100 (negative = reverse, e.g. RECOVERY_REWIND).
   INT8_MIN (-128) = "not available".  202 codes -> 8 bits, two's complement. */
#define APP_SPEED_MIN		(-100)
#define APP_SPEED_MAX		100
#define APP_SPEED_NA		(-128)
#define APP_SPEED_BITS		8u
#define APP_SPEED_CODES		((uint32_t)(APP_SPEED_MAX - APP_SPEED_MIN + 1) + 1u)
_Static_assert(APP_BITS_FOR(APP_SPEED_CODES) == APP_SPEED_BITS, "speed width");

/* Hump height -- owner: IMU Processing (Buddy 4).  Unsigned mm, 1 mm/LSB.
   Valid 0..62 mm, 63 = "not measured".  64 codes -> 6 bits. */
#define APP_HUMP_MM_MAX		62u
#define APP_HUMP_MM_NA		63u
#define APP_HUMP_BITS		6u
_Static_assert(APP_BITS_FOR(APP_HUMP_MM_MAX + 2u) == APP_HUMP_BITS, "hump width");
_Static_assert(APP_HUMP_MM_NA == APP_MASK(APP_HUMP_BITS), "hump sentinel");

/* Obstacle width -- owner: Obstacle Scanning (Buddy 5).  Unsigned cm.
   Valid 0..100 cm, 127 = "no obstacle / not measured", 101..126 invalid.
   102 codes -> 7 bits. */
#define APP_OBST_CM_MAX		100u
#define APP_OBST_CM_NA		127u
#define APP_OBST_BITS		7u
_Static_assert(APP_BITS_FOR(APP_OBST_CM_MAX + 2u) == APP_OBST_BITS, "obstacle width");
_Static_assert(APP_OBST_CM_NA == APP_MASK(APP_OBST_BITS), "obstacle sentinel");

/* Reconnect retry count -- owner: Wi-Fi Telemetry (Buddy 1).
   0..7, saturates at 7.  8 codes -> 3 bits. */
#define APP_RETRY_MAX		7u
#define APP_RETRY_BITS		3u
_Static_assert(APP_BITS_FOR(APP_RETRY_MAX + 1u) == APP_RETRY_BITS, "retry width");

/* Telemetry sequence: 8 bits, wraps 255 -> 0 (25.6 s at 10 Hz).
   Telemetry time: 16 bits in 10 ms units (= the 10 ms system tick),
   wraps every 655.36 s; seq + time together order frames in an outage. */
#define APP_TLM_SEQ_BITS	8u
#define APP_TLM_TIME_BITS	16u
#define APP_TLM_TIME_UNIT_MS	10u

/* ---------------------------------------------------------------------
 *  Shared fault mask (12 bits).  Lives in the event flag flg_fault
 *  (bits 0..11).  One OWNER per bit; only the owner sets/clears it, and
 *  only with tk_set_flg / tk_clr_flg (atomic in the kernel).
 *  Bit 6 is NOT stored in flg_fault: the Wi-Fi task copies it from the
 *  Safety latch (flg_safety SAFETY_ESTOP_LATCHED) when it builds a frame,
 *  so the ESTOP latch has a single source of truth.
 * --------------------------------------------------------------------- */
#define FLT_WIFI_LINK		(1u << 0)	/* B1: Wi-Fi association lost        */
#define FLT_MQTT_LINK		(1u << 1)	/* B1: MQTT broker connection lost   */
#define FLT_SOCK_TIMEOUT	(1u << 2)	/* B1: TCP not acked within 1.5 s    */
#define FLT_CMD_MALFORMED	(1u << 3)	/* B1: bad length/CRC/unknown cmd    */
#define FLT_TELEM_OVERFLOW	(1u << 4)	/* B1: telemetry_q or backlog dropped*/
#define FLT_PROTO_ERROR		(1u << 5)	/* B1: bad version / reserved bits   */
#define FLT_ESTOP_LATCHED	(1u << 6)	/* Safety (B4/B5), mirrored by B1    */
#define FLT_COLLISION		(1u << 7)	/* B4 IMU: shock > 2.5 g             */
#define FLT_IMU_I2C		(1u << 8)	/* B4 IMU: I2C read failure          */
#define FLT_LINE_LOST		(1u << 9)	/* B3 Line Following: LINE_FAULT     */
#define FLT_LINE_NOT_RECOVERED	(1u << 10)	/* B5 Obstacle: bypass recovery fail */
#define FLT_MOTOR		(1u << 11)	/* B2 Motion: motor/turn timeout     */
#define FLT_BITS		12u
#define FLT_ALL			((uint16_t)APP_MASK(FLT_BITS))
/* Bits owned by Buddy 1 (the only bits the Wi-Fi task may set/clear) */
#define FLT_B1_OWNED		(FLT_WIFI_LINK | FLT_MQTT_LINK | FLT_SOCK_TIMEOUT | \
				 FLT_CMD_MALFORMED | FLT_TELEM_OVERFLOW | FLT_PROTO_ERROR)
/* B1 event faults: reported in at least one published frame, then cleared */
#define FLT_B1_REPORT_ONCE	(FLT_CMD_MALFORMED | FLT_TELEM_OVERFLOW | FLT_PROTO_ERROR)
/* B1 link faults: set while the link is down, cleared on reconnection */
#define FLT_B1_LINK		(FLT_WIFI_LINK | FLT_MQTT_LINK | FLT_SOCK_TIMEOUT)
_Static_assert((FLT_B1_OWNED & FLT_ESTOP_LATCHED) == 0u, "B1 must not own ESTOP");
_Static_assert((FLT_B1_OWNED & ~(uint32_t)FLT_ALL) == 0u, "fault bit outside mask");

/* ---------------------------------------------------------------------
 *  Safety event flag flg_safety (owner: Emergency/Safety Handler, B4/B5)
 *  REQ bits: set by the requesting task, consumed and cleared by Safety.
 *  ESTOP_LATCHED: set and cleared ONLY by Safety (local reset required).
 * --------------------------------------------------------------------- */
#define SAFETY_REQ_COLLISION	(1u << 0)	/* set by IMU (B4)             */
#define SAFETY_REQ_OBSTACLE_STOP (1u << 1)	/* set by Obstacle (B5)        */
#define SAFETY_REQ_REMOTE_ABORT	(1u << 2)	/* set by Wi-Fi (B1)           */
#define SAFETY_ESTOP_LATCHED	(1u << 8)	/* Safety only; read by others */

/* =====================================================================
 *  TELEMETRY FRAME  (robot/<id>/state, Wi-Fi task -> broker)
 *  9 bytes, little endian, LSB-first inside each byte.
 *
 *  byte bits  field         width  notes
 *  0    0-1   version         2    must be APP_PROTO_VERSION
 *  0    2-4   vehicle state   3    APP_VS_*
 *  0    5-7   retry count     3    reconnect attempts, saturates at 7
 *  1    0-7   sequence        8    wraps 255 -> 0
 *  2-3  0-15  time           16    10 ms units, LE, wraps
 *  4    0-7   speed           8    int8 cm/s, -128 = n/a
 *  5    0-5   hump height     6    mm, 63 = n/a
 *  5    6     backlog         1    1 = buffered during an outage
 *  5    7     reserved        1    0
 *  6    0-6   obstacle width  7    cm, 127 = n/a
 *  6    7     reserved        1    0
 *  7-8  0-11  fault mask     12    FLT_*, LE
 *  8    4-7   reserved        4    0
 *  used 66 bits + 6 reserved = 72 bits = 9 bytes
 * ===================================================================== */
#define TLM_FRAME_LEN		9u

#define TLM_B0_VER_SHIFT	0u
#define TLM_B0_VER_MASK		0x03u
#define TLM_B0_STATE_SHIFT	2u
#define TLM_B0_STATE_MASK	0x1Cu
#define TLM_B0_RETRY_SHIFT	5u
#define TLM_B0_RETRY_MASK	0xE0u
#define TLM_B1_SEQ		1u		/* byte index */
#define TLM_B2_TIME		2u		/* byte index of u16 LE */
#define TLM_B4_SPEED		4u
#define TLM_B5_HUMP_SHIFT	0u
#define TLM_B5_HUMP_MASK	0x3Fu
#define TLM_B5_BACKLOG_MASK	0x40u
#define TLM_B5_RSVD_MASK	0x80u
#define TLM_B6_OBST_SHIFT	0u
#define TLM_B6_OBST_MASK	0x7Fu
#define TLM_B6_RSVD_MASK	0x80u
#define TLM_B7_FAULTS		7u		/* byte index of u16 LE */
#define TLM_B8_RSVD_MASK	0xF0u

#define TLM_USED_BITS		(APP_PROTO_VER_BITS + APP_VS_BITS + APP_RETRY_BITS + \
				 APP_TLM_SEQ_BITS + APP_TLM_TIME_BITS + APP_SPEED_BITS + \
				 APP_HUMP_BITS + 1u + APP_OBST_BITS + FLT_BITS)
#define TLM_RESERVED_BITS	6u

_Static_assert(TLM_USED_BITS == 66u, "telemetry allocation table changed");
_Static_assert(TLM_USED_BITS + TLM_RESERVED_BITS == 8u * TLM_FRAME_LEN,
	       "telemetry bits do not add up to the frame length");
_Static_assert((TLM_B0_VER_MASK | TLM_B0_STATE_MASK | TLM_B0_RETRY_MASK) == 0xFFu &&
	       (TLM_B0_VER_MASK & TLM_B0_STATE_MASK) == 0u &&
	       (TLM_B0_STATE_MASK & TLM_B0_RETRY_MASK) == 0u, "byte 0 layout");
_Static_assert((TLM_B5_HUMP_MASK | TLM_B5_BACKLOG_MASK | TLM_B5_RSVD_MASK) == 0xFFu &&
	       (TLM_B5_HUMP_MASK & TLM_B5_BACKLOG_MASK) == 0u, "byte 5 layout");
_Static_assert((TLM_B6_OBST_MASK | TLM_B6_RSVD_MASK) == 0xFFu, "byte 6 layout");
_Static_assert(((uint32_t)FLT_ALL >> 8) == (0xFFu & ~TLM_B8_RSVD_MASK), "byte 8 layout");
_Static_assert(TLM_B0_STATE_MASK == (APP_MASK(APP_VS_BITS) << TLM_B0_STATE_SHIFT) &&
	       TLM_B0_RETRY_MASK == (APP_MASK(APP_RETRY_BITS) << TLM_B0_RETRY_SHIFT) &&
	       TLM_B5_HUMP_MASK == APP_MASK(APP_HUMP_BITS) &&
	       TLM_B6_OBST_MASK == APP_MASK(APP_OBST_BITS), "masks match widths");

/* Decoded (in-memory) telemetry.  Task-local only; never sent as-is. */
typedef struct {
	uint8_t		version;	/* APP_PROTO_VERSION */
	uint8_t		veh_state;	/* APP_VS_* */
	uint8_t		retry;		/* 0..7 */
	uint8_t		seq;		/* 0..255 */
	uint16_t	time_10ms;	/* 0..65535 */
	int16_t		speed_cm_s;	/* -100..100 or APP_SPEED_NA */
	uint8_t		hump_mm;	/* 0..62 or APP_HUMP_MM_NA */
	uint8_t		obst_cm;	/* 0..100 or APP_OBST_CM_NA */
	uint16_t	faults;		/* FLT_* (12 bits) */
	bool		backlog;
} APP_TELEM;

static inline bool app_speed_valid(int32_t v)
{
	return (v >= APP_SPEED_MIN && v <= APP_SPEED_MAX) || v == APP_SPEED_NA;
}

static inline bool app_hump_valid(uint32_t v)
{
	return v <= APP_HUMP_MM_MAX || v == APP_HUMP_MM_NA;
}

static inline bool app_obst_valid(uint32_t v)
{
	return v <= APP_OBST_CM_MAX || v == APP_OBST_CM_NA;
}

/* Default values (used before any producer has reported) */
static inline void app_telem_default(APP_TELEM *t)
{
	t->version    = (uint8_t)APP_PROTO_VERSION;
	t->veh_state  = (uint8_t)APP_VS_INITIALIZING;
	t->retry      = 0u;
	t->seq        = 0u;
	t->time_10ms  = 0u;
	t->speed_cm_s = (int16_t)APP_SPEED_NA;
	t->hump_mm    = (uint8_t)APP_HUMP_MM_NA;
	t->obst_cm    = (uint8_t)APP_OBST_CM_NA;
	t->faults     = 0u;
	t->backlog    = false;
}

/* Encode.  Validates every field first; on error out[] is not written. */
static inline int app_telem_encode(const APP_TELEM *t, uint8_t out[TLM_FRAME_LEN])
{
	if (t->version != APP_PROTO_VERSION) {
		return APP_E_VERSION;
	}
	if (t->veh_state >= APP_VS_COUNT || t->retry > APP_RETRY_MAX ||
	    !app_speed_valid(t->speed_cm_s) || !app_hump_valid(t->hump_mm) ||
	    !app_obst_valid(t->obst_cm) || (t->faults & (uint16_t)~FLT_ALL) != 0u) {
		return APP_E_RANGE;
	}
	out[0] = (uint8_t)(((uint32_t)t->version << TLM_B0_VER_SHIFT) |
			   ((uint32_t)t->veh_state << TLM_B0_STATE_SHIFT) |
			   ((uint32_t)t->retry << TLM_B0_RETRY_SHIFT));
	out[TLM_B1_SEQ] = t->seq;
	app_put_u16le(&out[TLM_B2_TIME], t->time_10ms);
	out[TLM_B4_SPEED] = app_i8_to_byte(t->speed_cm_s);
	out[5] = (uint8_t)(((uint32_t)t->hump_mm & TLM_B5_HUMP_MASK) |
			   (t->backlog ? TLM_B5_BACKLOG_MASK : 0u));
	out[6] = (uint8_t)((uint32_t)t->obst_cm & TLM_B6_OBST_MASK);
	app_put_u16le(&out[TLM_B7_FAULTS], t->faults);
	return APP_OK;
}

/* Decode (used by tests and the PC tools; strict: rejects reserved bits) */
static inline int app_telem_decode(const uint8_t *in, uint32_t len, APP_TELEM *t)
{
	uint32_t	v;

	if (in == 0 || len != TLM_FRAME_LEN) {
		return APP_E_LEN;
	}
	if (((uint32_t)in[0] & TLM_B0_VER_MASK) >> TLM_B0_VER_SHIFT != APP_PROTO_VERSION) {
		return APP_E_VERSION;
	}
	if ((in[5] & TLM_B5_RSVD_MASK) != 0u || (in[6] & TLM_B6_RSVD_MASK) != 0u ||
	    (in[8] & TLM_B8_RSVD_MASK) != 0u) {
		return APP_E_RESERVED;
	}
	v = in[6] & TLM_B6_OBST_MASK;
	if (!app_obst_valid(v) || !app_speed_valid(app_byte_to_i8(in[TLM_B4_SPEED]))) {
		return APP_E_RANGE;
	}
	t->version    = (uint8_t)(((uint32_t)in[0] & TLM_B0_VER_MASK) >> TLM_B0_VER_SHIFT);
	t->veh_state  = (uint8_t)(((uint32_t)in[0] & TLM_B0_STATE_MASK) >> TLM_B0_STATE_SHIFT);
	t->retry      = (uint8_t)(((uint32_t)in[0] & TLM_B0_RETRY_MASK) >> TLM_B0_RETRY_SHIFT);
	t->seq        = in[TLM_B1_SEQ];
	t->time_10ms  = app_get_u16le(&in[TLM_B2_TIME]);
	t->speed_cm_s = (int16_t)app_byte_to_i8(in[TLM_B4_SPEED]);
	t->hump_mm    = (uint8_t)(in[5] & TLM_B5_HUMP_MASK);
	t->backlog    = (in[5] & TLM_B5_BACKLOG_MASK) != 0u;
	t->obst_cm    = (uint8_t)v;
	t->faults     = app_get_u16le(&in[TLM_B7_FAULTS]);
	return APP_OK;
}

/* Mark an already-encoded frame as "buffered during an outage" */
static inline void app_telem_set_backlog(uint8_t frame[TLM_FRAME_LEN])
{
	frame[5] = (uint8_t)(frame[5] | TLM_B5_BACKLOG_MASK);
}

/* =====================================================================
 *  COMMAND FRAME  (robot/<id>/cmd, broker -> Wi-Fi task)
 *  3 bytes.
 *
 *  byte bits  field     width  notes
 *  0    0-1   version     2    must be APP_PROTO_VERSION
 *  0    2-3   command     2    1 abort, 2 pause, 3 resume (0 invalid)
 *  0    4-7   reserved    4    must be 0
 *  1    0-7   sequence    8    serial-number arithmetic (wraps)
 *  2    0-7   CRC-8       8    poly 0x07, init 0x00, over bytes 0..1
 *  used 20 bits + 4 reserved = 24 bits = 3 bytes
 * ===================================================================== */
#define CMD_FRAME_LEN		3u

#define CMD_NONE		0u	/* invalid: an all-zero frame is not a command */
#define CMD_ABORT		1u
#define CMD_PAUSE		2u
#define CMD_RESUME		3u
#define CMD_TYPE_BITS		2u
#define CMD_TYPE_CODES		4u
_Static_assert(APP_BITS_FOR(CMD_TYPE_CODES) == CMD_TYPE_BITS, "command width");

#define CMD_B0_VER_SHIFT	0u
#define CMD_B0_VER_MASK		0x03u
#define CMD_B0_TYPE_SHIFT	2u
#define CMD_B0_TYPE_MASK	0x0Cu
#define CMD_B0_RSVD_MASK	0xF0u
#define CMD_B1_SEQ		1u
#define CMD_B2_CRC		2u
#define CMD_USED_BITS		(APP_PROTO_VER_BITS + CMD_TYPE_BITS + 8u + 8u)
#define CMD_RESERVED_BITS	4u
_Static_assert(CMD_USED_BITS + CMD_RESERVED_BITS == 8u * CMD_FRAME_LEN, "command bits");
_Static_assert((CMD_B0_VER_MASK | CMD_B0_TYPE_MASK | CMD_B0_RSVD_MASK) == 0xFFu &&
	       (CMD_B0_VER_MASK & CMD_B0_TYPE_MASK) == 0u, "command byte 0 layout");

/* CRC-8 (poly 0x07, init 0x00, no reflection, no final xor; "CRC-8/SMBUS").
   Check value: "123456789" -> 0xF4. */
static inline uint8_t app_crc8(const uint8_t *p, uint32_t n)
{
	uint8_t		crc = 0u;
	uint32_t	i, b;

	for (i = 0u; i < n; i++) {
		crc = (uint8_t)(crc ^ p[i]);
		for (b = 0u; b < 8u; b++) {
			crc = (uint8_t)(((crc & 0x80u) != 0u) ?
					(uint8_t)((uint32_t)crc << 1) ^ 0x07u :
					(uint8_t)((uint32_t)crc << 1));
		}
	}
	return crc;
}

typedef struct {
	uint8_t		type;	/* CMD_ABORT / CMD_PAUSE / CMD_RESUME */
	uint8_t		seq;
} APP_CMD;

static inline int app_cmd_encode(const APP_CMD *c, uint8_t out[CMD_FRAME_LEN])
{
	if (c->type == CMD_NONE || c->type > CMD_RESUME) {
		return APP_E_TYPE;
	}
	out[0] = (uint8_t)((APP_PROTO_VERSION << CMD_B0_VER_SHIFT) |
			   ((uint32_t)c->type << CMD_B0_TYPE_SHIFT));
	out[CMD_B1_SEQ] = c->seq;
	out[CMD_B2_CRC] = app_crc8(out, 2u);
	return APP_OK;
}

/* Structural decode: length, CRC, version, reserved bits, command type.
   Sequence/duplicate/ESTOP rules are applied by the Wi-Fi task. */
static inline int app_cmd_decode(const uint8_t *in, uint32_t len, APP_CMD *c)
{
	uint32_t	type;

	if (in == 0 || len != CMD_FRAME_LEN) {
		return APP_E_LEN;
	}
	if (app_crc8(in, 2u) != in[CMD_B2_CRC]) {
		return APP_E_CRC;
	}
	if (((uint32_t)in[0] & CMD_B0_VER_MASK) >> CMD_B0_VER_SHIFT != APP_PROTO_VERSION) {
		return APP_E_VERSION;
	}
	if ((in[0] & CMD_B0_RSVD_MASK) != 0u) {
		return APP_E_RESERVED;
	}
	type = ((uint32_t)in[0] & CMD_B0_TYPE_MASK) >> CMD_B0_TYPE_SHIFT;
	if (type == CMD_NONE) {
		return APP_E_TYPE;
	}
	c->type = (uint8_t)type;
	c->seq  = in[CMD_B1_SEQ];
	return APP_OK;
}

/* =====================================================================
 *  remote_cmd_q MESSAGE  (Wi-Fi task -> Vehicle Controller), 2 bytes
 *  byte 0 bits 0-1 command type (CMD_*), bits 2-7 reserved 0
 *  byte 1 sequence number of the accepted command
 * ===================================================================== */
#define RCMD_MSG_LEN		2u
#define RCMD_B0_TYPE_MASK	0x03u
#define RCMD_B0_RSVD_MASK	0xFCu

static inline int app_rcmd_pack(uint8_t out[RCMD_MSG_LEN], uint32_t type, uint32_t seq)
{
	if (type == CMD_NONE || type > CMD_RESUME || seq > 0xFFu) {
		return APP_E_RANGE;
	}
	out[0] = (uint8_t)(type & RCMD_B0_TYPE_MASK);
	out[1] = (uint8_t)seq;
	return APP_OK;
}

static inline int app_rcmd_unpack(const uint8_t in[RCMD_MSG_LEN], APP_CMD *c)
{
	if ((in[0] & RCMD_B0_RSVD_MASK) != 0u || (in[0] & RCMD_B0_TYPE_MASK) == CMD_NONE) {
		return APP_E_RANGE;
	}
	c->type = (uint8_t)(in[0] & RCMD_B0_TYPE_MASK);
	c->seq  = in[1];
	return APP_OK;
}

/* =====================================================================
 *  telemetry_q STATUS MESSAGE  (any producer -> Wi-Fi task), 4 bytes
 *  byte 0 bits 0-2 field id (TQ_FIELD_*), bits 3-7 reserved 0
 *  byte 1          value, encoded exactly as in the telemetry frame
 *  byte 2-3        enqueue timestamp, LE, 1.024 ms units (us >> 10)
 *  One producer per field (single writer): see docs.
 * ===================================================================== */
#define TQ_MSG_LEN		4u
#define TQ_FIELD_NONE		0u	/* invalid */
#define TQ_FIELD_VEH_STATE	1u	/* Vehicle Controller (B4) */
#define TQ_FIELD_SPEED		2u	/* Motion Control (B2)     */
#define TQ_FIELD_HUMP		3u	/* IMU Processing (B4)     */
#define TQ_FIELD_OBSTACLE	4u	/* Obstacle Scanning (B5)  */
#define TQ_FIELD_COUNT		5u	/* ids 0..4 (0 invalid)    */
#define TQ_FIELD_BITS		3u
#define TQ_B0_FIELD_MASK	0x07u
#define TQ_B0_RSVD_MASK		0xF8u
_Static_assert(APP_BITS_FOR(TQ_FIELD_COUNT) == TQ_FIELD_BITS, "field id width");

/* Range check of one status value (APP_OK, APP_E_TYPE or APP_E_RANGE) */
static inline int app_tq_check(uint32_t field, int32_t value)
{
	switch (field) {
	case TQ_FIELD_VEH_STATE:
		return (value >= 0 && value < (int32_t)APP_VS_COUNT) ? APP_OK : APP_E_RANGE;
	case TQ_FIELD_SPEED:
		return app_speed_valid(value) ? APP_OK : APP_E_RANGE;
	case TQ_FIELD_HUMP:
		return (value >= 0 && app_hump_valid((uint32_t)value)) ? APP_OK : APP_E_RANGE;
	case TQ_FIELD_OBSTACLE:
		return (value >= 0 && app_obst_valid((uint32_t)value)) ? APP_OK : APP_E_RANGE;
	default:
		return APP_E_TYPE;
	}
}

static inline int app_tq_pack(uint8_t out[TQ_MSG_LEN], uint32_t field,
			      int32_t value, uint16_t stamp)
{
	int	er = app_tq_check(field, value);

	if (er != APP_OK) {
		return er;
	}
	out[0] = (uint8_t)(field & TQ_B0_FIELD_MASK);
	out[1] = app_i8_to_byte(value);	/* low byte; speed is two's complement */
	app_put_u16le(&out[2], stamp);
	return APP_OK;
}

static inline int app_tq_unpack(const uint8_t in[TQ_MSG_LEN], uint32_t *field,
				int32_t *value, uint16_t *stamp)
{
	uint32_t	f = in[0] & TQ_B0_FIELD_MASK;
	int32_t		v;
	int		er;

	if ((in[0] & TQ_B0_RSVD_MASK) != 0u) {
		return APP_E_RESERVED;
	}
	v = (f == TQ_FIELD_SPEED) ? app_byte_to_i8(in[1]) : (int32_t)in[1];
	er = app_tq_check(f, v);
	if (er != APP_OK) {
		return er;
	}
	*field = f;
	*value = v;
	*stamp = app_get_u16le(&in[2]);
	return APP_OK;
}

/* =====================================================================
 * Buddy 4 IMU extensions (preserved from the independently tested v9/v10
 * IMU code).  These are for imu_evt_q and s4_imu_snapshot, not telemetry.
 *
 * The shared FLT_* / SAFETY_* definitions above are the only shared
 * fault and safety patterns.  Do NOT re-add the old provisional 32-bit
 * flg_faults allocation (whose bit numbers conflict with teammates).
 *
 * NOTE: imu_evt_q allows 0..127 mm; telemetry permits only 0..62 mm
 * (63 = not measured).  Convert/validate deliberately at the Wi-Fi
 * telemetry boundary; never truncate a hump value silently.
 * ===================================================================== */

/* ------------------------------------------------------------------ */
/* Generic field helpers (unsigned only)                               */
/* ------------------------------------------------------------------ */
static inline uint32_t ad_mask(uint32_t bits)
{
    return (bits >= 32U) ? 0xFFFFFFFFUL : ((1UL << bits) - 1UL);
}

static inline uint32_t ad_get(uint32_t word, uint32_t shift, uint32_t bits)
{
    return (word >> shift) & ad_mask(bits);
}

/* Returns word with the field replaced by value (value is masked). */
static inline uint32_t ad_put(uint32_t word, uint32_t shift, uint32_t bits,
                              uint32_t value)
{
    uint32_t m = ad_mask(bits) << shift;
    return (word & ~m) | ((value << shift) & m);
}

/* ------------------------------------------------------------------ */
/* Buddy 4 : imu_evt_q message  (16 bits, queue depth 8)               */
/*   [2:0]  TYPE       s4_evt_type_t                                   */
/*   [9:3]  HEIGHT_MM  0..127 mm (valid for HUMP_DONE when VALID = 1)  */
/*   [10]   VALID      1 = height is a real measurement                */
/*   [15:11] SEQ       wrapping counter, lets receivers see lost events*/
/* ------------------------------------------------------------------ */
typedef enum {
    S4_EVT_IMU_READY = 0,    /* calibration done, IMU data is usable   */
    S4_EVT_HUMP_START,       /* hump entered: VC should slow down      */
    S4_EVT_HUMP_DONE,        /* hump finished: HEIGHT_MM + VALID       */
    S4_EVT_SHOCK_MINOR,      /* minor shock, logged only               */
    S4_EVT_IMU_FAULT,        /* IMU unusable (I2C or calibration)      */
    S4_EVT_COUNT
} s4_evt_type_t;

#define S4_EVT_TYPE_SHIFT    0U
#define S4_EVT_TYPE_BITS     3U
#define S4_EVT_HEIGHT_SHIFT  3U
#define S4_EVT_HEIGHT_BITS   7U
#define S4_EVT_VALID_SHIFT   10U
#define S4_EVT_VALID_BITS    1U
#define S4_EVT_SEQ_SHIFT     11U
#define S4_EVT_SEQ_BITS      5U
#define S4_EVT_HEIGHT_MAX    127U

_Static_assert(S4_EVT_COUNT <= (1U << S4_EVT_TYPE_BITS),
               "event types do not fit in TYPE field");
_Static_assert(S4_EVT_TYPE_SHIFT + S4_EVT_TYPE_BITS == S4_EVT_HEIGHT_SHIFT,
               "TYPE/HEIGHT fields overlap or leave a gap");
_Static_assert(S4_EVT_HEIGHT_SHIFT + S4_EVT_HEIGHT_BITS == S4_EVT_VALID_SHIFT,
               "HEIGHT/VALID fields overlap or leave a gap");
_Static_assert(S4_EVT_VALID_SHIFT + S4_EVT_VALID_BITS == S4_EVT_SEQ_SHIFT,
               "VALID/SEQ fields overlap or leave a gap");
_Static_assert(S4_EVT_SEQ_SHIFT + S4_EVT_SEQ_BITS == 16U,
               "imu_evt message must be exactly 16 bits");

static inline uint16_t s4_evt_pack(s4_evt_type_t type, uint32_t height_mm,
                                   uint32_t valid, uint32_t seq)
{
    uint32_t w = 0UL;
    if (height_mm > S4_EVT_HEIGHT_MAX) { height_mm = S4_EVT_HEIGHT_MAX; } /* saturate */
    w = ad_put(w, S4_EVT_TYPE_SHIFT,   S4_EVT_TYPE_BITS,   (uint32_t)type);
    w = ad_put(w, S4_EVT_HEIGHT_SHIFT, S4_EVT_HEIGHT_BITS, height_mm);
    w = ad_put(w, S4_EVT_VALID_SHIFT,  S4_EVT_VALID_BITS,  valid);
    w = ad_put(w, S4_EVT_SEQ_SHIFT,    S4_EVT_SEQ_BITS,    seq);
    return (uint16_t)w;
}
static inline s4_evt_type_t s4_evt_type(uint16_t w)
{ return (s4_evt_type_t)ad_get(w, S4_EVT_TYPE_SHIFT, S4_EVT_TYPE_BITS); }
static inline uint32_t s4_evt_height_mm(uint16_t w)
{ return ad_get(w, S4_EVT_HEIGHT_SHIFT, S4_EVT_HEIGHT_BITS); }
static inline uint32_t s4_evt_valid(uint16_t w)
{ return ad_get(w, S4_EVT_VALID_SHIFT, S4_EVT_VALID_BITS); }
static inline uint32_t s4_evt_seq(uint16_t w)
{ return ad_get(w, S4_EVT_SEQ_SHIFT, S4_EVT_SEQ_BITS); }

/* ------------------------------------------------------------------ */
/* Buddy 4 : imu snapshot word (32 bits, ONE writer = IMU task)        */
/*   Written with a single aligned 32-bit store, read with a single    */
/*   load, so readers always see a consistent set of values.           */
/*   [7:0]   HEADING  binary angle, 256 steps = 360 deg (1.4 deg/step) */
/*   [15:8]  PITCH    offset-binary, 0.5 deg/step, 128 = level         */
/*   [23:16] ROLL     offset-binary, 0.5 deg/step, 128 = level         */
/*   [26:24] STATE    s4_imu_state_t                                   */
/*   [27]    VALID    1 = sensor data fresh and calibrated             */
/*   [31:28] SEQ      wrapping counter                                 */
/* ------------------------------------------------------------------ */
typedef enum {
    S4_ST_INIT_CALIBRATING = 0,
    S4_ST_MONITORING,
    S4_ST_HUMP_TRACKING,
    S4_ST_SHOCK_DETECTED,
    S4_ST_FAULT,
    S4_ST_COUNT
} s4_imu_state_t;

#define S4_SNAP_HEADING_SHIFT 0U
#define S4_SNAP_HEADING_BITS  8U
#define S4_SNAP_PITCH_SHIFT   8U
#define S4_SNAP_PITCH_BITS    8U
#define S4_SNAP_ROLL_SHIFT    16U
#define S4_SNAP_ROLL_BITS     8U
#define S4_SNAP_STATE_SHIFT   24U
#define S4_SNAP_STATE_BITS    3U
#define S4_SNAP_VALID_SHIFT   27U
#define S4_SNAP_VALID_BITS    1U
#define S4_SNAP_SEQ_SHIFT     28U
#define S4_SNAP_SEQ_BITS      4U

_Static_assert(S4_ST_COUNT <= (1U << S4_SNAP_STATE_BITS),
               "IMU states do not fit in STATE field");
_Static_assert(S4_SNAP_SEQ_SHIFT + S4_SNAP_SEQ_BITS == 32U,
               "imu snapshot must be exactly 32 bits");
_Static_assert(S4_SNAP_HEADING_SHIFT + S4_SNAP_HEADING_BITS == S4_SNAP_PITCH_SHIFT &&
               S4_SNAP_PITCH_SHIFT + S4_SNAP_PITCH_BITS == S4_SNAP_ROLL_SHIFT &&
               S4_SNAP_ROLL_SHIFT + S4_SNAP_ROLL_BITS == S4_SNAP_STATE_SHIFT &&
               S4_SNAP_STATE_SHIFT + S4_SNAP_STATE_BITS == S4_SNAP_VALID_SHIFT &&
               S4_SNAP_VALID_SHIFT + S4_SNAP_VALID_BITS == S4_SNAP_SEQ_SHIFT,
               "imu snapshot fields overlap or leave a gap");

static inline uint32_t s4_snap_pack(uint32_t heading_bam8, uint32_t pitch_ob8,
                                    uint32_t roll_ob8, s4_imu_state_t state,
                                    uint32_t valid, uint32_t seq)
{
    uint32_t w = 0UL;
    w = ad_put(w, S4_SNAP_HEADING_SHIFT, S4_SNAP_HEADING_BITS, heading_bam8);
    w = ad_put(w, S4_SNAP_PITCH_SHIFT,   S4_SNAP_PITCH_BITS,   pitch_ob8);
    w = ad_put(w, S4_SNAP_ROLL_SHIFT,    S4_SNAP_ROLL_BITS,    roll_ob8);
    w = ad_put(w, S4_SNAP_STATE_SHIFT,   S4_SNAP_STATE_BITS,   (uint32_t)state);
    w = ad_put(w, S4_SNAP_VALID_SHIFT,   S4_SNAP_VALID_BITS,   valid);
    w = ad_put(w, S4_SNAP_SEQ_SHIFT,     S4_SNAP_SEQ_BITS,     seq);
    return w;
}
static inline uint32_t s4_snap_heading(uint32_t w) { return ad_get(w, S4_SNAP_HEADING_SHIFT, S4_SNAP_HEADING_BITS); }
static inline uint32_t s4_snap_pitch(uint32_t w)   { return ad_get(w, S4_SNAP_PITCH_SHIFT,   S4_SNAP_PITCH_BITS); }
static inline uint32_t s4_snap_roll(uint32_t w)    { return ad_get(w, S4_SNAP_ROLL_SHIFT,    S4_SNAP_ROLL_BITS); }
static inline s4_imu_state_t s4_snap_state(uint32_t w)
{ return (s4_imu_state_t)ad_get(w, S4_SNAP_STATE_SHIFT, S4_SNAP_STATE_BITS); }
static inline uint32_t s4_snap_valid(uint32_t w)   { return ad_get(w, S4_SNAP_VALID_SHIFT,   S4_SNAP_VALID_BITS); }
static inline uint32_t s4_snap_seq(uint32_t w)     { return ad_get(w, S4_SNAP_SEQ_SHIFT,     S4_SNAP_SEQ_BITS); }

/* ------------------------------------------------------------------ */
/* Buddy 4 : Vehicle Controller states                              */
/* The team's APP_VS_* codes are authoritative for telemetry.      */
/* Keep the VC aliases aligned with those codes (8 states, not 7).  */
/* ------------------------------------------------------------------ */
typedef enum {
    VC_INITIALIZING     = APP_VS_INITIALIZING,
    VC_DRIVING          = APP_VS_DRIVING,
    VC_HUMP_CROSSING    = APP_VS_HUMP_CROSSING,
    VC_OBSTACLE_BYPASS  = APP_VS_OBSTACLE_BYPASS,
    VC_LINE_SEARCH      = APP_VS_LINE_SEARCH,
    VC_PAUSED           = APP_VS_PAUSED,
    VC_ERROR            = APP_VS_ERROR,
    VC_MISSION_COMPLETE = APP_VS_MISSION_COMPLETE,
    VC_STATE_COUNT      = APP_VS_COUNT
} vc_state_t;
_Static_assert(VC_STATE_COUNT == 8, "vehicle states must fit in 3 bits");
_Static_assert((uint32_t)VC_MISSION_COMPLETE == APP_VS_MISSION_COMPLETE,
               "vehicle state and telemetry codes differ");


#endif /* APP_DATA_H */
