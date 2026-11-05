/*
 * app_data.h
 * Central definitions for data shared between subsystems (Minimum-Bit
 * Data Representation, Week 8 decision).  Maintained by Buddy 4.
 *
 * RULES (agreed in the Tribunal 1 discussion)
 *  - Every packed layout is defined HERE, once, with _Static_assert checks.
 *  - Pack/unpack ONLY through the accessor functions below; never cast a
 *    byte buffer to a wider type (Cortex-M0+ has no unaligned access) and
 *    never use bit-fields for data that crosses a task boundary.
 *  - Use unsigned types for all shift/mask work (Barr-C 5.3).  Signed
 *    quantities are stored as offset-binary (value + offset).
 *  - Shared fault/safety information is an RTOS event flag
 *    (tk_set_flg / tk_clr_flg); NEVER "mask |= bit" on a shared variable.
 *
 * This file only needs <stdint.h>, so it also compiles on a PC for the
 * host unit tests in testing/subsystem4/host/.
 *
 * Other buddies: add your own section below the marker at the end.
 */

#ifndef APP_DATA_H
#define APP_DATA_H

#include <stdint.h>

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
/* Shared event-flag bit allocation  (PROPOSAL - team to confirm)      */
/* Each owner uses only its own byte so bits never collide.            */
/* ------------------------------------------------------------------ */

/* flg_faults : shared team fault mask (one kernel event flag).
 * Set   : tk_set_flg(flg_faults, FLT_xxx)
 * Clear : tk_clr_flg(flg_faults, ~FLT_xxx)   (clears bits that are 0) */
#define FLT_MOTOR_STALL      (1UL << 0)   /* Buddy 2 */
#define FLT_ENCODER          (1UL << 1)   /* Buddy 2 */
#define FLT_IMU_I2C          (1UL << 8)   /* Buddy 4: I2C timeout/NACK     */
#define FLT_IMU_CALIB        (1UL << 9)   /* Buddy 4: calibration failed   */
#define FLT_IMU_MAG_DISAGREE (1UL << 10)  /* Buddy 4: mag vs encoder turn  */
#define FLT_IMU_RANGE        (1UL << 11)  /* Buddy 4: sensor overflow      */
#define FLT_NO_ECHO          (1UL << 16)  /* Buddy 5 */
#define FLT_SERVO            (1UL << 17)  /* Buddy 5 */
#define FLT_WIFI_LOSS        (1UL << 24)  /* Buddy 1 */
#define FLT_LINE_LOST        (1UL << 25)  /* Buddy 3 */

/* flg_safety : events for the Emergency/Safety Event Handling Task. */
#define SAF_COLLISION        (1UL << 0)   /* from IMU (Buddy 4)            */
#define SAF_OBSTACLE_STOP    (1UL << 1)   /* from Obstacle (Buddy 5)       */
#define SAF_ESTOP_BUTTON     (1UL << 2)   /* E-stop button GP3             */
#define SAF_ABORT            (1UL << 3)   /* remote abort (Buddy 1)        */

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
/* Buddy 4 : Vehicle Controller state (3 bits)                         */
/* ------------------------------------------------------------------ */
typedef enum {
    VC_INITIALIZING = 0,
    VC_DRIVING,
    VC_HUMP_CROSSING,
    VC_OBSTACLE_BYPASS,
    VC_PAUSED,
    VC_ERROR,
    VC_MISSION_COMPLETE,
    VC_STATE_COUNT
} vc_state_t;
_Static_assert(VC_STATE_COUNT <= 8, "vehicle states must fit in 3 bits");

/* ================= Other buddies: add your layouts below ============ */

#endif /* APP_DATA_H */
