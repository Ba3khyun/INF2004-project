/*
 * imu_math.h
 * Subsystem 4 -- pure maths for the GY-511 (LSM303DLHC) IMU.
 *
 * No kernel or hardware calls in here, so it is unit-tested on a PC
 * (testing/subsystem4/host/) before it ever runs on the Pico.
 *
 * Vehicle frame used everywhere:  X = forward, Y = left, Z = up.
 *  - pitch  : nose-up is positive
 *  - roll   : left-side-up is positive
 *  - heading: degrees clockwise from magnetic north (0 = N, 90 = E)
 */
#ifndef IMU_MATH_H
#define IMU_MATH_H

#include <stdint.h>

/* ---- sensor scaling (see accel range +-4 g, high-resolution mode) ---- */
#define IMU_ACCEL_G_PER_COUNT     (0.002f / 16.0f) /* 2 mg per 12-bit LSB, left-justified */
#define IMU_MAG_GAUSS_PER_LSB_XY  (1.0f / 1100.0f) /* +-1.3 G range */
#define IMU_MAG_GAUSS_PER_LSB_Z   (1.0f / 980.0f)
#define IMU_MAG_OVERFLOW_RAW      (-4096)          /* DLHC overflow marker */

/* ---- thresholds: PROVISIONAL, to be tuned with logged data (A4.x) ---- */
#define IMU_SHOCK_MINOR_G         1.5f   /* A4.7: normal driving must stay below this   */
#define IMU_SHOCK_CRITICAL_G      2.5f   /* collision -> SAF_COLLISION                  */
#define IMU_HUMP_ENTRY_DEG        5.0f   /* pitch above this starts a hump              */
#define IMU_HUMP_EXIT_DEG         2.0f   /* |pitch| below this counts as flat           */
#define IMU_HUMP_ENTRY_SAMPLES    3U
#define IMU_HUMP_EXIT_SAMPLES     3U
#define IMU_HUMP_TIMEOUT_SAMPLES  300U   /* 3 s at 100 Hz                               */
#define IMU_HUMP_RETURN_FRAC      0.2f   /* height must fall back below 20 % of peak    */
#define IMU_HUMP_MIN_MM           2.0f   /* smaller peaks are treated as noise          */
#define IMU_CAL_SAMPLES           200U   /* 2 s at 100 Hz                               */
#define IMU_CAL_STD_MAX_G         0.02f  /* per-axis standard deviation limit           */
#define IMU_CAL_GRAVITY_MIN_G     0.8f
#define IMU_CAL_GRAVITY_MAX_G     1.2f

/* ---- axis mapping: which sensor axis is the vehicle's fwd/left/up ---- */
typedef struct {
    uint8_t src[3];    /* vehicle axis i takes sensor axis src[i] (0=X,1=Y,2=Z) */
    float   sign[3];   /* +1.0f or -1.0f                                        */
} imu_axis_map_t;

/* Default: board lying flat, sensor X forward, Y left, Z up.
 * CONFIRM with the tilt test (README step 5) and edit if different. */
#define IMU_AXIS_MAP_DEFAULT { {0U, 1U, 2U}, {1.0f, 1.0f, 1.0f} }

void  imu_map_axes(const imu_axis_map_t *map, const float s[3], float v[3]);

/* ---- float maths without libm (the project links without -lm) ---- */
float imu_fabsf(float x);
float imu_sqrtf(float x);
float imu_sinf(float x);           /* radians */
float imu_atan2f(float y, float x);/* radians, same quadrant rules as atan2f */

/* ---- unit conversion ---- */
float imu_accel_counts_to_g(int16_t raw);
void  imu_mag_counts_to_gauss(const int16_t raw[3], float gauss[3]);
float imu_vec_magnitude(const float v[3]);

/* ---- orientation (inputs in the VEHICLE frame) ---- */
/* long_accel_g: forward acceleration from the encoders, subtracted from the
 * forward axis so braking does not read as pitch.  Pass 0.0f to disable. */
float imu_pitch_deg(const float f[3], float long_accel_g);
float imu_roll_deg(const float f[3]);
/* Tilt-compensated heading, 0..360.  Returns -1.0f if it cannot be computed. */
float imu_heading_deg(const float f[3], const float m[3]);

/* ---- compact encodings (see app_data.h) ---- */
uint8_t imu_deg_to_bam8(float deg);          /* 256 steps = 360 deg           */
float   imu_bam8_to_deg(uint8_t code);
int32_t imu_bam8_diff(uint8_t a, uint8_t b); /* a - b wrapped to -128..127    */
uint8_t imu_deg_to_ob8(float deg);           /* offset-binary, 0.5 deg/step   */
float   imu_ob8_to_deg(uint8_t code);

/* ---- start-up calibration (level reference) ---- */
typedef struct {
    uint32_t n;
    float    sum[3];
    float    sumsq[3];
} imu_cal_acc_t;

typedef enum {
    IMU_CAL_PENDING = 0,
    IMU_CAL_OK,
    IMU_CAL_UNSTEADY,     /* the car moved during calibration         */
    IMU_CAL_BAD_GRAVITY   /* |a| not about 1 g: wrong range or scale  */
} imu_cal_result_t;

void imu_cal_reset(imu_cal_acc_t *c);
void imu_cal_add(imu_cal_acc_t *c, const float f[3]);
imu_cal_result_t imu_cal_finish(const imu_cal_acc_t *c,
                                float *pitch0_deg, float *roll0_deg);

/* ---- shock ---- */
typedef enum { IMU_SHOCK_NONE = 0, IMU_SHOCK_MINOR, IMU_SHOCK_CRITICAL } imu_shock_t;
imu_shock_t imu_shock_classify(float magnitude_g);

/* ---- hump height = running sum of sin(pitch) * distance ---- */
typedef enum { IMU_HUMP_IDLE = 0, IMU_HUMP_TRACKING } imu_hump_phase_t;
typedef enum {
    IMU_HUMP_NONE = 0,
    IMU_HUMP_STARTED,
    IMU_HUMP_DONE_VALID,
    IMU_HUMP_DONE_INVALID
} imu_hump_event_t;

typedef struct {
    imu_hump_phase_t phase;
    uint32_t above_cnt;
    uint32_t flat_cnt;
    uint32_t samples;
    float    pre_mm;     /* height gathered while the entry is debounced */
    float    height_mm;
    float    peak_mm;
} imu_hump_t;

void imu_hump_reset(imu_hump_t *h);
/* pitch_deg: level-corrected pitch.  delta_mm: distance moved since the last
 * call (from the encoders).  *peak_mm is set when DONE_* is returned. */
imu_hump_event_t imu_hump_update(imu_hump_t *h, float pitch_deg, float delta_mm,
                                 float *peak_mm);

#endif /* IMU_MATH_H */
