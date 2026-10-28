/*
 * imu_math.c
 * Subsystem 4 -- pure maths for the GY-511 IMU.  See imu_math.h.
 * Only unsigned types are used for shifts and masks (Barr-C 5.3).
 */
#include "imu_math.h"

#define RAD_TO_DEG        (57.29577951f)
#define DEG_TO_RAD        (0.01745329252f)
#define FULL_TURN_DEG     (360.0f)
#define BAM8_STEPS        (256.0f)
#define OB8_CENTRE        (128)
#define OB8_STEPS_PER_DEG (2.0f)
#define MIN_GRAVITY_G     (0.5f)
#define MIN_HORIZ_FIELD2  (1.0e-6f)

/* ------------------------------------------------------------------ */
/* Small self-contained float maths.                                   */
/* The project links without libm (-ffreestanding, no -lm), so sqrtf,  */
/* atan2f and sinf are provided here.  Accuracy is checked against     */
/* libm by the host tests (H-30 .. H-33).                              */
/* ------------------------------------------------------------------ */
#define M_PI_F        (3.14159265f)
#define M_HALF_PI_F   (1.57079633f)
#define M_TWO_PI_F    (6.28318531f)
#define M_PI_6_F      (0.52359878f)
#define M_SQRT3_F     (1.73205081f)
#define M_TAN_PI_12_F (0.26794919f)
#define SQRT_ITER     4U

float imu_fabsf(float x)
{
    return (x < 0.0f) ? -x : x;
}

float imu_sqrtf(float x)
{
    float scale = 1.0f;
    float g;
    uint32_t i;

    if (x <= 0.0f) {
        return 0.0f;
    }
    while (x >= 4.0f) {              /* bring x into [1, 4) */
        x *= 0.25f;
        scale *= 2.0f;
    }
    while (x < 1.0f) {
        x *= 4.0f;
        scale *= 0.5f;
    }
    g = 0.5f * (1.0f + x);           /* first guess, then Newton-Raphson */
    for (i = 0U; i < SQRT_ITER; i++) {
        g = 0.5f * (g + (x / g));
    }
    return g * scale;
}

float imu_sinf(float x)
{
    float x2;
    if ((x > M_PI_F) || (x < -M_PI_F)) {                  /* reduce to [-pi, pi] */
        float k = x / M_TWO_PI_F;
        int32_t n = (int32_t)((k >= 0.0f) ? (k + 0.5f) : (k - 0.5f));
        x -= (float)n * M_TWO_PI_F;
    }
    if (x > M_HALF_PI_F) {                                /* fold to [-pi/2, pi/2] */
        x = M_PI_F - x;
    } else if (x < -M_HALF_PI_F) {
        x = -M_PI_F - x;
    }
    x2 = x * x;
    return x * (1.0f + (x2 * (-1.0f / 6.0f + (x2 * (1.0f / 120.0f +
           (x2 * (-1.0f / 5040.0f + (x2 * (1.0f / 362880.0f)))))))));
}

/* atan for t in [-1, 1] */
static float atan_unit(float t)
{
    float offset = 0.0f;
    float t2;
    if (t > M_TAN_PI_12_F) {                  /* atan(t) = pi/6 + atan((sqrt3*t-1)/(sqrt3+t)) */
        t = ((M_SQRT3_F * t) - 1.0f) / (M_SQRT3_F + t);
        offset = M_PI_6_F;
    } else if (t < -M_TAN_PI_12_F) {
        t = ((M_SQRT3_F * t) + 1.0f) / (M_SQRT3_F - t);
        offset = -M_PI_6_F;
    }
    t2 = t * t;
    return offset + (t * (1.0f + (t2 * (-1.0f / 3.0f + (t2 * (1.0f / 5.0f +
           (t2 * (-1.0f / 7.0f + (t2 * (1.0f / 9.0f + (t2 * (-1.0f / 11.0f)))))))))))); 
}

float imu_atan2f(float y, float x)
{
    float ax = imu_fabsf(x);
    float ay = imu_fabsf(y);
    float a;

    if ((ax == 0.0f) && (ay == 0.0f)) {
        return 0.0f;
    }
    if (ay <= ax) {
        a = atan_unit(ay / ax);
    } else {
        a = M_HALF_PI_F - atan_unit(ax / ay);
    }
    if (x < 0.0f) {
        a = M_PI_F - a;
    }
    return (y < 0.0f) ? -a : a;
}

void imu_map_axes(const imu_axis_map_t *map, const float s[3], float v[3])
{
    uint32_t i;
    for (i = 0U; i < 3U; i++) {
        v[i] = map->sign[i] * s[map->src[i]];
    }
}

float imu_accel_counts_to_g(int16_t raw)
{
    return (float)raw * IMU_ACCEL_G_PER_COUNT;
}

void imu_mag_counts_to_gauss(const int16_t raw[3], float gauss[3])
{
    gauss[0] = (float)raw[0] * IMU_MAG_GAUSS_PER_LSB_XY;
    gauss[1] = (float)raw[1] * IMU_MAG_GAUSS_PER_LSB_XY;
    gauss[2] = (float)raw[2] * IMU_MAG_GAUSS_PER_LSB_Z;
}

float imu_vec_magnitude(const float v[3])
{
    return imu_sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

float imu_pitch_deg(const float f[3], float long_accel_g)
{
    float fx = f[0] - long_accel_g;
    float lateral = imu_sqrtf(f[1] * f[1] + f[2] * f[2]);
    return imu_atan2f(fx, lateral) * RAD_TO_DEG;
}

float imu_roll_deg(const float f[3])
{
    return imu_atan2f(f[1], f[2]) * RAD_TO_DEG;
}

float imu_heading_deg(const float f[3], const float m[3])
{
    float fm = imu_vec_magnitude(f);
    float up[3];
    float dot;
    float mh[3];
    float east_x;
    float heading;
    uint32_t i;

    if (fm < MIN_GRAVITY_G) {
        return -1.0f;
    }
    for (i = 0U; i < 3U; i++) {
        up[i] = f[i] / fm;
    }
    dot = m[0] * up[0] + m[1] * up[1] + m[2] * up[2];
    for (i = 0U; i < 3U; i++) {
        mh[i] = m[i] - dot * up[i];          /* horizontal part of the field */
    }
    if ((mh[0] * mh[0] + mh[1] * mh[1] + mh[2] * mh[2]) < MIN_HORIZ_FIELD2) {
        return -1.0f;
    }
    east_x  = mh[1] * up[2] - mh[2] * up[1]; /* forward axis . (north x up)  */
    heading = imu_atan2f(east_x, mh[0]) * RAD_TO_DEG;
    if (heading < 0.0f) {
        heading += FULL_TURN_DEG;
    }
    if (heading >= FULL_TURN_DEG) {
        heading -= FULL_TURN_DEG;
    }
    return heading;
}

/* ---------------- compact encodings ---------------- */
uint8_t imu_deg_to_bam8(float deg)
{
    int32_t turns = (int32_t)(deg / FULL_TURN_DEG);      /* truncates toward zero */
    float d = deg - ((float)turns * FULL_TURN_DEG);
    uint32_t code;
    if (d < 0.0f) {
        d += FULL_TURN_DEG;
    }
    if (d >= FULL_TURN_DEG) {
        d -= FULL_TURN_DEG;
    }
    code = (uint32_t)(d * (BAM8_STEPS / FULL_TURN_DEG) + 0.5f);
    return (uint8_t)(code & 0xFFUL);          /* 256 wraps to 0 */
}

float imu_bam8_to_deg(uint8_t code)
{
    return (float)code * (FULL_TURN_DEG / BAM8_STEPS);
}

int32_t imu_bam8_diff(uint8_t a, uint8_t b)
{
    uint8_t d = (uint8_t)(a - b);             /* modular arithmetic, unsigned */
    return (d > 127U) ? ((int32_t)d - 256) : (int32_t)d;
}

uint8_t imu_deg_to_ob8(float deg)
{
    float x = deg * OB8_STEPS_PER_DEG;
    int32_t steps = (int32_t)((x >= 0.0f) ? (x + 0.5f) : (x - 0.5f));
    int32_t code = steps + OB8_CENTRE;
    if (code < 0) { code = 0; }
    if (code > 255) { code = 255; }
    return (uint8_t)code;
}

float imu_ob8_to_deg(uint8_t code)
{
    return (float)((int32_t)code - OB8_CENTRE) / OB8_STEPS_PER_DEG;
}

/* ---------------- calibration ---------------- */
void imu_cal_reset(imu_cal_acc_t *c)
{
    uint32_t i;
    c->n = 0U;
    for (i = 0U; i < 3U; i++) {
        c->sum[i] = 0.0f;
        c->sumsq[i] = 0.0f;
    }
}

void imu_cal_add(imu_cal_acc_t *c, const float f[3])
{
    uint32_t i;
    for (i = 0U; i < 3U; i++) {
        c->sum[i]   += f[i];
        c->sumsq[i] += f[i] * f[i];
    }
    c->n++;
}

imu_cal_result_t imu_cal_finish(const imu_cal_acc_t *c,
                                float *pitch0_deg, float *roll0_deg)
{
    float mean[3];
    float nf;
    float mag;
    uint32_t i;

    if (c->n < IMU_CAL_SAMPLES) {
        return IMU_CAL_PENDING;
    }
    nf = (float)c->n;
    for (i = 0U; i < 3U; i++) {
        float var;
        mean[i] = c->sum[i] / nf;
        var = (c->sumsq[i] / nf) - (mean[i] * mean[i]);
        if (var < 0.0f) { var = 0.0f; }
        if (imu_sqrtf(var) > IMU_CAL_STD_MAX_G) {
            return IMU_CAL_UNSTEADY;
        }
    }
    mag = imu_vec_magnitude(mean);
    if (mag < IMU_CAL_GRAVITY_MIN_G || mag > IMU_CAL_GRAVITY_MAX_G) {
        return IMU_CAL_BAD_GRAVITY;
    }
    *pitch0_deg = imu_pitch_deg(mean, 0.0f);
    *roll0_deg  = imu_roll_deg(mean);
    return IMU_CAL_OK;
}

/* ---------------- shock ---------------- */
imu_shock_t imu_shock_classify(float magnitude_g)
{
    if (magnitude_g >= IMU_SHOCK_CRITICAL_G) { return IMU_SHOCK_CRITICAL; }
    if (magnitude_g >= IMU_SHOCK_MINOR_G)    { return IMU_SHOCK_MINOR; }
    return IMU_SHOCK_NONE;
}

/* ---------------- hump ---------------- */
void imu_hump_reset(imu_hump_t *h)
{
    h->phase = IMU_HUMP_IDLE;
    h->above_cnt = 0U;
    h->flat_cnt = 0U;
    h->samples = 0U;
    h->pre_mm = 0.0f;
    h->height_mm = 0.0f;
    h->peak_mm = 0.0f;
}

imu_hump_event_t imu_hump_update(imu_hump_t *h, float pitch_deg, float delta_mm,
                                 float *peak_mm)
{
    float rise = imu_sinf(pitch_deg * DEG_TO_RAD) * delta_mm;

    if (h->phase == IMU_HUMP_IDLE) {
        if (pitch_deg > IMU_HUMP_ENTRY_DEG) {
            h->above_cnt++;
            h->pre_mm += rise;
            if (h->above_cnt >= IMU_HUMP_ENTRY_SAMPLES) {
                h->phase = IMU_HUMP_TRACKING;
                h->height_mm = h->pre_mm;
                h->peak_mm = h->pre_mm;
                h->flat_cnt = 0U;
                h->samples = 0U;
                return IMU_HUMP_STARTED;
            }
        } else {
            h->above_cnt = 0U;
            h->pre_mm = 0.0f;
        }
        return IMU_HUMP_NONE;
    }

    /* TRACKING */
    h->height_mm += rise;
    if (h->height_mm > h->peak_mm) {
        h->peak_mm = h->height_mm;
    }
    h->samples++;
    if (imu_fabsf(pitch_deg) < IMU_HUMP_EXIT_DEG) {
        h->flat_cnt++;
    } else {
        h->flat_cnt = 0U;
    }

    if ((h->flat_cnt >= IMU_HUMP_EXIT_SAMPLES) &&
        (h->height_mm <= (h->peak_mm * IMU_HUMP_RETURN_FRAC))) {
        *peak_mm = h->peak_mm;
        h->phase = IMU_HUMP_IDLE;
        h->above_cnt = 0U;
        h->pre_mm = 0.0f;
        return (h->peak_mm >= IMU_HUMP_MIN_MM) ? IMU_HUMP_DONE_VALID
                                                : IMU_HUMP_DONE_INVALID;
    }
    if (h->samples >= IMU_HUMP_TIMEOUT_SAMPLES) {
        *peak_mm = h->peak_mm;
        h->phase = IMU_HUMP_IDLE;
        h->above_cnt = 0U;
        h->pre_mm = 0.0f;
        return IMU_HUMP_DONE_INVALID;
    }
    return IMU_HUMP_NONE;
}
