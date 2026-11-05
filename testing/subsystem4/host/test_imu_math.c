/*
 * test_imu_math.c
 * Host-side (PC) unit tests for subsystem 4: imu_math.c and app_data.h.
 * Build and run:   make        (in this folder)
 * Exit code 0 = all passed.  Record the output in ../TEST_LOG.md.
 */
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "app_data.h"
#include "imu_math.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(id, cond, desc) do { \
    if (cond) { g_pass++; printf("[HOST] %-8s PASS  %s\n", (id), (desc)); } \
    else      { g_fail++; printf("[HOST] %-8s FAIL  %s  (line %d)\n", (id), (desc), __LINE__); } \
} while (0)

#define NEAR(a, b, tol) (fabsf((float)(a) - (float)(b)) <= (float)(tol))
#define DEG2RAD 0.01745329252f
#define PI_D    3.14159265358979323846

/* ---- orientation model used to build test vectors --------------------
 * World frame (East, North, Up).  Vehicle heading psi (clockwise from N),
 * pitch theta (nose up +), roll phi (left side up +).  Returns the world
 * vector V expressed in the vehicle frame (X fwd, Y left, Z up).        */
static void to_body(float psi_d, float th_d, float ph_d, const float V[3], float out[3])
{
    float psi = psi_d * DEG2RAD, th = th_d * DEG2RAD, ph = ph_d * DEG2RAD;
    float xl[3] = { sinf(psi), cosf(psi), 0.0f };
    float yl[3] = { -cosf(psi), sinf(psi), 0.0f };
    float up[3] = { 0.0f, 0.0f, 1.0f };
    float x1[3], z1[3], y2[3], z2[3];
    int i;
    for (i = 0; i < 3; i++) {
        x1[i] = xl[i] * cosf(th) + up[i] * sinf(th);
        z1[i] = up[i] * cosf(th) - xl[i] * sinf(th);
        y2[i] = yl[i] * cosf(ph) + z1[i] * sinf(ph);
        z2[i] = z1[i] * cosf(ph) - yl[i] * sinf(ph);
    }
    out[0] = V[0]*x1[0] + V[1]*x1[1] + V[2]*x1[2];
    out[1] = V[0]*y2[0] + V[1]*y2[1] + V[2]*y2[2];
    out[2] = V[0]*z2[0] + V[1]*z2[1] + V[2]*z2[2];
}

static uint32_t lcg = 12345U;
static float noise(float amp)   /* deterministic pseudo-noise in [-amp, amp] */
{
    lcg = lcg * 1664525U + 1013904223U;
    return amp * (((float)(lcg >> 8) / 8388608.0f) - 1.0f);
}

int main(void)
{
    int psi, th, ph;
    int bad;
    float f[3], m[3];
    const float g_world[3] = { 0.0f, 0.0f, 1.0f };
    const float m_world[3] = { 0.0f, 0.25f, -0.05f };   /* ~0.25 G north, slight dip */

    /* --- conversions --- */
    CHECK("H-01", NEAR(imu_accel_counts_to_g(8000), 1.0f, 0.001f), "8000 raw counts = 1 g at +-4 g");
    CHECK("H-02", NEAR(imu_accel_counts_to_g(-8000), -1.0f, 0.001f), "negative counts convert");
    {
        int16_t raw[3] = { 1100, -550, 980 };
        float g[3];
        imu_mag_counts_to_gauss(raw, g);
        CHECK("H-03", NEAR(g[0], 1.0f, 0.001f) && NEAR(g[1], -0.5f, 0.001f) && NEAR(g[2], 1.0f, 0.001f),
              "mag counts -> gauss (XY 1100, Z 980 LSB/G)");
    }

    /* --- pitch / roll / heading recovered from a rotated world --- */
    bad = 0;
    for (th = -25; th <= 25; th += 10) {
        for (ph = -20; ph <= 20; ph += 20) {
            to_body(0.0f, (float)th, (float)ph, g_world, f);
            if (!NEAR(imu_pitch_deg(f, 0.0f), th, 0.05f)) { bad++; }
            if (!NEAR(imu_roll_deg(f), ph, 0.05f)) { bad++; }
        }
    }
    CHECK("H-04", bad == 0, "pitch and roll recovered for 15 tilt combinations");

    bad = 0;
    for (psi = 0; psi < 360; psi += 10) {
        for (th = -25; th <= 25; th += 10) {
            for (ph = -20; ph <= 20; ph += 20) {
                float h, d;
                to_body((float)psi, (float)th, (float)ph, g_world, f);
                to_body((float)psi, (float)th, (float)ph, m_world, m);
                h = imu_heading_deg(f, m);
                d = fabsf(h - (float)psi);
                if (d > 180.0f) { d = 360.0f - d; }
                if (h < 0.0f || d > 0.3f) { bad++; }
            }
        }
    }
    CHECK("H-05", bad == 0, "tilt-compensated heading correct for 36 headings x 15 tilts");

    {   /* flat car: the four compass points */
        float fl[3] = { 0.0f, 0.0f, 1.0f };
        float mn[3] = { 0.25f, 0.0f, -0.05f };    /* facing north: field along +X  */
        float me[3] = { 0.0f, 0.25f, -0.05f };    /* facing east: north is to left */
        float ms[3] = { -0.25f, 0.0f, -0.05f };
        float mw[3] = { 0.0f, -0.25f, -0.05f };
        CHECK("H-06", NEAR(imu_heading_deg(fl, mn), 0.0f, 0.1f) && NEAR(imu_heading_deg(fl, me), 90.0f, 0.1f) &&
                      NEAR(imu_heading_deg(fl, ms), 180.0f, 0.1f) && NEAR(imu_heading_deg(fl, mw), 270.0f, 0.1f),
              "flat car: N=0, E=90, S=180, W=270");
        {
            float zero[3] = { 0.0f, 0.0f, 0.0f };
            CHECK("H-07", imu_heading_deg(fl, zero) < 0.0f, "no horizontal field -> heading invalid (-1)");
        }
    }

    {   /* braking disturbance: 0.1 g deceleration */
        float fb[3];
        to_body(0.0f, 0.0f, 0.0f, g_world, fb);
        fb[0] += 0.1f;                                   /* braking adds +a/g on the forward axis */
        CHECK("H-08", NEAR(imu_pitch_deg(fb, 0.0f), 5.7f, 0.1f), "0.1 g of braking reads as ~5.7 deg false pitch");
        CHECK("H-09", NEAR(imu_pitch_deg(fb, 0.1f), 0.0f, 0.05f), "encoder acceleration removes the false pitch");
    }

    /* --- axis mapping --- */
    {
        imu_axis_map_t map = { {1U, 0U, 2U}, {1.0f, -1.0f, 1.0f} };  /* fwd=sensor Y, left=-sensor X */
        float s[3] = { 0.2f, 0.7f, 1.0f }, v[3];
        imu_map_axes(&map, s, v);
        CHECK("H-10", NEAR(v[0], 0.7f, 1e-6f) && NEAR(v[1], -0.2f, 1e-6f) && NEAR(v[2], 1.0f, 1e-6f),
              "axis map swaps and flips correctly");
    }

    /* --- compact encodings --- */
    CHECK("H-11", imu_deg_to_bam8(0.0f) == 0 && imu_deg_to_bam8(90.0f) == 64 &&
                  imu_deg_to_bam8(180.0f) == 128 && imu_deg_to_bam8(270.0f) == 192 &&
                  imu_deg_to_bam8(359.9f) == 0 && imu_deg_to_bam8(-90.0f) == 192,
          "BAM8 encode incl. wrap (359.9 -> 0, -90 -> 192)");
    CHECK("H-12", imu_bam8_diff(5, 250) == 11 && imu_bam8_diff(250, 5) == -11 && imu_bam8_diff(130, 0) == -126,
          "BAM8 difference wraps across 360 deg");
    bad = 0;
    for (psi = 0; psi < 360; psi++) {
        float back = imu_bam8_to_deg(imu_deg_to_bam8((float)psi));
        float d = fabsf(back - (float)psi);
        if (d > 180.0f) { d = 360.0f - d; }
        if (d > 0.71f) { bad++; }
    }
    CHECK("H-13", bad == 0, "BAM8 round-trip error <= 0.71 deg for every whole degree");
    CHECK("H-14", imu_deg_to_ob8(0.0f) == 128 && imu_deg_to_ob8(10.0f) == 148 && imu_deg_to_ob8(-10.0f) == 108 &&
                  imu_deg_to_ob8(100.0f) == 255 && imu_deg_to_ob8(-100.0f) == 0 && NEAR(imu_ob8_to_deg(148), 10.0f, 1e-6f),
          "offset-binary pitch/roll encode, clamp and decode");

    /* --- calibration --- */
    {
        imu_cal_acc_t c;
        float p0 = 0.0f, r0 = 0.0f, ft[3];
        int i;
        imu_cal_reset(&c);
        CHECK("H-15", imu_cal_finish(&c, &p0, &r0) == IMU_CAL_PENDING, "calibration pending until enough samples");
        for (i = 0; i < (int)IMU_CAL_SAMPLES; i++) {
            ft[0] = 0.05f + noise(0.003f); ft[1] = -0.02f + noise(0.003f); ft[2] = 0.998f + noise(0.003f);
            imu_cal_add(&c, ft);
        }
        CHECK("H-16", imu_cal_finish(&c, &p0, &r0) == IMU_CAL_OK && NEAR(p0, 2.86f, 0.3f) && NEAR(r0, -1.15f, 0.3f),
              "steady, level-ish car: calibration OK, pitch0/roll0 captured");
        imu_cal_reset(&c);
        for (i = 0; i < (int)IMU_CAL_SAMPLES; i++) {
            ft[0] = noise(0.2f); ft[1] = noise(0.2f); ft[2] = 1.0f + noise(0.2f);
            imu_cal_add(&c, ft);
        }
        CHECK("H-17", imu_cal_finish(&c, &p0, &r0) == IMU_CAL_UNSTEADY, "moving car: calibration rejected as UNSTEADY");
        imu_cal_reset(&c);
        for (i = 0; i < (int)IMU_CAL_SAMPLES; i++) {
            ft[0] = 0.0f; ft[1] = 0.0f; ft[2] = 0.5f;
            imu_cal_add(&c, ft);
        }
        CHECK("H-18", imu_cal_finish(&c, &p0, &r0) == IMU_CAL_BAD_GRAVITY, "|a| = 0.5 g: BAD_GRAVITY (wrong range/scale)");
    }

    /* --- shock --- */
    CHECK("H-19", imu_shock_classify(1.0f) == IMU_SHOCK_NONE && imu_shock_classify(1.49f) == IMU_SHOCK_NONE &&
                  imu_shock_classify(1.5f) == IMU_SHOCK_MINOR && imu_shock_classify(2.49f) == IMU_SHOCK_MINOR &&
                  imu_shock_classify(2.5f) == IMU_SHOCK_CRITICAL && imu_shock_classify(4.0f) == IMU_SHOCK_CRITICAL,
          "shock thresholds: <1.5 none, 1.5-2.5 minor, >=2.5 critical");

    /* --- hump height --- */
    {
        /* 150 mm/s, 10 ms samples -> 1.5 mm per sample.  Ramp 100 samples (150 mm of
         * surface) at 11.54 deg = 30 mm rise, 50 samples plateau, 100 samples down.
         * The crossing takes 2.5 s: it must stay under the 3 s hump timeout (A4.5). */
        const float slope = 11.54f;
        imu_hump_t h;
        imu_hump_event_t ev;
        float peak = 0.0f;
        int i, started = 0, done_valid = 0, done_invalid = 0;
        float bias[2] = { 0.0f, 0.3f };
        int b;

        for (b = 0; b < 2; b++) {
            imu_hump_reset(&h);
            started = 0; done_valid = 0; done_invalid = 0; peak = 0.0f;
            for (i = 0; i < 100 + 100 + 50 + 100 + 50; i++) {
                float pitch;
                if (i < 100)            { pitch = 0.0f; }
                else if (i < 200)       { pitch = slope; }
                else if (i < 250)       { pitch = 0.0f; }
                else if (i < 350)       { pitch = -slope; }
                else                    { pitch = 0.0f; }
                pitch += bias[b] + noise(0.3f);
                ev = imu_hump_update(&h, pitch, 1.5f, &peak);
                if (ev == IMU_HUMP_STARTED) { started++; }
                if (ev == IMU_HUMP_DONE_VALID) { done_valid++; }
                if (ev == IMU_HUMP_DONE_INVALID) { done_invalid++; }
            }
            if (b == 0) {
                CHECK("H-20", started == 1 && done_valid == 1 && done_invalid == 0 && NEAR(peak, 30.0f, 1.0f),
                      "hump (30 mm): one start, one valid result, peak within 1 mm");
            } else {
                CHECK("H-21", started == 1 && done_valid == 1 && NEAR(peak, 30.0f, 2.5f),
                      "hump with +0.3 deg pitch bias and noise: peak within 2.5 mm");
            }
        }

        /* ramp only, never returns -> timeout, invalid */
        imu_hump_reset(&h);
        started = 0; done_invalid = 0; done_valid = 0;
        for (i = 0; i < 600; i++) {
            float pitch = (i < 100) ? slope : 0.0f;
            ev = imu_hump_update(&h, pitch, 1.5f, &peak);
            if (ev == IMU_HUMP_STARTED) { started++; }
            if (ev == IMU_HUMP_DONE_INVALID) { done_invalid++; }
            if (ev == IMU_HUMP_DONE_VALID) { done_valid++; }
        }
        CHECK("H-22", started == 1 && done_invalid == 1 && done_valid == 0,
              "ramp that never comes back down ends by timeout as INVALID");

        /* noise alone never triggers */
        imu_hump_reset(&h);
        started = 0;
        for (i = 0; i < 2000; i++) {
            if (imu_hump_update(&h, noise(1.0f), 1.5f, &peak) == IMU_HUMP_STARTED) { started++; }
        }
        CHECK("H-23", started == 0, "+-1 deg pitch noise for 20 s never starts a hump");

        /* two consecutive humps both reported */
        imu_hump_reset(&h);
        started = 0; done_valid = 0;
        for (i = 0; i < 1200; i++) {
            float pitch = 0.0f;
            int k = i % 600;
            if (k >= 50 && k < 150) { pitch = slope; }
            else if (k >= 200 && k < 300) { pitch = -slope; }
            ev = imu_hump_update(&h, pitch, 1.5f, &peak);
            if (ev == IMU_HUMP_STARTED) { started++; }
            if (ev == IMU_HUMP_DONE_VALID) { done_valid++; }
        }
        CHECK("H-24", started == 2 && done_valid == 2, "two humps in a row are both detected and measured");
    }

    /* --- packed formats (app_data.h) --- */
    {
        int t, hgt, v, s;
        bad = 0;
        for (t = 0; t < (int)S4_EVT_COUNT; t++) {
            for (hgt = 0; hgt <= 127; hgt++) {
                for (v = 0; v <= 1; v++) {
                    for (s = 0; s < 32; s++) {
                        uint16_t w = s4_evt_pack((s4_evt_type_t)t, (uint32_t)hgt, (uint32_t)v, (uint32_t)s);
                        if (s4_evt_type(w) != (s4_evt_type_t)t || s4_evt_height_mm(w) != (uint32_t)hgt ||
                            s4_evt_valid(w) != (uint32_t)v || s4_evt_seq(w) != (uint32_t)s) { bad++; }
                    }
                }
            }
        }
        CHECK("H-25", bad == 0, "imu_evt word round-trips for every legal field value");
        CHECK("H-26", s4_evt_height_mm(s4_evt_pack(S4_EVT_HUMP_DONE, 200U, 1U, 0U)) == 127U,
              "height above 127 mm saturates instead of corrupting other fields");
        CHECK("H-27", s4_evt_seq(s4_evt_pack(S4_EVT_IMU_READY, 0U, 0U, 33U)) == 1U &&
                      s4_evt_type(s4_evt_pack(S4_EVT_IMU_READY, 0U, 0U, 33U)) == S4_EVT_IMU_READY,
              "out-of-range SEQ is masked and does not leak into neighbours");
        bad = 0;
        for (t = 0; t < (int)S4_ST_COUNT; t++) {
            for (s = 0; s < 16; s++) {
                uint32_t w = s4_snap_pack(0xA5U, 0x80U, 0x7FU, (s4_imu_state_t)t, (uint32_t)(s & 1), (uint32_t)s);
                if (s4_snap_heading(w) != 0xA5U || s4_snap_pitch(w) != 0x80U || s4_snap_roll(w) != 0x7FU ||
                    s4_snap_state(w) != (s4_imu_state_t)t || s4_snap_valid(w) != (uint32_t)(s & 1) ||
                    s4_snap_seq(w) != (uint32_t)s) { bad++; }
            }
        }
        CHECK("H-28", bad == 0, "imu snapshot word round-trips (all states, all SEQ)");
        CHECK("H-29", sizeof(uint16_t) == 2U && sizeof(uint32_t) == 4U, "fixed-width sizes as expected");
    }


    /* --- the self-contained maths (the Pico build has no libm) --- */
    {
        double worst = 0.0;
        float x;
        int ok = 1;
        for (x = 1.0e-6f; x < 1.0e6f; x *= 1.07f) {
            double rel = fabs((double)imu_sqrtf(x) - sqrt((double)x)) / sqrt((double)x);
            if (rel > worst) { worst = rel; }
        }
        ok = (worst < 1.0e-6) && (imu_sqrtf(0.0f) == 0.0f) && (imu_sqrtf(-4.0f) == 0.0f);
        CHECK("H-30", ok, "imu_sqrtf: relative error < 1e-6 from 1e-6 to 1e6, 0 and negative -> 0");
        printf("[HOST]          info  sqrt worst relative error = %.2e\n", worst);
    }
    {
        double worst = 0.0;
        float x;
        for (x = -20.0f; x <= 20.0f; x += 0.001f) {
            double e = fabs((double)imu_sinf(x) - sin((double)x));
            if (e > worst) { worst = e; }
        }
        CHECK("H-31", worst < 1.0e-5, "imu_sinf: absolute error < 1e-5 for -20..20 rad");
        printf("[HOST]          info  sin worst absolute error = %.2e\n", worst);
    }
    {
        double worst = 0.0;
        float yv, xv;
        for (yv = -10.0f; yv <= 10.0f; yv += 0.37f) {
            for (xv = -10.0f; xv <= 10.0f; xv += 0.41f) {
                double e = fabs((double)imu_atan2f(yv, xv) - atan2((double)yv, (double)xv));
                if (e > PI_D) { e = fabs(e - 2.0 * PI_D); }      /* +-pi branch cut */
                if (e > worst) { worst = e; }
            }
        }
        CHECK("H-32", worst < 1.0e-5 && imu_atan2f(0.0f, 0.0f) == 0.0f &&
                      NEAR(imu_atan2f(1.0f, 0.0f), 1.5707963f, 1e-5f) &&
                      NEAR(imu_atan2f(0.0f, -1.0f), 3.1415927f, 1e-5f) &&
                      NEAR(imu_atan2f(-1.0f, 0.0f), -1.5707963f, 1e-5f),
              "imu_atan2f: error < 1e-5 rad (0.0006 deg) over a grid, axes and origin correct");
        printf("[HOST]          info  atan2 worst absolute error = %.2e rad (%.5f deg)\n", worst, worst * 57.29578);
    }
    CHECK("H-33", imu_fabsf(-2.5f) == 2.5f && imu_deg_to_bam8(720.0f) == 0 && imu_deg_to_bam8(-450.0f) == 192 &&
                  imu_deg_to_bam8(725.0f) == 4 && imu_deg_to_bam8(-360.0f) == 0,
          "fabs and BAM8 normalisation for angles beyond +-360 deg");

    printf("[HOST] SUMMARY %d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
