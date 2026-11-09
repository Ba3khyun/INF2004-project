/*
 * imu_task.c
 * Subsystem 4 -- the IMU Processing Task.
 *
 * Every 10 ms (cyclic handler -> event flag) it reads the GY-511, then runs
 * the statechart of Section 6.4:
 *   INIT_CALIBRATING -> MONITORING <-> HUMP_TRACKING
 *   critical shock or I2C failure -> FAULT -> (reinit request) -> INIT_CALIBRATING
 *
 * Outputs (all compact, see app_data.h):
 *   s4_imu_snapshot  latest heading/pitch/roll/state, one 32-bit word
 *   imu_evt_q        discrete events (hump start/done, minor shock, fault)
 *   flg_safety       SAF_COLLISION on a critical shock
 *   flg_faults       FLT_IMU_* bits
 * The IMU never writes a motor command: the Vehicle Controller reacts to
 * HUMP_START (single-writer rule for motion_cmd_q).
 */
#include "subsystem4.h"
#include "imu_lsm303.h"
#include "imu_math.h"

/* RP2040 TIMER.TIMERAWL, free-running 1 MHz: used to time the task (NFR4.2). */
#define S4_TIMER_RAWL     (*(volatile UW *)0x40054028UL)
#define S4_DEBUG_CYCLES   100U          /* one console line per second */
#define S4_MM_S2_PER_G    9806.65f
#define S4_MAX_STEP_MM    1000U         /* larger distance jumps are ignored */

typedef struct {
    s4_imu_state_t state;
    uint32_t       fail_cnt;
    uint32_t       cal_attempts;
    uint32_t       snap_seq;
    uint32_t       evt_seq;
    uint32_t       shock_hold;
    float          pitch0_deg;
    float          roll0_deg;
    float          pitch_deg;
    float          roll_deg;
    float          heading_deg;
    UW             dist_prev;
    imu_cal_acc_t  cal;
    imu_hump_t     hump;
    imu_axis_map_t map;
#if S4_USE_BRAKE_COMP
    float          speed_prev_mm_s;
    float          accel_g_filt;
#endif
} s4_ctx_t;

static s4_ctx_t g;

/* statistics (written by the IMU task only, read by tests) */
static volatile UW g_cycles;
static volatile UW g_cycle_us_max;
static volatile UW g_period_us_max;
static volatile UW g_evt_dropped;
static volatile UW g_mag_ovf;
static volatile UW g_accel_mg;
static volatile UW g_mag_mgauss;
static volatile UW g_missed_ticks;
/* Monotonic ISR diagnostics: never reset by S4-U16 statistics test. */
static volatile UW g_handler_ticks;
static volatile UW g_tick_flag_errors;
static volatile UW g_reset_request;
/* IMU stall tracing: aligned, single-writer status snapshots. No I/O. */
static volatile ID g_imu_task_id = -1;
static volatile UW g_imu_stage = 0U;
static volatile UW g_imu_stage_us = 0U;
static volatile UW g_imu_wakeups = 0U;
static volatile UW g_imu_other_wait_errors = 0U;

/* Stage: 0=start, 1=wait tick, 2=woken, 3=accel I2C,
 * 4=mag I2C, 5=process sample, 6=cycle done,
 * 7=console output, 8=complete, 9=unexpected wait error. */
static void imu_diag_stage(UW stage)
{
    g_imu_stage_us = S4_TIMER_RAWL;
    g_imu_stage = stage;
}

EXPORT ID s4_diag_task_id(void) { return g_imu_task_id; }
EXPORT UW s4_diag_stage(void) { return g_imu_stage; }
EXPORT UW s4_diag_stage_age_ms(void)
{ return (S4_TIMER_RAWL - g_imu_stage_us) / 1000U; }
EXPORT UW s4_diag_wakeups(void) { return g_imu_wakeups; }
EXPORT UW s4_diag_other_wait_errors(void) { return g_imu_other_wait_errors; }


EXPORT volatile UW s4_imu_snapshot = 0U;
/* IMU task is the sole writer. VC reads these without stealing imu_evt_q. */
static volatile UW g_hump_start_count = 0U;
static volatile UW g_hump_done_word = 0U;
static UW g_hump_done_seq = 0U;


#if S4_DEBUG_PRINT
static const char *const k_state_name[S4_ST_COUNT] = {
    "INIT_CAL", "MONITOR", "HUMP", "SHOCK", "FAULT"
};
#endif

/* ------------------------------------------------------------------ */
/* Weak defaults so the subsystem builds before Buddy 2 provides these */
/* ------------------------------------------------------------------ */
__attribute__((weak)) UW motion_get_distance_mm(void)
{
    return 0U;
}

/* ------------------------------------------------------------------ */
/* Public read-only interface                                          */
/* ------------------------------------------------------------------ */
EXPORT UW s4_imu_get_snapshot(void)      { return s4_imu_snapshot; }
EXPORT UW s4_imu_hump_start_count(void) { return g_hump_start_count; }
EXPORT UW s4_imu_hump_done_word(void) { return g_hump_done_word; }

EXPORT s4_imu_state_t s4_imu_state(void) { return g.state; }
EXPORT UW s4_stat_cycles(void)           { return g_cycles; }
EXPORT UW s4_stat_cycle_us_max(void)     { return g_cycle_us_max; }
EXPORT UW s4_stat_period_us_max(void)    { return g_period_us_max; }
EXPORT UW s4_stat_events_dropped(void)   { return g_evt_dropped; }
EXPORT UW s4_stat_mag_overflow(void)     { return g_mag_ovf; }
EXPORT UW s4_stat_accel_mg(void)         { return g_accel_mg; }
EXPORT UW s4_stat_mag_mgauss(void)       { return g_mag_mgauss; }
EXPORT UW s4_stat_handler_ticks(void)   { return g_handler_ticks; }
EXPORT UW s4_stat_tick_flag_errors(void){ return g_tick_flag_errors; }
EXPORT UW s4_stat_wait_timeouts(void)   { return g_missed_ticks; }

EXPORT void s4_stat_reset(void)
{
    g_cycles = 0U;
    g_cycle_us_max = 0U;
    g_period_us_max = 0U;
    g_evt_dropped = 0U;
    g_reset_request = 1U;     /* the task restarts its period measurement */
}

EXPORT ER s4_imu_request_reinit(void)
{
    return tk_set_flg(flg_imu, S4_FLG_REINIT);
}

/* Cyclic handler (runs in handler context): only sets a flag. */
EXPORT void imu_cyc_handler(void *exinf)
{
    ER er;
    (void)exinf;
    g_handler_ticks++;  /* single ISR writer; 32-bit read-only outside handler */
    er = tk_set_flg(flg_imu, S4_FLG_TICK);
    if (er < E_OK) {
        g_tick_flag_errors++;
    }
}

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */
static void post_evt(s4_evt_type_t type, uint32_t height_mm, uint32_t valid)
{
    uint16_t w = s4_evt_pack(type, height_mm, valid, g.evt_seq);
    ER er;
    g.evt_seq++;
    /* Read-only event tap is independent of mbf_imu_evt capacity.
     * Do not synthesize or acknowledge a safety flag here. */
    if (type == S4_EVT_HUMP_START) {
        g_hump_start_count++;
    } else if (type == S4_EVT_HUMP_DONE) {
        g_hump_done_seq++;
        g_hump_done_word = ((g_hump_done_seq & 0xFFFFU) << 16) | (UW)w;
    }
    er = tk_snd_mbf(mbf_imu_evt, &w, (INT)sizeof(w), TMO_POL);
    if (er < E_OK) {
        g_evt_dropped++;           /* queue full: count it, never block */
    }
}

static void publish_snapshot(void)
{
    s4_imu_state_t shown = g.state;
    uint32_t valid = 0U;
    uint32_t heading_code = (uint32_t)imu_deg_to_bam8(g.heading_deg);
    uint32_t w;

    if ((g.state == S4_ST_MONITORING) || (g.state == S4_ST_HUMP_TRACKING)) {
        valid = 1U;
        if (g.shock_hold > 0U) {
            shown = S4_ST_SHOCK_DETECTED;
        }
    }
    w = s4_snap_pack(heading_code,
                     (uint32_t)imu_deg_to_ob8(g.pitch_deg),
                     (uint32_t)imu_deg_to_ob8(g.roll_deg),
                     shown, valid, g.snap_seq);
    g.snap_seq++;
    s4_imu_snapshot = w;           /* ONE aligned 32-bit store: readers never see a mix */
}

static void enter_fault(UW flt_bit)
{
    g.state = S4_ST_FAULT;
    imu_hump_reset(&g.hump);

    /*
     * The team has reserved only FLT_IMU_I2C
     * for an IMU communication failure.
     */
    if (flt_bit == FLT_IMU_I2C) {
        (void)tk_set_flg(flg_fault, FLT_IMU_I2C);
    }

    /* Always tell the Vehicle Controller about an IMU fault. */
    post_evt(S4_EVT_IMU_FAULT, 0U, 0U);

    tm_printf((UB*)
        "IMU: -> FAULT (reason=0x%X)\n",
        (UINT)flt_bit);
}

static void start_calibration(void)
{
    g.state = S4_ST_INIT_CALIBRATING;
    g.cal_attempts = 0U;
    g.fail_cnt = 0U;
    imu_cal_reset(&g.cal);
    imu_hump_reset(&g.hump);
    g.shock_hold = 0U;
}

static void try_reinit(void)
{
    ER er;

    er = imu_drv_init(s4_i2c_dev);

    if (er != E_OK) {
        (void)tk_set_flg(flg_fault, FLT_IMU_I2C);

        tm_printf((UB*)
            "IMU: reinit failed (err=%d)\n",
            (INT)er);
        return;
    }

    /* Clear only the IMU-owned I2C fault bit. */
    (void)tk_clr_flg(
        flg_fault,
        (UINT)(~FLT_IMU_I2C));

    start_calibration();

    tm_printf((UB*)
        "IMU: reinit OK, calibrating\n");
}

static void handle_calibration(const float f[3])
{
    imu_cal_result_t r;
    imu_cal_add(&g.cal, f);
    r = imu_cal_finish(&g.cal, &g.pitch0_deg, &g.roll0_deg);
    if (r == IMU_CAL_PENDING) {
        return;
    }
    if (r == IMU_CAL_OK) {
        g.state = S4_ST_MONITORING;
        imu_hump_reset(&g.hump);
        post_evt(S4_EVT_IMU_READY, 0U, 1U);
        tm_printf((UB*)"IMU: calibrated, level reference pitch0=%d roll0=%d (x0.1 deg)\n",
                  (INT)(g.pitch0_deg * 10.0f), (INT)(g.roll0_deg * 10.0f));
        return;
    }
    g.cal_attempts++;
    tm_printf((UB*)"IMU: calibration rejected (%s), attempt %u of %u\n",
              (r == IMU_CAL_UNSTEADY) ? "car moved" : "|a| not 1 g",
              (UINT)g.cal_attempts, (UINT)S4_CAL_MAX_ATTEMPTS);
    if (g.cal_attempts >= S4_CAL_MAX_ATTEMPTS) {
        /* Calibration failure has no dedicated bit in the team mask. */
        enter_fault(0U);
    } else {
        imu_cal_reset(&g.cal);
    }
}

static float step_distance_mm(void)
{
    UW now = motion_get_distance_mm();
    UW d = now - g.dist_prev;            /* modular: safe if the counter wraps */
    g.dist_prev = now;
    return (d > S4_MAX_STEP_MM) ? 0.0f : (float)d;
}

static void handle_monitoring(const float f[3], float mag_g, float delta_mm)
{
    imu_shock_t shock = imu_shock_classify(mag_g);
    imu_hump_event_t he;
    float peak = 0.0f;

    if (shock == IMU_SHOCK_CRITICAL) {
        /* Report collision through the team fault mask. */
        (void)tk_set_flg(flg_fault, FLT_COLLISION);

        /* Request emergency stop from the Safety Handler. */
        (void)tk_set_flg(flg_safety, SAFETY_REQ_COLLISION);
        g.state = S4_ST_FAULT;
        imu_hump_reset(&g.hump);
        tm_printf((UB*)"IMU: CRITICAL shock %d mg -> collision flag, FAULT\n", (INT)(mag_g * 1000.0f));
        return;
    }
    if ((shock == IMU_SHOCK_MINOR) && (g.shock_hold == 0U)) {
        post_evt(S4_EVT_SHOCK_MINOR, 0U, 0U);
        g.shock_hold = S4_SHOCK_HOLDOFF;
    } else if (g.shock_hold > 0U) {
        g.shock_hold--;
    }

    (void)f;
    he = imu_hump_update(&g.hump, g.pitch_deg, delta_mm, &peak);
    if (he == IMU_HUMP_STARTED) {
        g.state = S4_ST_HUMP_TRACKING;
        post_evt(S4_EVT_HUMP_START, 0U, 0U);              /* Vehicle Controller slows down */
    } else if ((he == IMU_HUMP_DONE_VALID) || (he == IMU_HUMP_DONE_INVALID)) {
        uint32_t mm = (uint32_t)(peak + 0.5f);
        g.state = S4_ST_MONITORING;
        post_evt(S4_EVT_HUMP_DONE, mm, (he == IMU_HUMP_DONE_VALID) ? 1U : 0U);
        tm_printf((UB*)"IMU: hump done, peak %u mm (%s)\n", (UINT)mm,
                  (he == IMU_HUMP_DONE_VALID) ? "valid" : "INVALID");
    }
}

/* One 10 ms cycle. */
static void run_cycle(void)
{
    int16_t raw_a[3];
    int16_t raw_m[3];
    UB ovf = 0U;
    float sa[3];
    float sm[3];
    float f[3];
    float m[3];
    float mag_g;
    float h;
    float delta_mm;
    float long_accel_g = 0.0f;
    ER ea;
    ER em;
    uint32_t i;

    if (g.state == S4_ST_FAULT) {
        publish_snapshot();
        return;
    }

    /* Device access remains sequential. Stage counters are read-only,
     * and do not perform console I/O inside the 10 ms sensor cycle. */
    imu_diag_stage(3U);
    ea = imu_drv_read_accel(raw_a);
    imu_diag_stage(4U);
    em = (ea == E_OK) ? imu_drv_read_mag(raw_m, &ovf) : ea;
    imu_diag_stage(5U);

    if ((ea != E_OK) || (em != E_OK)) {
        g.fail_cnt++;
        if (g.fail_cnt >= S4_I2C_FAIL_LIMIT) {
            enter_fault(FLT_IMU_I2C);
        }
        publish_snapshot();
        return;
    }
    g.fail_cnt = 0U;

    for (i = 0U; i < 3U; i++) {
        sa[i] = imu_accel_counts_to_g(raw_a[i]);
    }
    imu_mag_counts_to_gauss(raw_m, sm);
    imu_map_axes(&g.map, sa, f);
    imu_map_axes(&g.map, sm, m);
    mag_g = imu_vec_magnitude(f);
    g_accel_mg = (UW)(mag_g * 1000.0f);
    g_mag_mgauss = (UW)(imu_vec_magnitude(m) * 1000.0f);

    /* TEMPORARY: compare accelerometer and magnetometer.
    * Start printing after approximately 12 seconds to avoid
    * disturbing the initial automated timing tests.
    */

    delta_mm = step_distance_mm();
#if S4_USE_BRAKE_COMP
    {
        float v = delta_mm * (1000.0f / (float)S4_IMU_PERIOD_MS);          /* mm/s   */
        float a = (v - g.speed_prev_mm_s) * (1000.0f / (float)S4_IMU_PERIOD_MS); /* mm/s^2 */
        g.speed_prev_mm_s = v;
        g.accel_g_filt = (0.9f * g.accel_g_filt) + (0.1f * (a / S4_MM_S2_PER_G));
        long_accel_g = g.accel_g_filt;
    }
#endif

    g.pitch_deg = imu_pitch_deg(f, long_accel_g) - g.pitch0_deg;
    g.roll_deg  = imu_roll_deg(f) - g.roll0_deg;
    if (ovf != 0U) {
        g_mag_ovf++;                       /* keep the last heading */
    } else {
        h = imu_heading_deg(f, m);
        if (h >= 0.0f) {
            g.heading_deg = h;
        }
    }

    if (g.state == S4_ST_INIT_CALIBRATING) {
        handle_calibration(f);
    } else {
        handle_monitoring(f, mag_g, delta_mm);
    }
    publish_snapshot();
}

#if S4_DEBUG_PRINT
static void debug_print(UW cycle_us)
{
    tm_printf((UB*)"IMU %s hdg=%d pitch_x10=%d roll_x10=%d |a|=%dmg |B|=%dmG drop=%u i2cerr=%u cyc=%uus\n",
              k_state_name[g.state],
              (INT)g.heading_deg,
              (INT)(g.pitch_deg * 10.0f),
              (INT)(g.roll_deg * 10.0f),
              (INT)g_accel_mg, (INT)g_mag_mgauss,
              (UINT)g_evt_dropped, (UINT)imu_drv_error_count(), (UINT)cycle_us);
}
#endif

/* ------------------------------------------------------------------ */
/* The task                                                            */
/* ------------------------------------------------------------------ */
EXPORT void imu_task(INT stacd, void *exinf)
{
    UINT ptn = 0U;
    UW t_prev = 0U;
    UW t0;
    UW t1;
    UW cycle_us;
    UW period_us;
    UINT have_prev = 0U;
    ER er;
    static const imu_axis_map_t k_default_map = IMU_AXIS_MAP_DEFAULT;

    (void)stacd;
    (void)exinf;

    g_imu_task_id = tk_get_tid();
    imu_diag_stage(0U);
    g.map = k_default_map;
    g.heading_deg = 0.0f;
    start_calibration();

    
    er = imu_drv_init(s4_i2c_dev);

#if APP_DEBUG_IMU
    tm_printf((UB*)
        "[IMU INIT] imu_drv_init returned %d\n",
        (INT)er);
#endif

    if (er != E_OK) {
        enter_fault(FLT_IMU_I2C);
    }

    (void)tk_sta_cyc(cyc_imu);

    for (;;) {
        imu_diag_stage(1U);
        er = tk_wai_flg(flg_imu, S4_FLG_TICK | S4_FLG_REINIT,
                        TWF_ORW | TWF_BITCLR, &ptn, S4_TICK_WAIT_MS);
        if (er == E_TMOUT) {
            g_missed_ticks++;
            continue;
        }
        if (er < E_OK) {
            g_imu_other_wait_errors++;
            imu_diag_stage(9U);
            continue;
        }

        g_imu_wakeups++;
        imu_diag_stage(2U);
        t0 = S4_TIMER_RAWL;
        if (g_reset_request != 0U) {
            g_reset_request = 0U;
            have_prev = 0U;
        }
        if (((ptn & S4_FLG_REINIT) != 0U) && (g.state == S4_ST_FAULT)) {
            try_reinit();
        }
        if ((ptn & S4_FLG_TICK) != 0U) {
            run_cycle();
            g_cycles++;
            imu_diag_stage(6U);
        }
        t1 = S4_TIMER_RAWL;

        cycle_us = t1 - t0;                    /* unsigned: safe across wrap */
        if (cycle_us > g_cycle_us_max) {
            g_cycle_us_max = cycle_us;
        }
        if (have_prev != 0U) {
            period_us = t0 - t_prev;
            if (period_us > g_period_us_max) {
                g_period_us_max = period_us;
            }
        }
        t_prev = t0;
        have_prev = 1U;

#if S4_DEBUG_PRINT
        if (((ptn & S4_FLG_TICK) != 0U) &&
            ((g_cycles % S4_DEBUG_CYCLES) == 0U)) {
            imu_diag_stage(7U);
            debug_print(cycle_us);
        }
#endif
        imu_diag_stage(8U);
    }
}
