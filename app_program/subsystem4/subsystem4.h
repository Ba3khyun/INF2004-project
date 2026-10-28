/*
 *	subsystem4.h
 *	Subsystem 4 -- IMU subsystem (GY-511 / LSM303DLHC).  Student ID: 2502446.
 *
 *	Public interface of subsystem 4.  Other subsystems include it as
 *	"../subsystem4/subsystem4.h".
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "../app_data.h"

/* ---- GY-511 hardware (I2C0, Grove 3).  Pins are set in
 *      device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h (GP4 SDA, GP5 SCL). ---- */
#define S4_I2C_PORT          0
#define S4_I2C_SDA_PIN       4
#define S4_I2C_SCL_PIN       5
#define S4_ACCEL_ADDR        0x19
#define S4_MAG_ADDR          0x1E

/* ---- IMU task ---- */
#define S4_IMU_PERIOD_MS     10     /* 100 Hz (NFR4.1)                                  */
#define S4_IMU_PRIORITY      19     /* provisional: team plan is 18, TEAM.md band 18-19  */
#define S4_TICK_WAIT_MS      50     /* longest wait for a tick before counting a miss    */
#define S4_I2C_FAIL_LIMIT    3      /* NFR4.7: 3 failed reads in a row -> FAULT          */
#define S4_CAL_MAX_ATTEMPTS  3      /* NFR4.5: retries before FAULT                      */
#define S4_SHOCK_HOLDOFF     20     /* cycles (200 ms) between minor-shock events        */
#define S4_USE_BRAKE_COMP    0      /* 1 = subtract encoder acceleration from pitch      */
#define S4_DEBUG_PRINT       1      /* 1 = status line on the console once per second    */

/* ---- kernel objects owned by subsystem 4 ---- */
#define S4_FLG_TICK          0x01U  /* set by the 10 ms cyclic handler */
#define S4_FLG_REINIT        0x02U  /* request leaving FAULT           */
#define S4_EVT_QUEUE_DEPTH   8
#define S4_EVT_MSG_BYTES     2      /* uint16_t message (app_data.h)   */
#define S4_EVT_BUFSZ         64     /* 8 messages incl. kernel headers - checked by test S4-U17 */

IMPORT ID flg_imu;       /* tick + reinit requests for the IMU task          */
IMPORT ID cyc_imu;       /* 10 ms cyclic handler                             */
IMPORT ID mbf_imu_evt;   /* imu_evt_q: uint16_t messages, depth 8            */
IMPORT ID flg_faults;    /* shared team fault mask (event flag, app_data.h)  */
IMPORT ID flg_safety;    /* events for the Emergency/Safety task             */
IMPORT ID s4_i2c_dev;    /* device ID of "iica" (I2C0)                       */

/* ---- read-only status for other subsystems and tests ---- */
IMPORT volatile UW s4_imu_snapshot;                 /* packed word, see app_data.h */
IMPORT UW   s4_imu_get_snapshot(void);              /* one aligned 32-bit load     */
IMPORT s4_imu_state_t s4_imu_state(void);
IMPORT ER   s4_imu_request_reinit(void);            /* called after Safety reset   */

/* ---- statistics (tests, NFR4.2) ---- */
IMPORT UW   s4_stat_cycles(void);
IMPORT UW   s4_stat_cycle_us_max(void);             /* worst execution time        */
IMPORT UW   s4_stat_period_us_max(void);            /* worst gap between cycles    */
IMPORT UW   s4_stat_events_dropped(void);
IMPORT UW   s4_stat_mag_overflow(void);
IMPORT UW   s4_stat_accel_mg(void);                 /* last |a| in milli-g         */
IMPORT UW   s4_stat_mag_mgauss(void);               /* last |B| in milli-gauss     */
IMPORT void s4_stat_reset(void);

/* ---- Distance from Motion Control (Buddy 2): read-only, monotonic, mm.
 *      A weak default (returns 0) lives in imu_task.c until Buddy 2 provides it. ---- */
IMPORT UW   motion_get_distance_mm(void);

/* ---- compatibility interface for the example Subsystem 3 reporting ---- */
#define S4_SUMMARY_MAX 48
#define FLG_BATCH      0x0002
#define FLG_ALARM      0x0001
#define S4_PRI_MONITOR           18
#define S4_ALARM_MS              2000
#define S4_LED_TOGGLE_REPORTS    6

IMPORT ID mbf_summary;
IMPORT ID flg_events;
IMPORT ID alm_monitor;
IMPORT UW s4_lines_received;
IMPORT void monitor_task(INT stacd, void *exinf);
IMPORT void monitor_alarm_handler(void *exinf);

/* ---- tasks and handlers ---- */
IMPORT void imu_task(INT stacd, void *exinf);
IMPORT void imu_cyc_handler(void *exinf);

/* ---- lifecycle ---- */
IMPORT ER subsystem4_init(void);
IMPORT ER subsystem4_start(void);

#endif /* SUBSYSTEM4_H */
