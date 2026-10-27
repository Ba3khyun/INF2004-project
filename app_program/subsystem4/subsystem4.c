/*
 *	subsystem4.c
 *	Subsystem 4 -- IMU subsystem (GY-511 / LSM303DLHC).
 *	Subsystem 4 is responsible for reading the accelerometer and magnetometer
 *	registers, calculating pitch, roll and heading, and detecting humps and
 *	shocks.  It runs in its own task, imu_task(), which is started by
 *	subsystem4_start().
 */

#include "subsystem4.h"
#include "../../device/include/dev_i2c.h"

/* Implemented in the IMU task source file. */
IMPORT void imu_task(INT stacd, void *exinf);

/* Reporting objects shared with the monitor and Subsystem 3 */
EXPORT ID mbf_summary = -1;
EXPORT ID flg_events = -1;
EXPORT ID alm_monitor = -1;

/* Device ID returned by tk_opn_dev(). */
// *NOTE: saved device ID is currently private to subsystem4.c; we'll expose it to the IMU task in a later step
EXPORT ID s4_i2c_dev = -1; 

EXPORT ER subsystem4_init(void)
{
	T_CMBF cmbf = {0};
    T_CFLG cflg = {0};
    T_CALM calm = {0};

    /* Create the summary message buffer. */
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = 512;
    cmbf.maxmsz = S4_SUMMARY_MAX;

    mbf_summary = tk_cre_mbf(&cmbf);
    if (mbf_summary < E_OK) {
        return (ER)mbf_summary;
    }

    /* Create the event flag used by the reporting monitor. */
    cflg.flgatr = 0;  /* default attributes */
    cflg.iflgptn = 0;

    flg_events = tk_cre_flg(&cflg);
    if (flg_events < E_OK) {
        return (ER)flg_events;
    }

    /* Create the alarm handler used by the reporting monitor. */
    calm.almatr = TA_HLNG;
    calm.almhdr = monitor_alarm_handler;

    alm_monitor = tk_cre_alm(&calm);
    if (alm_monitor < E_OK) {
        return (ER)alm_monitor;
    }

	/* Open I2C unit 0, registered as "iica". */
	// s4_i2c_dev stores the returned device ID
    s4_i2c_dev = tk_opn_dev("iica", TD_UPDATE); 	// opens the I²C0 device for updating
	// If opening fails, returns the error rather than continuing as if the sensor were ready
    if (s4_i2c_dev < E_OK) {
        return (ER)s4_i2c_dev;
    }

    return E_OK;
}

EXPORT ER subsystem4_start(void)
{
	ID tid;

	tm_printf((UB*)"S4: starting monitor task\n");

    /* Start the original reporting monitor. */
    tid = app_start_task(
        monitor_task,
        S4_PRI_MONITOR,
        0,
        "monitor_task"
    );

    if (tid < E_OK) {
		tm_printf((UB*)"S4: monitor task start failed: %d\n", tid);
        return (ER)tid;
    }

    tm_printf((UB*)"S4: monitor task started, ID=%d\n", tid);
    tm_printf((UB*)"S4: starting IMU task\n");


	// Start the IMU task separately, with its own priority and name.  The IMU task will handle reading the sensor and processing the data.
	tid = app_start_task(
        imu_task,
        S4_IMU_PRIORITY,
        0,
        "imu_task"
    );

    if (tid < E_OK) {
        tm_printf((UB*)"S4: IMU task start failed: %d\n", tid);
        return (ER)tid;
    }

    tm_printf((UB*)"S4: IMU task started, ID=%d\n", tid);
    return E_OK;
}
