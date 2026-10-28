/*
 *	subsystem4.c
 *	Subsystem 4 -- IMU subsystem (GY-511 / LSM303DLHC).
 *	Reads the accelerometer and magnetometer, computes pitch, roll and
 *	heading, detects humps and shocks.  The work is done by imu_task(),
 *	woken every 10 ms by a cyclic handler.  This file only creates the
 *	kernel objects and starts the tasks.
 */

#include "subsystem4.h"
#include "../../device/include/dev_i2c.h"

/* Reporting objects shared with the monitor and Subsystem 3 (example code). */
EXPORT ID mbf_summary = -1;
EXPORT ID flg_events  = -1;
EXPORT ID alm_monitor = -1;

/* Objects owned by the IMU subsystem. */
EXPORT ID flg_imu     = -1;
EXPORT ID cyc_imu     = -1;
EXPORT ID mbf_imu_evt = -1;
EXPORT ID flg_faults  = -1;   /* TODO: move to safety_handler.c when it exists */
EXPORT ID flg_safety  = -1;   /* TODO: move to safety_handler.c when it exists */
EXPORT ID s4_i2c_dev  = -1;

EXPORT ER subsystem4_init(void)
{
	T_CMBF cmbf = {0};
	T_CFLG cflg = {0};
	T_CALM calm = {0};
	T_CCYC ccyc = {0};

	/* ---- example summary buffer (kept for Subsystem 3) ---- */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz = 512;
	cmbf.maxmsz = S4_SUMMARY_MAX;
	mbf_summary = tk_cre_mbf(&cmbf);
	if (mbf_summary < E_OK) { return (ER)mbf_summary; }

	cflg.flgatr = 0;
	cflg.iflgptn = 0;
	flg_events = tk_cre_flg(&cflg);
	if (flg_events < E_OK) { return (ER)flg_events; }

	calm.almatr = TA_HLNG;
	calm.almhdr = monitor_alarm_handler;
	alm_monitor = tk_cre_alm(&calm);
	if (alm_monitor < E_OK) { return (ER)alm_monitor; }

	/* ---- IMU: imu_evt_q, uint16_t messages, depth 8 ---- */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz = S4_EVT_BUFSZ;
	cmbf.maxmsz = S4_EVT_MSG_BYTES;
	mbf_imu_evt = tk_cre_mbf(&cmbf);
	if (mbf_imu_evt < E_OK) { return (ER)mbf_imu_evt; }

	/* ---- IMU: tick / reinit flag (one waiter: the IMU task) ---- */
	cflg.flgatr = TA_TFIFO;
	cflg.iflgptn = 0;
	flg_imu = tk_cre_flg(&cflg);
	if (flg_imu < E_OK) { return (ER)flg_imu; }

	/* ---- shared fault mask and safety events (several tasks may wait) ---- */
	cflg.flgatr = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_faults = tk_cre_flg(&cflg);
	if (flg_faults < E_OK) { return (ER)flg_faults; }
	flg_safety = tk_cre_flg(&cflg);
	if (flg_safety < E_OK) { return (ER)flg_safety; }

	/* ---- 10 ms cyclic handler (started by the IMU task once the sensor is ready) ---- */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = imu_cyc_handler;
	ccyc.cyctim = S4_IMU_PERIOD_MS;
	ccyc.cycphs = 0;
	cyc_imu = tk_cre_cyc(&ccyc);
	if (cyc_imu < E_OK) { return (ER)cyc_imu; }

	/* ---- open I2C unit 0 ("iica").  A failure is returned, not ignored. ---- */
	s4_i2c_dev = tk_opn_dev((UB*)"iica", TD_UPDATE);
	if (s4_i2c_dev < E_OK) { return (ER)s4_i2c_dev; }

	return E_OK;
}

EXPORT ER subsystem4_start(void)
{
	ID tid;

	/* Original reporting monitor (example, used by Subsystem 3). */
	tid = app_start_task(monitor_task, S4_PRI_MONITOR, 0, "monitor_task");
	if (tid < E_OK) { return (ER)tid; }

	/* The IMU task: reads the sensor and publishes events. */
	tid = app_start_task(imu_task, S4_IMU_PRIORITY, 0, "imu_task");
	return (tid < E_OK) ? (ER)tid : E_OK;
}
