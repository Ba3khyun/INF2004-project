/*
 *	subsystem3.c
 *	Subsystem 3 -- Initializes RTOS primitives and sensor hardware.
 */

#include "subsystem3.h"

EXPORT ID       mbx_samples;    /* IF-01[cite: 1] */
EXPORT ID       mtx_stats;    /* IF-04[cite: 1] */
EXPORT APP_STATS    app_stats;

EXPORT ER subsystem3_init(void)
{
	T_CMBX  cmbx = {0};
	T_CMTX  cmtx = {0};

	/* 1. Mailbox & Mutex Setup */
	cmbx.mbxatr = TA_TFIFO | TA_MFIFO;    /* FIFO waiters, FIFO messages[cite: 1] */
	mbx_samples = tk_cre_mbx(&cmbx);
	if (mbx_samples < E_OK) {
		return mbx_samples;
	}

	cmtx.mtxatr = TA_INHERIT;       /* Priority inheritance[cite: 1] */
	mtx_stats = tk_cre_mtx(&cmtx);
	if (mtx_stats < E_OK) {
		return mtx_stats;
	}

	/* Configure the barcode sensor input; the ADC driver configures GP26/27. */
	ER er = gpio_set_pin(BARCODE_PIN, GPIO_MODE_IN);
	if (er != E_OK) {
		return er;
	}
	set_w(GPIO(BARCODE_PIN), GPIO_PUE);

	return E_OK;
}

EXPORT ER subsystem3_start(void)
{
	/* Pinned to core 0 (TP_PRC1)[cite: 1] */
	ID tid = app_start_task(consumer_task, S3_PRI_CONSUMER, TP_PRC1,
				"consumer_task");
	return (tid < E_OK) ? tid : E_OK;
}