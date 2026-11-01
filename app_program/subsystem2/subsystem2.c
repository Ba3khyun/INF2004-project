/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion Control (Buddy 2) and the example Producer:
 *	creates their objects and starts their tasks.
 *
 *	APIs shown: tk_cre_sem, tk_cre_mpf, tk_cre_mbf, tk_snd_mbf
 */

#include "subsystem2.h"

EXPORT ID	sem_slots;	/* IF-02, see subsystem2.h */
EXPORT ID	mpf_samples;
EXPORT ID	motion_cmd_q;	/* Motion Control input, see subsystem2.h */
EXPORT ID	tid_motion;
EXPORT ID	cyc_motion;	/* 10 ms control tick */

EXPORT ER subsystem2_init(void)
{
	T_CSEM	csem = {0};
	T_CMPF	cmpf = {0};
	T_CMBF	cmbf = {0};

	/* Semaphore: a counter of free "slots".  The producer takes one
	   (tk_wai_sem) before each sample; the consumer gives one back
	   (tk_sig_sem).  It never exceeds the number of memory blocks, so
	   the producer can never run out of memory. */
	csem.sematr  = TA_TFIFO;	/* waiting tasks queue in FIFO order */
	csem.isemcnt = S2_POOL_COUNT;	/* initial count */
	csem.maxsem  = S2_POOL_COUNT;	/* maximum count */
	sem_slots = tk_cre_sem(&csem);
	if (sem_slots < E_OK) {
		return sem_slots;
	}

	/* Fixed-size memory pool: S2_POOL_COUNT blocks, each exactly one
	   SAMPLE_MSG.  Faster and more predictable than malloc(). */
	cmpf.mpfatr = TA_TFIFO | TA_RNG3;
	cmpf.mpfcnt = S2_POOL_COUNT;		/* number of blocks */
	cmpf.blfsz  = sizeof(SAMPLE_MSG);	/* size of one block */
	mpf_samples = tk_cre_mpf(&cmpf);
	if (mpf_samples < E_OK) {
		return mpf_samples;
	}

	/* motion_cmd_q: commands -> motion_task, MC_CMD_Q_DEPTH x 1 byte */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = MC_CMD_Q_DEPTH * APP_MBF_SLOT(MC_CMD_LEN);
	cmbf.maxmsz = MC_CMD_LEN;
	motion_cmd_q = tk_cre_mbf(&cmbf);
	if (motion_cmd_q < E_OK) {
		return motion_cmd_q;
	}

	/* cyc_motion: wakes motion_task every MC_TICK_MS.  Created stopped;
	   started in subsystem2_start() once the task it wakes exists. */
	{
		T_CCYC	ccyc = {0};

		ccyc.cycatr = TA_HLNG;
#if TK_SUPPORT_SMP
		ccyc.cycatr |= TA_ASSPRC;	/* run on core 1, beside motion_task */
		ccyc.assprc  = TP_PRC2;
#endif
		ccyc.cychdr = (FP)motion_cyclic_handler;
		ccyc.cyctim = MC_TICK_MS;
		ccyc.cycphs = MC_TICK_MS;
		cyc_motion = tk_cre_cyc(&ccyc);
	}
	return (cyc_motion < E_OK) ? cyc_motion : E_OK;
}

EXPORT ER subsystem2_start(void)
{
	ID tid;

	/* Motion Control on core 1 (TP_PRC2), away from the USB console and
	   the Wi-Fi task on core 0.  The encoder interrupt is enabled on the
	   same core, from inside the task. */
	tid = app_start_task(motion_task, S2_PRI_MOTION, TP_PRC2, "motion_task");
	if (tid < E_OK) {
		return tid;
	}
	tid_motion = tid;
	tid = tk_sta_cyc(cyc_motion);	/* start the 10 ms tick now */
	if (tid < E_OK) {
		return tid;
	}

	/* Pinned to core 1 (TP_PRC2): samples cross to subsystem 3 on core 0 */
	tid = app_start_task(producer_task, S2_PRI_PRODUCER, TP_PRC2,
			     "producer_task");
	return (tid < E_OK) ? tid : E_OK;
}

EXPORT ER motion_cmd_post(UB cmd)
{
	switch (cmd) {
	case MC_CMD_FORWARD:
	case MC_CMD_LEFT:
	case MC_CMD_RIGHT:
	case MC_CMD_STOP:
	case MC_CMD_CLEAR:
		break;
	default:
		return E_PAR;
	}
	return tk_snd_mbf(motion_cmd_q, &cmd, MC_CMD_LEN, TMO_POL);
}
