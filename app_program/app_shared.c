/*
 *	app_shared.c
 *	Shared kernel objects used across subsystems (see app.h).
 *
 *	INTERIM OWNER.  remote_cmd_q belongs to the Vehicle Controller (B4) and
 *	flg_safety / flg_fault to the Emergency/Safety Event Handler (B4/B5).
 *	Neither subsystem exists in the firmware yet, so the objects are made
 *	here, once, before every subsystemN_init(), so Buddy 1's Wi-Fi task can
 *	be built and tested against the agreed interface.  When the owners'
 *	code lands they move this creation into their own subsystem (same
 *	names, same formats) and delete this file.
 */

#include "app.h"

EXPORT ID	remote_cmd_q;
EXPORT ID	flg_safety;
EXPORT ID	flg_fault;

EXPORT ER app_shared_init(void)
{
	T_CMBF	cmbf = {0};
	T_CFLG	cflg = {0};

	/* remote_cmd_q: Wi-Fi -> Vehicle Controller, 4 messages of 2 bytes */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = REMOTE_CMD_Q_DEPTH * APP_MBF_SLOT(RCMD_MSG_LEN);
	cmbf.maxmsz = RCMD_MSG_LEN;
	remote_cmd_q = tk_cre_mbf(&cmbf);
	if (remote_cmd_q < E_OK) {
		return remote_cmd_q;
	}

	/* flg_safety: several tasks may wait (Safety, Motion Control) */
	cflg.flgatr  = TA_TPRI | TA_WMUL;
	cflg.iflgptn = 0;
	flg_safety = tk_cre_flg(&cflg);
	if (flg_safety < E_OK) {
		return flg_safety;
	}

	/* flg_fault: shared fault mask, FLT_* bits */
	flg_fault = tk_cre_flg(&cflg);
	return (flg_fault < E_OK) ? flg_fault : E_OK;
}
