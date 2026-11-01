/*
 *	subsystem1.c
 *	Subsystem 1 -- creates its objects and starts its tasks.
 *	  Wi-Fi Telemetry (Buddy 1): telemetry_q, state machine (wifi_telemetry.c)
 *	  LED heartbeat (template example, IF-05)
 *
 *	APIs shown: tk_cre_cyc, tk_sta_cyc, tk_cre_mbf
 */

#include "subsystem1.h"
#include "wifi_telemetry.h"

EXPORT ID	tid_led;	/* IF-05, see subsystem1.h */
EXPORT ID	cyc_led;

EXPORT ER subsystem1_init(void)
{
	T_CCYC	ccyc = {0};
	ER	er;

	/* Wi-Fi Telemetry: telemetry_q and the state machine */
	er = wifi_telemetry_init();
	if (er < E_OK) {
		return er;
	}

	/* Cyclic handler: once started, the kernel calls led_cyclic_handler
	   every S1_TICK_MS.  Without TA_STA it is created stopped; it is
	   started in subsystem1_start(), once the task it wakes exists. */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)led_cyclic_handler;
	ccyc.cyctim = S1_TICK_MS;	/* period, ms */
	ccyc.cycphs = S1_TICK_MS;	/* delay before the first call, ms */
	cyc_led = tk_cre_cyc(&ccyc);
	return (cyc_led < E_OK) ? cyc_led : E_OK;
}

EXPORT ER subsystem1_start(void)
{
	ER	er;

	er = wifi_telemetry_start();
	if (er < E_OK) {
		return er;
	}

	tid_led = app_start_task(led_task, S1_PRI_LED, 0, "led_task");
	if (tid_led < E_OK) {
		return tid_led;
	}
	return tk_sta_cyc(cyc_led);	/* start the cyclic handler now */
}
