/*
 *	drv_ultrasonic.c
 *	Subsystem 5 -- HC-SR04 ultrasonic distance sensor driver.
 *
 *	How an HC-SR04 measurement works
 *	--------------------------------
 *	  1. We hold TRIG high for >= 10 us.
 *	  2. The module sends eight 40 kHz sound pulses and raises ECHO.
 *	  3. ECHO stays high until the reflection comes back (or the module
 *	     gives up after ~38 ms if nothing reflects).
 *	  4. ECHO high time = time for sound to go there AND back, so
 *	        distance_cm = echo_us / 58
 *
 *	How we time ECHO
 *	----------------
 *	The RP2040 has a 64-bit hardware timer that counts once every
 *	microsecond (TIMERAWL = its low 32 bits).  We read it when ECHO rises
 *	and when it falls.  That is a short busy-wait (at most ~11.6 ms with
 *	S5_US_RANGE_MAX_CM = 200).  Higher-priority tasks may still preempt
 *	us; if that happens exactly on an edge the reading is a little long
 *	or short, which the median filter in us_ping_median() absorbs.
 *
 *	Integration note: the team design (Tribunal 1, A2.4) has ONE shared
 *	GPIO interrupt dispatcher owned by Buddy 2 that timestamps the ECHO
 *	edges into a ring buffer.  When it exists, only steps 3-4 of
 *	us_ping() change: read the two timestamps from that buffer instead
 *	of polling.  Everything above this file stays the same.
 */

#include "drv_ultrasonic.h"
#include "s5_config.h"

/* Longest ECHO pulse we wait for: maximum range, there and back */
#define US_ECHO_MAX_US		(S5_US_RANGE_MAX_CM * S5_US_US_PER_CM)

/* Timer self-check against the kernel clock.  The kernel ticks every
   CNF_TIMER_PERIOD = 10 ms (config/config.h), and tk_dly_tsk(n) adds one
   tick so it never sleeps LESS than n: a 200 ms sleep really lasts
   200..210 ms.  A long sleep keeps that error small (5 %).  The window
   rejects both a stopped/slow timer and one running too fast. */
#define US_CHECK_MS		200U
#define US_CHECK_MIN_US		195000U		/* 200 ms - 2.5 % */
#define US_CHECK_MAX_US		215000U		/* 200 ms + 1 tick + 2.5 % */

/* RP2040 watchdog TICK register: makes the 1 us tick for the timer from
   the 12 MHz crystal (datasheet 4.7).  Not in the BSP headers. */
#define US_WATCHDOG_TICK	(0x40058000UL + 0x2CUL)
#define US_TICK_ENABLE		(1UL << 9)

/* ------------------------------------------------------------------ */

EXPORT UW us_now_us(void)
{
	return in_w(TIMER_TIMERAWL);	/* raw read, no latching needed */
}

EXPORT void us_delay_us(UW us)
{
	UW	t0 = us_now_us();

	/* "<=" so we wait at least 'us' even if t0 was read late in a tick */
	while ((us_now_us() - t0) <= us) {
		/* spin */
	}
}

/*
 * Measure how many timer microseconds pass during a 10 ms kernel sleep.
 */
LOCAL UW timer_measure(void)
{
	UW	t0 = us_now_us();

	tk_dly_tsk(US_CHECK_MS);
	return (us_now_us() - t0);
}

/*
 * The timer counts "ticks" made by the watchdog's tick generator, which
 * divides clk_ref (the 12 MHz crystal) by its CYCLES field.  The Pico
 * SDK sets CYCLES = 12 at boot (12 MHz / 12 = 1 MHz), but this kernel's
 * start-up code (kernel/sysdepend/pico_rp2040/cpu_clock.c) never does,
 * so the timer can run at the wrong speed (measured on the bench: 636
 * counts in 20 ms instead of 20000).
 *
 * So: if the timer is stopped OR at the wrong speed, release it from
 * reset and set the tick generator to 1 MHz.  clr_w() uses the RP2040's
 * atomic clear alias and only RELEASES the reset (it never resets the
 * timer), so the count is not lost.  Anything else that uses the timer
 * (TinyUSB, WiFi delays) also needs 1 MHz, so this only makes it right.
 * The proper long-term place for this is startup_clock(): agree that
 * with the team, because it is shared kernel code.
 */
LOCAL ER timer_check(void)
{
	UW	dt = timer_measure();

	if ((dt < US_CHECK_MIN_US) || (dt > US_CHECK_MAX_US)) {
		tm_printf((UB*)"S5: us timer wrong (%u us in %u ms), setting 1 MHz tick\n",
			  dt, US_CHECK_MS);
		clr_w(RESETS_RESET, RESETS_RESET_TIMER);
		while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_TIMER) == 0U) {
			/* wait for the reset release to finish */
		}
		out_w(US_WATCHDOG_TICK, US_TICK_ENABLE | (UW)XOSC_MHz);
		dt = timer_measure();
	}

	tm_printf((UB*)"S5: timer check %u us in %u ms\n", dt, US_CHECK_MS);
	if ((dt < US_CHECK_MIN_US) || (dt > US_CHECK_MAX_US)) {
		tm_printf((UB*)"S5: ERROR timer is not 1 MHz, distances will be wrong\n");
		return E_SYS;
	}
	return E_OK;
}

EXPORT ER us_init(void)
{
	ER	er;

	er = gpio_set_pin(S5_PIN_US_TRIG, GPIO_MODE_OUT);
	if (er < E_OK) {
		return er;
	}
	(void)gpio_set_val(S5_PIN_US_TRIG, 0U);	/* idle low */

	er = gpio_set_pin(S5_PIN_US_ECHO, GPIO_MODE_IN);
	if (er < E_OK) {
		return er;
	}
	return timer_check();
}

/*
 * The module ignores a new trigger while ECHO is still high from the
 * previous ping, so wait (sleeping, not spinning) until it is low.
 * The limit is measured with the us timer, not by counting sleeps:
 * each tk_dly_tsk(1) really lasts up to one 10 ms kernel tick.
 */
LOCAL BOOL wait_echo_low(void)
{
	UW	t0 = us_now_us();

	while (gpio_get_val(S5_PIN_US_ECHO) != 0U) {
		if ((us_now_us() - t0) > (S5_US_IDLE_TMO_MS * 1000U)) {
			return FALSE;
		}
		tk_dly_tsk(1U);
	}
	return TRUE;
}

EXPORT US_STATUS us_ping(UH *p_cm)
{
	UW	t_start;
	UW	t_rise;
	UW	width_us;
	UW	cm;

	*p_cm = (UH)S5_US_RANGE_MAX_CM;

	/* 1. Module must be idle */
	if (!wait_echo_low()) {
		return US_STUCK;
	}

	/* 2. Trigger pulse */
	(void)gpio_set_val(S5_PIN_US_TRIG, 1U);
	us_delay_us(S5_US_TRIG_PULSE_US);
	(void)gpio_set_val(S5_PIN_US_TRIG, 0U);

	/* 3. Wait for ECHO to rise (the burst has gone out) */
	t_start = us_now_us();
	while (gpio_get_val(S5_PIN_US_ECHO) == 0U) {
		if ((us_now_us() - t_start) > S5_US_RISE_TMO_US) {
			return US_NO_ECHO;
		}
	}

	/* 4. Time how long ECHO stays high */
	t_rise = us_now_us();
	while (gpio_get_val(S5_PIN_US_ECHO) != 0U) {
		if ((us_now_us() - t_rise) > US_ECHO_MAX_US) {
			return US_OUT_OF_RANGE;	/* nothing within range */
		}
	}
	width_us = us_now_us() - t_rise;

	/* 5. Convert to cm, rounded to the nearest cm */
	cm = (width_us + (S5_US_US_PER_CM / 2U)) / S5_US_US_PER_CM;
	if (cm >= S5_US_RANGE_MAX_CM) {
		return US_OUT_OF_RANGE;
	}
	*p_cm = (UH)cm;
	return US_OK;
}

EXPORT US_STATUS us_ping_median(UINT n, UH *p_cm)
{
	UH		sorted[US_MEDIAN_MAX];
	UINT		got = 0U;
	UINT		i;
	UINT		j;
	UH		cm;
	US_STATUS	st;
	US_STATUS	last_err = US_NO_ECHO;

	if (n == 0U) {
		n = 1U;
	}
	if (n > US_MEDIAN_MAX) {
		n = US_MEDIAN_MAX;
	}

	for (i = 0U; i < n; i++) {
		if (i > 0U) {
			tk_dly_tsk(S5_PING_GAP_MS);	/* let echoes die out */
		}
		st = us_ping(&cm);
		if ((st == US_OK) || (st == US_OUT_OF_RANGE)) {
			/* insertion sort: keep sorted[0..got] in order */
			j = got;
			while ((j > 0U) && (sorted[j - 1U] > cm)) {
				sorted[j] = sorted[j - 1U];
				j--;
			}
			sorted[j] = cm;
			got++;
		} else {
			last_err = st;
		}
	}

	if (got == 0U) {
		*p_cm = (UH)S5_US_RANGE_MAX_CM;
		return last_err;	/* every ping failed */
	}

	*p_cm = sorted[got / 2U];	/* the middle value */
	return (*p_cm >= S5_US_RANGE_MAX_CM) ? US_OUT_OF_RANGE : US_OK;
}
