/*
 *	drv_servo.c
 *	Subsystem 5 -- hobby servo on PWM (GP12 = PWM slice 6, channel A).
 *
 *	A hobby servo wants a pulse every 20 ms (50 Hz).  The pulse WIDTH
 *	sets the angle: ~500 us = 0 deg, ~1500 us = 90 deg, ~2500 us = 180 deg.
 *
 *	PWM set-up
 *	----------
 *	  clk_sys = 125 MHz.  Divider 125  -> the PWM counter ticks at 1 MHz,
 *	  so one count = 1 us.  Wrap (TOP) at 19999 -> period 20000 us = 50 Hz.
 *	  Compare value (CC) = pulse width in us.  Easy to read on a scope.
 *
 *	Sharing note: slice 6 also drives GP13 (channel B).  Whoever uses GP13
 *	for PWM gets the same 50 Hz.  Record this in TEAM.md.
 */

#include "drv_servo.h"
#include "s5_config.h"

#define SERVO_PERIOD_US		20000U	/* 50 Hz */
#define SERVO_SLICE		((S5_PIN_SERVO >> 1U) & 0x07U)
#define SERVO_DIV_REG		(PWM_BASE + PWM_CHx_DIV + (SERVO_SLICE * 0x14U))
#define SERVO_DIV_INT_SHIFT	4U	/* DIV register: INT in bits 11:4 */

LOCAL UINT	servo_now_deg = S5_CENTRE_DEG;

EXPORT UW servo_pulse_us(UINT angle_deg)
{
	UW	deg = (angle_deg > S5_ANGLE_MAX_DEG) ? S5_ANGLE_MAX_DEG : angle_deg;

#if S5_SERVO_MIRROR
	deg = S5_ANGLE_MAX_DEG - deg;
#endif
	return (S5_SERVO_MIN_US
		+ (((S5_SERVO_MAX_US - S5_SERVO_MIN_US) * deg) / S5_ANGLE_MAX_DEG));
}

EXPORT ER servo_init(void)
{
	/* The kernel only takes the PWM block out of reset when USE_PTMR is
	   set (it is not, config/config.h), so do it here.  clr_w() is the
	   RP2040's atomic clear alias: harmless if Buddy 2's motor code does
	   the same on the other core.  Never RESET it again (set_w): that
	   would stop the motors' PWM too. */
	clr_w(RESETS_RESET, RESETS_RESET_PWM);
	while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U) {
		/* wait for the reset to finish */
	}

	out_w(SERVO_DIV_REG, (UW)SYSCLK << SERVO_DIV_INT_SHIFT);  /* 1 MHz */
	(void)pwm_set_wrap(S5_PIN_SERVO, SERVO_PERIOD_US - 1U);
	(void)pwm_set_cc(S5_PIN_SERVO, servo_pulse_us(S5_CENTRE_DEG));
	(void)pwm_set_pin(S5_PIN_SERVO);	/* GP12 function = PWM */
	(void)pwm_set_enabled(S5_PIN_SERVO, TRUE);

	servo_now_deg = S5_CENTRE_DEG;
	tk_dly_tsk(S5_CENTRE_DEG * S5_SERVO_MS_PER_DEG);	/* worst case */
	return E_OK;
}

EXPORT void servo_move(UINT angle_deg)
{
	UINT	deg = (angle_deg > S5_ANGLE_MAX_DEG) ? S5_ANGLE_MAX_DEG : angle_deg;
	UINT	delta = (deg > servo_now_deg) ? (deg - servo_now_deg)
					      : (servo_now_deg - deg);

	(void)pwm_set_cc(S5_PIN_SERVO, servo_pulse_us(deg));
	servo_now_deg = deg;

	/* The servo has no position feedback, so wait for the worst-case
	   travel time plus a settle time before pinging. */
	tk_dly_tsk((RELTIM)((delta * S5_SERVO_MS_PER_DEG) + S5_SERVO_SETTLE_MS));
}

EXPORT UINT servo_angle(void)
{
	return servo_now_deg;
}
