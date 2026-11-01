/*
 *	motion_control.c
 *	Subsystem 2 -- Motion Control (Buddy 2): motor, encoder and PID driver.
 *
 *	Ported from buddy2_motion_control/motion_control.c.  The behaviour is
 *	the same; only the hardware access changed:
 *	  Pico SDK (original)                 here
 *	  pwm_set_gpio_level(pin, level)      pwm_set_cc(pin, level)        libbsp
 *	  pwm_set_wrap/enabled(slice, ...)    pwm_set_wrap/enabled(pin, ...) libbsp
 *	  pwm_set_clkdiv(slice, 1.0f)         PWM CHn_DIV register
 *	  clock_get_hz(clk_sys)               SYSCLK (125 MHz, sysdef.h)
 *	  gpio_init/set_dir/put/pull_up       gpio_set_pin/val + pad register
 *	  gpio_set_irq_enabled_with_callback  tk_def_int(IO_IRQ_BANK0) + INTE
 *	  save_and_disable_interrupts         not needed: one aligned 32-bit
 *	                                      load is atomic on Cortex-M0+
 *
 *	Not ported: motors_move_forward_revolutions() and
 *	motors_turn_*_revolutions().  They busy-looped with sleep_ms() and
 *	nothing called them; task_motion.c runs the same algorithm as a
 *	non-blocking state machine on the 10 ms tick.
 *
 *	No Pico SDK headers are used, so this builds in every profile,
 *	including CONSOLE=uart WIFI=none.
 */

#include "motion_control.h"

/* ---------------------------------------------------------------------
 *  Pins and constants (unchanged from the original)
 * --------------------------------------------------------------------- */
#define MOTOR_L_A	8u
#define MOTOR_L_B	9u
#define MOTOR_R_A	10u
#define MOTOR_R_B	11u

/* Input pins for the two wheel encoders */
#define ENCODER_R_A	6u
#define ENCODER_L_A	7u

#define PWM_FREQUENCY_HZ	20000u

/* Encoder measurement for distance and speed calculations */
#define LEFT_PULSES_PER_REV	617.0f
#define RIGHT_PULSES_PER_REV	616.0f

#define PI_F		3.14159265f

/* ---------------------------------------------------------------------
 *  RP2040 registers not covered by libbsp (datasheet 4.5 PWM, 2.19 GPIO)
 * --------------------------------------------------------------------- */
#define PWM_CLK_HZ		((UW)SYSCLK * MHz)	/* clk_sys, 125 MHz */
#define PWM_SLICE(pin)		(((pin) >> 1) & 7u)
#define PWM_SLICE_CSR(s)	(PWM_BASE + PWM_CHx_CSR + (s) * 0x14u)
#define PWM_SLICE_DIV(s)	(PWM_BASE + PWM_CHx_DIV + (s) * 0x14u)
#define PWM_SLICE_CTR(s)	(PWM_BASE + PWM_CHx_CTR + (s) * 0x14u)
#define PWM_DIV_ONE		(1u << 4)	/* 8.4 fixed point: 1.0 */

/* IO_BANK0 raw interrupt and per-core enable/status, GPIO0..7 */
#define IO_INTR0		(IO_BANK0_BASE + 0x0F0u)
#define IO_PROC_INTE0(core)	(IO_BANK0_BASE + 0x100u + (core) * 0x30u)
#define IO_PROC_INTS0(core)	(IO_BANK0_BASE + 0x120u + (core) * 0x30u)
#define IO_EDGE_HIGH(pin)	(1u << (4u * (pin) + 3u))

#define INTNO_IO_BANK0		13	/* IO_IRQ_BANK0 */
#define ENCODER_INTPRI		2	/* same level as the ADC/I2C drivers */

#if (ENCODER_L_A > 7u) || (ENCODER_R_A > 7u)
#error "encoder pins must be GP0..GP7 (they share IO_INTR0)"
#endif
#if PWM_SLICE(MOTOR_L_A) != PWM_SLICE(MOTOR_L_B) || PWM_SLICE(MOTOR_R_A) != PWM_SLICE(MOTOR_R_B)
#error "each motor's A/B pins must be on one PWM slice"
#endif

#define ENC_MASK	(IO_EDGE_HIGH(ENCODER_L_A) | IO_EDGE_HIGH(ENCODER_R_A))

/* Written only by encoder_inthdr, read by the owning task */
LOCAL volatile uint32_t encoder_left_count = 0u;
LOCAL volatile uint32_t encoder_right_count = 0u;

LOCAL UW enc_core;		/* core whose INTE has our bits: 0 or 1 */
LOCAL UW pwm_top = 0u;		/* max PWM counter value */


LOCAL UW speed_to_pwm(float speed_percent)
{
	/* Ensure PWM remains between 0% and 100%. */
	if (speed_percent < 0.0f) {
		speed_percent = 0.0f;
	} else if (speed_percent > 100.0f) {
		speed_percent = 100.0f;
	}

	/* Convert percentage to PWM compare level. */
	float pwm_level = (speed_percent * (float)(pwm_top + 1u)) / 100.0f;

	return (UW)(pwm_level + 0.5f);
}


/* ==========================================================
 *  MOTORS SECTION
 * ========================================================== */

void motors_brake(void)
{
	/* LOW/LOW = Brake (Robo Pico datasheet). */
	pwm_set_cc(MOTOR_L_A, 0u);
	pwm_set_cc(MOTOR_L_B, 0u);
	pwm_set_cc(MOTOR_R_A, 0u);
	pwm_set_cc(MOTOR_R_B, 0u);
}

ER motors_init(void)
{
	static const UINT motor_pins[] = {MOTOR_L_A, MOTOR_L_B, MOTOR_R_A, MOTOR_R_B};
	static const UINT slices[] = {PWM_SLICE(MOTOR_L_A), PWM_SLICE(MOTOR_R_A)};
	UINT i;

	/* The kernel releases the PWM block from reset only when it uses it
	   as a physical timer (USE_PTMR), which it does not.  Clearing an
	   already clear reset bit is harmless (atomic CLR alias). */
	clr_w(RESETS_RESET, RESETS_RESET_PWM);
	while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0u) {
		;
	}

	/* Drive the pins low as plain GPIO before handing them to PWM. */
	for (i = 0u; i < 4u; ++i) {
		gpio_set_pin(motor_pins[i], GPIO_MODE_OUT);
		gpio_set_val(motor_pins[i], 0);
	}

	/* Divider 1.0; 125 MHz clock -> TOP 6249 -> 20 kHz. */
	pwm_top = (PWM_CLK_HZ / PWM_FREQUENCY_HZ) - 1u;

	for (i = 0u; i < 2u; ++i) {
		out_w(PWM_SLICE_CSR(slices[i]), 0u);	/* disabled, free-running */
		out_w(PWM_SLICE_DIV(slices[i]), PWM_DIV_ONE);
		out_w(PWM_SLICE_CTR(slices[i]), 0u);
	}
	for (i = 0u; i < 4u; ++i) {
		pwm_set_wrap(motor_pins[i], pwm_top);	/* per slice; twice is fine */
		pwm_set_cc(motor_pins[i], 0u);
	}
	for (i = 0u; i < 4u; ++i) {
		pwm_set_pin(motor_pins[i]);
	}
	motors_brake();
	pwm_set_enabled(MOTOR_L_A, TRUE);
	pwm_set_enabled(MOTOR_R_A, TRUE);

	return E_OK;
}

/* ========================
 *  Forward
 * ======================== */

/* Motor 1: A = PWM, B = LOW */
void motor1_forward(float speed_percent)
{
	pwm_set_cc(MOTOR_L_B, 0u);
	pwm_set_cc(MOTOR_L_A, speed_to_pwm(speed_percent));
}

/* Motor 2: A = PWM, B = LOW */
void motor2_forward(float speed_percent)
{
	pwm_set_cc(MOTOR_R_B, 0u);
	pwm_set_cc(MOTOR_R_A, speed_to_pwm(speed_percent));
}

void motors_forward(float speed_percent)
{
	motor1_forward(speed_percent);
	motor2_reverse(speed_percent);	/* motors are mirrored on the chassis */
}

/* ========================
 *  Reverse
 * ======================== */

/* Motor 1: A = LOW, B = PWM */
void motor1_reverse(float speed_percent)
{
	pwm_set_cc(MOTOR_L_A, 0u);
	pwm_set_cc(MOTOR_L_B, speed_to_pwm(speed_percent));
}

/* Motor 2: A = LOW, B = PWM */
void motor2_reverse(float speed_percent)
{
	pwm_set_cc(MOTOR_R_A, 0u);
	pwm_set_cc(MOTOR_R_B, speed_to_pwm(speed_percent));
}

void motors_reverse(float speed_percent)
{
	motor1_reverse(speed_percent);
	motor2_forward(speed_percent);
}

/* ========================
 *  Left / right (pivot)
 * ======================== */

void motors_left(float speed_percent)
{
	motor1_reverse(speed_percent);
	motor2_reverse(speed_percent);
}

void motors_right(float speed_percent)
{
	motor1_forward(speed_percent);
	motor2_forward(speed_percent);
}

/* ==========================================================
 *  ENCODER SECTION
 * ========================================================== */

/* IO_IRQ_BANK0 handler.  Runs on enc_core only, because only that core's
   INTE register has our bits.  Edge interrupts latch in INTR and are
   cleared by writing 1. */
LOCAL void encoder_inthdr(UINT intno)
{
	UW ints = in_w(IO_PROC_INTS0(enc_core)) & ENC_MASK;

	if ((ints & IO_EDGE_HIGH(ENCODER_L_A)) != 0u) {
		out_w(IO_INTR0, IO_EDGE_HIGH(ENCODER_L_A));
		encoder_left_count++;
	}
	if ((ints & IO_EDGE_HIGH(ENCODER_R_A)) != 0u) {
		out_w(IO_INTR0, IO_EDGE_HIGH(ENCODER_R_A));
		encoder_right_count++;
	}
	ClearInt(intno);
}

LOCAL void encoder_pin_init(UINT pin)
{
	gpio_set_pin(pin, GPIO_MODE_IN);
	clr_w(GPIO(pin), GPIO_PDE);	/* pull-down is on after reset */
	set_w(GPIO(pin), GPIO_PUE);	/* pull-up */
}

ER encoders_init(void)
{
	T_DINT	dint;
	ER	er;

	encoder_pin_init(ENCODER_L_A);
	encoder_pin_init(ENCODER_R_A);

	/* IO_IRQ_BANK0 is one interrupt for every GPIO.  This driver owns
	   it; another subsystem needing GPIO interrupts must share this
	   handler (record it in TEAM.md section 3). */
	enc_core = in_w(SIO_CPUID_ADDR) & 1u;
	out_w(IO_INTR0, ENC_MASK);		/* drop stale edges */

	dint.intatr = TA_HLNG;
	dint.inthdr = encoder_inthdr;
	er = tk_def_int(INTNO_IO_BANK0, &dint);
	if (er < E_OK) {
		return er;
	}
	set_w(IO_PROC_INTE0(enc_core), ENC_MASK);
	ClearInt(INTNO_IO_BANK0);
	EnableInt(INTNO_IO_BANK0, ENCODER_INTPRI);	/* this core's NVIC */
	return E_OK;
}

/* =============================
 *  Encoder counting
 * ============================= */

uint32_t encoder_get_left_count(void)
{
	return encoder_left_count;
}

uint32_t encoder_get_right_count(void)
{
	return encoder_right_count;
}

/* =============================
 *  Encoder measurement for distance and speed calculations
 * ============================= */

float encoder_left_revolutions(uint32_t pulses)
{
	return (float)pulses / LEFT_PULSES_PER_REV;
}

float encoder_right_revolutions(uint32_t pulses)
{
	return (float)pulses / RIGHT_PULSES_PER_REV;
}

float encoder_left_distance_cm(uint32_t pulses, float diameter_cm)
{
	return encoder_left_revolutions(pulses) * PI_F * diameter_cm;
}

float encoder_right_distance_cm(uint32_t pulses, float diameter_cm)
{
	return encoder_right_revolutions(pulses) * PI_F * diameter_cm;
}

/* ==========================================================
 *  PID controller (unchanged)
 * ========================================================== */

void pid_init(PIDController *pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->integral = 0.0f;
	pid->previous_error = 0.0f;

	/* Default output limits */
	pid->output_min = -20.0f;
	pid->output_max = 20.0f;
}

void pid_set_limits(PIDController *pid, float output_min, float output_max)
{
	if (output_min >= output_max) {
		return;
	}

	pid->output_min = output_min;
	pid->output_max = output_max;
}

float pid_update(PIDController *pid,
		 float target_speed,
		 float measured_speed,
		 float dt)
{
	if (dt <= 0.0f) {
		return 0.0f;
	}

	float error = target_speed - measured_speed;

	float derivative = (error - pid->previous_error) / dt;

	/* Test the proposed integral change before accepting it */
	float candidate_integral = pid->integral + error * dt;

	float output = pid->kp * error +
		       pid->ki * candidate_integral +
		       pid->kd * derivative;

	/* Anti-windup: stop accumulating while saturated in that direction */
	if (!((output > pid->output_max && error > 0.0f) ||
	      (output < pid->output_min && error < 0.0f))) {
		pid->integral = candidate_integral;
	}

	/* Recalculate with the accepted integral */
	output = pid->kp * error +
		 pid->ki * pid->integral +
		 pid->kd * derivative;

	/* Clamp output to the configured range */
	if (output > pid->output_max) {
		output = pid->output_max;
	} else if (output < pid->output_min) {
		output = pid->output_min;
	}

	pid->previous_error = error;

	return output;
}
