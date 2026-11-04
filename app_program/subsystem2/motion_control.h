/*
 *	motion_control.h
 *	Subsystem 2 -- Motion Control (Buddy 2): motor, encoder and PID driver.
 *
 *	Ported from buddy2_motion_control/motion_control.h (Pico SDK) to this
 *	micro T-Kernel port: libbsp gpio_* / pwm_*, direct RP2040 registers and
 *	tk_def_int() instead of hardware_pwm, gpio IRQ callbacks and sleep_ms.
 *
 *	PRIVATE to subsystem 2.  Only motion_task (task_motion.c) may call the
 *	motor and encoder functions: it is the owning task of the motor PWM
 *	slices and the encoder pins (TEAM.md section 3).  The PID and the
 *	revolution/distance helpers are pure functions and safe anywhere.
 *
 *	Pins (Robo Pico):  motor L = GP8/GP9 (PWM slice 4 A/B)
 *	                   motor R = GP10/GP11 (PWM slice 5 A/B)
 *	                   encoder R A/B = GP6/GP16, encoder L A/B = GP7/GP17
 */

#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "../app.h"
#include <stdint.h>
#include <stdbool.h>

/* Motors.  motors_init() must run on the owning task before any other
   motor call; it releases the PWM block from reset. */
ER   motors_init(void);
void motors_brake(void);

/* FORWARD */
void motor1_forward(float speed_percent);
void motor2_forward(float speed_percent);
void motors_forward(float speed_percent);

/* REVERSE */
void motor1_reverse(float speed_percent);
void motor2_reverse(float speed_percent);
void motors_reverse(float speed_percent);

/* LEFT & RIGHT (pivot) */
void motors_left(float speed_percent);
void motors_right(float speed_percent);

/* Single-channel encoder pulse counting (rising edges).
   encoders_init() enables the GPIO interrupt on the core it is called
   from, so call it from the owning task, which is pinned to one core. */
ER       encoders_init(void);

/* Shared IO_IRQ_BANK0 registration for one auxiliary GPIO on GP0..GP29.
   Both rising and falling edges are dispatched on the encoder-owning core.
   GP28 uses GPIO interrupt bank 3; GP6 and GP7 use bank 0.
   Register only after encoders_init() succeeds.  Callback runs in IRQ
   context: do not block, allocate, or print from it.
   The events argument contains the raw IO_BANK0 edge-bit mask. */
typedef void (*motion_gpio_irq_callback_t)(UINT gpio, UINT events);
ER motion_gpio_irq_register(UINT gpio, motion_gpio_irq_callback_t callback);
uint32_t encoder_get_left_count(void);
uint32_t encoder_get_right_count(void);

/* Wheel revolutions and distance */
float encoder_left_revolutions(uint32_t pulses);
float encoder_right_revolutions(uint32_t pulses);

float encoder_left_distance_cm(uint32_t pulses, float diameter_cm);
float encoder_right_distance_cm(uint32_t pulses, float diameter_cm);

/* Basic PID controller */
typedef struct
{
	float kp;
	float ki;
	float kd;

	float integral;
	float previous_error;

	/* Output limits */
	float output_min;
	float output_max;
} PIDController;

void  pid_init(PIDController *pid, float kp, float ki, float kd);

float pid_update(PIDController *pid,
		 float target_speed,
		 float measured_speed,
		 float dt);

/* So that output doesn't become negative or extremely large */
void  pid_set_limits(PIDController *pid,
		     float output_min,
		     float output_max);

#endif /* MOTION_CONTROL_H */
