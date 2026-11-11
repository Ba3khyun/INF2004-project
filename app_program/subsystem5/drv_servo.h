/*
 *	drv_servo.h
 *	Subsystem 5 -- scanning servo driver.  PRIVATE to S5.
 *	Call only from obstacle_task, the owner of the servo PWM channel.
 *
 *	Angles here are LOGICAL: 0 = car's right, 90 = ahead, 180 = left.
 *	S5_SERVO_MIRROR in s5_config.h flips them if your servo is mounted
 *	the other way round.
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include "subsystem5.h"

/* Set up 50 Hz PWM on S5_PIN_SERVO and centre the servo */
IMPORT ER	servo_init(void);

/* Pulse width (us) that the servo needs for a logical angle */
IMPORT UW	servo_pulse_us(UINT angle_deg);

/* Move to an angle and sleep until the servo has arrived */
IMPORT void	servo_move(UINT angle_deg);

/* Last commanded logical angle */
IMPORT UINT	servo_angle(void);

#endif /* DRV_SERVO_H */
