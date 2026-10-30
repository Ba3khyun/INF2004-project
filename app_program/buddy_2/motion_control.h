
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

// Motors
void motors_init(void);
void motors_brake(void);

// FORWARD
void motor1_forward(float speed_percent);
void motor2_forward(float speed_percent);
void motors_forward(float speed_percent);

//REVERSE
void motor1_reverse(float speed_percent);
void motor2_reverse(float speed_percent);
void motors_reverse(float speed_percent);

//LEFT & RIGHT ~
void motors_left(float speed_percent);
void motors_right(float speed_percent);

// Initial single-channel encoder pulse counting.
void encoders_init(void);
uint32_t encoder_get_left_count(void);
uint32_t encoder_get_right_count(void);

// Wheel revolutions
float encoder_left_revolutions(uint32_t pulses);
float encoder_right_revolutions(uint32_t pulses);

float encoder_left_distance_cm(uint32_t pulses, float diameter_cm);
float encoder_right_distance_cm(uint32_t pulses, float diameter_cm);


// Basic PID controller

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float previous_error;

    // Setting limits
    float output_min;
    float output_max;
} PIDController;


void pid_init(PIDController *pid, float kp, float ki, float kd);

float pid_update(PIDController *pid,
                 float target_speed,
                 float measured_speed,
                 float dt);

// So that output doesn't become negative or extremely large
void pid_set_limits(PIDController *pid,
                    float output_min,
                    float output_max);


// Distance-based movement
bool motors_move_forward_revolutions(float target_revolutions);

// Encoder-based turning
bool motors_turn_left_revolutions(float revolutions);
bool motors_turn_right_revolutions(float revolutions);

#endif
