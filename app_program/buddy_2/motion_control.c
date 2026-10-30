
#include "motion_control.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"

#define MOTOR_L_A 8u
#define MOTOR_L_B 9u
#define MOTOR_R_A 10u
#define MOTOR_R_B 11u

// Identify the input pins for the two wheel encoders
#define ENCODER_R_A 6u
#define ENCODER_L_A 7u

#define PWM_FREQUENCY_HZ 20000u

//  Encoder measurement for distance and speed calculations
#define LEFT_PULSES_PER_REV  617.0f
#define RIGHT_PULSES_PER_REV 616.0f

#define PI_F 3.14159265f

//  Distance-based Movement
#define MOVE_PWM 20.0f
#define MOVE_TIMEOUT_MS 4000u
#define MOVE_ENCODER_TIMEOUT_MS 250u
#define MOVE_UPDATE_MS 10u

//  Encoder-based Turning
#define TURN_PWM 20.0f
#define TURN_TIMEOUT_MS 4000u
#define TURN_ENCODER_TIMEOUT_MS 250u
#define TURN_UPDATE_MS 10u

static volatile uint32_t encoder_left_count = 0u;
static volatile uint32_t encoder_right_count = 0u;

static uint16_t pwm_top = 0u; // This stores Max PWM counter value


static uint16_t speed_to_pwm(float speed_percent)
{
    // Ensure PWM remains between 0% and 100%.
    if (speed_percent < 0.0f)
    {
        speed_percent = 0.0f;
    }
    else if (speed_percent > 100.0f)
    {
        speed_percent = 100.0f;
    }

    // Convert percentage to PWM compare level.
    float pwm_level =
        (speed_percent * (float)(pwm_top + 1u)) / 100.0f;

    return (uint16_t)(pwm_level + 0.5f);
}


// ==========================================================
//  MOTORS SECTION
// ==========================================================

void motors_brake(void)
{
    // LOW/LOW = Brake (Robo Pico datasheet).
    pwm_set_gpio_level(MOTOR_L_A, 0u);
    pwm_set_gpio_level(MOTOR_L_B, 0u);
    pwm_set_gpio_level(MOTOR_R_A, 0u);
    pwm_set_gpio_level(MOTOR_R_B, 0u);
}


void motors_init(void)
{
    const uint motor_pins[] = {MOTOR_L_A, MOTOR_L_B, MOTOR_R_A, MOTOR_R_B};

    // Set PWM compare levels to zero before enabling output on the GPIO pins.
    for (uint i = 0u; i < 4u; ++i) {
        gpio_init(motor_pins[i]);
        gpio_set_dir(motor_pins[i], GPIO_OUT);
        gpio_put(motor_pins[i], 0);
        pwm_set_gpio_level(motor_pins[i], 0u);
    }

    uint32_t system_clock = clock_get_hz(clk_sys);
    // Divider 1.0; 125 MHz clock -> TOP 6249 -> 20 kHz.
    pwm_top = (uint16_t)((system_clock / PWM_FREQUENCY_HZ) - 1u);

    uint left_slice = pwm_gpio_to_slice_num(MOTOR_L_A);
    uint right_slice = pwm_gpio_to_slice_num(MOTOR_R_A);

    pwm_set_enabled(left_slice, false);
    pwm_set_enabled(right_slice, false);
    pwm_set_clkdiv(left_slice, 1.0f);
    pwm_set_clkdiv(right_slice, 1.0f);
    pwm_set_wrap(left_slice, pwm_top);
    pwm_set_wrap(right_slice, pwm_top);

    for (uint i = 0u; i < 4u; ++i) {
        gpio_set_function(motor_pins[i], GPIO_FUNC_PWM);
    }
    motors_brake();
    pwm_set_enabled(left_slice, true);
    pwm_set_enabled(right_slice, true);
}

// ========================
//  going forwardddddd
// ========================

// -------- Motor 1 Forward --------
void motor1_forward(float speed_percent)
{
    // Motor 1: A = PWM, B = LOW
    pwm_set_gpio_level(MOTOR_L_B, 0u);
    pwm_set_gpio_level(MOTOR_L_A, speed_to_pwm(speed_percent));
}

// -------- Motor 2 Forward --------
void motor2_forward(float speed_percent)
{
    // Motor 2: A = PWM, B = LOW
    pwm_set_gpio_level(MOTOR_R_B, 0u);
    pwm_set_gpio_level(MOTOR_R_A, speed_to_pwm(speed_percent));
}

// ---- COMBINED Moving FORWARD ----
void motors_forward(float speed_percent)
{
    motor1_forward(speed_percent);
    motor2_reverse(speed_percent); //wheel in reverse so both sides dont spin the same
}


// =============================
//  REVERSINGG BACK or smth idk
// =============================

// -------- Motor 1 Reverse --------
void motor1_reverse(float speed_percent)
{
    uint16_t pwm_level = speed_to_pwm(speed_percent);

    // Motor 1: A = LOW, B = PWM
    pwm_set_gpio_level(MOTOR_L_A, 0u);
    pwm_set_gpio_level(MOTOR_L_B, speed_to_pwm(speed_percent));
}

// -------- Motor 2 Reverse --------
void motor2_reverse(float speed_percent)
{
   // Motor 2: A = LOW, B = PWM
    pwm_set_gpio_level(MOTOR_R_A, 0u);
    pwm_set_gpio_level(MOTOR_R_B, speed_to_pwm(speed_percent));
}

// ---- COMBINED Moving BACKWARDS ----
// literally the same as forward just flipped
void motors_reverse(float speed_percent)
{
    motor1_reverse(speed_percent);
    motor2_forward(speed_percent);
}

// =============================
//  LEFT (or at least i hope its left)
// =============================

void motors_left(float speed_percent)
{
    // Right Wheel backward
    motor1_reverse(speed_percent);
    // Left Wheel forward
    motor2_reverse(speed_percent);
}

// =============================
//  RIGHT (praying its right)
// =============================
void motors_right(float speed_percent)
{
    // Right Wheel forward
    motor1_forward(speed_percent);
    // Left Wheel backward
    motor2_forward(speed_percent);
}

static void encoder_irq_callback(uint gpio, uint32_t events)
{
    if ((events & GPIO_IRQ_EDGE_RISE) == 0u) {
        return;
    }
    if (gpio == ENCODER_L_A) {
        encoder_left_count++;
    } else if (gpio == ENCODER_R_A) {
        encoder_right_count++;
    }
}

// ==========================================================
//  ENCODER SECTION
// ==========================================================

void encoders_init(void)
{
    gpio_init(ENCODER_L_A);
    gpio_set_dir(ENCODER_L_A, GPIO_IN);
    gpio_pull_up(ENCODER_L_A);

    gpio_init(ENCODER_R_A);
    gpio_set_dir(ENCODER_R_A, GPIO_IN);
    gpio_pull_up(ENCODER_R_A);

    // One callback per core. Coordinate with other buddies before integration.
    gpio_set_irq_enabled_with_callback(
        ENCODER_L_A, GPIO_IRQ_EDGE_RISE, true, &encoder_irq_callback);
    gpio_set_irq_enabled(ENCODER_R_A, GPIO_IRQ_EDGE_RISE, true);
}

// =============================
//  Encoder Counting
// =============================

uint32_t encoder_get_left_count(void)
{
    uint32_t save = save_and_disable_interrupts();
    uint32_t count = encoder_left_count;
    restore_interrupts(save);
    return count;
}

uint32_t encoder_get_right_count(void)
{
    uint32_t save = save_and_disable_interrupts();
    uint32_t count = encoder_right_count;
    restore_interrupts(save);
    return count;
}

// =============================
//  Encoder measurement for distance and speed calculations
// =============================


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

// PID controller functions
void pid_init(PIDController *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;

    // Default output limits
    pid->output_min = -20.0f;
    pid->output_max = 20.0f;
}

void pid_set_limits(PIDController *pid,
                    float output_min,
                    float output_max)
{
    if (output_min >= output_max)
    {
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
    if (dt <= 0.0f)
    {
        return 0.0f;
    }

    float error = target_speed - measured_speed;

    float derivative =
        (error - pid->previous_error) / dt;

    // Tests the proposed integral change before accepting it
    float candidate_integral = 
        pid->integral + error * dt;

    float output =
        pid->kp * error +
        pid->ki * candidate_integral +
        pid->kd * derivative;

    // Anti-windup -  *prevents the PID controller from continuously accumulating errors
    if (!((output > pid->output_max && error > 0.0f) ||
          (output < pid->output_min && error < 0.0f)))
    {
        pid->integral = candidate_integral;
    }

    // Recalculate with accepted integral
    output =
        pid->kp * error +
        pid->ki * pid->integral +
        pid->kd * derivative;

    // Clamp output - *Keeps the returned correction inside the configured range
    if (output > pid->output_max)
    {
        output = pid->output_max;
    }
    else if (output < pid->output_min)
    {
        output = pid->output_min;
    }

    pid->previous_error = error;

    return output;
}

// ==========================================================
//  Distance-based Movement
// ==========================================================

bool motors_move_forward_revolutions(float target_revolutions)
{
    // Reject invalid movement requests.
    if (target_revolutions <= 0.0f)
    {
        motors_brake();
        return false;
    }

    motors_brake();

    // Record encoder counts before movement.
    uint32_t start_left = encoder_get_left_count();
    uint32_t start_right = encoder_get_right_count();

    uint32_t previous_left = start_left;
    uint32_t previous_right = start_right;

    uint64_t start_time = time_us_64();
    uint64_t last_left_pulse = start_time;
    uint64_t last_right_pulse = start_time;

    bool left_done = false;
    bool right_done = false;
    bool success = false;

    // Start both wheels moving forward.
    motor1_forward(MOVE_PWM);
    motor2_reverse(MOVE_PWM);

    while (true)
    {
        uint64_t now = time_us_64();

        uint32_t current_left = encoder_get_left_count();
        uint32_t current_right = encoder_get_right_count();

        uint32_t left_pulses = current_left - start_left;
        uint32_t right_pulses = current_right - start_right;

        if (current_left != previous_left)
        {
            last_left_pulse = now;
        }

        if (current_right != previous_right)
        {
            last_right_pulse = now;
        }

        float left_revolutions =
            encoder_left_revolutions(left_pulses);

        float right_revolutions =
            encoder_right_revolutions(right_pulses);

        // Stop the left wheel at its target.
        if (!left_done && left_revolutions >= target_revolutions)
        {
            motor1_forward(0.0f);
            left_done = true;
        }

        // Stop the right wheel at its target.
        if (!right_done && right_revolutions >= target_revolutions)
        {
            motor2_reverse(0.0f);
            right_done = true;
        }

        // Both wheels reached the target.
        if (left_done && right_done)
        {
            success = true;
            break;
        }

        // Prevent the motors running indefinitely.
        if (now - start_time >= MOVE_TIMEOUT_MS * 1000ULL)
        {
            break;
        }

        // Check encoder timeout for moving wheels only.
        if ((!left_done &&
             now - last_left_pulse >= MOVE_ENCODER_TIMEOUT_MS * 1000ULL) ||
            (!right_done &&
             now - last_right_pulse >= MOVE_ENCODER_TIMEOUT_MS * 1000ULL))
        {
            break;
        }

        previous_left = current_left;
        previous_right = current_right;

        sleep_ms(MOVE_UPDATE_MS);
    }

    // Always stop both motors before returning.
    motors_brake();

    return success;
}


// ==========================================================
//  Encoder-based Turning
// ==========================================================

static bool motors_turn_revolutions(float target_revolutions,
                                    bool turn_left)
{
    // Reject invalid turning requests.
    if (!(target_revolutions > 0.0f))
    {
        motors_brake();
        return false;
    }

    motors_brake();

    // Record encoder counts before turning.
    uint32_t start_left = encoder_get_left_count();
    uint32_t start_right = encoder_get_right_count();

    uint32_t previous_left = start_left;
    uint32_t previous_right = start_right;

    uint64_t start_time = time_us_64();
    uint64_t last_left_pulse = start_time;
    uint64_t last_right_pulse = start_time;

    bool left_done = false;
    bool right_done = false;
    bool success = false;

    // Start pivoting in the requested direction.
    if (turn_left)
    {
        motors_left(TURN_PWM);
    }
    else
    {
        motors_right(TURN_PWM);
    }

    while (true)
    {
        uint64_t now = time_us_64();

        uint32_t current_left = encoder_get_left_count();
        uint32_t current_right = encoder_get_right_count();

        uint32_t left_pulses = current_left - start_left;
        uint32_t right_pulses = current_right - start_right;

        if (current_left != previous_left)
        {
            last_left_pulse = now;
        }

        if (current_right != previous_right)
        {
            last_right_pulse = now;
        }

        float left_revolutions =
            encoder_left_revolutions(left_pulses);

        float right_revolutions =
            encoder_right_revolutions(right_pulses);

        // Stop the left wheel at its target.
        if (!left_done &&
            left_revolutions >= target_revolutions)
        {
            motor1_forward(0.0f);
            left_done = true;
        }

        // Stop the right wheel at its target.
        if (!right_done &&
            right_revolutions >= target_revolutions)
        {
            motor2_reverse(0.0f);
            right_done = true;
        }

        // Both wheels have reached their targets.
        if (left_done && right_done)
        {
            success = true;
            break;
        }

        // Prevent the motors running indefinitely.
        if (now - start_time >= TURN_TIMEOUT_MS * 1000ULL)
        {
            break;
        }

        // Check encoder timeout for moving wheels only.
        if ((!left_done &&
             now - last_left_pulse >=
                 TURN_ENCODER_TIMEOUT_MS * 1000ULL) ||
            (!right_done &&
             now - last_right_pulse >=
                 TURN_ENCODER_TIMEOUT_MS * 1000ULL))
        {
            break;
        }

        previous_left = current_left;
        previous_right = current_right;

        sleep_ms(TURN_UPDATE_MS);
    }

    motors_brake();

    return success;
}

bool motors_turn_left_revolutions(float revolutions)
{
    return motors_turn_revolutions(revolutions, true);
}

bool motors_turn_right_revolutions(float revolutions)
{
    return motors_turn_revolutions(revolutions, false);
}

