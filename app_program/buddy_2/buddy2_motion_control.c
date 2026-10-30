
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "motion_control.h"

// ========================
// Configuration
// ========================

#define CONTROL_INTERVAL_US 10000u
#define MOVE_TIMEOUT_US 4000000u
#define ENCODER_TIMEOUT_US 250000u

#define FORWARD_REVOLUTIONS 0.50f
#define TURN_REVOLUTIONS 0.25f

#define MOVE_PWM 20.0f

// ========================
// Motion States
// ========================

typedef enum
{
    MOTION_IDLE,
    MOTION_FORWARD,
    MOTION_LEFT,
    MOTION_RIGHT,
    MOTION_EMERGENCY_STOP
} MotionState;

static MotionState motion_state = MOTION_IDLE;

static bool left_done = false;
static bool right_done = false;

static uint32_t start_left = 0u;
static uint32_t start_right = 0u;

static uint32_t previous_left = 0u;
static uint32_t previous_right = 0u;

static uint64_t start_time = 0u;
static uint64_t last_left_pulse = 0u;
static uint64_t last_right_pulse = 0u;

static float target_revolutions = 0.0f;

// ========================
// Emergency Stop
// ========================

static void emergency_stop(const char *reason)
{
    // Stop immediately and latch emergency state.
    motors_brake();
    motion_state = MOTION_EMERGENCY_STOP;

    left_done = true;
    right_done = true;

    printf("\nEMERGENCY STOP: %s\n", reason);
    printf("Motors stopped. Type C to clear.\n");
}

static void clear_emergency_stop(void)
{
    if (motion_state != MOTION_EMERGENCY_STOP)
    {
        printf("No emergency stop to clear.\n");
        return;
    }

    motors_brake();
    motion_state = MOTION_IDLE;

    printf("\nEmergency stop cleared.\n");
    printf("Motion commands can be accepted again.\n");
}

// ========================
// Start Movement
// ========================

static void start_motion(MotionState command)
{
    if (motion_state == MOTION_EMERGENCY_STOP)
    {
        printf("Movement rejected: emergency stop active.\n");
        return;
    }

    if (motion_state != MOTION_IDLE)
    {
        printf("Movement rejected: robot is busy.\n");
        return;
    }

    motors_brake();

    start_left = encoder_get_left_count();
    start_right = encoder_get_right_count();

    previous_left = start_left;
    previous_right = start_right;

    start_time = time_us_64();
    last_left_pulse = start_time;
    last_right_pulse = start_time;

    left_done = false;
    right_done = false;

    motion_state = command;

    if (command == MOTION_FORWARD)
    {
        target_revolutions = FORWARD_REVOLUTIONS;

        printf("\nStarting FORWARD movement...\n");

        motor1_forward(MOVE_PWM);
        motor2_reverse(MOVE_PWM);
    }
    else if (command == MOTION_LEFT)
    {
        target_revolutions = TURN_REVOLUTIONS;

        printf("\nStarting LEFT turn...\n");

        motors_left(MOVE_PWM);
    }
    else if (command == MOTION_RIGHT)
    {
        target_revolutions = TURN_REVOLUTIONS;

        printf("\nStarting RIGHT turn...\n");

        motors_right(MOVE_PWM);
    }
}

// ========================
// Motion Update
// ========================

static void update_motion(void)
{
    if (motion_state == MOTION_IDLE ||
        motion_state == MOTION_EMERGENCY_STOP)
    {
        return;
    }

    uint64_t now = time_us_64();

    uint32_t current_left = encoder_get_left_count();
    uint32_t current_right = encoder_get_right_count();

    uint32_t left_pulses =
        current_left - start_left;

    uint32_t right_pulses =
        current_right - start_right;

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

    // Stop left motor when target is reached.
    if (!left_done &&
        left_revolutions >= target_revolutions)
    {
        motor1_forward(0.0f);
        left_done = true;
    }

    // Stop right motor when target is reached.
    if (!right_done &&
        right_revolutions >= target_revolutions)
    {
        motor2_reverse(0.0f);
        right_done = true;
    }

    // Both wheels reached the target.
    if (left_done && right_done)
    {
        motors_brake();
        motion_state = MOTION_IDLE;

        printf("\nMovement completed.\n");

        printf("Left revolutions: %.3f\n",
               (double)left_revolutions);

        printf("Right revolutions: %.3f\n",
               (double)right_revolutions);

        printf("Motors stopped.\n");
        return;
    }

    // Prevent movement from running indefinitely.
    if (now - start_time >= MOVE_TIMEOUT_US)
    {
        emergency_stop("Movement timeout");
        return;
    }

    // Stop when a moving wheel loses encoder pulses.
    if ((!left_done &&
         now - last_left_pulse >= ENCODER_TIMEOUT_US) ||
        (!right_done &&
         now - last_right_pulse >= ENCODER_TIMEOUT_US))
    {
        emergency_stop("Encoder timeout");
        return;
    }

    previous_left = current_left;
    previous_right = current_right;
}

// ========================
// Serial Commands
// ========================

static void process_command(int input)
{
    if (input == 'X' || input == 'x')
    {
        emergency_stop("Serial stop command");
        return;
    }

    if (input == 'C' || input == 'c')
    {
        clear_emergency_stop();
        return;
    }

    if (input == 'F' || input == 'f')
    {
        start_motion(MOTION_FORWARD);
    }
    else if (input == 'L' || input == 'l')
    {
        start_motion(MOTION_LEFT);
    }
    else if (input == 'R' || input == 'r')
    {
        start_motion(MOTION_RIGHT);
    }
}

// ========================
// Main Program
// ========================

int main(void)
{
    stdio_init_all();

    motors_init();
    motors_brake();
    encoders_init();

    bool monitor_was_connected = false;

    uint64_t next_update =
        time_us_64() + CONTROL_INTERVAL_US;

    while (true)
    {
        bool connected = stdio_usb_connected();

        if (!connected)
        {
            motors_brake();

            // A disconnected monitor must not leave
            // an active movement running.
            if (monitor_was_connected &&
                motion_state != MOTION_EMERGENCY_STOP)
            {
                motion_state = MOTION_EMERGENCY_STOP;
            }

            monitor_was_connected = false;
            sleep_ms(10);

            next_update =
                time_us_64() + CONTROL_INTERVAL_US;

            continue;
        }

        if (!monitor_was_connected)
        {
            monitor_was_connected = true;

            printf("\nBuddy 2 Motion Command Test\n");
            printf("F = Move forward\n");
            printf("L = Turn left\n");
            printf("R = Turn right\n");
            printf("X = Emergency stop\n");
            printf("C = Clear emergency stop\n");

            if (motion_state == MOTION_EMERGENCY_STOP)
            {
                printf("Emergency stop active. Type C to clear.\n");
            }
        }

        // Check serial commands before the motion update.
        int input = getchar_timeout_us(0);

        while (input != PICO_ERROR_TIMEOUT)
        {
            process_command(input);
            input = getchar_timeout_us(0);
        }

        uint64_t now = time_us_64();

        if (now >= next_update)
        {
            update_motion();

            next_update += CONTROL_INTERVAL_US;

            if (now >= next_update)
            {
                next_update = now + CONTROL_INTERVAL_US;
            }
        }

        sleep_ms(1);
    }
}
