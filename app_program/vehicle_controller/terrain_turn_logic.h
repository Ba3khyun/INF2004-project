#ifndef B4_TERRAIN_TURN_LOGIC_H
#define B4_TERRAIN_TURN_LOGIC_H
#include <stdint.h>

/* Pure C turn-angle agreement checker. No motor writes or RTOS calls.
 * Signed target and encoder angle: clockwise positive, CCW negative.
 * Encoder angle must come from Buddy 2's future per-wheel progress API:
 * current motion_get_distance_mm() cannot provide turning direction.
 * Magnetometer is a cross-check, not the sole authority. */
#define B4_TURN_TOLERANCE_DEG 10.0f
#define B4_TURN_DEFAULT_TIMEOUT_MS 3000u

typedef enum {
    B4_TURN_TRACKING = 0,
    B4_TURN_COMPLETE_AGREED,
    B4_TURN_COMPLETE_ENCODER_FALLBACK,
    B4_TURN_TIMEOUT,
    B4_TURN_INVALID
} b4_turn_result_t;

typedef struct {
    float start_heading_deg;
    float target_signed_deg;
    uint32_t started_ms;
    uint32_t timeout_ms;
    uint8_t active;
} b4_turn_checker_t;

float b4_turn_wrap_signed(float delta_deg);
int b4_turn_begin(b4_turn_checker_t *c, float start_heading_deg,
                  float target_signed_deg, uint32_t now_ms, uint32_t timeout_ms);
b4_turn_result_t b4_turn_check(const b4_turn_checker_t *c,
                               float encoder_signed_deg, float current_heading_deg,
                               int mag_valid, uint32_t now_ms,
                               float *out_disagreement_deg);

/* Hump event model: no motors. During a hump, a slowdown is REQUESTED,
 * but existing Buddy 2 command API lacks a variable-speed setter.
 * The live controller must stay at HALT until a reviewed speed API exists. */
typedef enum { B4_TERRAIN_NORMAL = 0, B4_TERRAIN_SLOW_REQUEST, B4_TERRAIN_HALT } b4_terrain_request_t;
b4_terrain_request_t b4_terrain_request(int hump_active, int imu_valid, int safety_clear);
#endif
