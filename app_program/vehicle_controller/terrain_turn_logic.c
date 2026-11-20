#include "terrain_turn_logic.h"

static float absf_local(float x) { return (x < 0.0f) ? -x : x; }
/* A heading delta is meaningful only for turns of at most 180 degrees. */
float b4_turn_wrap_signed(float d)
{
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}
int b4_turn_begin(b4_turn_checker_t *c, float start_heading_deg,
                  float target_signed_deg, uint32_t now_ms, uint32_t timeout_ms)
{
    if (!c || start_heading_deg < 0.0f || start_heading_deg >= 360.0f ||
        target_signed_deg == 0.0f || absf_local(target_signed_deg) > 180.0f ||
        timeout_ms == 0u) return 0;
    c->start_heading_deg = start_heading_deg;
    c->target_signed_deg = target_signed_deg;
    c->started_ms = now_ms;
    c->timeout_ms = timeout_ms;
    c->active = 1u;
    return 1;
}
b4_turn_result_t b4_turn_check(const b4_turn_checker_t *c,
                               float encoder_signed_deg, float current_heading_deg,
                               int mag_valid, uint32_t now_ms,
                               float *out_disagreement_deg)
{
    float mag_delta = 0.0f;
    float error = 0.0f;
    int encoder_at_target;
    int timed_out;
    if (out_disagreement_deg) *out_disagreement_deg = 0.0f;
    if (!c || !c->active || (mag_valid &&
        (current_heading_deg < 0.0f || current_heading_deg >= 360.0f)))
        return B4_TURN_INVALID;
    encoder_at_target = absf_local(encoder_signed_deg - c->target_signed_deg)
                        <= B4_TURN_TOLERANCE_DEG;
    timed_out = ((uint32_t)(now_ms - c->started_ms) >= c->timeout_ms);
    if (mag_valid) {
        mag_delta = b4_turn_wrap_signed(current_heading_deg - c->start_heading_deg);
        error = absf_local(mag_delta - encoder_signed_deg);
        if (out_disagreement_deg) *out_disagreement_deg = error;
        if (encoder_at_target && error <= B4_TURN_TOLERANCE_DEG)
            return B4_TURN_COMPLETE_AGREED;
    }
    /* The tribunal requires encoder fallback ONLY after the turn timeout.
     * Completion here is a reporting result; never authorizes motion. */
    if (timed_out) {
        if (encoder_at_target) return B4_TURN_COMPLETE_ENCODER_FALLBACK;
        return B4_TURN_TIMEOUT;
    }
    return B4_TURN_TRACKING;
}
b4_terrain_request_t b4_terrain_request(int hump_active, int imu_valid, int safety_clear)
{
    if (!imu_valid || !safety_clear) return B4_TERRAIN_HALT;
    return hump_active ? B4_TERRAIN_SLOW_REQUEST : B4_TERRAIN_NORMAL;
}
