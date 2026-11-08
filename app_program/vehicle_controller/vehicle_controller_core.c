#include "vehicle_controller_core.h"

void vc_core_init(vc_core_t *vc)
{
    if (!vc) return;
    *vc = (vc_core_t){0};
    vc->state = VC_INITIALIZING;
}

void vc_core_line(vc_core_t *vc, vc_line_intent_t intent, uint32_t now_ms)
{
    if (!vc) return;
    if (intent < VC_INTENT_NONE || intent > VC_INTENT_STOP) return;
    vc->intent = intent;
    vc->last_line_ms = now_ms;
    vc->line_received = 1;
    vc->line_visible = 1;
}

void vc_core_line_visible(vc_core_t *vc, int line_visible)
{
    if (vc) vc->line_visible = !!line_visible;
}

void vc_core_imu(vc_core_t *vc, vc_imu_event_t event, uint8_t valid, uint8_t height_mm)
{
    if (!vc) return;
    switch (event) {
    case VC_IMU_READY:
        vc->imu_ready = 1;
        vc->imu_fault = 0;
        vc->hump_active = 0;
        break;
    case VC_IMU_HUMP_START:
        if (vc->imu_ready && !vc->imu_fault) vc->hump_active = 1;
        break;
    case VC_IMU_HUMP_DONE:
        vc->hump_active = 0;
        vc->last_hump_valid = !!valid;
        vc->last_hump_mm = valid ? height_mm : 0;
        break;
    case VC_IMU_SHOCK_MINOR: break;
    case VC_IMU_FAULT:
        vc->imu_fault = 1;
        vc->imu_ready = 0;
        vc->hump_active = 0;
        break;
    default: break;
    }
}

void vc_core_set_safety(vc_core_t *vc, int request_pending, int estop_latched)
{
    if (!vc) return;
    vc->safety_request = !!request_pending;
    vc->safety_latched = !!estop_latched;
}
void vc_core_pause(vc_core_t *vc) { if (vc) vc->paused = 1; }
void vc_core_resume(vc_core_t *vc) { if (vc) vc->paused = 0; }
void vc_core_complete(vc_core_t *vc) { if (vc) vc->mission_complete = 1; }

vc_output_t vc_core_step(vc_core_t *vc, uint32_t now_ms)
{
    int stale;
    if (!vc) return VC_OUTPUT_HALT;
    /* Safety and sensor health dominate all movement intents. */
    if (vc->safety_latched || vc->safety_request || vc->imu_fault) {
        vc->state = VC_ERROR;
        return VC_OUTPUT_HALT;
    }
    if (vc->mission_complete) {
        vc->state = VC_MISSION_COMPLETE;
        return VC_OUTPUT_HALT;
    }
    if (vc->paused) {
        vc->state = VC_PAUSED;
        return VC_OUTPUT_HALT;
    }
    if (!vc->imu_ready) {
        vc->state = VC_INITIALIZING;
        return VC_OUTPUT_HALT;
    }
    stale = !vc->line_received || (uint32_t)(now_ms - vc->last_line_ms) > VC_LINE_TIMEOUT_MS;
    if (stale || vc->intent == VC_INTENT_STOP || vc->intent == VC_INTENT_NONE) {
        vc->state = VC_LINE_SEARCH;
        return VC_OUTPUT_HALT;
    }
    if (!vc->line_visible) {
        vc->state = VC_LINE_SEARCH;
        return VC_OUTPUT_HALT;
    }
    if (vc->hump_active) {
        vc->state = VC_HUMP_CROSSING;
        /* There is no speed control command in Buddy 2's API yet.
         * HALT is intentionally fail-safe until reduced-speed support exists. */
        return VC_OUTPUT_HALT;
    }
    if (vc->intent == VC_INTENT_LINE_LOST) {
        vc->state = VC_LINE_SEARCH;
        return VC_OUTPUT_HALT;
    }
    vc->state = VC_DRIVING;
    switch (vc->intent) {
    case VC_INTENT_FORWARD: return VC_OUTPUT_FORWARD;
    case VC_INTENT_STEER_LEFT: return VC_OUTPUT_STEER_LEFT;
    case VC_INTENT_STEER_RIGHT: return VC_OUTPUT_STEER_RIGHT;
    case VC_INTENT_PIVOT_LEFT: return VC_OUTPUT_PIVOT_LEFT;
    case VC_INTENT_PIVOT_RIGHT: return VC_OUTPUT_PIVOT_RIGHT;
    default: return VC_OUTPUT_HALT;
    }
}
