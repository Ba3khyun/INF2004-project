#include "vehicle_controller_motion_adapter.h"

/* Intentionally no RTOS include and no motor-command posting API. */
vma_result_t vma_preview(vc_output_t output,
                         const vma_inputs_t *inputs,
                         const vma_codes_t *codes)
{
    vma_result_t r = {0};
    if (!codes || !inputs) {
        r.blockers = VMA_BLOCK_BAD_INPUT;
        return r;
    }
    r.candidate_command = codes->halt;
    r.safe_command = codes->halt;

    switch (output) {
    case VC_OUTPUT_HALT:        break;
    case VC_OUTPUT_FORWARD:     r.candidate_command = codes->forward; break;
    case VC_OUTPUT_STEER_LEFT:  r.candidate_command = codes->steer_left; break;
    case VC_OUTPUT_STEER_RIGHT: r.candidate_command = codes->steer_right; break;
    case VC_OUTPUT_PIVOT_LEFT:  r.candidate_command = codes->pivot_left; break;
    case VC_OUTPUT_PIVOT_RIGHT: r.candidate_command = codes->pivot_right; break;
    default: r.blockers |= VMA_BLOCK_BAD_OUTPUT; break;
    }
    if (!inputs->safety_read_ok) r.blockers |= VMA_BLOCK_SAFETY_READ;
    if (!inputs->imu_healthy) r.blockers |= VMA_BLOCK_IMU;
    if (inputs->safety_request_active) r.blockers |= VMA_BLOCK_SAFETY_REQ;
    if (inputs->estop_latched) r.blockers |= VMA_BLOCK_ESTOP;
    if (!inputs->obstacle_source_verified) r.blockers |= VMA_BLOCK_B5_MISSING;
    if (!inputs->live_estop_verified) r.blockers |= VMA_BLOCK_STOP_UNVERIFIED;
    if (!inputs->motor_handoff_approved) r.blockers |= VMA_BLOCK_HANDOFF_PENDING;

    /* Model readiness is not motor authorization. This module never posts. */
    r.model_ready = (uint8_t)(r.blockers == 0u && output != VC_OUTPUT_HALT);
    return r;
}
