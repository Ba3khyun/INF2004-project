/* Buddy 4 Phase 5A: PURE command-preview adapter (no kernel/motor dependencies).
 * A preview is NEVER delivered to Buddy 2. This file has no motion_cmd_post().
 */
#ifndef VEHICLE_CONTROLLER_MOTION_ADAPTER_H
#define VEHICLE_CONTROLLER_MOTION_ADAPTER_H
#include <stdint.h>
#include "vehicle_controller_core.h"

typedef struct {
    uint8_t forward;
    uint8_t steer_left;
    uint8_t steer_right;
    uint8_t pivot_left;
    uint8_t pivot_right;
    uint8_t halt;
} vma_codes_t;

typedef struct {
    int safety_read_ok;
    int imu_healthy;
    int safety_request_active;
    int estop_latched;
    int obstacle_source_verified;
    int live_estop_verified;
    int motor_handoff_approved;
} vma_inputs_t;

#define VMA_BLOCK_SAFETY_READ      (1u << 0)
#define VMA_BLOCK_IMU             (1u << 1)
#define VMA_BLOCK_SAFETY_REQ      (1u << 2)
#define VMA_BLOCK_ESTOP           (1u << 3)
#define VMA_BLOCK_B5_MISSING      (1u << 4)
#define VMA_BLOCK_STOP_UNVERIFIED (1u << 5)
#define VMA_BLOCK_HANDOFF_PENDING (1u << 6)
#define VMA_BLOCK_BAD_OUTPUT      (1u << 7)
#define VMA_BLOCK_BAD_INPUT       (1u << 8)

typedef struct {
    uint8_t candidate_command; /* model suggestion, NOT a motor command */
    uint8_t safe_command;      /* always HALT; never posted */
    uint32_t blockers;
    uint8_t model_ready;       /* hypothetical readiness only */
} vma_result_t;

vma_result_t vma_preview(vc_output_t output,
                         const vma_inputs_t *inputs,
                         const vma_codes_t *codes);
#endif
