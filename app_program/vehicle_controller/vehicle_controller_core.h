#ifndef VEHICLE_CONTROLLER_CORE_H
#define VEHICLE_CONTROLLER_CORE_H

#include <stdint.h>

/* Pure decision engine: no RTOS, hardware, or motor calls. */
/* Team app_data.h defines the same vc_state_t enum.
 * The firmware adapter sets VC_CORE_USE_TEAM_STATE to reuse it. */
#ifndef VC_CORE_USE_TEAM_STATE
typedef enum {
    VC_INITIALIZING = 0,
    VC_DRIVING = 1,
    VC_HUMP_CROSSING = 2,
    VC_OBSTACLE_BYPASS = 3,
    VC_LINE_SEARCH = 4,
    VC_PAUSED = 5,
    VC_ERROR = 6,
    VC_MISSION_COMPLETE = 7
} vc_state_t; /* Matches APP_VS_* in team's app_data.h */
#endif

typedef enum {
    VC_INTENT_NONE = 0,
    VC_INTENT_FORWARD,
    VC_INTENT_STEER_LEFT,
    VC_INTENT_STEER_RIGHT,
    VC_INTENT_PIVOT_LEFT,
    VC_INTENT_PIVOT_RIGHT,
    VC_INTENT_LINE_LOST,
    VC_INTENT_STOP
} vc_line_intent_t;

typedef enum {
    VC_IMU_READY = 0,
    VC_IMU_HUMP_START,
    VC_IMU_HUMP_DONE,
    VC_IMU_SHOCK_MINOR,
    VC_IMU_FAULT
} vc_imu_event_t;

typedef enum {
    VC_OUTPUT_HALT = 0,
    VC_OUTPUT_FORWARD,
    VC_OUTPUT_STEER_LEFT,
    VC_OUTPUT_STEER_RIGHT,
    VC_OUTPUT_PIVOT_LEFT,
    VC_OUTPUT_PIVOT_RIGHT
} vc_output_t;

typedef struct {
    vc_state_t state;
    vc_line_intent_t intent;
    uint32_t last_line_ms;
    uint8_t line_received;
    uint8_t line_visible;
    uint8_t imu_ready;
    uint8_t imu_fault;
    uint8_t hump_active;
    uint8_t paused;
    uint8_t safety_latched;
    uint8_t safety_request;
    uint8_t mission_complete;
    uint8_t last_hump_valid;
    uint8_t last_hump_mm;
} vc_core_t;

#define VC_LINE_TIMEOUT_MS 500u

void vc_core_init(vc_core_t *vc);
void vc_core_line(vc_core_t *vc, vc_line_intent_t intent, uint32_t now_ms);
void vc_core_line_visible(vc_core_t *vc, int line_visible);
void vc_core_imu(vc_core_t *vc, vc_imu_event_t event, uint8_t valid, uint8_t height_mm);
void vc_core_set_safety(vc_core_t *vc, int request_pending, int estop_latched);
void vc_core_pause(vc_core_t *vc);
void vc_core_resume(vc_core_t *vc); /* Does not clear any safety latch. */
void vc_core_complete(vc_core_t *vc);
vc_output_t vc_core_step(vc_core_t *vc, uint32_t now_ms);

#endif
