#include "b4_input_bridge.h"
/* Matches the existing APP_CMD type and S5 public event constants.
 * Keep these numbers in sync with app_data.h / subsystem5.h. */
#define B4_CMD_ABORT  1u
#define B4_CMD_PAUSE  2u
#define B4_CMD_RESUME 3u
#define B4_S5_MONITOR 0u
#define B4_S5_CONTINUE 0u

void b4_input_bridge_init(b4_input_bridge_t *b)
{ if (b) *b = (b4_input_bridge_t){0}; }

void b4_input_bridge_remote(b4_input_bridge_t *b, unsigned command_type)
{
    if (!b) return;
    if (command_type == B4_CMD_ABORT) {
        b->remote_abort_latched = 1u;
        b->remote_count++;
    } else if (command_type == B4_CMD_PAUSE) {
        b->paused = 1u;
        b->remote_count++;
    } else if (command_type == B4_CMD_RESUME) {
        /* Resume never releases a terminal/abort/obstacle/safety latch. */
        if (!b->remote_abort_latched && !b->pickup_terminal) b->paused = 0u;
        b->remote_count++;
    }
}

void b4_input_bridge_obstacle(b4_input_bridge_t *b, unsigned state,
                              unsigned detected, unsigned action)
{
    if (!b) return;
    b->obstacle_seen = 1u;
    b->obstacle_count++;
    b->obstacle_action = (uint8_t)action; /* recommendation only */
    if (detected || action != B4_S5_CONTINUE) {
        b->obstacle_hold = 1u;
        b->obstacle_clear_evidence = 0u;
    } else if (state == B4_S5_MONITOR && action == B4_S5_CONTINUE) {
        b->obstacle_clear_evidence = 1u;
    }
}

void b4_input_bridge_obstacle_flag(b4_input_bridge_t *b, int stop, int sensor_fault)
{
    if (!b) return;
    b->obstacle_sensor_fault = !!sensor_fault;
    if (sensor_fault) b->obstacle_clear_evidence = 0u;
    if (stop) {
        b->obstacle_hold = 1u;
        b->obstacle_clear_evidence = 0u;
    }
}

void b4_input_bridge_refresh(b4_input_bridge_t *b, int fresh_line_visible)
{
    if (!b) return;
    /* A decision event alone cannot release a STOP. The obstacle task must
     * report MONITORING/CLEAR, the sensor must be healthy, and the line
     * must be freshly visible. This is only a SOFTWARE hold release. */
    if (b->obstacle_hold && b->obstacle_clear_evidence &&
        fresh_line_visible && !b->obstacle_sensor_fault)
        b->obstacle_hold = 0u;
}

void b4_input_bridge_pickup(b4_input_bridge_t *b)
{ if (b) b->pickup_terminal = 1u; }

int b4_input_bridge_halt(const b4_input_bridge_t *b)
{
    return !b || b->remote_abort_latched || b->pickup_terminal ||
           b->obstacle_hold || b->obstacle_sensor_fault;
}
