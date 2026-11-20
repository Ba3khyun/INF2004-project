#ifndef B4_INPUT_BRIDGE_H
#define B4_INPUT_BRIDGE_H
#include <stdint.h>
/* Software-only bridge state; no kernel calls and no motor/flag writes. */
typedef struct {
    uint8_t paused;
    uint8_t remote_abort_latched;
    uint8_t obstacle_hold;
    uint8_t obstacle_sensor_fault;
    uint8_t obstacle_clear_evidence;
    uint8_t obstacle_action;
    uint8_t pickup_terminal;
    uint8_t obstacle_seen;
    uint32_t obstacle_count;
    uint32_t remote_count;
} b4_input_bridge_t;
void b4_input_bridge_init(b4_input_bridge_t *b);
void b4_input_bridge_remote(b4_input_bridge_t *b, unsigned command_type);
void b4_input_bridge_obstacle(b4_input_bridge_t *b, unsigned state,
                              unsigned detected, unsigned action);
void b4_input_bridge_obstacle_flag(b4_input_bridge_t *b, int stop, int sensor_fault);
void b4_input_bridge_refresh(b4_input_bridge_t *b, int fresh_line_visible);
void b4_input_bridge_pickup(b4_input_bridge_t *b);
int b4_input_bridge_halt(const b4_input_bridge_t *b);
#endif
