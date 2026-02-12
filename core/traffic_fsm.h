#ifndef TRAFFIC_FSM_H
#define TRAFFIC_FSM_H

#include <stdint.h>
#include <stdbool.h>
#include "traffic_queue.h"

#define ROAD_COUNT 4
#define LANES_PER_ROAD 2

#define LANE_STRAIGHT_RIGHT 0
#define LANE_LEFT 1

typedef enum {
    STATE_INIT = 0,
    STATE_ALL_RED,
    STATE_NS_STRAIGHT,
    // TODO: Add rest of states
} TrafficState;

typedef enum {
    LIGHT_RED = 0,
    LIGHT_YELLOW,
    LIGHT_GREEN,
    LIGHT_LEFT_ARROW_GREEN,
    LIGHT_RIGHT_ARROW_GREEN
} LightColor;

typedef struct {
    TrafficState current_state;
    uint32_t current_step; // Global simulation timer
    uint32_t state_timer; // Current time spent in state

    // Structure: queues[ARRIVAL_DIRECTION][LANE]
    // Direction: 0=N, 1=E, 2=S, 3=W
    // Pas: 0=Straight/Right, 1=Left
    VehicleQueue queues[ROAD_COUNT][LANES_PER_ROAD];

    LightColor lights[ROAD_COUNT][LANES_PER_ROAD];

} TrafficSystem;

void traffic_init(TrafficSystem* sys);
bool traffic_add_vehicle(TrafficSystem* sys, const char* id, 
                        Direction start_node, Direction end_node, 
                        uint32_t arrival_time);
uint16_t traffic_get_queue_size(const TrafficSystem* sys, Direction road, uint8_t lane_idx);

#endif