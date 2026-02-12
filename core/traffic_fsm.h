#ifndef TRAFFIC_FSM_H
#define TRAFFIC_FSM_H

#include <stdint.h>
#include <stdbool.h>
#include "traffic_queue.h"

#define ROAD_COUNT 4
#define LANES_PER_ROAD 2

#define LANE_STRAIGHT_RIGHT 0
#define LANE_LEFT 1

#define DIRECTION_MOD 4
#define LEFT_TURN_DIFF 1

#define DEFAULT_TIMING {10, 5, 2, 3}

typedef struct {
    uint32_t green_st;
    uint32_t green_lt;
    uint32_t yellow;
    uint32_t all_red;
} TimingConfig;

typedef enum {
    STATE_ALL_RED = 0,

    STATE_NS_STRAIGHT,
    STATE_NS_STRAIGHT_YELLOW,

    STATE_NS_LEFT,
    STATE_NS_LEFT_YELLOW,

    STATE_EW_STRAIGHT,
    STATE_EW_STRAIGHT_YELLOW,

    STATE_EW_LEFT,
    STATE_EW_LEFT_YELLOW,
} TrafficState;

typedef enum {
    LIGHT_RED = 0,
    LIGHT_YELLOW,
    LIGHT_GREEN,
    LIGHT_LEFT_ARROW_GREEN,
    LIGHT_RIGHT_ARROW_GREEN
} LightColor;

typedef struct {
    TrafficState state;
    Direction road1;
    Direction road2;
    uint8_t lane;
    LightColor color;
    LightColor alt_color;
} StateLights;

typedef struct {
    TrafficState current;
    TrafficState next;
    uint8_t timing_idx;
} StateTransition;

typedef enum {
    TIMING_GREEN_ST = 0,
    TIMING_GREEN_LT = 1,
    TIMING_YELLOW = 2,
    TIMING_ALL_RED = 3,
    TIMING_COUNT
} TimingIndex;

typedef struct {
    TrafficState current_state;
    uint32_t current_step; // Global simulation timer
    uint32_t state_timer; // Current time spent in state

    TimingConfig timing;

    // Structure: queues[ARRIVAL_DIRECTION][LANE]
    // Direction: 0=N, 1=E, 2=S, 3=W
    // Pas: 0=Straight/Right, 1=Left
    VehicleQueue queues[ROAD_COUNT][LANES_PER_ROAD];

    LightColor lights[ROAD_COUNT][LANES_PER_ROAD];
} TrafficSystem;

void traffic_init(TrafficSystem* sys, TimingConfig config);
bool traffic_add_vehicle(TrafficSystem* sys, const char* id, 
                        Direction start_node, Direction end_node, 
                        uint32_t arrival_time);
uint8_t traffic_fsm_step(TrafficSystem* sys, char out_ids[][VEHICLE_ID_LEN]);

uint16_t traffic_get_queue_size(const TrafficSystem* sys, Direction road, uint8_t lane_idx);

#endif