#include <string.h>
#include "traffic_fsm.h"

static const StateLights STATE_LIGHTS[] = {
    {STATE_ALL_RED,            NORTH, SOUTH, LANE_STRAIGHT_RIGHT, LIGHT_RED, LIGHT_RED},
    {STATE_NS_STRAIGHT,        NORTH, SOUTH, LANE_STRAIGHT_RIGHT, LIGHT_GREEN,  LIGHT_RED},
    {STATE_NS_STRAIGHT_YELLOW, NORTH, SOUTH, LANE_STRAIGHT_RIGHT, LIGHT_YELLOW, LIGHT_RED},
    {STATE_NS_LEFT,           NORTH, SOUTH, LANE_LEFT,           LIGHT_GREEN,  LIGHT_RED},
    {STATE_NS_LEFT_YELLOW,    NORTH, SOUTH, LANE_LEFT,           LIGHT_YELLOW, LIGHT_RED},
    {STATE_EW_STRAIGHT,       EAST,  WEST,  LANE_STRAIGHT_RIGHT, LIGHT_GREEN,  LIGHT_RED},
    {STATE_EW_STRAIGHT_YELLOW, EAST,  WEST,  LANE_STRAIGHT_RIGHT, LIGHT_YELLOW, LIGHT_RED},
    {STATE_EW_LEFT,           EAST,  WEST,  LANE_LEFT,           LIGHT_GREEN,  LIGHT_RED},
    {STATE_EW_LEFT_YELLOW,    EAST,  WEST,  LANE_LEFT,           LIGHT_YELLOW, LIGHT_RED},
};

static const StateTransition STATE_TRANSITIONS[] = {
    {STATE_ALL_RED,              STATE_NS_STRAIGHT,        TIMING_ALL_RED},
    {STATE_NS_STRAIGHT,          STATE_NS_STRAIGHT_YELLOW, TIMING_GREEN_ST},
    {STATE_NS_STRAIGHT_YELLOW,   STATE_NS_LEFT,           TIMING_YELLOW},
    {STATE_NS_LEFT,             STATE_NS_LEFT_YELLOW,     TIMING_GREEN_LT},
    {STATE_NS_LEFT_YELLOW,      STATE_EW_STRAIGHT,        TIMING_YELLOW},
    {STATE_EW_STRAIGHT,         STATE_EW_STRAIGHT_YELLOW, TIMING_GREEN_ST},
    {STATE_EW_STRAIGHT_YELLOW,  STATE_EW_LEFT,           TIMING_YELLOW},
    {STATE_EW_LEFT,            STATE_EW_LEFT_YELLOW,     TIMING_GREEN_LT},
    {STATE_EW_LEFT_YELLOW,     STATE_ALL_RED,            TIMING_YELLOW},
};

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

static inline bool is_left_turn(Direction start, Direction end) {
    return ((end - start + DIRECTION_MOD) % DIRECTION_MOD) == LEFT_TURN_DIFF;
}

static inline uint8_t get_lane_for_turn(Direction start, Direction end) {
    return is_left_turn(start, end) ? LANE_LEFT : LANE_STRAIGHT_RIGHT;
}

static void set_lights_for_state(TrafficSystem* sys) {
    memset(sys->lights, LIGHT_RED, sizeof(sys->lights));
    
    for (uint_fast8_t i = 0; i < ARRAY_SIZE(STATE_LIGHTS); i++) {
        if (sys->current_state == STATE_LIGHTS[i].state) {
            sys->lights[STATE_LIGHTS[i].road1][STATE_LIGHTS[i].lane] = STATE_LIGHTS[i].color;
            sys->lights[STATE_LIGHTS[i].road2][STATE_LIGHTS[i].lane] = STATE_LIGHTS[i].color;
            return;
        }
    }
}

static uint8_t process_discharges(TrafficSystem* sys, char out_ids[][VEHICLE_ID_LEN]) {
    uint8_t discharged = 0;
    
    for (uint_fast8_t road = 0; road < ROAD_COUNT; road++) {
        for (uint_fast8_t lane = 0; lane < LANES_PER_ROAD; lane++) {
            if (sys->lights[road][lane] != LIGHT_GREEN) {
                continue;
            }
            
            VehicleQueue* q = &sys->queues[road][lane];
            if (!queue_is_empty(q)) {
                uint32_t wait;
                queue_dequeue(q, out_ids[discharged], sys->current_step, &wait);
                discharged++;
            }
        }
    }
    
    return discharged;
}

static TrafficState get_next_state(const TrafficSystem* sys) {
    const uint32_t* timings = (const uint32_t*)&sys->timing;
    
    for (uint_fast8_t i = 0; i < ARRAY_SIZE(STATE_TRANSITIONS); i++) {
        uint32_t timeout = timings[STATE_TRANSITIONS[i].timing_idx];
        
        if (sys->current_state == STATE_TRANSITIONS[i].current &&
            sys->state_timer >= timeout) {
            return STATE_TRANSITIONS[i].next;
        }
    }
    return sys->current_state;
}

void traffic_init(TrafficSystem* sys, TimingConfig config) {
    if (!sys) return;
    
    memset(sys, 0, sizeof(TrafficSystem));
    sys->timing = config;
    
    for (uint_fast8_t road = 0; road < ROAD_COUNT; road++) {
        for (uint_fast8_t lane = 0; lane < LANES_PER_ROAD; lane++) {
            queue_init(&sys->queues[road][lane]);
            sys->lights[road][lane] = LIGHT_RED;
        }
    }
    
    sys->current_state = STATE_ALL_RED;
    sys->state_timer = 0;
}

bool traffic_add_vehicle(TrafficSystem* sys, const char* id, 
                         Direction start, Direction end, 
                         uint32_t arrival_time) {
    if (!sys || start == end || start >= ROAD_COUNT || end >= ROAD_COUNT) {
        return false;
    }
    
    uint8_t lane = get_lane_for_turn(start, end);
    return queue_enqueue(&sys->queues[start][lane], id, start, end, arrival_time);
}

uint8_t traffic_fsm_step(TrafficSystem* sys, char out_ids[][VEHICLE_ID_LEN]) {
    if (!sys || !out_ids) return 0;
    
    sys->current_step++;
    sys->state_timer++;
    
    TrafficState next_state = get_next_state(sys);
    
    if (next_state != sys->current_state) {
        sys->current_state = next_state;
        sys->state_timer = 0;
    }
    
    set_lights_for_state(sys);
    return process_discharges(sys, out_ids);
}

uint16_t traffic_get_queue_size(const TrafficSystem* sys, Direction road, uint8_t lane) {
    if (!sys || road >= ROAD_COUNT || lane >= LANES_PER_ROAD) {
        return 0;
    }
    
    return queue_count(&sys->queues[road][lane]);
}