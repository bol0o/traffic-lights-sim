/**
 * @file traffic_fsm.c
 * @brief Implementation of the Finite State Machine for intersection control.
 * 
 * 11.02.26 Paweł Bolek
 */

#include <string.h>
#include "traffic_fsm.h"

// --- INTERNAL DATA STRUCTURES ---

typedef struct {
    TrafficState state;
    Direction road1;
    Direction road2;
    uint8_t lane;
    LightColor color;
} StateLight;

typedef struct {
    TrafficState current;
    TrafficState next;
    uint8_t timing_idx;
} StateTransition;

/**
 * @brief Table mapping specific FSM states to physical light colors.
 */
static const StateLight STATE_LIGHTS[] = {
    {STATE_NS_RED_YELLOW, NORTH, SOUTH, LANE_STRAIGHT_RIGHT, LIGHT_RED_YELLOW},
    {STATE_NS_STRAIGHT, NORTH, SOUTH, LANE_STRAIGHT_RIGHT, LIGHT_GREEN},
    {STATE_NS_STRAIGHT_YELLOW, NORTH, SOUTH, LANE_STRAIGHT_RIGHT, LIGHT_YELLOW},

    {STATE_NS_LEFT_RED_YELLOW, NORTH, SOUTH, LANE_LEFT, LIGHT_RED_YELLOW},
    {STATE_NS_LEFT, NORTH, SOUTH, LANE_LEFT, LIGHT_GREEN},
    {STATE_NS_LEFT, EAST, WEST, LANE_STRAIGHT_RIGHT, LIGHT_RIGHT_ARROW_GREEN},
    {STATE_NS_LEFT_YELLOW, NORTH, SOUTH, LANE_LEFT, LIGHT_YELLOW},

    {STATE_EW_RED_YELLOW, EAST, WEST, LANE_STRAIGHT_RIGHT, LIGHT_RED_YELLOW},
    {STATE_EW_STRAIGHT, EAST, WEST, LANE_STRAIGHT_RIGHT, LIGHT_GREEN},
    {STATE_EW_STRAIGHT_YELLOW, EAST, WEST, LANE_STRAIGHT_RIGHT, LIGHT_YELLOW},

    {STATE_EW_LEFT_RED_YELLOW, EAST, WEST, LANE_LEFT, LIGHT_RED_YELLOW},
    {STATE_EW_LEFT, EAST, WEST, LANE_LEFT, LIGHT_GREEN},
    {STATE_EW_LEFT, NORTH, SOUTH, LANE_STRAIGHT_RIGHT, LIGHT_RIGHT_ARROW_GREEN},
    {STATE_EW_LEFT_YELLOW, EAST, WEST, LANE_LEFT, LIGHT_YELLOW},
};

/**
 * @brief Table defining deterministic state transitions and their required durations.
 */
static const StateTransition STATE_TRANSITIONS[] = {
    [STATE_ALL_RED]            = {STATE_ALL_RED, STATE_NS_RED_YELLOW, 3},

    [STATE_NS_RED_YELLOW]      = {STATE_NS_RED_YELLOW, STATE_NS_STRAIGHT, 4},
    [STATE_NS_STRAIGHT]        = {STATE_NS_STRAIGHT, STATE_NS_STRAIGHT_YELLOW, 0},
    [STATE_NS_STRAIGHT_YELLOW] = {STATE_NS_STRAIGHT_YELLOW, STATE_NS_LEFT_RED_YELLOW, 2},
    
    [STATE_NS_LEFT_RED_YELLOW] = {STATE_NS_LEFT_RED_YELLOW, STATE_NS_LEFT, 4},
    [STATE_NS_LEFT]            = {STATE_NS_LEFT, STATE_NS_LEFT_YELLOW, 1},
    [STATE_NS_LEFT_YELLOW]     = {STATE_NS_LEFT_YELLOW, STATE_EW_RED_YELLOW, 2},
    
    [STATE_EW_RED_YELLOW]      = {STATE_EW_RED_YELLOW, STATE_EW_STRAIGHT, 4},
    [STATE_EW_STRAIGHT]        = {STATE_EW_STRAIGHT, STATE_EW_STRAIGHT_YELLOW, 0},
    [STATE_EW_STRAIGHT_YELLOW] = {STATE_EW_STRAIGHT_YELLOW, STATE_EW_LEFT_RED_YELLOW, 2},
    
    [STATE_EW_LEFT_RED_YELLOW] = {STATE_EW_LEFT_RED_YELLOW, STATE_EW_LEFT, 4},
    [STATE_EW_LEFT]            = {STATE_EW_LEFT, STATE_EW_LEFT_YELLOW, 1},
    [STATE_EW_LEFT_YELLOW]     = {STATE_EW_LEFT_YELLOW, STATE_NS_RED_YELLOW, 2},
};

// --- HELPER FUNCTIONS (static, some inline for speed) ---

/**
 * @brief Retrieves the target duration (in steps) for a given timing index.
 */
static inline uint32_t get_timing_value(const TrafficSystem* sys, uint8_t idx) {
    switch(idx) {
        case 0: { return sys->timing.green_st; }
        case 1: { return sys->timing.green_lt; }
        case 2: { return sys->timing.yellow; }
        case 3: { return sys->timing.all_red; }
        case 4: { return sys->timing.red_yellow; }
        
        default: { return 0; }
    }
}

/**
 * @brief Determines mathematically if a route constitutes a left turn.
 */
static inline bool is_left_turn(Direction start, Direction end) {
    return ((end - start + DIRECTION_MOD) % DIRECTION_MOD) == LEFT_TURN_DIFF;
}

/**
 * @brief Resolves which lane queue a vehicle should join based on its destination.
 */
static inline uint8_t get_lane_for_turn(Direction start, Direction end) {
    return is_left_turn(start, end) ? LANE_LEFT : LANE_STRAIGHT_RIGHT;
}

static inline bool is_green_phase(TrafficState state) {
    return (state == STATE_NS_STRAIGHT || state == STATE_NS_LEFT ||
            state == STATE_EW_STRAIGHT || state == STATE_EW_LEFT);
}

static inline bool is_yellow_phase(TrafficState state) {
    return (state == STATE_NS_STRAIGHT_YELLOW || state == STATE_NS_LEFT_YELLOW ||
            state == STATE_EW_STRAIGHT_YELLOW || state == STATE_EW_LEFT_YELLOW);
}

/**
 * @brief Maps a green phase state to an array index (0-3).
 * Useful for accessing phase_skip_counters array.
 */
static int get_phase_idx(TrafficState state) {
    if (state == STATE_NS_STRAIGHT) { return 0; }
    if (state == STATE_NS_LEFT) { return 1; }
    if (state == STATE_EW_STRAIGHT) { return 2; }
    if (state == STATE_EW_LEFT) { return 3; }

    return -1;
}

/**
 * @brief Returns the corresponding RED_YELLOW preparation state for a given green phase.
 */
static TrafficState get_preparation_state(TrafficState green_phase) {
    switch(green_phase) {
        case STATE_NS_STRAIGHT: { return STATE_NS_RED_YELLOW; }
        case STATE_NS_LEFT: { return STATE_NS_LEFT_RED_YELLOW; }
        case STATE_EW_STRAIGHT: {  return STATE_EW_RED_YELLOW; }
        case STATE_EW_LEFT: { return STATE_EW_LEFT_RED_YELLOW; }
        default: { return STATE_ALL_RED; }
    }
}

/**
 * @brief Returns the next sequential green phase in the standard cycle.
 */
static TrafficState get_next_green_phase(TrafficState green_phase) {
    switch(green_phase) {
        case STATE_NS_STRAIGHT: { return STATE_NS_LEFT; }
        case STATE_NS_LEFT: { return STATE_EW_STRAIGHT; }
        case STATE_EW_STRAIGHT: { return STATE_EW_LEFT; }
        case STATE_EW_LEFT: { return STATE_NS_STRAIGHT; }
        default: { return STATE_NS_STRAIGHT; }
    }
}

/**
 * @brief Returns the next sequential phase in the standard cycle.
 */
static TrafficState get_phase_after_yellow(TrafficState yellow_phase) {
    switch(yellow_phase) {
        case STATE_NS_STRAIGHT_YELLOW: { return STATE_NS_LEFT; }
        case STATE_NS_LEFT_YELLOW: { return STATE_EW_STRAIGHT; }
        case STATE_EW_STRAIGHT_YELLOW: { return STATE_EW_LEFT; }
        case STATE_EW_LEFT_YELLOW: { return STATE_NS_STRAIGHT; }
        default: { return STATE_NS_STRAIGHT; }
    }
}

/**
 * @brief Evaluates if both opposing lanes for a given phase are currently empty.
 */
static bool is_phase_empty(const TrafficSystem* sys, TrafficState state) {
    switch (state) {
        case STATE_NS_STRAIGHT:
            { return queue_is_empty(&sys->queues[NORTH][LANE_STRAIGHT_RIGHT]) && 
                   queue_is_empty(&sys->queues[SOUTH][LANE_STRAIGHT_RIGHT]); }
        case STATE_NS_LEFT:
            { return queue_is_empty(&sys->queues[NORTH][LANE_LEFT]) && 
                   queue_is_empty(&sys->queues[SOUTH][LANE_LEFT]); }
        case STATE_EW_STRAIGHT:
            { return queue_is_empty(&sys->queues[EAST][LANE_STRAIGHT_RIGHT]) && 
                   queue_is_empty(&sys->queues[WEST][LANE_STRAIGHT_RIGHT]); }
        case STATE_EW_LEFT:
            { return queue_is_empty(&sys->queues[EAST][LANE_LEFT]) && 
                   queue_is_empty(&sys->queues[WEST][LANE_LEFT]); }
        default: { return false; }
    }
}

// --- CORE FSM LOGIC ---

/**
 * @brief Computes the next state of the intersection based on current timers and queue loads.
 * 
 * @details Implements Phase Skipping logic. If a target phase is empty, it skips to the 
 * next one, unless the starvation limit (skip_limit) has been reached.
 */
static TrafficState get_next_state(TrafficSystem* sys) {
    if (sys->current_state >= ARRAY_SIZE(STATE_TRANSITIONS)) {
        return STATE_ALL_RED; 
    }

    const StateTransition* transition = &STATE_TRANSITIONS[sys->current_state];

    // Wait until the timer for the current state expires
    if (sys->state_timer < get_timing_value(sys, transition->timing_idx)) {
        return sys->current_state;
    }

    // RULE 1: Static transitions (Green -> Yellow, Red/Yellow -> Green)
    if (!is_yellow_phase(sys->current_state) && sys->current_state != STATE_ALL_RED) {
        return transition->next;
    }

    // RULE 2: Phase selection (End of Yellow, or waking up from All-Red)
    TrafficState candidate_green = get_phase_after_yellow(sys->current_state);

    // Scan ahead up to 4 phases to skip empty queues
    for (uint8_t checked = 0; checked < 4; checked++) {
        int phase_idx = get_phase_idx(candidate_green);
        
        // Failsafe against invalid phase lookup
        if (phase_idx < 0) {
            return STATE_ALL_RED; 
        }
        
        // If phase has vehicles OR starvation limit is reached -> Execute this phase
        if (!is_phase_empty(sys, candidate_green) || 
            sys->phase_skip_counters[phase_idx] >= sys->timing.skip_limit) {
            
            sys->phase_skip_counters[phase_idx] = 0; // Reset starvation counter
            return get_preparation_state(candidate_green);
        }
        
        // Phase is empty -> increment starvation counter and test the next one
        sys->phase_skip_counters[phase_idx]++;
        candidate_green = get_next_green_phase(candidate_green);
    }
    
    // Intersection is completely empty - retreat to ALL_RED
    return STATE_ALL_RED;
}

/**
 * @brief Translates the state into physical light signals for all lanes.
 */
static void set_lights_for_state(TrafficSystem* sys) {
    // First set all lights to red
    memset(sys->lights, LIGHT_RED, sizeof(sys->lights));
    
    // Apply specific lights from state table
    for (uint8_t i = 0; i < ARRAY_SIZE(STATE_LIGHTS); i++) {
        if (sys->current_state == STATE_LIGHTS[i].state) {
            sys->lights[STATE_LIGHTS[i].road1][STATE_LIGHTS[i].lane] = STATE_LIGHTS[i].color;
            sys->lights[STATE_LIGHTS[i].road2][STATE_LIGHTS[i].lane] = STATE_LIGHTS[i].color;
        }
    }
}

/**
 * @brief Iterates through all queues and dequeues vehicles that have a green light
 * 
 * @details Implements logic for permissive right turns
 */
static uint8_t process_discharges(TrafficSystem* sys, char out_ids[][VEHICLE_ID_LEN]) {
    uint8_t discharged = 0;
    
    for (uint8_t road = 0; road < ROAD_COUNT; road++) {
        for (uint8_t lane = 0; lane < LANES_PER_ROAD; lane++) {
            LightColor color = sys->lights[road][lane];
            VehicleQueue* q = &sys->queues[road][lane];
            
            if (queue_is_empty(q)) continue;

            // Only green lights allow vehicles to pass
            if (color == LIGHT_GREEN || color == LIGHT_RIGHT_ARROW_GREEN) {
                
                // For green arrow, only let right-turning vehicles through
                if (color == LIGHT_RIGHT_ARROW_GREEN) {
                    Vehicle v;
                    if (queue_peek(q, &v)) {
                        Direction right_target = (road + 3) % DIRECTION_MOD;
                        if (v.end_road != right_target) {
                            continue; // Not turning right - stays in queue
                        }
                    }
                }
                
                // Dequeue the vehicle and record its ID
                uint32_t wait_time;
                queue_dequeue(q, out_ids[discharged++], sys->current_step, &wait_time);
            }
        }
    }
    
    return discharged;
}

/**
 * @brief Determines if the current green phase should be extended based on queue length
 */
static bool should_extend_current_phase(const TrafficSystem* sys) {
    if (!is_green_phase(sys->current_state)) return false;
    
    // Check all lanes that currently have green light
    for (uint8_t road = 0; road < ROAD_COUNT; road++) {
        for (uint8_t lane = 0; lane < LANES_PER_ROAD; lane++) {
            if (!(sys->lights[road][lane] == LIGHT_GREEN)) {
                continue;
            }

            if (queue_count(&sys->queues[road][lane]) >= sys->timing.ext_threshold) {
                return true;
            }
            
        }
    }

    return false;
}

// --- PUBLIC API IMPLEMENTATION ---

void traffic_init(TrafficSystem* sys, TimingConfig config) {
    if (!sys) return;
    
    memset(sys, 0, sizeof(TrafficSystem));
    sys->timing = config;
    
    for (uint8_t road = 0; road < ROAD_COUNT; road++) {
        for (uint8_t lane = 0; lane < LANES_PER_ROAD; lane++) {
            queue_init(&sys->queues[road][lane]);
        }
    }
    
    sys->current_state = STATE_ALL_RED;
    sys->state_timer = 0;
    set_lights_for_state(sys);
}

bool traffic_add_vehicle(TrafficSystem* sys, const char* id, Direction start, Direction end, uint32_t arrival_time) {
    if (!sys || start == end || 
        start >= ROAD_COUNT || end >= ROAD_COUNT) {
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
    
    // Green Extension Logic
    if (next_state != sys->current_state && is_green_phase(sys->current_state)) {
        if (should_extend_current_phase(sys) && sys->extension_timer < sys->timing.max_ext) {
            sys->extension_timer++;
            next_state = sys->current_state; // Stay in current green phase
        }
    }
    
    // Perform state transition if needed
    if (next_state != sys->current_state) {
        sys->current_state = next_state;
        sys->state_timer = 0;
        sys->extension_timer = 0;
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