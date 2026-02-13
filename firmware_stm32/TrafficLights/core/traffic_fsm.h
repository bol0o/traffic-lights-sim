/**
 * @file traffic_fsm.h
 * @brief FSM for intersection control.
 * @details This module implements a Finite State Machine
 * to control traffic lights and manage vehicle queues. Designed for embedded environments.
 * 
 * 11.02.26, Paweł Bolek
 */

#ifndef TRAFFIC_FSM_H
#define TRAFFIC_FSM_H

#include <stdint.h>
#include <stdbool.h>
#include "traffic_queue.h"

// --- CONSTANTS ---
#define ROAD_COUNT 4
#define LANES_PER_ROAD 2

#define LANE_STRAIGHT_RIGHT 0
#define LANE_LEFT 1

#define DIRECTION_MOD 4   // Must match ROAD_COUNT
#define LEFT_TURN_DIFF 1   // (start + 1) % 4 = left turn

#define VEHICLE_ID_LEN 32  // Max length for vehicle ID strings

// Default optimal timings found via python
// {green_st, green_lt, yellow, all_red, ext_threshold, max_ext, skip_limit}
#define DEFAULT_TIMING {4, 3, 2, 3, 1, 1, 15, 2}
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

// --- DATA TYPES ---

/**
 * @brief Traffic system configuration parameters.
 */
typedef struct {
    uint32_t green_st; // Base green time for straight/right
    uint32_t green_lt; // Base green time for left turns
    uint32_t yellow; // Yellow light duration
    uint32_t all_red; // All-red clearance interval
    uint32_t red_yellow; // Transition state
    uint32_t ext_threshold; // Queue length to trigger green extension
    uint32_t max_ext; // Maximum extra green time (steps)
    uint32_t skip_limit; // Max times a phase can be skipped if empty
} TimingConfig;

/**
 * @brief Enumeration of all possible FSM states.
 * 
 * @details The states sequence through 4 main phases:
 * NS-Straight -> NS-Left -> EW-Straight -> EW-Left
 * Each main phase has a preparation state (RED_YELLOW) and a closing state (YELLOW)
 */
typedef enum {
    STATE_ALL_RED = 0,

    STATE_NS_RED_YELLOW,
    STATE_NS_STRAIGHT,
    STATE_NS_STRAIGHT_YELLOW,

    STATE_NS_LEFT_RED_YELLOW,
    STATE_NS_LEFT,
    STATE_NS_LEFT_YELLOW,
    
    STATE_EW_RED_YELLOW,
    STATE_EW_STRAIGHT,
    STATE_EW_STRAIGHT_YELLOW,

    STATE_EW_LEFT_RED_YELLOW,
    STATE_EW_LEFT,
    STATE_EW_LEFT_YELLOW,
} TrafficState;

/**
 * @brief Physical state of a single traffic light
 */
typedef enum {
    LIGHT_RED = 0,
    LIGHT_YELLOW,
    LIGHT_GREEN,
    LIGHT_RED_YELLOW,
    LIGHT_RIGHT_ARROW_GREEN
} LightColor;

// --- FSM SYSTEM STRUCTURE ---

typedef struct {
    TrafficState current_state;
    uint32_t current_step; // Global simulation timer
    uint32_t state_timer; // Time spent in current state
    
    TimingConfig timing;
    
    /** Matrix of vehicle queues: queues[ARRIVAL_DIRECTION][LANE] */
    VehicleQueue queues[ROAD_COUNT][LANES_PER_ROAD];

    /** Matrix of light states matching the physical layout */
    LightColor lights[ROAD_COUNT][LANES_PER_ROAD];
    
    /** Starvation prevention counters for each of the 4 main phases */
    uint8_t phase_skip_counters[ROAD_COUNT];
    
    /** Current accumulated extra green steps (resets on phase change) */
    uint32_t extension_timer;
} TrafficSystem;

// --- PUBLIC API ---

/**
 * @brief Initializes the traffic state machine with given timings.
 * 
 * @param sys Pointer to TrafficSystem structure
 * @param config Timing configuration
 */
void traffic_init(TrafficSystem* sys, TimingConfig config);

/**
 * @brief Attempts to add a vehicle to the appropriate lane queue.
 * 
 * @param sys Pointer to TrafficSystem
 * @param id Vehicle identifier string
 * @param start Road where vehicle appears
 * @param end Destination road
 * @param arrival_time Simulation step when vehicle arrived
 * 
 * @return true if added successfully, false on error
 */
bool traffic_add_vehicle(TrafficSystem* sys, const char* id, 
                         Direction start, Direction end, 
                         uint32_t arrival_time);

/**
 * @brief Executes one simulation step of the FSM.
 * 
 * @param sys Pointer to TrafficSystem
 * @param out_ids Array to store IDs of vehicles that left in this step
 * 
 * @return Number of vehicles that left the intersection
 */
uint8_t traffic_fsm_step(TrafficSystem* sys, char out_ids[][VEHICLE_ID_LEN]);

/**
 * @brief Returns the current number of vehicles waiting in a specific lane.
 * 
 * @param sys Pointer to TrafficSystem
 * @param road Direction of approach
 * @param lane_idx Lane index (LANE_STRAIGHT_RIGHT or LANE_LEFT)
 * 
 * @return Number of vehicles in queue
 */
uint16_t traffic_get_queue_size(const TrafficSystem* sys, Direction road, uint8_t lane_idx);

#endif // TRAFFIC_FSM_H