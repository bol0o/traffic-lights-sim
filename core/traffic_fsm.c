#include <string.h>
#include <stdio.h>
#include "traffic_fsm.h"

// Assuming right-hand traffic and: N=0, E=1, S=2, W=3
static bool is_left_turn(Direction start, Direction end) {
    // 0 -> 1 (N->E): +1 (Left turn)
    // 1 -> 2 (E->S): +1 (Left turn)
    // and so on
    int diff = (end - start + 4) % 4;
    return diff == 1; 
}

void traffic_init(TrafficSystem* sys) {
    if (!sys) return;

    memset(sys, 0, sizeof(TrafficSystem));

    for (int road = 0; road < ROAD_COUNT; road++) {
        for (int lane = 0; lane < LANES_PER_ROAD; lane++) {
            queue_init(&sys->queues[road][lane]);
            
            sys->lights[road][lane] = LIGHT_RED;
        }
    }

    sys->current_state = STATE_ALL_RED;
    sys->current_step = 0;
    sys->state_timer = 0;
}

bool traffic_add_vehicle(TrafficSystem* sys, const char* id, 
                         Direction start_node, Direction end_node, 
                         uint32_t arrival_time) {
    if (!sys) return false;

    // TODO: Handle U-Turn
    if (start_node == end_node) return false;

    uint8_t target_lane;
    
    if (is_left_turn(start_node, end_node)) {
        target_lane = LANE_LEFT;
    } else {
        target_lane = LANE_STRAIGHT_RIGHT;
    }

    VehicleQueue* q = &sys->queues[start_node][target_lane];
    
    return queue_enqueue(q, id, start_node, end_node, arrival_time);
}

uint16_t traffic_get_queue_size(const TrafficSystem* sys, Direction road, uint8_t lane_idx) {
    if (!sys || lane_idx >= LANES_PER_ROAD) return 0;
    return queue_count(&sys->queues[road][lane_idx]);
}