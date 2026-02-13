#include "test_utils.h"
#include "traffic_fsm.h"
#include <stdio.h>

int tests_run = 0;
int tests_failed = 0;

TrafficSystem create_test_system() {
    TrafficSystem sys;
    TimingConfig test_config = {
        .green_st = 10,
        .green_lt = 5,
        .yellow = 2,
        .all_red = 3,
        .ext_threshold = 2,
        .max_ext = 5,
        .skip_limit = 2
    };
    traffic_init(&sys, test_config);
    return sys;
}

TrafficSystem create_minimal_system() {
    TrafficSystem sys;
    TimingConfig minimal_config = {
        .green_st = 2,
        .green_lt = 2,
        .yellow = 1,
        .all_red = 1,
        .ext_threshold = 1,
        .max_ext = 3,
        .skip_limit = 1
    };
    traffic_init(&sys, minimal_config);
    return sys;
}

void advance_to_state(TrafficSystem* sys, TrafficState target_state, char out_ids[][VEHICLE_ID_LEN]) {
    int max_steps = 1000;
    int steps = 0;
    
    while (sys->current_state != target_state && steps < max_steps) {
        traffic_fsm_step(sys, out_ids);
        steps++;
    }
}

void fill_lane(TrafficSystem* sys, Direction road, uint8_t lane, int count) {
    char buf[16];
    for (int i = 0; i < count; i++) {
        sprintf(buf, "c%d_%d", road, i);
        Direction end = (lane == LANE_LEFT) ? (road + 1) % 4 : (road + 2) % 4;
        traffic_add_vehicle(sys, buf, road, end, 0);
    }
}

void test_fsm_initialization() {
    TrafficSystem sys;
    TimingConfig config = DEFAULT_TIMING;
    traffic_init(&sys, config);
    
    ASSERT_EQ_INT(sys.current_state, STATE_ALL_RED, "Initial state should be ALL_RED");
    ASSERT_EQ_INT(sys.current_step, 0, "Initial step should be 0");
    ASSERT_EQ_INT(sys.state_timer, 0, "State timer should be 0");    
    ASSERT_EQ_INT(sys.timing.green_st, 4, "Default green_st should be 4");
    ASSERT_EQ_INT(sys.timing.green_lt, 3, "Default green_lt should be 3");
    ASSERT_EQ_INT(sys.timing.yellow, 2, "Default yellow should be 2");
    ASSERT_EQ_INT(sys.timing.all_red, 3, "Default all_red should be 3");
    
    uint16_t count = traffic_get_queue_size(&sys, NORTH, LANE_STRAIGHT_RIGHT);
    ASSERT_EQ_INT(count, 0, "Queues should be empty after init");
}

void test_fsm_init_with_custom_config() {
    TrafficSystem sys;
    TimingConfig custom_config = {
        .green_st = 15,
        .green_lt = 8,
        .yellow = 3,
        .all_red = 2,
        .ext_threshold = 4,
        .max_ext = 7,
        .skip_limit = 3
    };
    
    traffic_init(&sys, custom_config);
    
    ASSERT_EQ_INT(sys.timing.green_st, 15, "Custom green_st should be 15");
    ASSERT_EQ_INT(sys.timing.green_lt, 8, "Custom green_lt should be 8");
    ASSERT_EQ_INT(sys.timing.yellow, 3, "Custom yellow should be 3");
    ASSERT_EQ_INT(sys.timing.all_red, 2, "Custom all_red should be 2");
}

void test_routing_straight_and_right() {
    TrafficSystem sys = create_test_system();

    bool res = traffic_add_vehicle(&sys, "car_straight", NORTH, SOUTH, 10);
    ASSERT_TRUE(res, "Should successfully add vehicle going straight");

    ASSERT_EQ_INT(traffic_get_queue_size(&sys, NORTH, LANE_STRAIGHT_RIGHT), 1, "Straight/right lane should have 1 car");
    ASSERT_EQ_INT(traffic_get_queue_size(&sys, NORTH, LANE_LEFT), 0, "Left lane should be empty");

    res = traffic_add_vehicle(&sys, "car_right", NORTH, WEST, 12);
    ASSERT_TRUE(res, "Should successfully add vehicle turning right");

    ASSERT_EQ_INT(traffic_get_queue_size(&sys, NORTH, LANE_STRAIGHT_RIGHT), 2, "Straight/Right lane should have 2 cars");
}

void test_routing_left_turn() {
    TrafficSystem sys = create_test_system();

    bool res = traffic_add_vehicle(&sys, "car_left", NORTH, EAST, 15);
    ASSERT_TRUE(res, "Should successfully add vehicle turning left");

    ASSERT_EQ_INT(traffic_get_queue_size(&sys, NORTH, LANE_LEFT), 1, "Left lane should have 1 car");
    ASSERT_EQ_INT(traffic_get_queue_size(&sys, NORTH, LANE_STRAIGHT_RIGHT), 0, "Straight/Right lane should be empty");
}

void test_queue_overflow_via_fsm() {
    TrafficSystem sys = create_test_system();
    char buf[16];

    for (int i = 0; i < MAX_VEHICLES_PER_ROAD; i++) {
        sprintf(buf, "c%d", i);
        traffic_add_vehicle(&sys, buf, NORTH, SOUTH, 0);
    }

    ASSERT_EQ_INT(traffic_get_queue_size(&sys, NORTH, LANE_STRAIGHT_RIGHT), MAX_VEHICLES_PER_ROAD, "Lane should be full");

    bool res = traffic_add_vehicle(&sys, "overflow", NORTH, SOUTH, 100);
    ASSERT_TRUE(!res, "Should return false when lane is full");
}

void test_fsm_state_transitions() {
    TrafficSystem sys = create_test_system();
    char dummy_ids[8][32];

    traffic_add_vehicle(&sys, "car1", NORTH, SOUTH, 0);

    traffic_fsm_step(&sys, dummy_ids);
    traffic_fsm_step(&sys, dummy_ids);
    ASSERT_EQ_INT(sys.current_state, STATE_ALL_RED, "Should remain in ALL_RED until timer expires");
    
    traffic_fsm_step(&sys, dummy_ids);
    ASSERT_EQ_INT(sys.current_state, STATE_NS_RED_YELLOW, "Should transition to PREP (NS_RED_YELLOW)");
    ASSERT_EQ_INT(sys.state_timer, 0, "Timer should reset");
    
    traffic_fsm_step(&sys, dummy_ids);
    ASSERT_EQ_INT(sys.current_state, STATE_NS_STRAIGHT, "Should now be in main GREEN phase");
}

void test_fsm_transitions_with_custom_times() {
    TrafficSystem sys;
    TimingConfig custom_config = {
        .green_st = 7, .green_lt = 4, .yellow = 1, .all_red = 2,
        .ext_threshold = 2, .max_ext = 5, .skip_limit = 2
    };
    traffic_init(&sys, custom_config);
    char dummy_ids[8][32];

    traffic_add_vehicle(&sys, "car1", NORTH, SOUTH, 0);

    traffic_fsm_step(&sys, dummy_ids);
    ASSERT_EQ_INT(sys.current_state, STATE_ALL_RED, "Still in ALL_RED at step 1");
    
    traffic_fsm_step(&sys, dummy_ids);
    ASSERT_EQ_INT(sys.current_state, STATE_NS_RED_YELLOW, "Transitions after ALL_RED=2");
}

void test_light_colors_logic() {
    TrafficSystem sys = create_test_system();
    char dummy_ids[8][32];

    traffic_add_vehicle(&sys, "car1", NORTH, SOUTH, 0);
    advance_to_state(&sys, STATE_NS_STRAIGHT, dummy_ids);
    ASSERT_EQ_INT(sys.lights[NORTH][LANE_STRAIGHT_RIGHT], LIGHT_GREEN, "North straight should be GREEN");
    ASSERT_EQ_INT(sys.lights[NORTH][LANE_LEFT], LIGHT_RED, "North left should be RED during straight phase");
    
    advance_to_state(&sys, STATE_NS_STRAIGHT_YELLOW, dummy_ids);
    ASSERT_EQ_INT(sys.lights[NORTH][LANE_STRAIGHT_RIGHT], LIGHT_YELLOW, "North straight should be YELLOW");
    
    traffic_add_vehicle(&sys, "car2", NORTH, EAST, 0);
    advance_to_state(&sys, STATE_NS_LEFT, dummy_ids);
    ASSERT_EQ_INT(sys.lights[NORTH][LANE_LEFT], LIGHT_GREEN, "North left should be GREEN during left phase");
    ASSERT_EQ_INT(sys.lights[NORTH][LANE_STRAIGHT_RIGHT], LIGHT_RED, "North straight should be RED during left phase");
}

void test_vehicle_discharge_logic() {
    TrafficSystem sys = create_test_system();
    char out_ids[ROAD_COUNT * LANES_PER_ROAD][VEHICLE_ID_LEN];
    uint8_t count;

    traffic_add_vehicle(&sys, "car_1", NORTH, SOUTH, 0);
    ASSERT_EQ_INT(1, traffic_get_queue_size(&sys, NORTH, LANE_STRAIGHT_RIGHT), "Car is waiting in the queue");

    traffic_fsm_step(&sys, out_ids);
    traffic_fsm_step(&sys, out_ids);

    count = traffic_fsm_step(&sys, out_ids);
    ASSERT_EQ_INT(sys.current_state, STATE_NS_RED_YELLOW, "State must be NS_RED_YELLOW");
    ASSERT_EQ_INT(0, count, "No one leaves on RED_YELLOW");

    count = traffic_fsm_step(&sys, out_ids);
    ASSERT_EQ_INT(sys.current_state, STATE_NS_STRAIGHT, "Should be NS_STRAIGHT");
    ASSERT_EQ_INT(1, count, "Car leaves on GREEN");
    ASSERT_STR_EQ(out_ids[0], "car_1", "ID of the car matches");
}

void test_fsm_full_cycle_wrap() {
    TrafficSystem sys = create_test_system();
    char dummy_ids[8][32];

    traffic_add_vehicle(&sys, "c1", NORTH, SOUTH, 0);
    traffic_add_vehicle(&sys, "c2", EAST, WEST, 0);
    traffic_add_vehicle(&sys, "c3", NORTH, EAST, 0);
    traffic_add_vehicle(&sys, "c4", EAST, SOUTH, 0);

    advance_to_state(&sys, STATE_NS_STRAIGHT, dummy_ids);
    ASSERT_EQ_INT(sys.current_state, STATE_NS_STRAIGHT, "Started cycle");

    advance_to_state(&sys, STATE_EW_STRAIGHT, dummy_ids);
    ASSERT_EQ_INT(sys.current_state, STATE_EW_STRAIGHT, "Hit middle of cycle");
    
    traffic_add_vehicle(&sys, "c5", NORTH, SOUTH, 100); 
    advance_to_state(&sys, STATE_NS_STRAIGHT, dummy_ids);
    ASSERT_EQ_INT(sys.current_state, STATE_NS_STRAIGHT, "FSM wrapped back successfully");
}

void test_left_turn_gets_green_time() {
    TrafficSystem sys = create_test_system();
    char dummy_ids[8][32];
    
    traffic_add_vehicle(&sys, "c1", NORTH, EAST, 0);
    advance_to_state(&sys, STATE_NS_LEFT, dummy_ids);
    
    ASSERT_EQ_INT(sys.state_timer, 0, "Timer reset at start of left turn phase");
    
    for(int i = 0; i < 5; i++) {
        ASSERT_EQ_INT(sys.current_state, STATE_NS_LEFT, "Still in left turn phase");
        traffic_fsm_step(&sys, dummy_ids);
    }
    
    ASSERT_EQ_INT(sys.current_state, STATE_NS_LEFT_YELLOW, "Should transition to left yellow after green_lt time");
}

void test_zero_timing_config() {
    TrafficSystem sys;
    TimingConfig zero_config = {0};
    traffic_init(&sys, zero_config);
    char dummy_ids[8][32];
    
    traffic_fsm_step(&sys, dummy_ids);
    ASSERT_TRUE(true, "Should handle zero timings without crashing");
}

void test_phase_skipping_empty_lanes() {
    TrafficSystem sys;
    TimingConfig config = {
        .green_st = 2, .green_lt = 2, .yellow = 1, .all_red = 1,
        .ext_threshold = 1, .max_ext = 3, .skip_limit = 2
    };
    traffic_init(&sys, config);
    char dummy_ids[8][32];
    
    traffic_add_vehicle(&sys, "car1", NORTH, SOUTH, 0);
    traffic_add_vehicle(&sys, "car2", EAST, WEST, 0);
    
    advance_to_state(&sys, STATE_NS_STRAIGHT_YELLOW, dummy_ids);
    
    traffic_fsm_step(&sys, dummy_ids); 
    
    ASSERT_EQ_INT(sys.current_state, STATE_EW_RED_YELLOW, "Should skip NS_LEFT prep and go to EW_STRAIGHT prep");
}

void test_green_extension_basic() {
    TrafficSystem sys;
    TimingConfig config = {
        .green_st = 2, .green_lt = 2, .yellow = 1, .all_red = 1,
        .ext_threshold = 1, .max_ext = 3, .skip_limit = 2
    };
    traffic_init(&sys, config);
    char dummy_ids[8][32];
    
    fill_lane(&sys, NORTH, LANE_STRAIGHT_RIGHT, 4);
    
    advance_to_state(&sys, STATE_NS_STRAIGHT, dummy_ids);
    
    traffic_fsm_step(&sys, dummy_ids);
    traffic_fsm_step(&sys, dummy_ids);
    
    traffic_fsm_step(&sys, dummy_ids);
    
    ASSERT_EQ_INT(sys.current_state, STATE_NS_STRAIGHT, "Should still be in NS_STRAIGHT");
}

void test_green_arrow_right_turns() {
    TrafficSystem sys = create_test_system();
    char out_ids[8][32];
    
    traffic_add_vehicle(&sys, "right_east", EAST, NORTH, 0);
    traffic_add_vehicle(&sys, "left_north", NORTH, EAST, 0);
    
    uint8_t initial_queue = traffic_get_queue_size(&sys, EAST, LANE_STRAIGHT_RIGHT);
    ASSERT_EQ_INT(initial_queue, 1, "Vehicle should be in queue initially");
    
    bool reached_left_phase = false;
    bool discharged_on_arrow = false;
    
    for(int i = 0; i < 15; i++) {
        traffic_fsm_step(&sys, out_ids);
        
        ASSERT_TRUE(sys.current_state != STATE_EW_STRAIGHT, "Reached normal green phase");
        
        if (sys.current_state == STATE_NS_LEFT) {
            reached_left_phase = true;
            
            if (traffic_get_queue_size(&sys, EAST, LANE_STRAIGHT_RIGHT) == 0) {
                discharged_on_arrow = true;
                break;
            }
        }
    }
    
    ASSERT_TRUE(reached_left_phase, "System should enter NS_LEFT phase");
    ASSERT_TRUE(discharged_on_arrow, "Vehicle MUST discharge during NS_LEFT via Green Arrow");
}

int main() {
    printf("\n=== TRAFFIC FSM TESTS ===\n\n");

    RUN_TEST(test_fsm_initialization);
    RUN_TEST(test_fsm_init_with_custom_config);
    RUN_TEST(test_routing_straight_and_right);
    RUN_TEST(test_routing_left_turn);
    RUN_TEST(test_queue_overflow_via_fsm);
    RUN_TEST(test_fsm_state_transitions);
    RUN_TEST(test_fsm_transitions_with_custom_times);
    RUN_TEST(test_light_colors_logic);
    RUN_TEST(test_vehicle_discharge_logic);
    RUN_TEST(test_fsm_full_cycle_wrap);
    RUN_TEST(test_left_turn_gets_green_time);
    RUN_TEST(test_zero_timing_config);
    RUN_TEST(test_phase_skipping_empty_lanes);
    RUN_TEST(test_green_extension_basic);
    RUN_TEST(test_green_arrow_right_turns);

    PRINT_TEST_RESULTS();

    return (tests_failed == 0) ? 0 : 1;
}