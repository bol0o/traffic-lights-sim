#include "test_utils.h"
#include "traffic_fsm.h"

int tests_run = 0;
int tests_failed = 0;

TrafficSystem create_test_system() {
    TrafficSystem sys;
    TimingConfig test_config = {
        .green_st = 10,
        .green_lt = 5,
        .yellow = 2,
        .all_red = 3
    };
    traffic_init(&sys, test_config);
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

void test_fsm_initialization() {
    TrafficSystem sys;
    TimingConfig config = DEFAULT_TIMING;
    traffic_init(&sys, config);
    
    ASSERT_EQ_INT(sys.current_state, STATE_ALL_RED, "Initial state should be ALL_RED");
    ASSERT_EQ_INT(sys.current_step, 0, "Initial step should be 0");
    ASSERT_EQ_INT(sys.state_timer, 0, "State timer should be 0");
    ASSERT_EQ_INT(sys.timing.green_st, 10, "Default green_st should be 10");
    ASSERT_EQ_INT(sys.timing.green_lt, 5, "Default green_lt should be 5");
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
        .all_red = 2
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

    for(uint32_t i = 0; i < sys.timing.all_red; i++) {
        traffic_fsm_step(&sys, dummy_ids);
    }
    ASSERT_EQ_INT(sys.current_state, STATE_NS_STRAIGHT, "Should transition to NS_STRAIGHT after ALL_RED time");
    ASSERT_EQ_INT(sys.state_timer, 0, "Timer should reset after transition");

    for(uint32_t i = 0; i < sys.timing.green_st; i++) {
        traffic_fsm_step(&sys, dummy_ids);
    }
    ASSERT_EQ_INT(sys.current_state, STATE_NS_STRAIGHT_YELLOW, "Should transition to YELLOW after GREEN_ST time");
    ASSERT_EQ_INT(sys.state_timer, 0, "Timer should reset after transition");
    
    for(uint32_t i = 0; i < sys.timing.yellow; i++) {
        traffic_fsm_step(&sys, dummy_ids);
    }
    ASSERT_EQ_INT(sys.current_state, STATE_NS_LEFT, "Should transition to NS_LEFT after YELLOW time");
}

void test_fsm_transitions_with_custom_times() {
    TrafficSystem sys;
    TimingConfig custom_config = {
        .green_st = 7,
        .green_lt = 4,
        .yellow = 1,
        .all_red = 2
    };
    traffic_init(&sys, custom_config);
    char dummy_ids[8][32];

    for(uint32_t i = 0; i < sys.timing.all_red; i++) traffic_fsm_step(&sys, dummy_ids);
    ASSERT_EQ_INT(sys.current_state, STATE_NS_STRAIGHT, "Should transition after custom ALL_RED=2");
    
    for(uint32_t i = 0; i < sys.timing.green_st; i++) traffic_fsm_step(&sys, dummy_ids);
    ASSERT_EQ_INT(sys.current_state, STATE_NS_STRAIGHT_YELLOW, "Should transition after custom GREEN_ST=7");
    
    for(uint32_t i = 0; i < sys.timing.yellow; i++) traffic_fsm_step(&sys, dummy_ids);
    ASSERT_EQ_INT(sys.current_state, STATE_NS_LEFT, "Should transition after custom YELLOW=1");
}

void test_light_colors_logic() {
    TrafficSystem sys = create_test_system();
    char dummy_ids[8][32];

    advance_to_state(&sys, STATE_NS_STRAIGHT, dummy_ids);

    ASSERT_EQ_INT(sys.lights[NORTH][LANE_STRAIGHT_RIGHT], LIGHT_GREEN, "North straight should be GREEN");
    ASSERT_EQ_INT(sys.lights[NORTH][LANE_LEFT], LIGHT_RED, "North left should be RED during straight phase");
    
    advance_to_state(&sys, STATE_NS_STRAIGHT_YELLOW, dummy_ids);
    ASSERT_EQ_INT(sys.lights[NORTH][LANE_STRAIGHT_RIGHT], LIGHT_YELLOW, "North straight should be YELLOW");
    
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

    for(uint32_t i = 0; i < sys.timing.all_red - 1; i++) {
        count = traffic_fsm_step(&sys, out_ids);
        ASSERT_EQ_INT(0, count, "No one leaves on red light");
    }

    count = traffic_fsm_step(&sys, out_ids);
    ASSERT_EQ_INT(1, count, "Car leaves on green");
    ASSERT_EQ_INT(sys.current_state, STATE_NS_STRAIGHT, "Should be NS_STRAIGHT");
    ASSERT_STR_EQ(out_ids[0], "car_1", "ID of the car matches");
}

void test_fsm_full_cycle_wrap() {
    TrafficSystem sys = create_test_system();
    char dummy_ids[8][32];

    int total_cycle_time = sys.timing.all_red + 
                           (sys.timing.green_st + sys.timing.yellow) * 2 +
                           (sys.timing.green_lt + sys.timing.yellow) * 2;

    for(int i = 0; i < total_cycle_time; i++) {
        traffic_fsm_step(&sys, dummy_ids);
    }

    ASSERT_EQ_INT(sys.current_state, STATE_ALL_RED, "FSM should wrap back to ALL_RED after full cycle");
}

void test_left_turn_gets_green_time() {
    TrafficSystem sys = create_test_system();
    char dummy_ids[8][32];
    
    advance_to_state(&sys, STATE_NS_LEFT, dummy_ids);
    
    ASSERT_EQ_INT(sys.state_timer, 0, "Timer reset at start of left turn phase");
    
    for(uint32_t i = 0; i < sys.timing.green_lt - 1; i++) {
        traffic_fsm_step(&sys, dummy_ids);
        ASSERT_EQ_INT(sys.current_state, STATE_NS_LEFT, "Still in left turn phase");
    }
    
    traffic_fsm_step(&sys, dummy_ids);
    ASSERT_EQ_INT(sys.current_state, STATE_NS_LEFT_YELLOW, "Should transition to left yellow after green_lt time");
}

void test_zero_timing_config() {
    TrafficSystem sys;
    TimingConfig zero_config = {0, 0, 0, 0};
    traffic_init(&sys, zero_config);
    char dummy_ids[8][32];
    
    traffic_fsm_step(&sys, dummy_ids);
    ASSERT_EQ_INT(sys.current_state, STATE_NS_STRAIGHT, "Should immediately transition to NS_STRAIGHT");
    
    traffic_fsm_step(&sys, dummy_ids);
    ASSERT_EQ_INT(sys.current_state, STATE_NS_STRAIGHT_YELLOW, "Should immediately transition to YELLOW");
}

int main() {
    printf("=== FSM TESTS ===\n\n");

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

    PRINT_TEST_RESULTS();

    return (tests_failed == 0) ? 0 : 1;
}