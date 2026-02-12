#include "test_utils.h"
#include "traffic_fsm.h"

int tests_run = 0;
int tests_failed = 0;

void test_fsm_initialization() {
    TrafficSystem sys;
    traffic_init(&sys);

    ASSERT_EQ_INT(sys.current_state, STATE_ALL_RED, "Initial state should be ALL_RED");
    ASSERT_EQ_INT(sys.current_step, 0, "Initial step should be 0");
    ASSERT_EQ_INT(sys.state_timer, 0, "State timer should be 0");

    uint16_t count = traffic_get_queue_size(&sys, NORTH, LANE_STRAIGHT_RIGHT);
    ASSERT_EQ_INT(count, 0, "Queues should be empty after init");
}

void test_routing_straight_and_right() {
    TrafficSystem sys;
    traffic_init(&sys);

    // Case 1: North -> South (Heading straight)
    bool res = traffic_add_vehicle(&sys, "car_straight", NORTH, SOUTH, 10);
    ASSERT_TRUE(res, "Should successfully add vehicle going straight");

    ASSERT_EQ_INT(traffic_get_queue_size(&sys, NORTH, LANE_STRAIGHT_RIGHT), 1, "Straight/right lane should have 1 car");
    ASSERT_EQ_INT(traffic_get_queue_size(&sys, NORTH, LANE_LEFT), 0, "Left lane should be empty");

    // Case 2: North -> West (Heading right)
    res = traffic_add_vehicle(&sys, "car_right", NORTH, WEST, 12);
    ASSERT_TRUE(res, "Should successfully add vehicle turning right");

    ASSERT_EQ_INT(traffic_get_queue_size(&sys, NORTH, LANE_STRAIGHT_RIGHT), 2, "Straight/Right lane should have 2 cars");
}

void test_routing_left_turn() {
    TrafficSystem sys;
    traffic_init(&sys);

    bool res = traffic_add_vehicle(&sys, "car_left", NORTH, EAST, 15);
    ASSERT_TRUE(res, "Should successfully add vehicle turning left");

    ASSERT_EQ_INT(traffic_get_queue_size(&sys, NORTH, LANE_LEFT), 1, "Left lane should have 1 car");
    ASSERT_EQ_INT(traffic_get_queue_size(&sys, NORTH, LANE_STRAIGHT_RIGHT), 0, "Straight/Right lane should be empty");
}

void test_queue_overflow_via_fsm() {
    TrafficSystem sys;
    traffic_init(&sys);
    char buf[16];

    for (int i = 0; i < MAX_VEHICLES_PER_ROAD; i++) {
        sprintf(buf, "c%d", i);
        traffic_add_vehicle(&sys, buf, NORTH, SOUTH, 0);
    }

    ASSERT_EQ_INT(traffic_get_queue_size(&sys, NORTH, LANE_STRAIGHT_RIGHT), MAX_VEHICLES_PER_ROAD, "Lane should be full");

    bool res = traffic_add_vehicle(&sys, "overflow", NORTH, SOUTH, 100);
    ASSERT_TRUE(!res, "Should return false when lane is full");
}

int main() {
    printf("=== FSM TESTS ===\n\n");

    RUN_TEST(test_fsm_initialization);
    RUN_TEST(test_routing_straight_and_right);
    RUN_TEST(test_routing_left_turn);
    RUN_TEST(test_queue_overflow_via_fsm);

    PRINT_TEST_RESULTS();

    return (tests_failed == 0) ? 0 : 1;
}