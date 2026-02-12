#include "traffic_queue.h"
#include "test_utils.h"

int tests_run = 0;
int tests_failed = 0;

void test_initialization() {
    VehicleQueue q;
    queue_init(&q);

    ASSERT_EQ_INT(queue_count(&q), 0, "Queue should be empty after init");
    ASSERT_TRUE(queue_is_empty(&q), "is_empty flag should be true");
    ASSERT_TRUE(!queue_is_full(&q), "is_full flag should be false");
    ASSERT_EQ_INT(q.head, 0, "Head should be 0");
    ASSERT_EQ_INT(q.tail, 0, "Tail should be 0");
}

void test_enqueue_dequeue_basic() {
    VehicleQueue q;
    queue_init(&q);
    char buffer[VEHICLE_ID_LEN];

    ASSERT_TRUE(queue_enqueue(&q, "car1", NORTH, SOUTH, 10), "Enqueue failed");
    ASSERT_EQ_INT(queue_count(&q), 1, "Count should be 1");
    ASSERT_TRUE(!queue_is_empty(&q), "Queue should not be empty");

    ASSERT_TRUE(queue_dequeue(&q, buffer, 15, NULL), "Dequeue failed");
    ASSERT_STR_EQ(buffer, "car1", "ID mismatch");
    ASSERT_EQ_INT(queue_count(&q), 0, "Count should be 0");
}

void test_full_queue_protection() {
    VehicleQueue q;
    queue_init(&q);
    char name_buf[16];

    for (int i = 0; i < MAX_VEHICLES_PER_ROAD; i++) {
        sprintf(name_buf, "c%d", i);
        if (!queue_enqueue(&q, name_buf, NORTH, SOUTH, 0)) {
            ASSERT_TRUE(false, "Failed to fill the queue to limit");
        }
    }

    ASSERT_TRUE(queue_is_full(&q), "Queue should be full");
    
    bool result = queue_enqueue(&q, "overflow", NORTH, SOUTH, 0);
    ASSERT_TRUE(!result, "Enqueue on full queue should return false");
    ASSERT_EQ_INT(queue_count(&q), MAX_VEHICLES_PER_ROAD, "Counter exceeded MAX limit");
}

void test_circular_behavior() {
    VehicleQueue q;
    queue_init(&q);
    
    for (int i = 0; i < MAX_VEHICLES_PER_ROAD; i++) {
        queue_enqueue(&q, "fill", NORTH, SOUTH, 0);
    }

    for (int i = 0; i < 10; i++) {
        queue_dequeue(&q, NULL, 0, NULL);
    }

    for (int i = 0; i < 5; i++) {
        char id[10];
        sprintf(id, "w%d", i);
        ASSERT_TRUE(queue_enqueue(&q, id, NORTH, SOUTH, 0), "Wrap-around enqueue failed");
    }

    ASSERT_EQ_INT(q.tail, 5, "Tail index incorrect after wrap");
    ASSERT_EQ_INT(queue_count(&q), MAX_VEHICLES_PER_ROAD - 10 + 5, "Incorrect count after wrap");
}

void test_wait_time_statistics() {
    VehicleQueue q;
    queue_init(&q);
    uint32_t wait_time = 0;

    // Car 1: Arrives at 100, Leaves at 150 -> Wait 50
    queue_enqueue(&q, "t1", NORTH, SOUTH, 100);
    queue_dequeue(&q, NULL, 150, &wait_time);
    ASSERT_EQ_INT(wait_time, 50, "Incorrect wait time calculated");
    ASSERT_EQ_INT(q.max_wait_time, 50, "max_wait_time statistic incorrect");

    // Car 2: Arrives at 200, Leaves at 210 -> Wait 10
    queue_enqueue(&q, "t2", NORTH, SOUTH, 200);
    queue_dequeue(&q, NULL, 210, &wait_time); 
    ASSERT_EQ_INT(wait_time, 10, "Incorrect wait time for second car");
    ASSERT_EQ_INT(q.max_wait_time, 50, "max_wait_time should retain highest value");
}

void test_peek() {
    VehicleQueue q;
    queue_init(&q);
    Vehicle v;

    queue_enqueue(&q, "peek", NORTH, SOUTH, 10);
    ASSERT_TRUE(queue_peek(&q, &v), "Peek operation failed");
    ASSERT_STR_EQ(v.id, "peek", "Peek returned wrong vehicle ID");
    ASSERT_EQ_INT(queue_count(&q), 1, "Peek should not remove element");
}

int main() {
    printf("=== QUEUE TESTS ===\n\n");

    RUN_TEST(test_initialization);
    RUN_TEST(test_enqueue_dequeue_basic);
    RUN_TEST(test_full_queue_protection);
    RUN_TEST(test_circular_behavior);
    RUN_TEST(test_wait_time_statistics);
    RUN_TEST(test_peek);

    PRINT_TEST_RESULTS();

    return (tests_failed == 0) ? 0 : 1;
}