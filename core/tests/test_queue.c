#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "traffic_queue.h"

int tests_run = 0;
int tests_failed = 0;

#define LOG_FAIL(msg) printf(" [FAIL] %s:%d: %s\n", __FILE__, __LINE__, msg)

#define ASSERT_TRUE(cond, msg) do { \
    if (!(cond)) { \
        LOG_FAIL(msg); \
        tests_failed++; \
        return; \
    } \
} while(0)

#define ASSERT_EQ_INT(val1, val2, msg) do { \
    int v1 = (val1); \
    int v2 = (val2); \
    if (v1 != v2) { \
        printf(" [FAIL] %s:%d: %s (Expected: %d, Got: %d)\n", \
               __FILE__, __LINE__, msg, v1, v2); \
        tests_failed++; \
        return; \
    } \
} while(0)

#define ASSERT_STR_EQ(str1, str2, msg) do { \
    const char* s1 = (str1); \
    const char* s2 = (str2); \
    if (strcmp(s1, s2) != 0) { \
        printf(" [FAIL] %s:%d: %s (Expected: '%s', Got: '%s')\n", \
               __FILE__, __LINE__, msg, s1, s2); \
        tests_failed++; \
        return; \
    } \
} while(0)

#define RUN_TEST(func) do { \
    printf("Running test: %-35s ...", #func); \
    int failed_before = tests_failed; \
    func(); \
    if (tests_failed == failed_before) { \
        printf(" [PASS]\n"); \
    } \
    tests_run++; \
} while(0)

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
    printf("=== TESTS ===\n\n");

    RUN_TEST(test_initialization);
    RUN_TEST(test_enqueue_dequeue_basic);
    RUN_TEST(test_full_queue_protection);
    RUN_TEST(test_circular_behavior);
    RUN_TEST(test_wait_time_statistics);
    RUN_TEST(test_peek);

    printf("\n=== RESULTS ===\n");
    printf("Run:      %d\n", tests_run);
    printf("Passed:   %d\n", tests_run - tests_failed);
    printf("Failed:   %d\n", tests_failed);

    return (tests_failed == 0) ? 0 : 1;
}