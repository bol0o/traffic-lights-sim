#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern int tests_run;
extern int tests_failed;

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

#define PRINT_TEST_RESULTS() do { \
    printf("\n=== RESULTS ===\n"); \
    printf("Run:      %d\n", tests_run); \
    printf("Passed:   %d\n", tests_run - tests_failed); \
    printf("Failed:   %d\n", tests_failed); \
} while(0)

#endif