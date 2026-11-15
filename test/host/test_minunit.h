/**
 * @file test_minunit.h
 * @brief Tiny assertion/test-runner helpers (no external test framework needed).
 *
 * A test is a `static void name(void)`. ASSERT_* macros record a failure and return from the
 * test function. RUN_TEST executes a test and counts it.
 */
#ifndef TEST_MINUNIT_H
#define TEST_MINUNIT_H

#include <stdio.h>
#include <string.h>

/** Global counters, defined in test_main.c. */
extern int g_tests_run;
extern int g_tests_failed;
/** Set to 1 by a failing assertion inside the running test. */
extern int g_current_failed;

/** Fail the current test with a message unless @p cond holds. */
#define ASSERT_TRUE(cond)                                                                    \
    do {                                                                                     \
        if (!(cond)) {                                                                       \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                         \
            g_current_failed = 1;                                                            \
            return;                                                                          \
        }                                                                                    \
    } while (0)

/** Compare two integers (printed as signed long long on failure). */
#define ASSERT_EQ(expected, actual)                                                          \
    do {                                                                                     \
        long long e_ = (long long)(expected), a_ = (long long)(actual);                      \
        if (e_ != a_) {                                                                      \
            printf("  FAIL %s:%d: %s == %s (expected %lld, got %lld)\n", __FILE__, __LINE__, \
                   #expected, #actual, e_, a_);                                              \
            g_current_failed = 1;                                                            \
            return;                                                                          \
        }                                                                                    \
    } while (0)

/** Compare two byte buffers of @p len bytes. */
#define ASSERT_MEM_EQ(expected, actual, len)                                                 \
    do {                                                                                     \
        if (memcmp((expected), (actual), (len)) != 0) {                                      \
            printf("  FAIL %s:%d: memory differs (%s vs %s)\n", __FILE__, __LINE__,          \
                   #expected, #actual);                                                      \
            g_current_failed = 1;                                                            \
            return;                                                                          \
        }                                                                                    \
    } while (0)

/** Run one test function and update the counters. */
#define RUN_TEST(fn)                                                                         \
    do {                                                                                     \
        g_current_failed = 0;                                                                \
        printf("[ RUN  ] %s\n", #fn);                                                        \
        fn();                                                                                \
        g_tests_run++;                                                                       \
        if (g_current_failed) {                                                              \
            g_tests_failed++;                                                                \
        }                                                                                    \
    } while (0)

#endif /* TEST_MINUNIT_H */
