/**
 * @file test_main.c
 * @brief Test runner entry point: runs all suites and returns non-zero on any failure.
 */
#include <stdio.h>

#include "test_minunit.h"

int g_tests_run;
int g_tests_failed;
int g_current_failed;

void run_crc_tests(void);
void run_framer_tests(void);
void run_codec_tests(void);
void run_gatt_codec_tests(void);
void run_core_tests(void);

int main(void)
{
    run_crc_tests();
    run_framer_tests();
    run_codec_tests();
    run_gatt_codec_tests();
    run_core_tests();
    printf("\n%d tests run, %d failed\n", g_tests_run, g_tests_failed);
    return g_tests_failed == 0 ? 0 : 1;
}
