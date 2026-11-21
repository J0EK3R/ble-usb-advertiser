/**
 * @file test_scanfilter.c
 * @brief Tests for the scan report filter (dedupe, refresh, LRU, budget) and the address cache.
 */
#include "bc2_scanfilter.h"
#include "test_minunit.h"

static const uint8_t A1[6] = {1, 1, 1, 1, 1, 1};
static const uint8_t A2[6] = {2, 2, 2, 2, 2, 2};
static const uint8_t D1[3] = {1, 2, 3};
static const uint8_t D2[3] = {1, 2, 4};

/** New advertiser is reported, an identical PDU is a duplicate, changed data is reported. */
static void test_sf_dedupe(void)
{
    bc2_scanfilter_t f;
    bc2_scanfilter_init(&f, 1000, false);
    ASSERT_EQ(BC2_SF_REPORT, bc2_scanfilter_check(&f, A1, 0, false, D1, 3, 0));
    ASSERT_EQ(BC2_SF_DUPLICATE, bc2_scanfilter_check(&f, A1, 0, false, D1, 3, 10));
    ASSERT_EQ(BC2_SF_REPORT, bc2_scanfilter_check(&f, A1, 0, false, D2, 3, 20));   /* data changed */
    ASSERT_EQ(BC2_SF_REPORT, bc2_scanfilter_check(&f, A2, 0, false, D1, 3, 30));   /* other address */
    ASSERT_EQ(BC2_SF_REPORT, bc2_scanfilter_check(&f, A1, 1, false, D2, 3, 40));   /* other addr type */
}

/** ADV and SCAN_RSP of one address are tracked separately (LEGO names live in the response). */
static void test_sf_scan_response_flag(void)
{
    bc2_scanfilter_t f;
    bc2_scanfilter_init(&f, 1000, false);
    ASSERT_EQ(BC2_SF_REPORT, bc2_scanfilter_check(&f, A1, 0, false, D1, 3, 0));
    ASSERT_EQ(BC2_SF_REPORT, bc2_scanfilter_check(&f, A1, 0, true, D1, 3, 1));
    ASSERT_EQ(BC2_SF_DUPLICATE, bc2_scanfilter_check(&f, A1, 0, true, D1, 3, 2));
}

/** An unchanged advertiser is re-reported after refresh_ms (also across the 32 bit clock wrap). */
static void test_sf_refresh(void)
{
    bc2_scanfilter_t f;
    bc2_scanfilter_init(&f, 500, false);
    ASSERT_EQ(BC2_SF_REPORT, bc2_scanfilter_check(&f, A1, 0, false, D1, 3, 0xFFFFFF00u));
    ASSERT_EQ(BC2_SF_DUPLICATE, bc2_scanfilter_check(&f, A1, 0, false, D1, 3, 0xFFFFFF00u + 499u));
    ASSERT_EQ(BC2_SF_REPORT, bc2_scanfilter_check(&f, A1, 0, false, D1, 3, 0xFFFFFF00u + 500u));
}

/** "Report duplicates" mode disables the dedupe. */
static void test_sf_report_all(void)
{
    bc2_scanfilter_t f;
    bc2_scanfilter_init(&f, 1000, true);
    for (uint32_t t = 0; t < 5; t++) {
        ASSERT_EQ(BC2_SF_REPORT, bc2_scanfilter_check(&f, A1, 0, false, D1, 3, t));
    }
}

/** At most 150 reports per second; the budget refills in the next window. */
static void test_sf_budget(void)
{
    bc2_scanfilter_t f;
    bc2_scanfilter_init(&f, 1000, true);                 /* report_all so every PDU needs budget */
    for (unsigned i = 0; i < BC2_SCANFILTER_BUDGET; i++) {
        ASSERT_EQ(BC2_SF_REPORT, bc2_scanfilter_check(&f, A1, 0, false, D1, 3, 5000 + i));
    }
    ASSERT_EQ(BC2_SF_OVER_BUDGET, bc2_scanfilter_check(&f, A1, 0, false, D1, 3, 5500));
    ASSERT_EQ(BC2_SF_OVER_BUDGET, bc2_scanfilter_check(&f, A2, 0, false, D1, 3, 5999));
    ASSERT_EQ(2, f.dropped_budget);
    ASSERT_EQ(BC2_SF_REPORT, bc2_scanfilter_check(&f, A1, 0, false, D1, 3, 6000));  /* new window */
}

/** A PDU refused by the budget is not remembered, so it is reported once budget is available. */
static void test_sf_budget_keeps_entry_unseen(void)
{
    bc2_scanfilter_t f;
    bc2_scanfilter_init(&f, 1000, false);
    for (unsigned i = 0; i < BC2_SCANFILTER_BUDGET; i++) {            /* use up the budget */
        uint8_t a[6] = {9, 9, 9, 9, (uint8_t)(i >> 8), (uint8_t)i};
        ASSERT_EQ(BC2_SF_REPORT, bc2_scanfilter_check(&f, a, 0, false, D1, 3, 100));
    }
    ASSERT_EQ(BC2_SF_OVER_BUDGET, bc2_scanfilter_check(&f, A1, 0, false, D1, 3, 200));
    ASSERT_EQ(BC2_SF_REPORT, bc2_scanfilter_check(&f, A1, 0, false, D1, 3, 1100));
}

/** The table holds 48 advertisers; the least recently reported one is evicted first. */
static void test_sf_lru_eviction(void)
{
    bc2_scanfilter_t f;
    bc2_scanfilter_init(&f, 10000, false);
    for (unsigned i = 0; i < BC2_SCANFILTER_ENTRIES; i++) {
        uint8_t a[6] = {0, 0, 0, 0, 0, (uint8_t)(i + 1)};
        ASSERT_EQ(BC2_SF_REPORT, bc2_scanfilter_check(&f, a, 0, false, D1, 3, i * 10u));
    }
    const uint8_t extra[6] = {0, 0, 0, 0, 1, 0};
    ASSERT_EQ(BC2_SF_REPORT, bc2_scanfilter_check(&f, extra, 0, false, D1, 3, 1000));  /* evicts #1 */
    const uint8_t first[6] = {0, 0, 0, 0, 0, 1};
    const uint8_t second[6] = {0, 0, 0, 0, 0, 2};
    ASSERT_EQ(BC2_SF_DUPLICATE, bc2_scanfilter_check(&f, second, 0, false, D1, 3, 1001));
    ASSERT_EQ(BC2_SF_REPORT, bc2_scanfilter_check(&f, first, 0, false, D1, 3, 1002));  /* forgotten */
}

static void test_addrcache_basic(void)
{
    bc2_addrcache_t c;
    uint8_t type = 9;
    bc2_addrcache_init(&c);
    ASSERT_TRUE(!bc2_addrcache_get(&c, A1, &type));
    bc2_addrcache_put(&c, A1, 1);
    bc2_addrcache_put(&c, A2, 0);
    ASSERT_TRUE(bc2_addrcache_get(&c, A1, &type));
    ASSERT_EQ(1, type);
    bc2_addrcache_put(&c, A1, 0);                        /* update in place */
    ASSERT_TRUE(bc2_addrcache_get(&c, A1, &type));
    ASSERT_EQ(0, type);
}

/** 64 entries; a recently used entry survives while the least recently used one is evicted. */
static void test_addrcache_lru(void)
{
    bc2_addrcache_t c;
    uint8_t type;
    bc2_addrcache_init(&c);
    for (unsigned i = 0; i < BC2_ADDRCACHE_ENTRIES; i++) {
        const uint8_t a[6] = {0, 0, 0, 0, 0, (uint8_t)(i + 1)};
        bc2_addrcache_put(&c, a, 1);
    }
    const uint8_t first[6] = {0, 0, 0, 0, 0, 1};
    ASSERT_TRUE(bc2_addrcache_get(&c, first, &type));                 /* touch #1: now #2 is the oldest */
    const uint8_t extra[6] = {0, 0, 0, 0, 1, 0};
    bc2_addrcache_put(&c, extra, 0);
    const uint8_t second[6] = {0, 0, 0, 0, 0, 2};
    ASSERT_TRUE(!bc2_addrcache_get(&c, second, &type));
    ASSERT_TRUE(bc2_addrcache_get(&c, first, &type));
    ASSERT_TRUE(bc2_addrcache_get(&c, extra, &type));
    ASSERT_EQ(0, type);
}

void run_scanfilter_tests(void)
{
    RUN_TEST(test_sf_dedupe);
    RUN_TEST(test_sf_scan_response_flag);
    RUN_TEST(test_sf_refresh);
    RUN_TEST(test_sf_report_all);
    RUN_TEST(test_sf_budget);
    RUN_TEST(test_sf_budget_keeps_entry_unseen);
    RUN_TEST(test_sf_lru_eviction);
    RUN_TEST(test_addrcache_basic);
    RUN_TEST(test_addrcache_lru);
}
