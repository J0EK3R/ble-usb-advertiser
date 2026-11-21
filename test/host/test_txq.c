/**
 * @file test_txq.c
 * @brief Tests for the TX frame queue: FIFO order, wrap-around, full queue, drop classes.
 */
#include "bc2_txq.h"
#include "test_minunit.h"

/** Build a frame of @p len bytes filled with @p fill. */
static void make(uint8_t *buf, size_t len, uint8_t fill)
{
    memset(buf, fill, len);
}

static void test_txq_fifo(void)
{
    uint8_t storage[200], f[BC2_TXQ_MAX_FRAME], out[BC2_TXQ_MAX_FRAME];
    bc2_txq_t q;
    bc2_txq_init(&q, storage, sizeof(storage));
    ASSERT_EQ(0, bc2_txq_pop(&q, out));
    make(f, 10, 0xA1);
    ASSERT_TRUE(bc2_txq_push(&q, f, 10, false));
    make(f, 20, 0xB2);
    ASSERT_TRUE(bc2_txq_push(&q, f, 20, true));
    ASSERT_EQ(2, bc2_txq_frames(&q));
    ASSERT_EQ(32, bc2_txq_used(&q));
    ASSERT_EQ(10, bc2_txq_pop(&q, out));
    ASSERT_EQ(0xA1, out[9]);
    ASSERT_EQ(20, bc2_txq_pop(&q, out));
    ASSERT_EQ(0xB2, out[0]);
    ASSERT_EQ(0, bc2_txq_used(&q));
    ASSERT_EQ(0, bc2_txq_pop(&q, out));
}

/** Frames wrap around the end of the storage without corruption. */
static void test_txq_wrap(void)
{
    uint8_t storage[64], f[BC2_TXQ_MAX_FRAME], out[BC2_TXQ_MAX_FRAME];
    bc2_txq_t q;
    bc2_txq_init(&q, storage, sizeof(storage));
    for (int round = 0; round < 50; round++) {          /* far more data than the storage size */
        const size_t len = (size_t)(10 + round % 13);
        make(f, len, (uint8_t)round);
        ASSERT_TRUE(bc2_txq_push(&q, f, len, false));
        ASSERT_EQ(len, bc2_txq_pop(&q, out));
        for (size_t i = 0; i < len; i++) {
            ASSERT_EQ((uint8_t)round, out[i]);
        }
    }
    ASSERT_EQ(0, q.dropped_critical);
}

/** Critical frames fill the buffer completely, then they are rejected and counted. */
static void test_txq_full_critical(void)
{
    uint8_t storage[40], f[BC2_TXQ_MAX_FRAME];
    bc2_txq_t q;
    bc2_txq_init(&q, storage, sizeof(storage));
    make(f, 19, 1);                                     /* 20 bytes per entry: two fit exactly */
    ASSERT_TRUE(bc2_txq_push(&q, f, 19, false));
    ASSERT_TRUE(bc2_txq_push(&q, f, 19, false));
    ASSERT_TRUE(!bc2_txq_push(&q, f, 1, false));
    ASSERT_EQ(1, q.dropped_critical);
    ASSERT_EQ(0, q.dropped_droppable);
}

/** Droppable frames are rejected above half fill level, so critical frames always find room. */
static void test_txq_droppable_limit(void)
{
    uint8_t storage[100], f[BC2_TXQ_MAX_FRAME], out[BC2_TXQ_MAX_FRAME];
    bc2_txq_t q;
    bc2_txq_init(&q, storage, sizeof(storage));
    make(f, 24, 7);                                     /* 25 bytes per entry */
    ASSERT_TRUE(bc2_txq_push(&q, f, 24, true));
    ASSERT_TRUE(bc2_txq_push(&q, f, 24, true));         /* 50 bytes = exactly half */
    ASSERT_TRUE(!bc2_txq_push(&q, f, 24, true));
    ASSERT_TRUE(!bc2_txq_push(&q, f, 1, true));
    ASSERT_EQ(2, q.dropped_droppable);
    ASSERT_TRUE(bc2_txq_push(&q, f, 24, false));        /* critical still fits */
    ASSERT_TRUE(bc2_txq_push(&q, f, 24, false));
    ASSERT_TRUE(!bc2_txq_push(&q, f, 24, false));
    ASSERT_EQ(1, q.dropped_critical);
    ASSERT_EQ(24, bc2_txq_pop(&q, out));                /* draining makes room again */
    ASSERT_TRUE(!bc2_txq_push(&q, f, 24, true));        /* 75 used > half: still refused */
}

static void test_txq_invalid_sizes(void)
{
    uint8_t storage[200], f[BC2_TXQ_MAX_FRAME + 1];
    bc2_txq_t q;
    bc2_txq_init(&q, storage, sizeof(storage));
    memset(f, 0, sizeof(f));
    ASSERT_TRUE(!bc2_txq_push(&q, f, 0, false));
    ASSERT_TRUE(!bc2_txq_push(&q, f, BC2_TXQ_MAX_FRAME + 1, false));
    ASSERT_TRUE(bc2_txq_push(&q, f, BC2_TXQ_MAX_FRAME, false));
}

void run_txq_tests(void)
{
    RUN_TEST(test_txq_fifo);
    RUN_TEST(test_txq_wrap);
    RUN_TEST(test_txq_full_critical);
    RUN_TEST(test_txq_droppable_limit);
    RUN_TEST(test_txq_invalid_sizes);
}
