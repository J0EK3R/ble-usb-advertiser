/**
 * @file fake_ble.h
 * @brief Fake BLE adapter, fake I/O and a helper harness for the bc2_core tests.
 *
 * The harness records every BLE call, can inject errors, captures bytes written to the "USB port"
 * and provides a controllable clock, so core behaviour can be tested without hardware.
 */
#ifndef FAKE_BLE_H
#define FAKE_BLE_H

#include "bc2_core.h"

#define FAKE_MAX_CALLS 64
#define FAKE_MAX_OUT 2048

/** Kinds of recorded BLE calls. */
typedef enum { CALL_START, CALL_UPDATE, CALL_STOP } fake_call_kind_t;

/** One recorded BLE call. */
typedef struct {
    fake_call_kind_t kind;
    uint8_t set_id;
    uint16_t mfr_id;
    uint8_t data_len;
    uint8_t data[32];
    uint8_t flags;
} fake_call_t;

/** Complete fake environment around one bc2_core_t. */
typedef struct {
    bc2_core_t core;
    /* BLE fake */
    bool ready;
    uint8_t max_sets;
    fake_call_t calls[FAKE_MAX_CALLS];
    int call_count;
    bc2_status_t start_result;      /**< status returned by start() */
    bc2_status_t update_result;     /**< status returned by update() */
    uint16_t error_rc;              /**< rc reported with BLE errors */
    int stop_failures;              /**< number of upcoming stop() calls that fail */
    bool failure_pending;           /**< take_failure() returns (failure_set, failure_reason) once */
    uint8_t failure_set;
    uint8_t failure_reason;
    /* IO fake */
    uint8_t out[FAKE_MAX_OUT];      /**< bytes written by the core */
    size_t out_len;
    uint32_t now_us;                /**< value returned by now_us() */
    uint32_t us_per_call;           /**< now_us() advances by this amount per call */
    int hard_resets;
    /* clock used by the tests */
    uint32_t now_ms;
    uint8_t seq;
} fake_env_t;

/** Initialise the environment (ready BLE, 5 sets, success results) and the core. */
void fake_init(fake_env_t *env);

/** Send a request built from type and payload to the core (valid frame, auto seq). */
void fake_send(fake_env_t *env, uint8_t type, const uint8_t *payload, uint8_t len);

/** Send HELLO with the given keep-alive and return after processing. */
void fake_hello(fake_env_t *env, uint16_t keepalive_ms);

/** Send SET_START (flags 0x01, interval 32, tx 127). */
void fake_start(fake_env_t *env, uint8_t set_id, uint16_t mfr, const uint8_t *data, uint8_t len);

/**
 * Parse the captured output and return frame number @p index (0-based).
 * @return true if such a frame exists
 */
bool fake_out_frame(const fake_env_t *env, int index, bc2_frame_t *fr);

/** Number of frames in the captured output. */
int fake_out_count(const fake_env_t *env);

/** Forget captured output and recorded BLE calls. */
void fake_clear(fake_env_t *env);

/** Count recorded BLE calls of one kind. */
int fake_count_calls(const fake_env_t *env, fake_call_kind_t kind);

#endif /* FAKE_BLE_H */
