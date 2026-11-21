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
#include "bc2_gatt.h"

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

#define FAKE_MAX_GCALLS 128
#define FAKE_MAX_GEVENTS 64

/** Kinds of recorded GATT adapter calls. */
typedef enum {
    GCALL_SCAN_START, GCALL_SCAN_STOP, GCALL_CONNECT, GCALL_CANCEL, GCALL_TERMINATE, GCALL_DISCOVER,
    GCALL_READ, GCALL_WRITE, GCALL_WRITE_CMD, GCALL_SUBSCRIBE
} fake_gcall_kind_t;

/** One recorded GATT adapter call. */
typedef struct {
    fake_gcall_kind_t kind;
    uint8_t conn_id;
    uint16_t handle;                /**< handle / cccd */
    uint8_t mode;                   /**< SUBSCRIBE mode, CONNECT addr_type, SCAN_START flags */
    uint8_t len;
    uint8_t data[BC2_GATT_MAX_VALUE];
    uint8_t addr[6];
    uint32_t duration_ms;           /**< CONNECT duration */
    uint16_t itvl_min, itvl_max;    /**< CONNECT intervals */
    uint32_t at_ms;                 /**< fake clock when the call happened */
} fake_gcall_t;

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
    bool refuse_droppable;          /**< write_droppable() refuses (TX queue pressure) */
    /* GATT adapter fake */
    bool gatt_attached;             /**< false = core built without GATT */
    uint8_t max_conns;              /**< value of ops.max_conns() */
    fake_gcall_t gcalls[FAKE_MAX_GCALLS];
    int gcall_count;
    bc2_status_t gresult[GCALL_SUBSCRIBE + 1];   /**< status returned per call kind */
    uint16_t g_rc;                  /**< rc reported with BLE errors */
    int write_cmd_busy;             /**< upcoming write_cmd() calls that answer BUSY */
    bc2_gatt_event_t gev[FAKE_MAX_GEVENTS];      /**< events the adapter "has queued" */
    int gev_head, gev_tail;
    bc2_gatt_adapter_stats_t gstats;
    /* clock used by the tests */
    uint32_t now_ms;
    uint8_t seq;
} fake_env_t;

/** Initialise the environment (ready BLE, 5 sets, GATT adapter with 4 connections) and the core. */
void fake_init(fake_env_t *env);

/** Same, with @p max_conns connections announced by the GATT adapter. */
void fake_init_conns(fake_env_t *env, uint8_t max_conns);

/** Same, but the core is built without a GATT adapter. */
void fake_init_no_gatt(fake_env_t *env);

/** Advance the fake clock to @p now_ms and run bc2_core_tick(). */
void fake_tick(fake_env_t *env, uint32_t now_ms);

/** Queue an adapter event (delivered at the next tick). */
void fake_push_event(fake_env_t *env, const bc2_gatt_event_t *ev);

/** Convenience builders for adapter events (queued, delivered at the next tick). */
void fake_ev_link_up(fake_env_t *env, uint8_t conn_id, uint16_t mtu, uint16_t itvl);
void fake_ev_link_down(fake_env_t *env, uint8_t conn_id, uint16_t reason);
void fake_ev_connect_failed(fake_env_t *env, uint8_t conn_id, uint16_t reason);
void fake_ev_op_done(fake_env_t *env, uint8_t conn_id, uint8_t status, uint16_t code, const uint8_t *data, uint8_t len);
void fake_ev_notify(fake_env_t *env, uint8_t conn_id, uint16_t handle, const uint8_t *data, uint8_t len);

/** Count recorded GATT calls of one kind (optionally for one conn_id, 0xFF = any). */
int fake_count_gcalls(const fake_env_t *env, fake_gcall_kind_t kind, uint8_t conn_id);

/** Last recorded GATT call of a kind, or NULL. */
const fake_gcall_t *fake_last_gcall(const fake_env_t *env, fake_gcall_kind_t kind);

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
