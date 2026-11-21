/**
 * @file bc2_gatt.h
 * @brief GATT central / scanner logic of the stick (protocol 1.1): connection table, connect
 *        scheduler, pending procedures, failsafe writes and the safety stop.
 *
 * Pure C11, no ESP-IDF includes. The radio is reached only through bc2_gatt_ops_t (implemented by
 * the NimBLE adapter on the device, by a fake in the unit tests). Like bc2_core the module is NOT
 * thread safe: the caller serialises every call with the core mutex. The adapter never calls back
 * into this module; it queues bc2_gatt_event_t values that the core polls (bc2_gatt_pump()).
 *
 * Connection slot life cycle (one slot per host chosen conn_id):
 *
 *   FREE --CONNECT--> WAIT --initiator free--> ATTEMPT --link up--> CONNECTED
 *     ^                 ^                         |  \--cancel--> CANCEL
 *     |                 +----- retry -------------+
 *     +----- link down / attempt ended ---- TERMINATING <--- DISCONNECT / safety / timeout
 */
#ifndef BC2_GATT_H
#define BC2_GATT_H

#include "bc2_gatt_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Timing constants of the GATT logic (milliseconds). */
#define BC2_GATT_PROC_TIMEOUT_MS 5000u      /**< READ/WRITE/SUBSCRIBE/DISCOVER without answer -> TIMEOUT */
#define BC2_GATT_ATTEMPT_SLICE_MS 3000u     /**< a connect attempt yields after this if others wait */
#define BC2_GATT_RETRY_DELAY_MS 200u        /**< pause before a failed attempt is retried */
#define BC2_GATT_CANCEL_GUARD_MS 2000u      /**< a cancelled attempt must report back within this time */
#define BC2_GATT_FAILSAFE_FLUSH_MS 40u      /**< delay between the last failsafe write and the terminate */
#define BC2_GATT_FAILSAFE_MAX_WAIT_MS 200u  /**< failsafe writes refused with BUSY are retried this long */
#define BC2_GATT_TERMINATE_RETRY_MS 500u    /**< terminate request is repeated this often until the link is gone */
#define BC2_GATT_VERIFY_MS 1000u            /**< links must be gone this long after a safety stop */
#define BC2_GATT_DROP_REPORT_MS 1000u       /**< minimum spacing of EVT_DROPPED per kind */
#define BC2_GATT_MAX_EVENTS_PER_PUMP 64u    /**< adapter events handled per bc2_gatt_pump() call */

/** Event kinds posted by the adapter (bc2_gatt_event_t.kind). */
typedef enum {
    BC2_GEV_LINK_UP,          /**< link established and MTU exchange done: u16a = mtu, u16b = itvl, addr */
    BC2_GEV_LINK_DOWN,        /**< link ended: code = HCI reason */
    BC2_GEV_CONNECT_FAILED,   /**< connect attempt ended without a link: code = reason */
    BC2_GEV_OP_DONE,          /**< READ/WRITE/SUBSCRIBE finished: status, code (att error / rc), flags, data */
    BC2_GEV_NOTIFY,           /**< notification: u16a = value handle, flags (bit0 indication, bit1 truncated), data */
    BC2_GEV_SCAN_REPORT,      /**< advertising PDU that passed the filter: addr, addr_type, rssi, flags, data */
    BC2_GEV_SERVICE,          /**< discovered service: u16a = start, u16b = end, data = uuid (2 or 16 bytes) */
    BC2_GEV_CHAR,             /**< discovered characteristic: u16a = value handle, u16b = cccd handle, code = props, data = uuid */
    BC2_GEV_DISCOVER_DONE,    /**< discovery finished: status */
    BC2_GEV_SCAN_STOPPED      /**< scan ended by itself: code = rc */
} bc2_gev_kind_t;

/** One adapter event (fixed size so it can travel through a FreeRTOS queue). */
typedef struct {
    uint8_t kind;                          /**< bc2_gev_kind_t */
    uint8_t conn_id;
    uint8_t status;                        /**< bc2_status_t for OP_DONE and DISCOVER_DONE */
    uint8_t flags;
    uint8_t addr_type;
    int8_t rssi;
    uint8_t len;                           /**< valid bytes in data */
    uint16_t code;
    uint16_t u16a;
    uint16_t u16b;
    uint8_t addr[6];                       /**< display order */
    uint8_t data[BC2_GATT_MAX_VALUE];
} bc2_gatt_event_t;

/** Counters and figures the adapter hands over with bc2_gatt_ops_t.stats (deltas are reset by the call). */
typedef struct {
    uint16_t dropped_scan;                 /**< scan reports dropped since the last call (filter budget, full queue) */
    uint16_t dropped_notify;               /**< notifications dropped since the last call */
    bool state_lost;                       /**< a state event could not be queued: link state is unknown */
    uint32_t free_heap;                    /**< bytes, 0 = unknown */
    uint32_t min_free_heap;                /**< bytes, 0 = unknown */
} bc2_gatt_adapter_stats_t;

/**
 * Radio adapter operations. All calls are non blocking and are made under the core mutex; results
 * arrive later as events. Status values are bc2_status_t; @p rc receives a NimBLE return code for
 * BC2_ERR_BLE_ERROR.
 */
typedef struct {
    void *ctx;                                                            /**< passed to every call */
    uint8_t (*max_conns)(void *ctx);                                      /**< simultaneous links supported */
    bc2_status_t (*scan_start)(void *ctx, const bc2_req_scan_start_t *p, uint16_t *rc);
    void (*scan_stop)(void *ctx);
    /** Start ONE connect attempt (the controller has a single initiator). duration_ms 0 = until cancelled. */
    bc2_status_t (*connect)(void *ctx, uint8_t conn_id, const uint8_t addr[6], uint8_t addr_type,
                            uint16_t itvl_min, uint16_t itvl_max, uint32_t duration_ms, uint16_t *rc);
    void (*connect_cancel)(void *ctx);                                    /**< outcome: CONNECT_FAILED or LINK_UP */
    bool (*terminate)(void *ctx, uint8_t conn_id);                        /**< request link termination; outcome: LINK_DOWN */
    bc2_status_t (*discover)(void *ctx, uint8_t conn_id, uint16_t *rc);
    bc2_status_t (*read)(void *ctx, uint8_t conn_id, uint16_t handle, uint16_t *rc);
    bc2_status_t (*write)(void *ctx, uint8_t conn_id, uint16_t handle, const uint8_t *data, uint8_t len,
                          uint16_t *rc);
    bc2_status_t (*write_cmd)(void *ctx, uint8_t conn_id, uint16_t handle, const uint8_t *data, uint8_t len,
                              uint16_t *rc);
    bc2_status_t (*subscribe)(void *ctx, uint8_t conn_id, uint16_t cccd, uint8_t mode, uint16_t *rc);
    bool (*poll_event)(void *ctx, bc2_gatt_event_t *ev);                  /**< next queued event, false if none */
    void (*stats)(void *ctx, bc2_gatt_adapter_stats_t *out);
} bc2_gatt_ops_t;

/** Output side of the module (implemented by bc2_core). */
typedef struct {
    void *ctx;
    /** Send a frame; @p droppable frames may be refused (returns false). */
    bool (*send)(void *ctx, const bc2_frame_t *frame, bool droppable);
    void (*hard_reset)(void *ctx);                                        /**< last resort of the safety stop */
} bc2_gatt_out_t;

/** Connection slot states. */
typedef enum {
    BC2_GC_FREE,
    BC2_GC_WAIT,           /**< CONNECT accepted, waiting for the initiator */
    BC2_GC_ATTEMPT,        /**< the initiator is working on it */
    BC2_GC_CANCEL,         /**< attempt cancel requested, outcome pending */
    BC2_GC_CONNECTED,
    BC2_GC_TERMINATING     /**< link termination requested, LINK_DOWN pending */
} bc2_gc_state_t;

/** One stored failsafe write. */
typedef struct {
    uint8_t len;                                      /**< 0 = slot empty */
    uint16_t handle;
    uint8_t data[BC2_GATT_FAILSAFE_MAX_DATA];
} bc2_failsafe_t;

/** Bookkeeping of one connection slot. */
typedef struct {
    uint8_t state;                                    /**< bc2_gc_state_t */
    uint8_t addr[6];
    uint8_t addr_type;
    bool background;
    bool has_deadline;                                /**< overall connect deadline (not for background connects) */
    uint32_t deadline_ms;
    uint16_t itvl_min, itvl_max;
    uint32_t attempt_start_ms;
    uint32_t retry_at_ms;                             /**< WAIT slots are not tried before this time */
    uint8_t cancel_reason;                            /**< why the attempt is cancelled (0 = only to requeue) */
    bool announced;                                   /**< EVT_CONNECTED was sent for this link */
    uint16_t mtu;
    /* termination */
    uint8_t term_reason;                              /**< reason reported in EVT_DISCONNECTED (0 = use the stack's) */
    bool run_failsafe;                                /**< failsafe writes are due before the terminate */
    uint8_t fs_next;                                  /**< next failsafe slot to write */
    bool term_sent;
    bool safety_kill;                                 /**< terminated by a safety stop: must be gone within 2 s */
    uint32_t term_due_ms;                             /**< earliest time of the terminate request */
    uint32_t term_limit_ms;                           /**< latest time of the terminate request */
    uint32_t term_retry_ms;                           /**< next repeat of a terminate request */
    /* pending procedure */
    bool pend_active;
    uint8_t pend_type;                                /**< BC2_MSG_READ / WRITE / SUBSCRIBE / DISCOVER */
    uint8_t pend_seq;
    uint16_t pend_handle;
    uint32_t pend_deadline_ms;
    uint8_t n_services, n_chars;                      /**< discovery counters (saturating) */
    bc2_failsafe_t failsafe[BC2_GATT_FAILSAFE_SLOTS];
} bc2_gconn_t;

/** State of the GATT module. Treat as opaque; initialise with bc2_gatt_init(). */
typedef struct {
    bc2_gatt_ops_t ops;
    bc2_gatt_out_t out;
    bool present;                                     /**< an adapter is attached (ops.max_conns != NULL) */
    bc2_gconn_t conn[BC2_GATT_MAX_CONNS_LIMIT];
    bool scanning;
    int8_t attempt_conn;                              /**< conn_id of the running attempt, -1 if none */
    uint32_t cancel_sent_ms;
    uint8_t rr_next;                                  /**< round robin start for the next attempt */
    uint16_t dropped_scan, dropped_notify;            /**< totals since HELLO (wrap) */
    uint16_t reported_scan, reported_notify;          /**< totals already announced by EVT_DROPPED */
    uint32_t last_drop_report_ms;
    uint32_t free_heap, min_free_heap;
    uint8_t verify_stage;                             /**< 0 idle, 1 first check pending, 2 second check pending */
    uint32_t verify_at_ms;
} bc2_gatt_t;

/** Initialise. @p ops may be NULL (no GATT support: every request is answered NOT_SUPPORTED). */
void bc2_gatt_init(bc2_gatt_t *g, const bc2_gatt_ops_t *ops, const bc2_gatt_out_t *out);

/** True if an adapter is attached. */
bool bc2_gatt_present(const bc2_gatt_t *g);

/** Value for HELLO_RSP gatt_flags / max_conns (0 without adapter). */
uint8_t bc2_gatt_flags(const bc2_gatt_t *g);
uint8_t bc2_gatt_max_conns(const bc2_gatt_t *g);

/**
 * Handle one GATT request frame (type 0x30..0x3A); the answer is sent from here.
 * The session check was done by the caller.
 */
void bc2_gatt_handle_request(bc2_gatt_t *g, const bc2_frame_t *fr, uint32_t now_ms);

/** Poll and process adapter events (bounded per call). */
void bc2_gatt_pump(bc2_gatt_t *g, uint32_t now_ms);

/** Periodic work: deadlines, connect scheduler, failsafe/terminate sequencing, verification, EVT_DROPPED. */
void bc2_gatt_tick(bc2_gatt_t *g, uint32_t now_ms);

/**
 * Safety stop (protocol section 17.3): failsafe writes + terminate of every link, cancel attempts,
 * stop the scan. Used for keep-alive timeout, HELLO and the button.
 * @param reason BC2_REASON_SAFETY (events report it as EVT_DISCONNECTED reason)
 */
void bc2_gatt_safety_stop(bc2_gatt_t *g, uint32_t now_ms, uint8_t reason);

/** HELLO: safety stop, then forget all counters and failsafe settings of the previous session. */
void bc2_gatt_session_reset(bc2_gatt_t *g, uint32_t now_ms);

/** True if a scan runs or any connection slot is in use (LED "active"). */
bool bc2_gatt_active(const bc2_gatt_t *g);

#ifdef __cplusplus
}
#endif

#endif /* BC2_GATT_H */
