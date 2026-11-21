/**
 * @file bc2_gatt_protocol.h
 * @brief Protocol 1.1 additions: scanning and GATT central messages (TYPE ranges 0x30-0x3A,
 *        0xA0-0xA1 and 0xC0-0xC8). Constants, request decoders and response/event encoders.
 *
 * Pure C11, no ESP-IDF includes. The authoritative description is docs/protocol.md sections 13-18;
 * keep both in sync. All integers are little-endian. Every message fits into one frame (payload
 * <= 60 bytes), so no fragmentation exists.
 */
#ifndef BC2_GATT_PROTOCOL_H
#define BC2_GATT_PROTOCOL_H

#include "bc2_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* Message types                                                              */
/* ------------------------------------------------------------------------- */

/** GATT message TYPE bytes (protocol 1.1). */
typedef enum {
    /* host -> device requests, never retransmitted by the host */
    BC2_MSG_SCAN_START = 0x30,
    BC2_MSG_SCAN_STOP = 0x31,
    BC2_MSG_CONNECT = 0x32,
    BC2_MSG_DISCONNECT = 0x33,
    BC2_MSG_DISCOVER = 0x34,
    BC2_MSG_READ = 0x35,
    BC2_MSG_WRITE = 0x36,           /**< with response */
    BC2_MSG_WRITE_CMD = 0x37,       /**< without response */
    BC2_MSG_SUBSCRIBE = 0x38,
    BC2_MSG_GET_GATT_STATUS = 0x39,
    BC2_MSG_SET_FAILSAFE = 0x3A,
    /* device -> host responses */
    BC2_MSG_READ_RSP = 0xA0,
    BC2_MSG_GATT_STATUS_RSP = 0xA1,
    /* device -> host events (SEQ 0) */
    BC2_MSG_EVT_NOTIFY = 0xC0,
    BC2_MSG_EVT_CONNECTED = 0xC1,
    BC2_MSG_EVT_DISCONNECTED = 0xC2,
    BC2_MSG_EVT_SCAN_REPORT = 0xC3,
    BC2_MSG_EVT_GATT_SERVICE = 0xC4,
    BC2_MSG_EVT_GATT_CHAR = 0xC5,
    BC2_MSG_EVT_DISCOVER_DONE = 0xC6,
    BC2_MSG_EVT_SCAN_STOPPED = 0xC7,
    BC2_MSG_EVT_DROPPED = 0xC8
} bc2_gatt_msg_type_t;

/** True for the request types 0x30..0x3A. */
bool bc2_is_gatt_request(uint8_t type);

/* ------------------------------------------------------------------------- */
/* Limits and defaults                                                        */
/* ------------------------------------------------------------------------- */

#define BC2_GATT_MAX_VALUE 56u            /**< largest value in WRITE / READ_RSP / EVT_NOTIFY */
#define BC2_GATT_MAX_SCAN_DATA 31u        /**< largest advertising data of a scan report */
#define BC2_GATT_MAX_CONN_ID 15u          /**< conn_id is host chosen, 0..15 (u16 masks) */
#define BC2_GATT_MAX_CONNS_LIMIT 16u      /**< protocol maximum of simultaneous links */
#define BC2_GATT_FAILSAFE_SLOTS 4u        /**< failsafe writes per connection */
#define BC2_GATT_FAILSAFE_MAX_DATA 20u    /**< bytes of one failsafe write */
#define BC2_GATT_DEFAULT_MTU 23u          /**< ATT MTU when the exchange failed / is unsupported */

#define BC2_SCAN_DEFAULT_INTERVAL 160u    /**< 100 ms in 0.625 ms units */
#define BC2_SCAN_DEFAULT_WINDOW 80u       /**< 50 ms */
#define BC2_SCAN_MIN_INTERVAL 4u          /**< 2.5 ms */
#define BC2_SCAN_MAX_INTERVAL 16384u      /**< 10.24 s */
#define BC2_SCAN_DEFAULT_REFRESH_MS 1000u
#define BC2_SCAN_MAX_REFRESH_MS 10000u
#define BC2_CONNECT_DEFAULT_TIMEOUT_MS 10000u
#define BC2_CONNECT_DEFAULT_ITVL_MIN 12u  /**< 15 ms in 1.25 ms units */
#define BC2_CONNECT_DEFAULT_ITVL_MAX 24u  /**< 30 ms */
#define BC2_CONNECT_ITVL_LOWEST 6u        /**< 7.5 ms */
#define BC2_CONNECT_ITVL_HIGHEST 3200u    /**< 4 s */
#define BC2_ADDR_TYPE_AUTO 0xFFu          /**< CONNECT addr_type: use the type seen while scanning */

/** HELLO_RSP gatt_flags bits. */
#define BC2_GATT_FLAG_SCAN 0x01u
#define BC2_GATT_FLAG_CENTRAL 0x02u
#define BC2_GATT_FLAG_FAILSAFE 0x04u

/** SCAN_START flags. */
#define BC2_SCAN_FLAG_ACTIVE 0x01u
#define BC2_SCAN_FLAG_DUPLICATES 0x02u
#define BC2_SCAN_FLAG_RESERVED 0xFCu

/** CONNECT flags. */
#define BC2_CONNECT_FLAG_BACKGROUND 0x01u
#define BC2_CONNECT_FLAG_RESERVED 0xFEu

/** EVT_SCAN_REPORT flags. */
#define BC2_REPORT_CONNECTABLE 0x01u
#define BC2_REPORT_SCAN_RSP 0x02u
#define BC2_REPORT_DIRECTED 0x04u

/** EVT_NOTIFY / READ_RSP flags. */
#define BC2_VALUE_INDICATION 0x01u        /**< EVT_NOTIFY only */
#define BC2_VALUE_TRUNCATED 0x02u         /**< EVT_NOTIFY (bit1) */
#define BC2_READ_TRUNCATED 0x01u          /**< READ_RSP (bit0) */

/** EVT_DISCONNECTED phase values. */
#define BC2_PHASE_CONNECTING 0u
#define BC2_PHASE_CONNECTED 1u

/** Stick specific EVT_DISCONNECTED reasons (values below 0xF0 are HCI error codes). */
#define BC2_REASON_CONNECT_TIMEOUT 0xF0u  /**< connect attempt timed out */
#define BC2_REASON_CANCELLED 0xF1u        /**< cancelled by DISCONNECT while connecting */
#define BC2_REASON_SAFETY 0xF2u           /**< stopped by stick safety (timeout / HELLO / button) */
#define BC2_REASON_INTERNAL 0xF3u         /**< stick internal error (procedure TIMEOUT, ...) */

/** EVT_SCAN_STOPPED reasons. */
#define BC2_SCAN_STOP_ERROR 1u
#define BC2_SCAN_STOP_SAFETY 2u

/** EVT_DROPPED kinds. */
#define BC2_DROP_SCAN 1u
#define BC2_DROP_NOTIFY 2u

/** SUBSCRIBE modes. */
#define BC2_SUB_OFF 0u
#define BC2_SUB_NOTIFY 1u
#define BC2_SUB_INDICATE 2u

/* ------------------------------------------------------------------------- */
/* Decoded requests                                                           */
/* ------------------------------------------------------------------------- */

/** SCAN_START with defaults already applied (0 values replaced). */
typedef struct {
    uint8_t flags;
    uint16_t interval;       /**< 0.625 ms units */
    uint16_t window;         /**< 0.625 ms units */
    uint16_t refresh_ms;
} bc2_req_scan_start_t;

/** CONNECT with defaults already applied. */
typedef struct {
    uint8_t conn_id;
    uint8_t addr[6];         /**< display order (byte 0 = first octet of AA:BB:..) */
    uint8_t addr_type;       /**< 0 public, 1 random, BC2_ADDR_TYPE_AUTO */
    uint8_t flags;
    uint16_t timeout_ms;     /**< 0 for background connects */
    uint16_t itvl_min;       /**< 1.25 ms units */
    uint16_t itvl_max;
} bc2_req_connect_t;

/** READ. */
typedef struct {
    uint8_t conn_id;
    uint16_t handle;
} bc2_req_read_t;

/** WRITE and WRITE_CMD. @c data points into the frame payload. */
typedef struct {
    uint8_t conn_id;
    uint16_t handle;
    uint8_t data_len;        /**< 1..BC2_GATT_MAX_VALUE */
    const uint8_t *data;
} bc2_req_write_t;

/** SUBSCRIBE. */
typedef struct {
    uint8_t conn_id;
    uint16_t cccd_handle;
    uint8_t mode;            /**< BC2_SUB_* */
} bc2_req_subscribe_t;

/** SET_FAILSAFE. @c data points into the frame payload. */
typedef struct {
    uint8_t conn_id;
    uint8_t slot;            /**< 0..BC2_GATT_FAILSAFE_SLOTS-1 */
    uint16_t handle;
    uint8_t data_len;        /**< 0 clears the slot */
    const uint8_t *data;
} bc2_req_set_failsafe_t;

/**
 * Decoders: length first (BAD_LENGTH), then values (BAD_PARAM), then size (PAYLOAD_TOO_LARGE).
 * They only check what is visible in the frame; limits that depend on the device state (conn_id
 * against max_conns, MTU, link state) are checked by the core afterwards.
 */
bc2_status_t bc2_decode_scan_start(const bc2_frame_t *f, bc2_req_scan_start_t *out);
bc2_status_t bc2_decode_connect(const bc2_frame_t *f, bc2_req_connect_t *out);
/** DISCONNECT and DISCOVER: a single conn_id byte (range checked). */
bc2_status_t bc2_decode_conn_id(const bc2_frame_t *f, uint8_t *conn_id);
bc2_status_t bc2_decode_read(const bc2_frame_t *f, bc2_req_read_t *out);
bc2_status_t bc2_decode_write(const bc2_frame_t *f, bc2_req_write_t *out);
bc2_status_t bc2_decode_subscribe(const bc2_frame_t *f, bc2_req_subscribe_t *out);
bc2_status_t bc2_decode_set_failsafe(const bc2_frame_t *f, bc2_req_set_failsafe_t *out);

/* ------------------------------------------------------------------------- */
/* Responses and events                                                       */
/* ------------------------------------------------------------------------- */

/** Content of GATT_STATUS_RSP. */
typedef struct {
    uint8_t scanning;
    uint16_t connecting_mask;    /**< bit n = conn_id n waits for / runs a connect attempt */
    uint16_t connected_mask;     /**< bit n = conn_id n is connected */
    uint16_t dropped_scan;       /**< total dropped scan reports since HELLO (wraps) */
    uint16_t dropped_notify;     /**< total dropped notifications since HELLO (wraps) */
    uint32_t free_heap;          /**< current free heap in bytes (0 = unknown), for the connection spike */
    uint32_t min_free_heap;      /**< lowest free heap since boot in bytes (0 = unknown) */
} bc2_gatt_status_t;

/** READ_RSP. @return false if @p len > BC2_GATT_MAX_VALUE. */
bool bc2_encode_read_rsp(bc2_frame_t *out, uint8_t seq, uint8_t conn_id, uint16_t handle,
                         uint8_t flags, const uint8_t *data, uint8_t len);
void bc2_encode_gatt_status_rsp(bc2_frame_t *out, uint8_t seq, const bc2_gatt_status_t *st);
/** EVT_NOTIFY. @return false if @p len > BC2_GATT_MAX_VALUE. */
bool bc2_encode_evt_notify(bc2_frame_t *out, uint8_t conn_id, uint16_t handle, uint8_t flags,
                           const uint8_t *data, uint8_t len);
void bc2_encode_evt_connected(bc2_frame_t *out, uint8_t conn_id, uint16_t mtu, uint16_t itvl,
                              const uint8_t addr[6]);
void bc2_encode_evt_disconnected(bc2_frame_t *out, uint8_t conn_id, uint8_t phase, uint8_t reason);
/** EVT_SCAN_REPORT. @return false if @p len > BC2_GATT_MAX_SCAN_DATA. */
bool bc2_encode_evt_scan_report(bc2_frame_t *out, const uint8_t addr[6], uint8_t addr_type,
                                int8_t rssi, uint8_t flags, const uint8_t *data, uint8_t len);
/** EVT_GATT_SERVICE. @p uuid_len must be 2 or 16. */
bool bc2_encode_evt_gatt_service(bc2_frame_t *out, uint8_t conn_id, uint16_t start, uint16_t end,
                                 const uint8_t *uuid, uint8_t uuid_len);
/** EVT_GATT_CHAR. @p uuid_len must be 2 or 16. */
bool bc2_encode_evt_gatt_char(bc2_frame_t *out, uint8_t conn_id, uint16_t value_handle,
                              uint16_t cccd_handle, uint8_t props, const uint8_t *uuid,
                              uint8_t uuid_len);
void bc2_encode_evt_discover_done(bc2_frame_t *out, uint8_t conn_id, uint8_t status,
                                  uint8_t n_services, uint8_t n_chars);
void bc2_encode_evt_scan_stopped(bc2_frame_t *out, uint8_t reason, uint16_t rc);
void bc2_encode_evt_dropped(bc2_frame_t *out, uint8_t kind, uint16_t count);

#ifdef __cplusplus
}
#endif

#endif /* BC2_GATT_PROTOCOL_H */
