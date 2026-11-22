/**
 * @file bc2_protocol.h
 * @brief Wire protocol of the BC2 USB BLE advertiser: constants, framing, message codec.
 *
 * Pure C11, no ESP-IDF includes, so it can be unit tested on a Linux host.
 * The authoritative description of the protocol is docs/protocol.md; keep both in sync.
 */
#ifndef BC2_PROTOCOL_H
#define BC2_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* Protocol and firmware version                                              */
/* ------------------------------------------------------------------------- */

#define BC2_PROTO_MAJOR 1      /**< Incompatible protocol changes bump this. */
#define BC2_PROTO_MINOR 1      /**< Additive protocol changes bump this (1.1 = GATT central, docs/protocol.md 13+). */
#define BC2_FW_MAJOR 0         /**< Firmware version reported in HELLO_RSP. */
#define BC2_FW_MINOR 2
#define BC2_FW_PATCH 2

/* ------------------------------------------------------------------------- */
/* Framing constants                                                          */
/* ------------------------------------------------------------------------- */

#define BC2_SOF 0xB2u               /**< Start-of-frame marker byte. */
#define BC2_MIN_LEN 2u              /**< Smallest LEN value (TYPE + SEQ, empty payload). */
#define BC2_MAX_LEN 62u             /**< Largest LEN value (TYPE + SEQ + 60 payload bytes). */
#define BC2_MAX_PAYLOAD 60u         /**< Largest payload of one frame. */
#define BC2_MAX_FRAME 66u           /**< SOF + LEN + 62 + CRC(2). */
#define BC2_FRAME_TIMEOUT_MS 200u   /**< A frame must complete within this time after its first byte. */

/* ------------------------------------------------------------------------- */
/* Limits shared with the host                                                */
/* ------------------------------------------------------------------------- */

#define BC2_MAX_DATA_WITH_FLAGS 24u     /**< Max raw bytes per set when the AD flags field is added. */
#define BC2_MAX_DATA_NO_FLAGS 27u       /**< Max raw bytes per set without AD flags field. */
#define BC2_MIN_INTERVAL 32u            /**< Shortest advertising interval, 0.625 ms units (20 ms). */
#define BC2_MAX_INTERVAL 16384u         /**< Longest advertising interval, 0.625 ms units (10.24 s). */
#define BC2_TX_POWER_MAX 127            /**< tx_power value meaning "maximum of the controller". */
#define BC2_KEEPALIVE_MIN_MS 200u       /**< Lower clamp of the keep-alive timeout. */
#define BC2_KEEPALIVE_MAX_MS 10000u     /**< Upper clamp of the keep-alive timeout. */
#define BC2_KEEPALIVE_DEFAULT_MS 1000u  /**< Used when the host sends 0. */
#define BC2_SET_ID_SUMMARY 0xFFu        /**< GET_STATUS set_id value requesting the device summary. */
#define BC2_MAGIC "BC2ADV"              /**< 6 byte magic at the start of HELLO_RSP. */
#define BC2_MAGIC_LEN 6u
#define BC2_HELLO_RSP_LEN 32u           /**< Payload length of HELLO_RSP (27 bytes in protocol 1.0). */

/** SET_START flag bits. */
#define BC2_FLAG_ADD_AD_FLAGS 0x01u     /**< bit0: add AD flags field 02 01 06. */
#define BC2_FLAG_PDU_SHIFT 1u           /**< bits1-2: PDU type. */
#define BC2_FLAG_PDU_MASK 0x06u
#define BC2_FLAG_RESERVED_MASK 0xF8u    /**< bits3-7 must be zero. */

/** Advertising PDU types (value of the PDU bits in the flags byte). */
typedef enum {
    BC2_PDU_ADV_IND = 0,         /**< connectable + scannable (default, like Android). */
    BC2_PDU_ADV_SCAN_IND = 1,    /**< scannable only. */
    BC2_PDU_ADV_NONCONN_IND = 2  /**< neither connectable nor scannable. */
} bc2_pdu_type_t;

/* ------------------------------------------------------------------------- */
/* Message types and status codes                                             */
/* ------------------------------------------------------------------------- */

/** Message TYPE byte. Ranges are documented in docs/protocol.md section 3. */
typedef enum {
    BC2_MSG_HELLO = 0x01,
    BC2_MSG_PING = 0x02,
    BC2_MSG_SET_START = 0x10,
    BC2_MSG_SET_UPDATE = 0x11,
    BC2_MSG_SET_STOP = 0x12,
    BC2_MSG_STOP_ALL = 0x13,
    BC2_MSG_GET_STATUS = 0x14,
    BC2_MSG_ACK = 0x80,
    BC2_MSG_HELLO_RSP = 0x81,
    BC2_MSG_PONG = 0x82,
    BC2_MSG_SET_STATUS_RSP = 0x83,
    BC2_MSG_EVT_SET_STOPPED = 0xE0,
    BC2_MSG_EVT_BUTTON = 0xE1,
    BC2_MSG_EVT_TIMEOUT = 0xE2
} bc2_msg_type_t;

/**
 * TYPE ranges of the GATT messages (protocol 1.1, see bc2_gatt_protocol.h) and the part of the
 * ranges that is still reserved. A device answers every type it does not implement, including the
 * reserved ones and all device-to-host types, with ACK(UNKNOWN_CMD).
 */
#define BC2_RESERVED_GATT_REQ_FIRST 0x3Bu   /**< host->device, still reserved (0x30-0x3A are used). */
#define BC2_RESERVED_GATT_REQ_LAST 0x4Fu
#define BC2_RESERVED_GATT_RSP_FIRST 0xA2u   /**< device->host responses, still reserved (0xA0-0xA1 used). */
#define BC2_RESERVED_GATT_RSP_LAST 0xBFu
#define BC2_RESERVED_GATT_EVT_FIRST 0xC9u   /**< device->host events, still reserved (0xC0-0xC8 used). */
#define BC2_RESERVED_GATT_EVT_LAST 0xDFu

/** Status code carried in ACK frames. */
typedef enum {
    BC2_OK = 0,
    BC2_ERR_UNKNOWN_CMD = 1,
    BC2_ERR_BAD_LENGTH = 2,
    BC2_ERR_BAD_PARAM = 3,
    BC2_ERR_NO_SESSION = 4,
    BC2_ERR_VERSION_UNSUPPORTED = 5,
    BC2_ERR_ALREADY_RUNNING = 6,
    BC2_ERR_SET_NOT_FOUND = 7,
    BC2_ERR_PAYLOAD_TOO_LARGE = 8,
    BC2_ERR_BLE_ERROR = 9,
    BC2_ERR_NO_RESOURCES = 10,
    /* ---- protocol 1.1 (GATT) ---- */
    BC2_ERR_BUSY = 11,            /**< a procedure is already running on the connection / controller queue full */
    BC2_ERR_NOT_CONNECTED = 12,   /**< conn_id has no connected link */
    BC2_ERR_TIMEOUT = 13,         /**< GATT procedure did not finish within 5 s (link is terminated) */
    BC2_ERR_ATT_ERROR = 14,       /**< peer answered with an ATT error, ACK extra = att_error u8 */
    BC2_ERR_DISCONNECTED = 15,    /**< link was lost while the request was pending */
    BC2_ERR_NOT_SUPPORTED = 16    /**< feature not built into this firmware */
} bc2_status_t;

/** Reason byte of EVT_SET_STOPPED. */
typedef enum {
    BC2_STOP_BLE_ERROR = 1,           /**< the controller reported an error for the set */
    BC2_STOP_DISCONNECT_FAILED = 2    /**< connection teardown / restart of the set failed */
} bc2_stop_reason_t;

/* ------------------------------------------------------------------------- */
/* CRC-16/CCITT-FALSE                                                         */
/* ------------------------------------------------------------------------- */

/** Initial value of the CRC. */
#define BC2_CRC_INIT 0xFFFFu

/**
 * Continue a CRC-16/CCITT-FALSE computation (poly 0x1021, no reflection, xorout 0).
 * @param crc  running CRC (BC2_CRC_INIT for the first call)
 * @param data bytes to add
 * @param len  number of bytes
 * @return updated CRC
 */
uint16_t bc2_crc16_update(uint16_t crc, const uint8_t *data, size_t len);

/** CRC of a complete buffer (init 0xFFFF). Check value of "123456789" is 0x29B1. */
uint16_t bc2_crc16(const uint8_t *data, size_t len);

/* ------------------------------------------------------------------------- */
/* Frames and the streaming framer                                            */
/* ------------------------------------------------------------------------- */

/** A decoded frame (without SOF/LEN/CRC). */
typedef struct {
    uint8_t type;                         /**< message TYPE */
    uint8_t seq;                          /**< sequence number chosen by the host */
    uint8_t len;                          /**< payload length 0..BC2_MAX_PAYLOAD */
    uint8_t payload[BC2_MAX_PAYLOAD];     /**< payload bytes */
} bc2_frame_t;

/**
 * Serialize a frame to wire format (SOF, LEN, TYPE, SEQ, PAYLOAD, CRC little-endian).
 * @param frame frame to send
 * @param out   output buffer
 * @param cap   capacity of @p out
 * @return number of bytes written, 0 if the frame is invalid or the buffer too small
 */
size_t bc2_frame_encode(const bc2_frame_t *frame, uint8_t *out, size_t cap);

/**
 * Streaming parser state. Bytes are appended with bc2_framer_push() and complete frames are
 * fetched with bc2_framer_next(); see docs/protocol.md section 2 for the resync rules.
 */
typedef struct {
    uint8_t buf[BC2_MAX_FRAME];   /**< bytes received but not yet consumed, buf[0] is the candidate SOF */
    size_t len;                   /**< number of valid bytes in buf */
    uint32_t start_ms;            /**< time the candidate frame started (for the 200 ms timeout) */
    uint16_t parse_errors;        /**< frames dropped because of bad LEN, bad CRC or timeout (wraps) */
} bc2_framer_t;

/** Reset a framer, including its error counter. */
void bc2_framer_init(bc2_framer_t *f);

/**
 * Append one received byte. If a partial frame has been waiting for more than
 * BC2_FRAME_TIMEOUT_MS it is dropped first (counted as parse error).
 * Call bc2_framer_next() until it returns false after every push.
 * @param now_ms monotonic millisecond clock (wrap-safe)
 */
void bc2_framer_push(bc2_framer_t *f, uint8_t byte, uint32_t now_ms);

/**
 * Try to extract the next complete, CRC-valid frame from the buffered bytes.
 * Garbage before a SOF is skipped silently; bad LEN or bad CRC drops only the SOF byte
 * and rescans, so a valid frame that started inside a corrupted one is still found.
 * @return true and fills @p out if a frame was found
 */
bool bc2_framer_next(bc2_framer_t *f, bc2_frame_t *out);

/* ------------------------------------------------------------------------- */
/* Decoded requests                                                           */
/* ------------------------------------------------------------------------- */

/** HELLO request. */
typedef struct {
    uint8_t proto_major;
    uint8_t proto_minor;
    uint16_t keepalive_ms;   /**< requested value, not yet clamped */
} bc2_req_hello_t;

/** SET_START request. @c data points into the frame payload. */
typedef struct {
    uint8_t set_id;
    uint8_t flags;
    uint16_t interval;       /**< 0.625 ms units */
    int8_t tx_power;         /**< dBm or BC2_TX_POWER_MAX */
    uint16_t mfr_id;
    uint8_t data_len;
    const uint8_t *data;
} bc2_req_set_start_t;

/** SET_UPDATE request. @c data points into the frame payload. */
typedef struct {
    uint8_t set_id;
    uint16_t mfr_id;
    uint8_t data_len;
    const uint8_t *data;
} bc2_req_set_update_t;

/**
 * Decode functions validate length first (BAD_LENGTH), then values (BAD_PARAM),
 * then size limits (PAYLOAD_TOO_LARGE). They return BC2_OK on success.
 */
bc2_status_t bc2_decode_hello(const bc2_frame_t *f, bc2_req_hello_t *out);
bc2_status_t bc2_decode_set_start(const bc2_frame_t *f, bc2_req_set_start_t *out);
bc2_status_t bc2_decode_set_update(const bc2_frame_t *f, bc2_req_set_update_t *out);
/** Requests with a single set_id byte: SET_STOP and GET_STATUS. */
bc2_status_t bc2_decode_set_id(const bc2_frame_t *f, uint8_t *set_id);
/** Requests without payload: PING and STOP_ALL. */
bc2_status_t bc2_decode_empty(const bc2_frame_t *f);

/** Maximum raw data length allowed for a set started with @p flags. */
uint8_t bc2_max_data_len(uint8_t flags);

/** Size of a legacy advertising payload in bytes. */
#define BC2_LEGACY_ADV_MAX 31u

/**
 * Build the advertising data of a set: optional flags field `02 01 06` followed by one
 * manufacturer specific data structure `len FF mfr_lo mfr_hi data...`.
 * @param flags  SET_START flag byte (only the AD flags bit is evaluated)
 * @param mfr_id manufacturer id (little endian on air)
 * @param data   raw manufacturer data
 * @param len    number of raw bytes, must fit bc2_max_data_len(flags)
 * @param out    buffer of at least BC2_LEGACY_ADV_MAX bytes
 * @return bytes written, 0 if @p len is too large
 */
size_t bc2_build_adv_data(uint8_t flags, uint16_t mfr_id, const uint8_t *data, uint8_t len, uint8_t *out);

/* ------------------------------------------------------------------------- */
/* Responses and events                                                       */
/* ------------------------------------------------------------------------- */

/** Content of HELLO_RSP. */
typedef struct {
    uint8_t status;                 /**< always BC2_OK, errors use ACK */
    uint8_t proto_major;
    uint8_t proto_minor;
    uint8_t fw_major;
    uint8_t fw_minor;
    uint8_t fw_patch;
    uint8_t max_sets;
    uint8_t max_data_with_flags;
    uint8_t max_data_no_flags;
    uint16_t min_interval;
    uint16_t max_interval;
    uint16_t keepalive_ms;          /**< effective (clamped) value */
    uint8_t device_id[6];           /**< Bluetooth public MAC */
    uint8_t gatt_flags;             /**< bit0 scan, bit1 GATT central, bit2 failsafe write (0 = no GATT) */
    uint8_t max_conns;              /**< simultaneous GATT links (0 if GATT is not built in) */
    uint8_t max_value_len;          /**< largest GATT value in one frame (56) */
    uint8_t max_scan_data;          /**< largest advertising data in a scan report (31) */
    uint8_t failsafe_slots;         /**< failsafe writes per connection (4, 0 if unsupported) */
} bc2_hello_rsp_t;

/** Content of PONG. */
typedef struct {
    uint8_t state;                  /**< 0 idle, 1 advertising */
    uint16_t active_mask;           /**< bit n set = set n running */
    uint16_t parse_errors;
    uint32_t uptime_ms;
} bc2_pong_t;

/** Content of SET_STATUS_RSP. @c data is copied by the encoder. */
typedef struct {
    uint8_t set_id;
    uint8_t running;
    uint8_t flags;
    uint16_t interval;
    int8_t tx_power;
    uint16_t mfr_id;
    uint8_t data_len;
    const uint8_t *data;
} bc2_set_status_t;

/** Build ACK(orig_type, status, extra). @return false if @p extra_len is too large. */
bool bc2_encode_ack(bc2_frame_t *out, uint8_t seq, uint8_t orig_type, uint8_t status,
                    const uint8_t *extra, uint8_t extra_len);
void bc2_encode_hello_rsp(bc2_frame_t *out, uint8_t seq, const bc2_hello_rsp_t *rsp);
void bc2_encode_pong(bc2_frame_t *out, uint8_t seq, const bc2_pong_t *pong);
bool bc2_encode_set_status(bc2_frame_t *out, uint8_t seq, const bc2_set_status_t *st);
void bc2_encode_evt_set_stopped(bc2_frame_t *out, uint8_t set_id, uint8_t reason);
void bc2_encode_evt_button(bc2_frame_t *out);
void bc2_encode_evt_timeout(bc2_frame_t *out, uint16_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* BC2_PROTOCOL_H */
