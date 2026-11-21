/**
 * @file bc2_gatt_codec.c
 * @brief Decoding of the GATT requests and encoding of the GATT responses/events of protocol 1.1.
 *
 * All integers are little-endian. Payload layouts are documented in docs/protocol.md section 14-16.
 */
#include <string.h>

#include "bc2_gatt_protocol.h"

/* ---- little-endian helpers ---- */

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    put_u16(p, (uint16_t)(v & 0xFFFFu));
    put_u16(p + 2, (uint16_t)(v >> 16));
}

/** Initialise an outgoing frame header. */
static void frame_begin(bc2_frame_t *f, uint8_t type, uint8_t seq)
{
    f->type = type;
    f->seq = seq;
    f->len = 0;
}

bool bc2_is_gatt_request(uint8_t type)
{
    return type >= BC2_MSG_SCAN_START && type <= BC2_MSG_SET_FAILSAFE;
}

/* ---- request decoding ---- */

/** True if (interval, window) is a valid scan timing pair (both in 0.625 ms units). */
static bool scan_timing_valid(uint16_t interval, uint16_t window)
{
    return interval >= BC2_SCAN_MIN_INTERVAL && interval <= BC2_SCAN_MAX_INTERVAL &&
           window >= BC2_SCAN_MIN_INTERVAL && window <= BC2_SCAN_MAX_INTERVAL && window <= interval;
}

bc2_status_t bc2_decode_scan_start(const bc2_frame_t *f, bc2_req_scan_start_t *out)
{
    if (f->len != 7) {
        return BC2_ERR_BAD_LENGTH;
    }
    out->flags = f->payload[0];
    out->interval = get_u16(&f->payload[1]);
    out->window = get_u16(&f->payload[3]);
    out->refresh_ms = get_u16(&f->payload[5]);
    if ((out->flags & BC2_SCAN_FLAG_RESERVED) != 0 || out->refresh_ms > BC2_SCAN_MAX_REFRESH_MS) {
        return BC2_ERR_BAD_PARAM;
    }
    if (out->interval == 0 && out->window == 0) {          /* 0/0 = firmware defaults */
        out->interval = BC2_SCAN_DEFAULT_INTERVAL;
        out->window = BC2_SCAN_DEFAULT_WINDOW;
    } else if (!scan_timing_valid(out->interval, out->window)) {
        return BC2_ERR_BAD_PARAM;
    }
    if (out->refresh_ms == 0) {
        out->refresh_ms = BC2_SCAN_DEFAULT_REFRESH_MS;
    }
    return BC2_OK;
}

bc2_status_t bc2_decode_connect(const bc2_frame_t *f, bc2_req_connect_t *out)
{
    if (f->len != 15) {
        return BC2_ERR_BAD_LENGTH;
    }
    const uint8_t *p = f->payload;
    out->conn_id = p[0];
    memcpy(out->addr, &p[1], 6);
    out->addr_type = p[7];
    out->flags = p[8];
    out->timeout_ms = get_u16(&p[9]);
    out->itvl_min = get_u16(&p[11]);
    out->itvl_max = get_u16(&p[13]);
    if (out->conn_id > BC2_GATT_MAX_CONN_ID || (out->flags & BC2_CONNECT_FLAG_RESERVED) != 0 ||
        (out->addr_type > 1 && out->addr_type != BC2_ADDR_TYPE_AUTO)) {
        return BC2_ERR_BAD_PARAM;
    }
    if (out->itvl_min == 0 && out->itvl_max == 0) {        /* 0/0 = firmware defaults */
        out->itvl_min = BC2_CONNECT_DEFAULT_ITVL_MIN;
        out->itvl_max = BC2_CONNECT_DEFAULT_ITVL_MAX;
    } else if (out->itvl_min < BC2_CONNECT_ITVL_LOWEST || out->itvl_max > BC2_CONNECT_ITVL_HIGHEST ||
               out->itvl_min > out->itvl_max) {
        return BC2_ERR_BAD_PARAM;
    }
    if (out->flags & BC2_CONNECT_FLAG_BACKGROUND) {        /* background connects have no timeout */
        out->timeout_ms = 0;
    } else if (out->timeout_ms == 0) {
        out->timeout_ms = BC2_CONNECT_DEFAULT_TIMEOUT_MS;
    }
    return BC2_OK;
}

bc2_status_t bc2_decode_conn_id(const bc2_frame_t *f, uint8_t *conn_id)
{
    if (f->len != 1) {
        return BC2_ERR_BAD_LENGTH;
    }
    *conn_id = f->payload[0];
    return *conn_id > BC2_GATT_MAX_CONN_ID ? BC2_ERR_BAD_PARAM : BC2_OK;
}

bc2_status_t bc2_decode_read(const bc2_frame_t *f, bc2_req_read_t *out)
{
    if (f->len != 3) {
        return BC2_ERR_BAD_LENGTH;
    }
    out->conn_id = f->payload[0];
    out->handle = get_u16(&f->payload[1]);
    /* ATT handle 0 is invalid by definition. */
    return (out->conn_id > BC2_GATT_MAX_CONN_ID || out->handle == 0) ? BC2_ERR_BAD_PARAM : BC2_OK;
}

bc2_status_t bc2_decode_write(const bc2_frame_t *f, bc2_req_write_t *out)
{
    if (f->len < 4) {                                      /* header 3 bytes + at least 1 data byte */
        return BC2_ERR_BAD_LENGTH;
    }
    out->conn_id = f->payload[0];
    out->handle = get_u16(&f->payload[1]);
    out->data_len = (uint8_t)(f->len - 3u);
    out->data = &f->payload[3];
    if (out->conn_id > BC2_GATT_MAX_CONN_ID || out->handle == 0) {
        return BC2_ERR_BAD_PARAM;
    }
    return out->data_len > BC2_GATT_MAX_VALUE ? BC2_ERR_PAYLOAD_TOO_LARGE : BC2_OK;
}

bc2_status_t bc2_decode_subscribe(const bc2_frame_t *f, bc2_req_subscribe_t *out)
{
    if (f->len != 4) {
        return BC2_ERR_BAD_LENGTH;
    }
    out->conn_id = f->payload[0];
    out->cccd_handle = get_u16(&f->payload[1]);
    out->mode = f->payload[3];
    if (out->conn_id > BC2_GATT_MAX_CONN_ID || out->cccd_handle == 0 || out->mode > BC2_SUB_INDICATE) {
        return BC2_ERR_BAD_PARAM;
    }
    return BC2_OK;
}

bc2_status_t bc2_decode_set_failsafe(const bc2_frame_t *f, bc2_req_set_failsafe_t *out)
{
    if (f->len < 4) {                                      /* conn, slot, handle(2), data 0..20 */
        return BC2_ERR_BAD_LENGTH;
    }
    out->conn_id = f->payload[0];
    out->slot = f->payload[1];
    out->handle = get_u16(&f->payload[2]);
    out->data_len = (uint8_t)(f->len - 4u);
    out->data = &f->payload[4];
    if (out->conn_id > BC2_GATT_MAX_CONN_ID || out->slot >= BC2_GATT_FAILSAFE_SLOTS ||
        (out->data_len > 0 && out->handle == 0)) {
        return BC2_ERR_BAD_PARAM;
    }
    return out->data_len > BC2_GATT_FAILSAFE_MAX_DATA ? BC2_ERR_PAYLOAD_TOO_LARGE : BC2_OK;
}

/* ---- response and event encoding ---- */

bool bc2_encode_read_rsp(bc2_frame_t *out, uint8_t seq, uint8_t conn_id, uint16_t handle,
                         uint8_t flags, const uint8_t *data, uint8_t len)
{
    if (len > BC2_GATT_MAX_VALUE) {
        return false;
    }
    frame_begin(out, BC2_MSG_READ_RSP, seq);
    out->payload[0] = conn_id;
    put_u16(&out->payload[1], handle);
    out->payload[3] = flags;
    if (len > 0) {
        memcpy(&out->payload[4], data, len);
    }
    out->len = (uint8_t)(4u + len);
    return true;
}

void bc2_encode_gatt_status_rsp(bc2_frame_t *out, uint8_t seq, const bc2_gatt_status_t *st)
{
    frame_begin(out, BC2_MSG_GATT_STATUS_RSP, seq);
    out->payload[0] = st->scanning;
    put_u16(&out->payload[1], st->connecting_mask);
    put_u16(&out->payload[3], st->connected_mask);
    put_u16(&out->payload[5], st->dropped_scan);
    put_u16(&out->payload[7], st->dropped_notify);
    put_u32(&out->payload[9], st->free_heap);
    put_u32(&out->payload[13], st->min_free_heap);
    out->len = 17;
}

bool bc2_encode_evt_notify(bc2_frame_t *out, uint8_t conn_id, uint16_t handle, uint8_t flags,
                           const uint8_t *data, uint8_t len)
{
    if (len > BC2_GATT_MAX_VALUE) {
        return false;
    }
    frame_begin(out, BC2_MSG_EVT_NOTIFY, 0);               /* events always use SEQ 0 */
    out->payload[0] = conn_id;
    put_u16(&out->payload[1], handle);
    out->payload[3] = flags;
    if (len > 0) {
        memcpy(&out->payload[4], data, len);
    }
    out->len = (uint8_t)(4u + len);
    return true;
}

void bc2_encode_evt_connected(bc2_frame_t *out, uint8_t conn_id, uint16_t mtu, uint16_t itvl,
                              const uint8_t addr[6])
{
    frame_begin(out, BC2_MSG_EVT_CONNECTED, 0);
    out->payload[0] = conn_id;
    put_u16(&out->payload[1], mtu);
    put_u16(&out->payload[3], itvl);
    memcpy(&out->payload[5], addr, 6);
    out->len = 11;
}

void bc2_encode_evt_disconnected(bc2_frame_t *out, uint8_t conn_id, uint8_t phase, uint8_t reason)
{
    frame_begin(out, BC2_MSG_EVT_DISCONNECTED, 0);
    out->payload[0] = conn_id;
    out->payload[1] = phase;
    out->payload[2] = reason;
    out->len = 3;
}

bool bc2_encode_evt_scan_report(bc2_frame_t *out, const uint8_t addr[6], uint8_t addr_type,
                                int8_t rssi, uint8_t flags, const uint8_t *data, uint8_t len)
{
    if (len > BC2_GATT_MAX_SCAN_DATA) {
        return false;
    }
    frame_begin(out, BC2_MSG_EVT_SCAN_REPORT, 0);
    memcpy(out->payload, addr, 6);
    out->payload[6] = addr_type;
    out->payload[7] = (uint8_t)rssi;
    out->payload[8] = flags;
    out->payload[9] = len;
    if (len > 0) {
        memcpy(&out->payload[10], data, len);
    }
    out->len = (uint8_t)(10u + len);
    return true;
}

bool bc2_encode_evt_gatt_service(bc2_frame_t *out, uint8_t conn_id, uint16_t start, uint16_t end,
                                 const uint8_t *uuid, uint8_t uuid_len)
{
    if (uuid_len != 2 && uuid_len != 16) {
        return false;
    }
    frame_begin(out, BC2_MSG_EVT_GATT_SERVICE, 0);
    out->payload[0] = conn_id;
    put_u16(&out->payload[1], start);
    put_u16(&out->payload[3], end);
    out->payload[5] = uuid_len;
    memcpy(&out->payload[6], uuid, uuid_len);
    out->len = (uint8_t)(6u + uuid_len);
    return true;
}

bool bc2_encode_evt_gatt_char(bc2_frame_t *out, uint8_t conn_id, uint16_t value_handle,
                              uint16_t cccd_handle, uint8_t props, const uint8_t *uuid,
                              uint8_t uuid_len)
{
    if (uuid_len != 2 && uuid_len != 16) {
        return false;
    }
    frame_begin(out, BC2_MSG_EVT_GATT_CHAR, 0);
    out->payload[0] = conn_id;
    put_u16(&out->payload[1], value_handle);
    put_u16(&out->payload[3], cccd_handle);
    out->payload[5] = props;
    out->payload[6] = uuid_len;
    memcpy(&out->payload[7], uuid, uuid_len);
    out->len = (uint8_t)(7u + uuid_len);
    return true;
}

void bc2_encode_evt_discover_done(bc2_frame_t *out, uint8_t conn_id, uint8_t status,
                                  uint8_t n_services, uint8_t n_chars)
{
    frame_begin(out, BC2_MSG_EVT_DISCOVER_DONE, 0);
    out->payload[0] = conn_id;
    out->payload[1] = status;
    out->payload[2] = n_services;
    out->payload[3] = n_chars;
    out->len = 4;
}

void bc2_encode_evt_scan_stopped(bc2_frame_t *out, uint8_t reason, uint16_t rc)
{
    frame_begin(out, BC2_MSG_EVT_SCAN_STOPPED, 0);
    out->payload[0] = reason;
    put_u16(&out->payload[1], rc);
    out->len = 3;
}

void bc2_encode_evt_dropped(bc2_frame_t *out, uint8_t kind, uint16_t count)
{
    frame_begin(out, BC2_MSG_EVT_DROPPED, 0);
    out->payload[0] = kind;
    put_u16(&out->payload[1], count);
    out->len = 3;
}
