/**
 * @file bc2_codec.c
 * @brief Decoding of requests and encoding of responses/events (all integers little-endian).
 */
#include <string.h>

#include "bc2_protocol.h"

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

uint8_t bc2_max_data_len(uint8_t flags)
{
    return (flags & BC2_FLAG_ADD_AD_FLAGS) ? BC2_MAX_DATA_WITH_FLAGS : BC2_MAX_DATA_NO_FLAGS;
}

size_t bc2_build_adv_data(uint8_t flags, uint16_t mfr_id, const uint8_t *data, uint8_t len, uint8_t *out)
{
    if (len > bc2_max_data_len(flags)) {
        return 0;
    }
    size_t n = 0;
    if (flags & BC2_FLAG_ADD_AD_FLAGS) {
        out[n++] = 0x02;      /* AD structure length */
        out[n++] = 0x01;      /* AD type: flags */
        out[n++] = 0x06;      /* LE general discoverable, BR/EDR not supported (as Android adds) */
    }
    out[n++] = (uint8_t)(len + 3u);              /* AD length: type + 2 byte company id + data */
    out[n++] = 0xFF;                             /* AD type: manufacturer specific data */
    out[n++] = (uint8_t)(mfr_id & 0xFFu);
    out[n++] = (uint8_t)(mfr_id >> 8);
    if (len > 0) {
        memcpy(&out[n], data, len);
    }
    return n + len;
}

/* ---- request decoding ---- */

bc2_status_t bc2_decode_empty(const bc2_frame_t *f)
{
    return f->len == 0 ? BC2_OK : BC2_ERR_BAD_LENGTH;
}

bc2_status_t bc2_decode_set_id(const bc2_frame_t *f, uint8_t *set_id)
{
    if (f->len != 1) {
        return BC2_ERR_BAD_LENGTH;
    }
    *set_id = f->payload[0];
    return BC2_OK;
}

bc2_status_t bc2_decode_hello(const bc2_frame_t *f, bc2_req_hello_t *out)
{
    if (f->len != 4) {
        return BC2_ERR_BAD_LENGTH;
    }
    out->proto_major = f->payload[0];
    out->proto_minor = f->payload[1];
    out->keepalive_ms = get_u16(&f->payload[2]);
    return BC2_OK;
}

/** Fixed part of SET_START before the data bytes: id, flags, interval(2), tx, mfr(2), data_len. */
#define SET_START_FIXED 8u
/** Fixed part of SET_UPDATE before the data bytes: id, mfr(2), data_len. */
#define SET_UPDATE_FIXED 4u

bc2_status_t bc2_decode_set_start(const bc2_frame_t *f, bc2_req_set_start_t *out)
{
    if (f->len < SET_START_FIXED) {
        return BC2_ERR_BAD_LENGTH;
    }
    const uint8_t data_len = f->payload[7];
    if (f->len != SET_START_FIXED + data_len) {            /* data_len must match the frame exactly */
        return BC2_ERR_BAD_LENGTH;
    }
    out->set_id = f->payload[0];
    out->flags = f->payload[1];
    out->interval = get_u16(&f->payload[2]);
    out->tx_power = (int8_t)f->payload[4];
    out->mfr_id = get_u16(&f->payload[5]);
    out->data_len = data_len;
    out->data = &f->payload[SET_START_FIXED];

    if ((out->flags & BC2_FLAG_RESERVED_MASK) != 0 ||
        ((out->flags & BC2_FLAG_PDU_MASK) >> BC2_FLAG_PDU_SHIFT) > BC2_PDU_ADV_NONCONN_IND ||
        out->interval < BC2_MIN_INTERVAL || out->interval > BC2_MAX_INTERVAL) {
        return BC2_ERR_BAD_PARAM;
    }
    if (data_len > bc2_max_data_len(out->flags)) {
        return BC2_ERR_PAYLOAD_TOO_LARGE;
    }
    return BC2_OK;
}

bc2_status_t bc2_decode_set_update(const bc2_frame_t *f, bc2_req_set_update_t *out)
{
    if (f->len < SET_UPDATE_FIXED) {
        return BC2_ERR_BAD_LENGTH;
    }
    const uint8_t data_len = f->payload[3];
    if (f->len != SET_UPDATE_FIXED + data_len) {
        return BC2_ERR_BAD_LENGTH;
    }
    out->set_id = f->payload[0];
    out->mfr_id = get_u16(&f->payload[1]);
    out->data_len = data_len;
    out->data = &f->payload[SET_UPDATE_FIXED];
    /* The flags of the running set are unknown here: only the absolute maximum is checked;
     * bc2_core applies the stricter limit when the set was started with AD flags. */
    return data_len > BC2_MAX_DATA_NO_FLAGS ? BC2_ERR_PAYLOAD_TOO_LARGE : BC2_OK;
}

/* ---- response encoding ---- */

bool bc2_encode_ack(bc2_frame_t *out, uint8_t seq, uint8_t orig_type, uint8_t status,
                    const uint8_t *extra, uint8_t extra_len)
{
    if (extra_len > BC2_MAX_PAYLOAD - 2u) {
        return false;
    }
    frame_begin(out, BC2_MSG_ACK, seq);
    out->payload[0] = orig_type;
    out->payload[1] = status;
    if (extra_len > 0) {
        memcpy(&out->payload[2], extra, extra_len);
    }
    out->len = (uint8_t)(2u + extra_len);
    return true;
}

void bc2_encode_hello_rsp(bc2_frame_t *out, uint8_t seq, const bc2_hello_rsp_t *rsp)
{
    frame_begin(out, BC2_MSG_HELLO_RSP, seq);
    uint8_t *p = out->payload;
    memcpy(p, BC2_MAGIC, BC2_MAGIC_LEN);                   /* identifies this firmware on any COM port */
    p[6] = rsp->status;
    p[7] = rsp->proto_major;
    p[8] = rsp->proto_minor;
    p[9] = rsp->fw_major;
    p[10] = rsp->fw_minor;
    p[11] = rsp->fw_patch;
    p[12] = rsp->max_sets;
    p[13] = rsp->max_data_with_flags;
    p[14] = rsp->max_data_no_flags;
    put_u16(&p[15], rsp->min_interval);
    put_u16(&p[17], rsp->max_interval);
    put_u16(&p[19], rsp->keepalive_ms);
    memcpy(&p[21], rsp->device_id, 6);
    out->len = BC2_HELLO_RSP_LEN;
}

void bc2_encode_pong(bc2_frame_t *out, uint8_t seq, const bc2_pong_t *pong)
{
    frame_begin(out, BC2_MSG_PONG, seq);
    out->payload[0] = pong->state;
    put_u16(&out->payload[1], pong->active_mask);
    put_u16(&out->payload[3], pong->parse_errors);
    put_u32(&out->payload[5], pong->uptime_ms);
    out->len = 9;
}

bool bc2_encode_set_status(bc2_frame_t *out, uint8_t seq, const bc2_set_status_t *st)
{
    if (st->data_len > BC2_MAX_DATA_NO_FLAGS) {
        return false;
    }
    frame_begin(out, BC2_MSG_SET_STATUS_RSP, seq);
    uint8_t *p = out->payload;
    p[0] = st->set_id;
    p[1] = st->running;
    p[2] = st->flags;
    put_u16(&p[3], st->interval);
    p[5] = (uint8_t)st->tx_power;
    put_u16(&p[6], st->mfr_id);
    p[8] = st->data_len;
    if (st->data_len > 0) {
        memcpy(&p[9], st->data, st->data_len);
    }
    out->len = (uint8_t)(9u + st->data_len);
    return true;
}

void bc2_encode_evt_set_stopped(bc2_frame_t *out, uint8_t set_id, uint8_t reason)
{
    frame_begin(out, BC2_MSG_EVT_SET_STOPPED, 0);          /* events always use SEQ 0 */
    out->payload[0] = set_id;
    out->payload[1] = reason;
    out->len = 2;
}

void bc2_encode_evt_button(bc2_frame_t *out)
{
    frame_begin(out, BC2_MSG_EVT_BUTTON, 0);
}

void bc2_encode_evt_timeout(bc2_frame_t *out, uint16_t timeout_ms)
{
    frame_begin(out, BC2_MSG_EVT_TIMEOUT, 0);
    put_u16(out->payload, timeout_ms);
    out->len = 2;
}
