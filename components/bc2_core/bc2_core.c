/**
 * @file bc2_core.c
 * @brief Implementation of the session state machine, see bc2_core.h and docs/protocol.md.
 *
 * Request validation order: unknown TYPE -> (HELLO) -> session check -> payload length ->
 * parameter values -> state (set running / not running) -> size limit of the set.
 */
#include <string.h>

#include "bc2_core.h"

/* ------------------------------------------------------------------------- */
/* Small helpers                                                              */
/* ------------------------------------------------------------------------- */

/** True while @p now_ms is before @p until_ms (wrap-safe for intervals < 2^31 ms). */
static bool before(uint32_t now_ms, uint32_t until_ms)
{
    return (int32_t)(until_ms - now_ms) > 0;
}

/** Number of usable sets: BLE adapter value capped by the table capacity. */
static uint8_t max_sets(const bc2_core_t *core)
{
    const uint8_t n = core->ble.max_sets(core->ble.ctx);
    return n > BC2_CORE_MAX_SETS ? (uint8_t)BC2_CORE_MAX_SETS : n;
}

/** Serialize and send a frame; transport errors are ignored (safety never depends on the host). */
static void send_frame(bc2_core_t *core, const bc2_frame_t *frame)
{
    uint8_t wire[BC2_MAX_FRAME];
    const size_t n = bc2_frame_encode(frame, wire, sizeof(wire));
    if (n > 0) {
        core->io.write(core->io.ctx, wire, n);
    }
}

/** Send ACK(orig_type, status, extra). */
static void send_ack(bc2_core_t *core, uint8_t seq, uint8_t orig_type, uint8_t status,
                     const uint8_t *extra, uint8_t extra_len)
{
    bc2_frame_t out;
    if (bc2_encode_ack(&out, seq, orig_type, status, extra, extra_len)) {
        send_frame(core, &out);
    }
}

/** Make the LED show ERROR for BC2_ERROR_DISPLAY_MS. */
static void flag_error(bc2_core_t *core, uint32_t now_ms)
{
    core->error_active = true;
    core->error_until_ms = now_ms + BC2_ERROR_DISPLAY_MS;
}

/** Bitmask of running sets (bit n = set n). */
static uint16_t active_mask(const bc2_core_t *core)
{
    uint16_t mask = 0;
    for (uint8_t i = 0; i < BC2_CORE_MAX_SETS; i++) {
        if (core->slots[i].running) {
            mask = (uint16_t)(mask | (1u << i));
        }
    }
    return mask;
}

/**
 * Stop one running set. A failed stop is retried once; if the controller may still be
 * advertising the chip is reset, because silence is more important than the session.
 */
static void stop_set(bc2_core_t *core, uint8_t set_id)
{
    if (!core->ble.stop(core->ble.ctx, set_id) && !core->ble.stop(core->ble.ctx, set_id)) {
        core->io.hard_reset(core->io.ctx);
    }
    core->slots[set_id].running = false;
}

/** Stop every running set. */
static void stop_all(bc2_core_t *core)
{
    for (uint8_t i = 0; i < BC2_CORE_MAX_SETS; i++) {
        if (core->slots[i].running) {
            stop_set(core, i);
        }
    }
}

/** True for the request types a host may send (everything else is UNKNOWN_CMD). */
static bool is_known_request(uint8_t type)
{
    switch (type) {
    case BC2_MSG_HELLO:
    case BC2_MSG_PING:
    case BC2_MSG_SET_START:
    case BC2_MSG_SET_UPDATE:
    case BC2_MSG_SET_STOP:
    case BC2_MSG_STOP_ALL:
    case BC2_MSG_GET_STATUS:
        return true;
    default:
        return false;   /* includes the reserved GATT ranges and all device->host types */
    }
}

/* ------------------------------------------------------------------------- */
/* Request handlers                                                           */
/* ------------------------------------------------------------------------- */

/** Clamp the requested keep-alive timeout (0 = default). */
static uint16_t effective_keepalive(uint16_t requested)
{
    if (requested == 0) {
        return BC2_KEEPALIVE_DEFAULT_MS;
    }
    if (requested < BC2_KEEPALIVE_MIN_MS) {
        return BC2_KEEPALIVE_MIN_MS;
    }
    return requested > BC2_KEEPALIVE_MAX_MS ? (uint16_t)BC2_KEEPALIVE_MAX_MS : requested;
}

/** HELLO: version check, reset everything, open the session, answer with the capabilities. */
static void handle_hello(bc2_core_t *core, const bc2_frame_t *fr)
{
    bc2_req_hello_t req;
    const bc2_status_t st = bc2_decode_hello(fr, &req);
    if (st != BC2_OK) {
        send_ack(core, fr->seq, fr->type, st, NULL, 0);
        return;
    }
    if (req.proto_major != BC2_PROTO_MAJOR) {
        send_ack(core, fr->seq, fr->type, BC2_ERR_VERSION_UNSUPPORTED, NULL, 0);
        return;
    }
    stop_all(core);                                     /* HELLO always starts from silence */
    memset(core->slots, 0, sizeof(core->slots));
    core->framer.parse_errors = 0;
    core->seen_parse_errors = 0;
    core->session = true;
    core->timed_out = false;
    core->keepalive_ms = effective_keepalive(req.keepalive_ms);

    bc2_hello_rsp_t rsp = {.status = BC2_OK, .proto_major = BC2_PROTO_MAJOR,
                           .proto_minor = BC2_PROTO_MINOR, .fw_major = BC2_FW_MAJOR,
                           .fw_minor = BC2_FW_MINOR, .fw_patch = BC2_FW_PATCH,
                           .max_sets = max_sets(core),
                           .max_data_with_flags = BC2_MAX_DATA_WITH_FLAGS,
                           .max_data_no_flags = BC2_MAX_DATA_NO_FLAGS,
                           .min_interval = BC2_MIN_INTERVAL, .max_interval = BC2_MAX_INTERVAL,
                           .keepalive_ms = core->keepalive_ms};
    core->ble.device_id(core->ble.ctx, rsp.device_id);
    bc2_frame_t out;
    bc2_encode_hello_rsp(&out, fr->seq, &rsp);
    send_frame(core, &out);
}

/** Fill a PONG from the current state and send it. */
static void send_pong(bc2_core_t *core, uint8_t seq, uint32_t now_ms)
{
    const uint16_t mask = active_mask(core);
    const bc2_pong_t pong = {.state = mask != 0 ? 1u : 0u, .active_mask = mask,
                             .parse_errors = core->framer.parse_errors, .uptime_ms = now_ms};
    bc2_frame_t out;
    bc2_encode_pong(&out, seq, &pong);
    send_frame(core, &out);
}

/** SET_START: validate, hand to the BLE adapter, record the slot. */
static void handle_set_start(bc2_core_t *core, const bc2_frame_t *fr, uint32_t now_ms)
{
    bc2_req_set_start_t req;
    bc2_status_t st = bc2_decode_set_start(fr, &req);
    if (st == BC2_OK && req.set_id >= max_sets(core)) {
        st = BC2_ERR_BAD_PARAM;
    }
    if (st == BC2_OK && core->slots[req.set_id].running) {
        st = BC2_ERR_ALREADY_RUNNING;
    }
    if (st != BC2_OK) {
        send_ack(core, fr->seq, fr->type, st, NULL, 0);
        return;
    }

    const bc2_set_params_t params = {.flags = req.flags, .interval = req.interval,
                                     .tx_power = req.tx_power, .mfr_id = req.mfr_id,
                                     .data_len = req.data_len, .data = req.data};
    int8_t applied_tx = req.tx_power;
    uint16_t rc = 0;
    st = core->ble.start(core->ble.ctx, req.set_id, &params, &applied_tx, &rc);
    if (st != BC2_OK) {
        if (st == BC2_ERR_BLE_ERROR || st == BC2_ERR_NO_RESOURCES) {
            flag_error(core, now_ms);
        }
        const uint8_t rc_le[2] = {(uint8_t)(rc & 0xFFu), (uint8_t)(rc >> 8)};
        send_ack(core, fr->seq, fr->type, st, rc_le, st == BC2_ERR_BLE_ERROR ? 2u : 0u);
        return;
    }

    bc2_slot_t *slot = &core->slots[req.set_id];
    slot->running = true;
    slot->flags = req.flags;
    slot->interval = req.interval;
    slot->tx_power = applied_tx;
    slot->mfr_id = req.mfr_id;
    slot->data_len = req.data_len;
    memcpy(slot->data, req.data, req.data_len);
    const uint8_t tx = (uint8_t)applied_tx;
    send_ack(core, fr->seq, fr->type, BC2_OK, &tx, 1);
}

/** SET_UPDATE: in-place data change of a running set; ACK carries the processing time. */
static void handle_set_update(bc2_core_t *core, const bc2_frame_t *fr, uint32_t now_ms, uint32_t t0_us)
{
    bc2_req_set_update_t req;
    bc2_status_t st = bc2_decode_set_update(fr, &req);
    if (st == BC2_OK && (req.set_id >= max_sets(core) || !core->slots[req.set_id].running)) {
        st = BC2_ERR_SET_NOT_FOUND;
    }
    if (st == BC2_OK && req.data_len > bc2_max_data_len(core->slots[req.set_id].flags)) {
        st = BC2_ERR_PAYLOAD_TOO_LARGE;
    }
    if (st != BC2_OK) {
        send_ack(core, fr->seq, fr->type, st, NULL, 0);
        return;
    }

    uint16_t rc = 0;
    st = core->ble.update(core->ble.ctx, req.set_id, req.mfr_id, req.data, req.data_len, &rc);
    if (st != BC2_OK) {
        flag_error(core, now_ms);
        const uint8_t rc_le[2] = {(uint8_t)(rc & 0xFFu), (uint8_t)(rc >> 8)};
        send_ack(core, fr->seq, fr->type, st, rc_le, st == BC2_ERR_BLE_ERROR ? 2u : 0u);
        return;
    }
    bc2_slot_t *slot = &core->slots[req.set_id];
    slot->mfr_id = req.mfr_id;
    slot->data_len = req.data_len;
    memcpy(slot->data, req.data, req.data_len);
    core->activity_active = true;
    core->activity_until_ms = now_ms + BC2_ACTIVITY_FLASH_MS;

    const uint32_t dt = core->io.now_us(core->io.ctx) - t0_us;   /* wraps correctly */
    const uint8_t extra[4] = {(uint8_t)(dt & 0xFFu), (uint8_t)((dt >> 8) & 0xFFu),
                              (uint8_t)((dt >> 16) & 0xFFu), (uint8_t)(dt >> 24)};
    send_ack(core, fr->seq, fr->type, BC2_OK, extra, sizeof(extra));
}

/** SET_STOP: stop one set. */
static void handle_set_stop(bc2_core_t *core, const bc2_frame_t *fr)
{
    uint8_t id = 0;
    bc2_status_t st = bc2_decode_set_id(fr, &id);
    if (st == BC2_OK && (id >= max_sets(core) || !core->slots[id].running)) {
        st = BC2_ERR_SET_NOT_FOUND;
    }
    if (st == BC2_OK) {
        stop_set(core, id);
    }
    send_ack(core, fr->seq, fr->type, st, NULL, 0);
}

/** GET_STATUS: summary (PONG) or one set (SET_STATUS_RSP). */
static void handle_get_status(bc2_core_t *core, const bc2_frame_t *fr, uint32_t now_ms)
{
    uint8_t id = 0;
    const bc2_status_t st = bc2_decode_set_id(fr, &id);
    if (st != BC2_OK) {
        send_ack(core, fr->seq, fr->type, st, NULL, 0);
        return;
    }
    if (id == BC2_SET_ID_SUMMARY) {
        send_pong(core, fr->seq, now_ms);
        return;
    }
    bc2_set_status_t rsp = {.set_id = id};                   /* all zero for sets that do not exist */
    if (id < BC2_CORE_MAX_SETS && core->slots[id].running) {
        const bc2_slot_t *slot = &core->slots[id];
        rsp.running = 1;
        rsp.flags = slot->flags;
        rsp.interval = slot->interval;
        rsp.tx_power = slot->tx_power;
        rsp.mfr_id = slot->mfr_id;
        rsp.data_len = slot->data_len;
        rsp.data = slot->data;
    }
    bc2_frame_t out;
    if (bc2_encode_set_status(&out, fr->seq, &rsp)) {
        send_frame(core, &out);
    }
}

/** Dispatch one valid frame. */
static void handle_frame(bc2_core_t *core, const bc2_frame_t *fr, uint32_t now_ms)
{
    const uint32_t t0_us = core->io.now_us(core->io.ctx);
    core->last_rx_ms = now_ms;                               /* any valid frame is a keep-alive */

    if (!is_known_request(fr->type)) {
        send_ack(core, fr->seq, fr->type, BC2_ERR_UNKNOWN_CMD, NULL, 0);
        return;
    }
    if (fr->type == BC2_MSG_HELLO) {
        handle_hello(core, fr);
        return;
    }
    if (!core->session) {
        send_ack(core, fr->seq, fr->type, BC2_ERR_NO_SESSION, NULL, 0);
        return;
    }
    switch (fr->type) {
    case BC2_MSG_PING: {
        const bc2_status_t st = bc2_decode_empty(fr);
        if (st == BC2_OK) {
            send_pong(core, fr->seq, now_ms);
        } else {
            send_ack(core, fr->seq, fr->type, st, NULL, 0);
        }
        break;
    }
    case BC2_MSG_SET_START:
        handle_set_start(core, fr, now_ms);
        break;
    case BC2_MSG_SET_UPDATE:
        handle_set_update(core, fr, now_ms, t0_us);
        break;
    case BC2_MSG_SET_STOP:
        handle_set_stop(core, fr);
        break;
    case BC2_MSG_STOP_ALL: {
        const bc2_status_t st = bc2_decode_empty(fr);
        if (st == BC2_OK) {
            stop_all(core);
        }
        send_ack(core, fr->seq, fr->type, st, NULL, 0);
        break;
    }
    default:   /* BC2_MSG_GET_STATUS, the only remaining known request */
        handle_get_status(core, fr, now_ms);
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                 */
/* ------------------------------------------------------------------------- */

void bc2_core_init(bc2_core_t *core, const bc2_ble_ops_t *ble, const bc2_io_ops_t *io)
{
    memset(core, 0, sizeof(*core));
    core->ble = *ble;
    core->io = *io;
    core->keepalive_ms = BC2_KEEPALIVE_DEFAULT_MS;
    bc2_framer_init(&core->framer);
}

void bc2_core_on_bytes(bc2_core_t *core, const uint8_t *data, size_t len, uint32_t now_ms)
{
    bc2_frame_t fr;
    for (size_t i = 0; i < len; i++) {
        bc2_framer_push(&core->framer, data[i], now_ms);
        while (bc2_framer_next(&core->framer, &fr)) {
            handle_frame(core, &fr, now_ms);
        }
    }
    if (core->framer.parse_errors != core->seen_parse_errors) {   /* new corrupt frames: show ERROR */
        core->seen_parse_errors = core->framer.parse_errors;
        flag_error(core, now_ms);
    }
}

void bc2_core_tick(bc2_core_t *core, uint32_t now_ms)
{
    /* Sets that failed on their own (restart after a connection attempt etc.) */
    uint8_t id = 0;
    uint8_t reason = 0;
    while (core->ble.take_failure(core->ble.ctx, &id, &reason)) {
        if (id < BC2_CORE_MAX_SETS && core->slots[id].running) {
            core->slots[id].running = false;
            bc2_frame_t evt;
            bc2_encode_evt_set_stopped(&evt, id, reason);
            send_frame(core, &evt);
            flag_error(core, now_ms);
        }
    }

    /* Keep-alive: no valid frame for keepalive_ms ends the session and silences everything. */
    if (core->session && (uint32_t)(now_ms - core->last_rx_ms) >= core->keepalive_ms) {
        stop_all(core);
        core->session = false;
        core->timed_out = true;
        bc2_frame_t evt;
        bc2_encode_evt_timeout(&evt, core->keepalive_ms);
        send_frame(core, &evt);
    }
}

void bc2_core_on_button(bc2_core_t *core, uint32_t now_ms)
{
    stop_all(core);                                          /* session is kept on purpose */
    core->button_active = true;
    core->button_until_ms = now_ms + BC2_BUTTON_DISPLAY_MS;
    bc2_frame_t evt;
    bc2_encode_evt_button(&evt);
    send_frame(core, &evt);
}

bc2_led_state_t bc2_core_led_state(const bc2_core_t *core, uint32_t now_ms)
{
    if (core->error_active && before(now_ms, core->error_until_ms)) {
        return BC2_LED_ERROR;
    }
    if (core->button_active && before(now_ms, core->button_until_ms)) {
        return BC2_LED_BUTTON_STOP;
    }
    if (core->timed_out) {
        return BC2_LED_TIMEOUT;
    }
    if (core->session) {
        return active_mask(core) != 0 ? BC2_LED_ADVERTISING : BC2_LED_CONNECTED;
    }
    return core->ble.ready(core->ble.ctx) ? BC2_LED_NO_HOST : BC2_LED_BOOT;
}

bool bc2_core_activity_flash(const bc2_core_t *core, uint32_t now_ms)
{
    return core->activity_active && before(now_ms, core->activity_until_ms);
}
