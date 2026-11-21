# BC2 USB BLE Advertiser - Serial Protocol 1.1

This document is the contract between the stick firmware and the BrickController2 (BC2) host
implementation. Protocol **1.0** (sections 1-12) covers advertising, protocol **1.1** (sections 13-19)
adds the BLE scanner and GATT central (connect, discover, read, write, subscribe). 1.1 is purely
additive: a 1.0 host works unchanged. **Protocol 1.1 is frozen**: task 010 (BC2 side) is implemented
against this document.

The reference implementations are `components/bc2_protocol` (C) and `tools/bc2adv.py` (Python); both
are tested against the golden vectors in sections 10 and 18.

Contents: 1 Transport, 2 Frame format, 3 Message types, 4 Requests, 5 Responses and events,
6 Status codes, 7 Safety behaviour, 8 Version policy, 9 Host port settings and detection,
10 Golden vectors, 11 Notes for the BC2 host implementation (advertising), 12 Verified telegram sizes,
13 GATT overview and capabilities, 14 GATT requests, 15 GATT responses, 16 GATT events,
17 Flow control, safety and radio scheduling, 18 Golden vectors 1.1, 19 Notes for the BC2 host
implementation (GATT).

## 1. Transport

- USB CDC-ACM serial port (ESP32-S3 USB Serial/JTAG peripheral, no driver needed on Windows 10/11).
  Baud rate and parity are irrelevant (virtual port), 8N1 is assumed by host libraries.
- Byte stream, all multi-byte integers are **little-endian**. Full duplex: the device answers every
  request and may send asynchronous events.
- The firmware console and log output are disabled, so the port carries only protocol frames. The
  ROM bootloader may print text right after a reset; the parser skips garbage (section 2) and the
  host flushes its input after opening the port and then sends `HELLO`.

## 2. Frame format

```
+------+-----+------+-----+-----------------+------------+
| SOF  | LEN | TYPE | SEQ | PAYLOAD (0..60) | CRC16 (LE) |
| 0xB2 | u8  | u8   | u8  |                 | u16        |
+------+-----+------+-----+-----------------+------------+
```

- `SOF` = 0xB2.
- `LEN` = number of bytes of `TYPE + SEQ + PAYLOAD` (2..62). Frame size on the wire = `LEN + 4` (6..66).
- `CRC16` = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0) over
  `LEN, TYPE, SEQ, PAYLOAD`, transmitted low byte first. Check value of ASCII `123456789` is `0x29B1`.
- `SEQ`: chosen by the host per request (wraps at 255); the device copies it into the response.
  Events use SEQ 0, so hosts must not use 0 for requests and must not rely on SEQ to match events.

Parser rules (device and host, identical):

1. Skip bytes until a SOF is found (garbage is skipped silently, it is not an error).
2. Read `LEN`. If `LEN < 2` or `LEN > 62`: discard **only the SOF byte**, count a parse error, hunt again.
3. Wait for `LEN + 4` bytes in total. On CRC mismatch discard **only the SOF byte**, count a parse
   error and re-scan the already buffered bytes (so a valid frame that started inside a corrupted one
   is not lost).
4. A frame that is not complete 200 ms after its first byte was received is dropped (counted as a
   parse error) when the next byte arrives. This protects against a half frame blocking the parser.
5. No escaping is needed: resync relies on LEN + CRC.

`parse_errors` is a 16 bit counter that wraps; it is reset by `HELLO` and reported in `PONG`.
A frame with a bad CRC is never answered (its SEQ cannot be trusted).

## 3. Message types

| Type | Name | Dir | Description |
|------|------|-----|-------------|
| 0x01 | HELLO | H->D | Open a session, negotiate version and keep-alive. Also stops all sets. |
| 0x02 | PING | H->D | Keep-alive. Answered with PONG. |
| 0x10 | SET_START | H->D | Create and start an advertising set. |
| 0x11 | SET_UPDATE | H->D | Replace the payload of a running set. |
| 0x12 | SET_STOP | H->D | Stop one set. |
| 0x13 | STOP_ALL | H->D | Stop all sets. |
| 0x14 | GET_STATUS | H->D | Query the device summary or one set. |
| 0x30-0x3A | SCAN_START ... SET_FAILSAFE | H->D | Protocol 1.1: scanner and GATT central requests (section 14). |
| 0x80 | ACK | D->H | Generic response (original type, status, optional extra bytes). |
| 0x81 | HELLO_RSP | D->H | Capabilities. |
| 0x82 | PONG | D->H | Response to PING and to GET_STATUS(0xFF). |
| 0x83 | SET_STATUS_RSP | D->H | Response to GET_STATUS(set_id). |
| 0xA0-0xA1 | READ_RSP, GATT_STATUS_RSP | D->H | Protocol 1.1 responses (section 15). |
| 0xC0-0xC8 | EVT_NOTIFY ... EVT_DROPPED | D->H | Protocol 1.1 asynchronous events (section 16). |
| 0xE0 | EVT_SET_STOPPED | D->H | A set ended without a host request. |
| 0xE1 | EVT_BUTTON | D->H | Button pressed, all sets were stopped. |
| 0xE2 | EVT_TIMEOUT | D->H | Keep-alive timeout, all sets were stopped (best effort). |

### 3.1 TYPE ranges

A device that receives a request with any type that it does not implement, including all reserved
types and all device-to-host types, answers `ACK(orig_type, UNKNOWN_CMD)` like for any other unknown
type (also before `HELLO`). Hosts must not send them. A 1.0 firmware answers the 1.1 request types
this way, so a host can never harm it; the capability block of HELLO_RSP (section 13.1) tells a host
what is supported.

| Range | Direction | Use |
|-------|-----------|-----|
| 0x20 - 0x2F | H->D | reserved: future advertising / device management requests |
| 0x30 - 0x3A | H->D | **protocol 1.1: scanner and GATT central requests (section 14)** |
| 0x3B - 0x4F | H->D | reserved: further GATT requests |
| 0xA0 - 0xA1 | D->H | **protocol 1.1: responses to GATT requests (section 15)** |
| 0xA2 - 0xBF | D->H | reserved |
| 0xC0 - 0xC8 | D->H | **protocol 1.1: asynchronous GATT events (section 16)** |
| 0xC9 - 0xDF | D->H | reserved |
| 0xE3 - 0xEF | D->H | future device events |
| 0xF0 - 0xFF | both | experimental / vendor use, never assigned |

Order of checks in the device for a request: unknown TYPE (UNKNOWN_CMD) -> HELLO handling ->
session present (NO_SESSION) -> payload length (BAD_LENGTH) -> parameter values (BAD_PARAM) ->
state (ALREADY_RUNNING / SET_NOT_FOUND) -> size limit (PAYLOAD_TOO_LARGE).

## 4. Requests

### HELLO (0x01) - `proto_major u8, proto_minor u8, keepalive_ms u16`

- Accepted if `proto_major` equals the device major (1); minor differences are compatible (the device
  reports its own minor). Otherwise `ACK(HELLO, VERSION_UNSUPPORTED)` and nothing else changes.
- `keepalive_ms` = requested safety timeout. The device clamps it to [200, 10000]; 0 = default 1000.
  The effective value is returned in HELLO_RSP. The host must send a valid frame (PING recommended)
  at least every `keepalive_ms / 4`.
- Effects: stops all sets, clears the set table, resets `parse_errors`, starts the session, restarts
  the keep-alive timer, clears the TIMEOUT state. Response: `HELLO_RSP`.

### PING (0x02) - empty

Resets the keep-alive timer (any valid frame does; PING is the cheapest). Response: `PONG`.

### SET_START (0x10) - `set_id u8, flags u8, interval u16, tx_power i8, mfr_id u16, data_len u8, data[data_len]`

- `set_id`: 0 .. `max_sets`-1 (host chosen, maps 1:1 to a NimBLE advertising instance).
- `flags`: bit 0 = add the AD flags field `02 01 06` (default 1, like Android);
  bits 1-2 = PDU type: 0 = ADV_IND (connectable + scannable, default), 1 = ADV_SCAN_IND,
  2 = ADV_NONCONN_IND, 3 = reserved (BAD_PARAM); bits 3-7 reserved, must be 0.
- `interval`: advertising interval in 0.625 ms units, min = max = this value (no jitter band).
  Allowed range `min_interval .. max_interval` from HELLO_RSP (32 = 20 ms .. 16384 = 10.24 s).
- `tx_power`: dBm; `127` = maximum supported by the controller. The firmware clamps to the
  controller range and returns the applied value in the ACK extra.
- `mfr_id`: company identifier, placed little-endian into the AD structure.
- `data_len` <= 24 with flags bit 0, <= 27 without (reported as `max_data_*` in HELLO_RSP). The
  value must match the frame length exactly. Empty data (0) is allowed.
- AD data built by the firmware: `[02 01 06]` (if bit 0) + `[data_len+3, FF, mfr_lo, mfr_hi, data...]`.
  Only legacy advertising PDUs are used (<= 31 bytes, channels 37/38/39).
- Response: `ACK(SET_START, status, extra = applied_tx_power i8)`. With status OK the set is on air.
  `ALREADY_RUNNING` if the id is active. With `BLE_ERROR` the extra is `rc u16` (NimBLE return code).

### SET_UPDATE (0x11) - `set_id u8, mfr_id u16, data_len u8, data[data_len]`

- The set must be running (`SET_NOT_FOUND` otherwise). Same size limits as at start (`PAYLOAD_TOO_LARGE`
  if above the limit of the set's flags). Flags, interval and TX power stay unchanged.
- Implemented as an in-place data update of the running instance (no stop/start); the new data goes
  out with the next advertising event. Response `ACK(SET_UPDATE, OK, extra = processing_time_us u32)`:
  time from the start of handling the frame to the controller having accepted the data.
  With `BLE_ERROR` the ACK extra is `rc u16` (NimBLE return code), as for SET_START; the set may then be
  silent, in which case the device also emits `EVT_SET_STOPPED` and frees the slot.
- Updates are applied in arrival order. The host need not wait for the ACK before sending the next
  update but should coalesce (latest wins) when ACKs lag.

### SET_STOP (0x12) - `set_id u8`

Stops the set and frees the slot. `SET_NOT_FOUND` if not running. Response `ACK`.

### STOP_ALL (0x13) - empty

Stops all sets. Always `OK`. Response `ACK`.

### GET_STATUS (0x14) - `set_id u8`

`0xFF` = device summary (answered with `PONG`), otherwise `SET_STATUS_RSP` for that set.

## 5. Responses and events

**ACK (0x80)**: `orig_type u8, status u8, extra[0..]` (extra defined per request above, otherwise empty).

**HELLO_RSP (0x81)**, 32 bytes (27 bytes in protocol 1.0; the bytes 27..31 are described in section 13.1):

| Offset | Field |
|--------|-------|
| 0..5 | magic `"BC2ADV"` (ASCII) |
| 6 | status (0) |
| 7, 8 | proto_major, proto_minor |
| 9, 10, 11 | fw_major, fw_minor, fw_patch |
| 12 | max_sets |
| 13 | max_data_len_with_flags (24) |
| 14 | max_data_len_no_flags (27) |
| 15..16 | min_interval (u16, 32) |
| 17..18 | max_interval (u16, 16384) |
| 19..20 | keepalive_ms (u16, effective) |
| 21..26 | device_id: Bluetooth MAC, display order |
| 27..31 | GATT capability block (protocol 1.1, section 13.1) |

On version mismatch the device sends `ACK(HELLO, VERSION_UNSUPPORTED)` instead. The magic lets a host
identify this firmware on any COM port.

**PONG (0x82)**, 9 bytes: `state u8` (0 = no set running, 1 = advertising), `active_mask u16`
(bit n = set n running), `parse_errors u16`, `uptime_ms u32`.

**SET_STATUS_RSP (0x83)**: `set_id u8, running u8, flags u8, interval u16, tx_power i8, mfr_id u16,
data_len u8, data[]` (`running = 0` and all other fields zero if the set does not exist).

**EVT_SET_STOPPED (0xE0)**: `set_id u8, reason u8` with reason 1 = BLE controller error,
2 = restart after a connection attempt failed. **EVT_BUTTON (0xE1)**: empty, sent after the button
stopped all sets. **EVT_TIMEOUT (0xE2)**: `timeout_ms u16`, sent after the keep-alive timeout.
Events are best effort (the host may be gone); after reconnecting the host re-queries with GET_STATUS.
Hosts must ignore unknown trailing bytes in responses and unknown event types.

## 6. Status codes

| Code | Name | Meaning |
|------|------|---------|
| 0 | OK | |
| 1 | UNKNOWN_CMD | TYPE not implemented (includes reserved ranges, section 3.1; also 1.1 types on a 1.0 firmware) |
| 2 | BAD_LENGTH | payload length does not match the request layout |
| 3 | BAD_PARAM | value out of range / reserved bits set / set_id >= max_sets |
| 4 | NO_SESSION | HELLO required first (also after a keep-alive timeout) |
| 5 | VERSION_UNSUPPORTED | proto_major mismatch |
| 6 | ALREADY_RUNNING | SET_START on an active id |
| 7 | SET_NOT_FOUND | update/stop on an id that is not running |
| 8 | PAYLOAD_TOO_LARGE | data_len above the limit |
| 9 | BLE_ERROR | NimBLE/controller error; ACK extra = `rc u16` (0xFFFF = BLE host not synced yet) |
| 10 | NO_RESOURCES | out of memory or advertising instances; for CONNECT also: `max_conns` links already used |
| 11 | BUSY | (1.1) a procedure is already running on this connection / the controller has no free buffer; retry later |
| 12 | NOT_CONNECTED | (1.1) `conn_id` has no connected link |
| 13 | TIMEOUT | (1.1) a GATT procedure did not finish within 5 s (the stick then also terminates that link) |
| 14 | ATT_ERROR | (1.1) the peer answered with an ATT error, ACK extra = `att_error u8` |
| 15 | DISCONNECTED | (1.1) the link was lost while the request was pending |
| 16 | NOT_SUPPORTED | (1.1) GATT is not built into this firmware |

Lost or corrupt frames are detected by a response timeout (suggested 100 ms). All requests are
idempotent except SET_START, which answers `ALREADY_RUNNING` on a duplicate; a host that knows its
first SET_START may have arrived treats that as success.

## 7. Safety behaviour (normative)

- Boot: no advertising, no session.
- A session ends (all sets stopped, state "no session") on: keep-alive timeout, and `HELLO`
  (restart). With protocol 1.1 every GATT link and the scan are stopped as well (section 17.3). USB unplug removes the power of the stick, which also ends advertising.
- Keep-alive timeout: if no **valid frame** was received for `keepalive_ms` (garbage and corrupt frames
  do not count), the device stops all sets, ends the session, sends EVT_TIMEOUT (best effort) and shows
  the TIMEOUT LED state. The host must send HELLO again; other requests answer NO_SESSION. This also
  covers "COM port closed" and crashed host software, because the device cannot see the line state.
- Button short press: stops all sets, **keeps** the session, sends EVT_BUTTON. With protocol 1.1 it also
  drops all GATT links and stops the scan (section 17.3).
- A failed stop request is retried once; if the controller may still be advertising the firmware resets
  the chip (guaranteed silence). The task watchdog resets the chip if the USB or core task hangs.

## 8. Version policy

`proto_major` changes on incompatible changes, `proto_minor` on additive ones (new optional flag bits,
new message types from the reserved ranges, longer response payloads at the end). Minor 1 is the GATT
extension (sections 13-19); a host that wants GATT checks `proto_minor >= 1` and `gatt_flags & 2` in
HELLO_RSP. Request layouts are
strict: the device rejects payloads of unexpected length with BAD_LENGTH, except that a future minor
may extend request layouts at the end, which the device then only accepts from hosts that announced
that minor in HELLO.

## 9. Host port settings and detection

- Open the port with DTR and RTS **de-asserted** (pyserial: create `Serial()`, set `dtr=False`,
  `rts=False`, then `open()`; .NET: `DtrEnable = false`, `RtsEnable = false`). On the USB Serial/JTAG
  peripheral certain DTR/RTS sequences reset the chip or put it into the ROM bootloader (that is how
  esptool works). Do not toggle them.
- After opening: flush input, send HELLO with a 200 ms timeout, retry up to 3 times (the chip may still
  be booting).
- Auto-detection: candidate ports have VID `0x303A` / PID `0x1001` (the fixed identity of the ESP32-S3
  USB Serial/JTAG peripheral, shared with all ESP32-S3 dev boards). Open each candidate, send HELLO and
  accept the port whose HELLO_RSP starts with the magic `BC2ADV`. Close all other ports immediately and
  never keep another board's port open. There is no custom USB product string (decision of the product
  owner, no TinyUSB backend).

## 10. Golden vectors

All frames are given as hex bytes on the wire. They are checked byte-exact by
`test/host/test_codec.c` and `tools/tests/test_framing.py`. The HELLO request vector below uses protocol
minor 0 (a 1.0 host); a 1.1 host sends minor 1, the device accepts both. The vectors of the 1.1 messages are in
section 18.

| Frame | Bytes |
|-------|-------|
| HELLO seq 1, proto 1.0, keepalive 1000 | `B2 06 01 01 01 00 E8 03 D6 60` |
| PING seq 2 | `B2 02 02 02 DC E4` |
| SET_START seq 3: set 0, flags 0x01, interval 32, tx 127, mfr 0xFFF0, data 01..08 | `B2 12 10 03 00 01 20 00 7F F0 FF 08 01 02 03 04 05 06 07 08 F5 48` |
| SET_UPDATE seq 4: set 0, mfr 0xFFF0, data A1 A2 A3 A4 | `B2 0A 11 04 00 F0 FF 04 A1 A2 A3 A4 52 B1` |
| SET_STOP seq 5, set 0 | `B2 03 12 05 00 EA CD` |
| STOP_ALL seq 6 | `B2 02 13 06 1A 94` |
| GET_STATUS seq 7, summary | `B2 03 14 07 FF D8 07` |
| HELLO_RSP seq 1: proto 1.1, fw 0.2.0, 5 sets, 24/27, interval 32..16384, keepalive 1000, id 24:0A:BC:11:22:33, GATT block flags 7, 4 connections, 56 byte values, 31 byte scan data, 4 failsafe slots | `B2 22 81 01 42 43 32 41 44 56 00 01 01 00 02 00 05 18 1B 20 00 00 40 E8 03 24 0A BC 11 22 33 07 04 38 1F 04 DA 6E` |
| ACK seq 3 for SET_START, OK, applied tx 127 | `B2 05 80 03 10 00 7F 06 38` |
| ACK seq 4 for SET_UPDATE, OK, 891 us | `B2 08 80 04 11 00 7B 03 00 00 D4 57` |
| PONG seq 2: advertising, set 0, 0 errors, uptime 10000 ms | `B2 0B 82 02 01 01 00 00 00 10 27 00 00 5C D9` |
| EVT_TIMEOUT, 1000 ms | `B2 04 E2 00 E8 03 F0 58` |

AD data example for the SET_START above (flags bit set):
`02 01 06 0B FF F0 FF 01 02 03 04 05 06 07 08` (15 bytes on air).

## 11. Notes for the BC2 host implementation (advertising, task 008)

- **Keep-alive**: BC2 refreshes advertising data only on change or every 2 s. The host implementation
  must therefore send a dedicated `PING` every **<= 250 ms** (with the default 1000 ms timeout),
  independent of data changes and from its own timer, otherwise an idle controller makes the stick
  fall silent and drop the session. After EVT_TIMEOUT or NO_SESSION: HELLO, then SET_START again.
- **Mapping of `IBluetoothLEAdvertiserDevice`**: `StartAdvertiseAsync` -> SET_START,
  `UpdateAdvertisedDataAsync` -> SET_UPDATE (the manufacturer id is passed on every update),
  `StopAdvertiseAsync` -> SET_STOP. One `BluetoothAdvertisingDeviceHandler` = one set id.
- **Interval**: BC2 always uses `AdvertisingInterval.Min` -> `32` (20 ms). Suggested mapping for the other
  enum values: Low 160 (100 ms), Medium 400 (250 ms), High 1600 (1 s). The receiver only sees new data
  once per interval, so `Min` gives the best responsiveness.
- **TX power**: BC2 always uses `TxPowerLevel.Max` -> `127`.
- **Payload layout**: send the Android layout (flags bit 0 set, raw data = encrypted RF payload from
  `CryptTools.GetRfPayload` without the Windows 3 byte offset, see section 12) so the toys see the same
  bytes as from an Android phone.
- **Default PDU**: ADV_IND (flags bits 1-2 = 0), like Android's connectable + scannable legacy set. Which
  PDU type the toys accept is verified on hardware (docs/verification.md).
- **Sets**: at most `max_sets` (5) sets run concurrently; several BC2 devices need several set ids.
  MK4 and MK5 share one handler, CaDA uses one per car.
- **Updates**: send SET_UPDATE without waiting for the previous ACK, coalesce (latest wins) when ACKs lag.
- **Port**: DTR/RTS off, flush, HELLO with retries, detection by VID/PID + magic (section 9).

## 12. Verified telegram sizes (BrickController2 source, read-only analysis)

The size of the "raw data" sent as manufacturer data is `3 (header) + seed + telegram + 2 (checksum)`
(`CryptTools.GetRfPayload`), written into a fixed-size array and padded with filler bytes. Layout of the
Android platform services (the layout the stick reproduces):

| Vendor | Manufacturer id | Telegram (before crypto) | Seed | Raw data on air (Android) | Source |
|--------|-----------------|--------------------------|------|---------------------------|--------|
| MouldKing (MK3, MK3.8, MK4, MK5, MK6) | 0xFFF0 | 8 bytes (connect, MK3 base), 10 bytes (MK3.8, MK4, MK5, MK6 base) | 5 | 18 or 20 bytes, padded with filler to **24** | `MKPlatformService` (PayloadLength 24) |
| JieStar (SCM4, SCM8) | 0xFFF0 | 8 bytes | 5 | 18 bytes, padded to **24** | `JieStarPlatformService` (24) |
| PowerBox (A series, M battery) | 0xFF00 | 8 bytes | 5 | **18** bytes (exactly 3+5+8+2) | `PowerBoxPlatformService` (18) |
| CaDA V1 race car | 0xC200 | 16 bytes | 3 | **24** bytes (exactly 3+3+16+2, no padding) | `CaDAPlatformService` (24) |
| CaDA V2 race car (Rev2 encoder) | 0xC200 | 16 bytes, sent unencrypted | - | **16** bytes | `ICaDAPlatformService.TryGetRfPayloadV2` |

Result: the maximum is **24 raw bytes**, reached by MK/JieStar (padding) and CaDA V1 (exact). That is
exactly `max_data_len_with_flags` (24), so every known device fits with the AD flags field enabled and the
full legacy PDU is used (3 + 4 + 24 = 31 bytes). No device needs the 27 byte no-flags mode.

Windows (WinUI) services use `PayloadLength = 24 + 3` (26 for PowerBox): they prepend 3 dummy bytes so that
the payload sits at the same offset as on Android, because the Windows publisher cannot add the flags
field. That is 27 bytes, which would fit only with flags bit 0 cleared. The stick can add the flags field
itself, so task 008 must **not** use the Windows 3 byte offset but send the Android layout (<= 24 bytes).
The iOS services use 26 bytes (20 for PowerBox) for a different on-air layout; they are not relevant for
the stick.

# Protocol 1.1: scanner and GATT central

## 13. GATT overview and capabilities

Besides advertising, the stick can scan, connect to several peripherals (GATT central) and read, write
and subscribe on them, so that BC2 on Windows can use all its BLE devices through the stick. Advertising,
scanning and the GATT links run at the same time; the controller schedules the radio itself.

- Frame format, CRC, parser rules and SEQ rules of protocol 1.0 are unchanged. Every 1.1 message fits into
  one frame (payload <= 60 bytes), so there is no fragmentation. Values larger than **56 bytes** are
  truncated by the stick and flagged.
- All 1.1 requests require a session (`NO_SESSION` otherwise). Order of checks: unknown TYPE ->
  session -> payload length (`BAD_LENGTH`) -> parameter values (`BAD_PARAM`) -> state -> resources.
- **GATT requests are never retransmitted by the host** (USB is reliable, a duplicate write would be
  executed twice). A request is answered exactly once, by an ACK or by its specific response type.
  Hosts wait up to 6 s for the answer of a GATT request (the stick answers within 5 s).
- `conn_id` is chosen by the host, **0..15**, and names a link while it exists. At most `max_conns`
  conn ids are in use at the same time (connected or connecting); one more CONNECT answers `NO_RESOURCES`.
- `addr` fields are 6 bytes in **display order** (byte 0 is the first octet of `AA:BB:CC:DD:EE:FF`, the
  same order as `device_id` in HELLO_RSP). `uuid` fields are 2 or 16 bytes in ATT wire order
  (little-endian; `00001623-1212-efde-1623-785feabcd123` is `23 D1 BC EA 5F 78 23 16 DE EF 12 12 23 16 00 00`).
- The stick keeps no GATT state across sessions: every accepted HELLO starts clean (section 17.3).

### 13.1 Capability block in HELLO_RSP (bytes 27..31)

| Offset | Field |
|--------|-------|
| 27 | `gatt_flags` u8: bit0 scan supported, bit1 GATT central supported, bit2 failsafe write supported (0 = no GATT built in) |
| 28 | `max_conns` u8: simultaneous GATT links the firmware supports together with 5 advertising sets and the scan (1..16; 0 if GATT is not built in). **Hosts must read it and never assume a value.** The shipped firmware announces the value documented in `hardware.md` |
| 29 | `max_value_len` u8 (56): largest GATT value in WRITE / READ_RSP / EVT_NOTIFY |
| 30 | `max_scan_data` u8 (31): largest advertising data in EVT_SCAN_REPORT |
| 31 | `failsafe_slots` u8 (4): failsafe writes per connection (0 if bit2 of `gatt_flags` is 0) |

A host that sees `proto_minor < 1` or `gatt_flags & 2 == 0` must not send GATT requests (a 1.0 firmware
answers them with `UNKNOWN_CMD`, a firmware built without GATT with `NOT_SUPPORTED`).

## 14. GATT requests (host -> device, TYPE 0x30-0x3A)

| Type | Name | Payload | Answer |
|------|------|---------|--------|
| 0x30 | SCAN_START | `flags u8, interval u16, window u16, refresh_ms u16` (7 bytes) | ACK, immediately |
| 0x31 | SCAN_STOP | empty | ACK, always OK |
| 0x32 | CONNECT | `conn_id u8, addr[6], addr_type u8, flags u8, timeout_ms u16, itvl_min u16, itvl_max u16` (**15 bytes**) | ACK immediately; the result comes as EVT_CONNECTED / EVT_DISCONNECTED |
| 0x33 | DISCONNECT | `conn_id u8` | ACK immediately (OK even if the id is free); EVT_DISCONNECTED follows if a link or attempt existed |
| 0x34 | DISCOVER | `conn_id u8` | ACK immediately, then EVT_GATT_SERVICE / EVT_GATT_CHAR events, finally EVT_DISCOVER_DONE |
| 0x35 | READ | `conn_id u8, handle u16` | READ_RSP, or ACK with an error status |
| 0x36 | WRITE (with response) | `conn_id u8, handle u16, data[1..56]` | ACK **after** the ATT write response |
| 0x37 | WRITE_CMD (no response) | `conn_id u8, handle u16, data[1..56]` | ACK as soon as the data is queued in the controller |
| 0x38 | SUBSCRIBE | `conn_id u8, cccd_handle u16, mode u8` (0 off, 1 notify, 2 indicate) | ACK **after** the CCCD write completed |
| 0x39 | GET_GATT_STATUS | empty | GATT_STATUS_RSP |
| 0x3A | SET_FAILSAFE | `conn_id u8, slot u8, handle u16, data[0..20]` | ACK immediately |

All integers are little-endian. ACK extras: errors with status `ATT_ERROR` carry `att_error u8`, with
`BLE_ERROR` they carry `rc u16` (NimBLE return code); everything else has no extra.

### SCAN_START

- `flags`: bit0 = active scan (send scan requests, needed for names in scan responses), bit1 = report
  duplicates (disables the stick-side duplicate filter), bits 2-7 must be 0.
- `interval`, `window`: 0.625 ms units, 4..16384, `window <= interval`. 0/0 selects the default 160 / 80
  (100 ms / 50 ms). Any other combination with a 0 is `BAD_PARAM`.
- `refresh_ms`: an unchanged advertiser is re-reported after this time (0 = default 1000, max 10000).
- The stick reports a PDU when its (address, address type, scan-response flag, hash of the data)
  combination is new or changed, or was last reported more than `refresh_ms` ago. The table has 48
  entries (least recently reported is evicted). RSSI is not part of the key. At most **150 reports per
  second** are sent; the rest is dropped and counted (section 17.1). ADV and SCAN_RSP PDUs of one device
  are reported separately (flag bit1 of EVT_SCAN_REPORT); the host merges them per address.
- `ALREADY_RUNNING` if a scan is active (SCAN_STOP first to change parameters). Only legacy advertising
  PDUs are reported (no BC2 device uses extended advertising).
- While a connect attempt runs the scan keeps running if the controller allows it; otherwise the stick
  pauses the scan during the attempt and resumes it afterwards (no event).

### CONNECT

- `addr_type`: 0 public, 1 random, **0xFF = auto**: the type seen while scanning (cache of the 64 most
  recent advertisers), public if the address was never seen. `GetKnownDevice(address)` in BC2 only has the
  address, so a host should scan before it connects, or use auto after a recent scan.
- `flags` bit0 = **background connect**: no timeout, keeps trying until connected or until DISCONNECT (for
  hubs that are switched on later, such as the DUPLO train hub). Other bits must be 0.
- `timeout_ms`: 0 = default 10000, ignored for background connects. The time counts from the request
  (waiting for a free initiator included).
- `itvl_min` / `itvl_max`: connection interval in 1.25 ms units, 6..3200, `min <= max`; 0/0 = default
  12 / 24 (15..30 ms). Slave latency 0 and supervision timeout 4 s are fixed. Peer requests to change the
  parameters are accepted.
- Errors: `BAD_PARAM`; `ALREADY_RUNNING` (`conn_id` in use, this address is already connected / connecting, or
  the old link of this id is still being torn down after a safety stop); `NO_RESOURCES` (`max_conns` ids
  already in use, or a controller/memory error when the attempt starts, then reported as EVT_DISCONNECTED
  reason 0xF3 instead).
- The controller can have **one** connection attempt at a time. The stick therefore queues the CONNECT
  requests and works through them round robin; an attempt that has run 3 s while another request waits is
  cancelled and requeued (so a pending background connect never blocks other connections).
- Result: **EVT_CONNECTED** after the stick's own MTU exchange finished (requested MTU 64; a peer that
  does not support it, answers with an ATT error or is silent for 2 s gives `mtu` 23, never a failure).
  If the attempt ends without a link (timeout, cancel, error), **EVT_DISCONNECTED** with phase 0.

### DISCONNECT, DISCOVER, READ, WRITE, WRITE_CMD, SUBSCRIBE

- DISCONNECT on a connected link terminates it; on a waiting or running attempt it cancels the attempt
  (EVT_DISCONNECTED, phase 0, reason 0xF1). A pending procedure of that link is answered `DISCONNECTED`.
- DISCOVER performs a full discovery: all services, then the characteristics of all services, with the CCCD
  handle (descriptor 0x2902) for characteristics that have the notify or indicate property. The result is
  the event stream of section 16. The 5 s timeout restarts with every discovery event.
- READ is a single ATT read (up to MTU-1 bytes); the value is truncated to 56 bytes (flag in READ_RSP).
- WRITE / WRITE_CMD: `data_len` 1..56 and at most `mtu-3`, otherwise `PAYLOAD_TOO_LARGE`. Handle 0 is
  `BAD_PARAM`. Use the **value handle** found by discovery.
- SUBSCRIBE uses the **CCCD handle** reported by discovery (not the value handle). Mode 1 on a
  characteristic without the notify property is answered by the peer with an ATT error. Notifications
  are active from the moment the CCCD write is issued: the SUBSCRIBE ACK is sent first, then the first
  EVT_NOTIFY (hubs such as the DUPLO train hub send their first notification immediately).
- Common state errors: `NOT_CONNECTED` if the link is not connected (a link that is still discovering is
  connected); `BUSY` if another procedure (READ, WRITE, SUBSCRIBE, DISCOVER) is running on that
  connection. WRITE_CMD is not a procedure: it is allowed at any time, `BUSY` only when the controller
  has no free buffer (retry after >= 5 ms). BC2 serialises operations per device, so a second procedure
  is rare; the stick does **not** queue (BUSY instead) and stays stateless.
- A procedure without answer after 5 s ends with `TIMEOUT` and the stick terminates the link (failsafe
  writes first, EVT_DISCONNECTED reason 0xF3).

### GET_GATT_STATUS, SET_FAILSAFE

- SET_FAILSAFE stores up to `failsafe_slots` (4) "safe state" writes per connection (`slot` 0..3, `handle` =
  value handle, `data` up to 20 bytes, e.g. "motors off"). Several slots exist because a hub such as the
  LEGO WeDo 2.0 needs one write per motor port. `data_len` 0 clears the slot. The stick executes all
  non-empty slots in slot order as WRITE_CMD **before it terminates the link** in the situations of
  section 17.3 (keep-alive timeout, HELLO, button, procedure TIMEOUT, internal errors). It does **not**
  execute them on a normal DISCONNECT (the host does its own cleanup). The slots are cleared when the link
  goes away. Whether a device needs one depends on the hub (see `verification.md` G10).

## 15. GATT responses (device -> host, TYPE 0xA0-0xA1)

- **READ_RSP 0xA0** (SEQ = request SEQ): `conn_id u8, handle u16, flags u8, data[0..56]`; flags bit0 = data truncated to 56 bytes.
- **GATT_STATUS_RSP 0xA1** (SEQ = request SEQ), 17 bytes: `scanning u8, connecting_mask u16, connected_mask u16,
  dropped_scan u16, dropped_notify u16, free_heap u32, min_free_heap u32`. Bit n of a mask = `conn_id` n
  (connecting = waiting for or running an attempt). The dropped counters count since the last HELLO
  (wrapping). The heap figures (bytes, 0 = unknown) serve the connection spike (`verification.md`).

## 16. GATT events (device -> host, TYPE 0xC0-0xC8, SEQ 0)

| Type | Name | Payload |
|------|------|---------|
| 0xC0 | EVT_NOTIFY | `conn_id u8, handle u16 (value handle), flags u8 (bit0 indication, bit1 truncated to 56), data[0..56]` |
| 0xC1 | EVT_CONNECTED | `conn_id u8, mtu u16, itvl u16 (1.25 ms units), addr[6]` (11 bytes). Sent after the stick's own MTU exchange. The link is usable from here |
| 0xC2 | EVT_DISCONNECTED | `conn_id u8, phase u8 (0 = connect attempt ended without a link, 1 = a connected link ended), reason u8` |
| 0xC3 | EVT_SCAN_REPORT | `addr[6], addr_type u8 (0 public, 1 random), rssi i8, flags u8 (bit0 connectable, bit1 scan response, bit2 directed), data_len u8, data[0..31]` (raw AD structures) |
| 0xC4 | EVT_GATT_SERVICE | `conn_id u8, start u16, end u16, uuid_len u8 (2 or 16), uuid[]` |
| 0xC5 | EVT_GATT_CHAR | `conn_id u8, value_handle u16, cccd_handle u16 (0 = none), props u8 (ATT characteristic properties: 0x02 read, 0x04 write without response, 0x08 write, 0x10 notify, 0x20 indicate), uuid_len u8, uuid[]` |
| 0xC6 | EVT_DISCOVER_DONE | `conn_id u8, status u8, n_services u8, n_chars u8` (status: 0 OK, 12 `NOT_CONNECTED`/15 `DISCONNECTED` link lost, 13 `TIMEOUT`, 9 `BLE_ERROR`) |
| 0xC7 | EVT_SCAN_STOPPED | `reason u8 (1 = controller error, 2 = stopped by safety / button), rc u16` |
| 0xC8 | EVT_DROPPED | `kind u8 (1 = scan reports, 2 = notifications), count u16` (dropped since the last EVT_DROPPED, at most one per kind and second) |

`EVT_DISCONNECTED.reason` is the HCI error code of the stack (0x08 supervision timeout, 0x13 remote user
terminated, 0x16 local host terminated, 0x3E failed to establish, ...) or a stick specific value:
**0xF0** connect attempt timed out, **0xF1** cancelled by DISCONNECT while connecting, **0xF2** stopped by the
stick's safety logic (keep-alive timeout / HELLO / button), **0xF3** stick internal error (procedure TIMEOUT,
controller error, lost internal event).

Ordering guarantees: the events of one connection keep the order in which they happened. The answer of a
request (ACK) is sent **before** the events it causes. Discovery is delivered service by service in
this order: all EVT_GATT_SERVICE first, then the EVT_GATT_CHAR entries of all services in handle order (the host
assigns a characteristic to a service by the handle range `start..end`), finally EVT_DISCOVER_DONE (`n_services`
and `n_chars` count what was reported, saturating at 255). Up to 24 services and 24 characteristics per
service are reported, anything beyond is skipped. After a safety stop with an accepted HELLO the events of
the old links (EVT_DISCONNECTED 0xF2) may still arrive; their `conn_id`s stay busy until then.

## 17. Flow control, safety and radio scheduling

### 17.1 Flow control and loss policy

- Responses and state events (EVT_CONNECTED, EVT_DISCONNECTED, EVT_GATT_*, EVT_DISCOVER_DONE, EVT_SCAN_STOPPED,
  EVT_TIMEOUT, EVT_BUTTON, EVT_DROPPED) are **never dropped** in normal operation. They are only lost when
  the host does not read the port at all for a long time and the stick's 3 KiB TX queue is full.
- EVT_NOTIFY and EVT_SCAN_REPORT are droppable: they are refused when the TX queue is more than half full, and
  scan reports additionally above 150 per second or when the stick's internal event queue is more than half
  full. Drops are counted and reported by EVT_DROPPED and GATT_STATUS_RSP.
- The host must read the port continuously (BC2 does). A host that does not read only delays the stick's TX
  task, never the BLE stack or the safety timers.

### 17.2 Latency (targets, measured with `gatt-bench`)

WRITE_CMD ACK < 5 ms, notification to USB frame < 10 ms, WRITE with response about 1-2 connection intervals
plus 3 ms (so about 33-66 writes per second at 15-30 ms).

### 17.3 Safety rules (normative, extends section 7)

A **safety stop** does the following: for every link that is connected, execute its failsafe writes (if any;
the terminate request is sent up to 40 ms after the last write so the writes are on air first), then
terminate the link; cancel connect attempts and drop queued CONNECT requests (EVT_DISCONNECTED reason 0xF2);
stop the scan (EVT_SCAN_STOPPED reason 2); stop all advertising sets (as in section 7). It is executed on:
keep-alive timeout (before EVT_TIMEOUT), an accepted HELLO (the session restarts), the button short press
(the session is kept), and internal fatal errors. A procedure TIMEOUT executes the failsafe writes and
terminates only the affected link.

One second after a safety stop the firmware verifies that all links are gone; a link that survived is
terminated once more and if it is still there one second later the chip is reset (like a failed advertising
stop). Unplugging USB removes the power. A peripheral may keep running after the link drops (device
specific); SET_FAILSAFE is the mechanism to improve that.

### 17.4 Radio scheduling

The controller time-slices scan, advertising (up to 5 sets at 20 ms) and the connection events itself. The
defaults leave airtime: scan 100 ms interval / 50 ms window, connection interval 15..30 ms. BC2 should scan only
while its scan page is open. Connect attempts are serialised (section 14, CONNECT).

## 18. Golden vectors 1.1

All frames as hex bytes on the wire. The payloads were written by hand and framed with the Python reference
implementation; they are checked byte-exact by `test/host/test_gatt_codec.c` (C) and
`tools/tests/test_gatt_framing.py` (Python). The 18 byte MouldKing DIY frame is
`CC AA BB 01 80 80 80 80 80 80 80 80 80 80 80 80 66 33`, the DUPLO motor frame `08 00 81 00 11 51 00 00`.

| Frame | Bytes |
|-------|-------|
| SCAN_START seq 8: active, all defaults | `B2 09 30 08 01 00 00 00 00 00 00 8E 96` |
| SCAN_START seq 9: active + duplicates, 160 / 80, refresh 1000 ms | `B2 09 30 09 03 A0 00 50 00 E8 03 D5 8C` |
| SCAN_STOP seq 10 | `B2 02 31 0A 12 35` |
| CONNECT seq 11: conn 0, 90:84:2B:01:02:03, type auto, background, default interval | `B2 11 32 0B 00 90 84 2B 01 02 03 FF 01 00 00 00 00 00 00 D0 EB` |
| CONNECT seq 12: conn 2, AA:BB:CC:DD:EE:FF, random, timeout 5000 ms, interval 12..24 | `B2 11 32 0C 02 AA BB CC DD EE FF 01 00 88 13 0C 00 18 00 D3 7C` |
| DISCONNECT seq 13, conn 1 | `B2 03 33 0D 01 94 E5` |
| DISCOVER seq 14, conn 1 | `B2 03 34 0E 01 57 35` |
| READ seq 15: conn 1, handle 0x0012 | `B2 05 35 0F 01 12 00 67 E4` |
| WRITE seq 16: conn 0, handle 0x0010, DUPLO motor frame (8 bytes) | `B2 0D 36 10 00 10 00 08 00 81 00 11 51 00 00 03 58` |
| WRITE_CMD seq 17: conn 1, handle 0x002A, MouldKing DIY frame (18 bytes) | `B2 17 37 11 01 2A 00 CC AA BB 01 80 80 80 80 80 80 80 80 80 80 80 80 66 33 83 62` |
| WRITE_CMD seq 18: conn 1, handle 0x002A, 56 data bytes 00..37 (largest frame) | `B2 3D 37 12 01 2A 00 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F 20 21 22 23 24 25 26 27 28 29 2A 2B 2C 2D 2E 2F 30 31 32 33 34 35 36 37 23 F4` |
| SUBSCRIBE seq 19: conn 0, CCCD 0x0011, notify | `B2 06 38 13 00 11 00 01 DF FC` |
| GET_GATT_STATUS seq 20 | `B2 02 39 14 44 4F` |
| SET_FAILSAFE seq 21: conn 1, slot 0, handle 0x002A, MouldKing DIY frame | `B2 18 3A 15 01 00 2A 00 CC AA BB 01 80 80 80 80 80 80 80 80 80 80 80 80 66 33 34 1C` |
| SET_FAILSAFE seq 22: conn 1, slot 1 cleared | `B2 06 3A 16 01 01 2A 00 13 98` |
| READ_RSP seq 15: conn 1, handle 0x0012, data A1 B2 | `B2 08 A0 0F 01 12 00 00 A1 B2 22 91` |
| GATT_STATUS_RSP seq 20: scanning, conn 2 connecting, conn 0+1 connected, 2 scan drops, heap 0x28000 / min 0x14000 | `B2 13 A1 14 01 04 00 03 00 02 00 00 00 00 80 02 00 00 40 01 00 56 F8` |
| ACK seq 16 for WRITE: ATT_ERROR, att error 0x03 (write not permitted) | `B2 05 80 10 36 0E 03 0F 10` |
| ACK seq 11 for CONNECT: OK | `B2 04 80 0B 32 00 34 D6` |
| EVT_NOTIFY: conn 0, handle 0x0011, data 04 00 45 00 01 | `B2 0B C0 00 00 11 00 00 04 00 45 00 01 AC 42` |
| EVT_CONNECTED: conn 0, mtu 64, interval 24 (30 ms), 90:84:2B:01:02:03 | `B2 0D C1 00 00 40 00 18 00 90 84 2B 01 02 03 B0 12` |
| EVT_DISCONNECTED: conn 0, was connected, reason 0x08 (supervision timeout) | `B2 05 C2 00 00 01 08 13 88` |
| EVT_SCAN_REPORT: 90:84:2B:01:02:03 random, -60 dBm, connectable, LEGO manufacturer data 0x0397 | `B2 19 C3 00 90 84 2B 01 02 03 01 C4 01 0D 02 01 06 09 FF 97 03 00 41 20 00 00 00 B9 69` |
| EVT_GATT_SERVICE: conn 0, handles 0x0010-0x0020, 00001623-1212-efde-1623-785feabcd123 | `B2 18 C4 00 00 10 00 20 00 10 23 D1 BC EA 5F 78 23 16 DE EF 12 12 23 16 00 00 FB 31` |
| EVT_GATT_SERVICE: conn 0, handles 0x0030-0x003F, 0x180A | `B2 0A C4 00 00 30 00 3F 00 02 0A 18 21 CF` |
| EVT_GATT_CHAR: conn 0, value 0x0011, CCCD 0x0012, props 0x1E, 00001624-1212-efde-1623-785feabcd123 | `B2 19 C5 00 00 11 00 12 00 1E 10 23 D1 BC EA 5F 78 23 16 DE EF 12 12 24 16 00 00 8F 18` |
| EVT_DISCOVER_DONE: conn 0, OK, 2 services, 5 characteristics | `B2 06 C6 00 00 00 02 05 FD 55` |
| EVT_SCAN_STOPPED: controller error, rc 15 | `B2 05 C7 00 01 0F 00 73 3E` |
| EVT_DROPPED: 42 notifications | `B2 05 C8 00 02 2A 00 C9 FB` |

## 19. Notes for the BC2 host implementation (GATT, task 010)

- **Capabilities first**: read the HELLO_RSP capability block; use GATT only if `proto_minor >= 1` and
  `gatt_flags & 2`. Use `max_conns` as the limit of simultaneous devices (never a constant).
- **Scan data**: the stick forwards raw AD bytes. Merge ADV and SCAN_RSP per address (as the WinUI scanner does);
  LEGO hubs need the active scan and a non-empty name from the scan response. BC2 evaluates AD 0xFF
  (manufacturer data, first two bytes = company id), AD 0x06/0x07 (128-bit service UUIDs) and AD 0x09 (name).
  Request an active scan (`flags` bit0).
- **Connecting**: remember the address type from the scan or pass 0xFF. For `autoConnect` devices (LEGO DUPLO
  train hub) use the background flag; `DISCONNECT` cancels it. BC2 never requests an MTU: the stick negotiates
  64 itself, so a 38 byte BuWizz 3 write works.
- **Discovery** returns the full service tree at once, which maps to `ConnectAndDiscoverServicesAsync`. Keep the
  handles: all later requests use handles, not UUIDs. Notifications arrive with the value handle.
- **Operations**: `WriteAsync` = WRITE, `WriteNoResponseAsync` = WRITE_CMD, `ReadAsync` = READ,
  `EnableNotificationAsync` = SUBSCRIBE(mode 1) with the CCCD handle, `DisableNotificationAsync` = SUBSCRIBE(mode 0).
  Notify only: BC2 never uses indications.
- **BUSY and timing**: retry WRITE_CMD after >= 5 ms on `BUSY`. WRITE with response is limited by the
  connection interval (about 33-66 per second). Do not send a second procedure on a connection while one is
  pending.
- **Keep-alive**: the 250 ms PING of section 11 also protects the GATT links: without it the stick drops all
  links after `keepalive_ms`. After EVT_TIMEOUT / NO_SESSION: HELLO, then reconnect everything.
- **Disconnects**: handle EVT_DISCONNECTED for every conn id; reason 0xF2 means the stick dropped the link by
  itself (safety), not the peripheral.
- **Failsafe**: optional. A host that knows a device keeps running after the link drops can store a "stop"
  write with SET_FAILSAFE after connecting. Per-device stop frames: DUPLO train hub one write of
  `08 00 81 00 11 51 00 00` to its characteristic, MouldKing DIY the 18 byte neutral frame, WeDo 2.0 two writes
  (`01 01 01 00`, `02 01 01 00`); whether these hubs need it is a hardware question (`verification.md` G10).
