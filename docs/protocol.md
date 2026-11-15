# BC2 USB BLE Advertiser - Serial Protocol 1.0

This document is the contract between the stick firmware (task 007) and the BrickController2
host implementation (task 008). The reference implementations are
`components/bc2_protocol` (C) and `tools/bc2adv.py` (Python); both are tested against the golden
vectors in section 10.

Contents: 1 Transport, 2 Frame format, 3 Message types, 4 Requests, 5 Responses and events,
6 Status codes, 7 Safety behaviour, 8 Version policy, 9 Host port settings and detection,
10 Golden vectors, 11 Notes for the BC2 host implementation (task 008), 12 Verified telegram sizes.

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
  Events use SEQ 0, so hosts should not use 0 for requests.

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
| 0x80 | ACK | D->H | Generic response (original type, status, optional extra bytes). |
| 0x81 | HELLO_RSP | D->H | Capabilities. |
| 0x82 | PONG | D->H | Response to PING and to GET_STATUS(0xFF). |
| 0x83 | SET_STATUS_RSP | D->H | Response to GET_STATUS(set_id). |
| 0xE0 | EVT_SET_STOPPED | D->H | A set ended without a host request. |
| 0xE1 | EVT_BUTTON | D->H | Button pressed, all sets were stopped. |
| 0xE2 | EVT_TIMEOUT | D->H | Keep-alive timeout, all sets were stopped (best effort). |

### 3.1 Reserved TYPE ranges (not implemented in 1.x)

The following ranges are **reserved for future messages** and are not implemented. A device that
receives a request with any type that it does not implement, including all reserved types and all
device-to-host types, answers `ACK(orig_type, UNKNOWN_CMD)` like for any other unknown type (also
before `HELLO`). Hosts must not send them.

| Range | Direction | Reserved for |
|-------|-----------|--------------|
| 0x20 - 0x2F | H->D | future advertising / device management requests |
| 0x30 - 0x4F | H->D | future GATT requests (stick as GATT central / proxy: scan, connect, discover, read, write, subscribe) |
| 0xA0 - 0xBF | D->H | responses to the future GATT requests |
| 0xC0 - 0xDF | D->H | asynchronous GATT events (notifications, disconnects) |
| 0xE3 - 0xEF | D->H | future device events |
| 0xF0 - 0xFF | both | experimental / vendor use, never assigned |

When GATT messages are added they will be announced with a higher `proto_minor` in HELLO_RSP, so a
host can check support before sending them.

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

**HELLO_RSP (0x81)**, 27 bytes:

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
| 1 | UNKNOWN_CMD | TYPE not implemented (includes reserved ranges, section 3.1) |
| 2 | BAD_LENGTH | payload length does not match the request layout |
| 3 | BAD_PARAM | value out of range / reserved bits set / set_id >= max_sets |
| 4 | NO_SESSION | HELLO required first (also after a keep-alive timeout) |
| 5 | VERSION_UNSUPPORTED | proto_major mismatch |
| 6 | ALREADY_RUNNING | SET_START on an active id |
| 7 | SET_NOT_FOUND | update/stop on an id that is not running |
| 8 | PAYLOAD_TOO_LARGE | data_len above the limit |
| 9 | BLE_ERROR | NimBLE/controller error; ACK extra = `rc u16` (0xFFFF = BLE host not synced yet) |
| 10 | NO_RESOURCES | out of memory or advertising instances |

Lost or corrupt frames are detected by a response timeout (suggested 100 ms). All requests are
idempotent except SET_START, which answers `ALREADY_RUNNING` on a duplicate; a host that knows its
first SET_START may have arrived treats that as success.

## 7. Safety behaviour (normative)

- Boot: no advertising, no session.
- A session ends (all sets stopped, state "no session") on: keep-alive timeout, and `HELLO`
  (restart). USB unplug removes the power of the stick, which also ends advertising.
- Keep-alive timeout: if no **valid frame** was received for `keepalive_ms` (garbage and corrupt frames
  do not count), the device stops all sets, ends the session, sends EVT_TIMEOUT (best effort) and shows
  the TIMEOUT LED state. The host must send HELLO again; other requests answer NO_SESSION. This also
  covers "COM port closed" and crashed host software, because the device cannot see the line state.
- Button short press: stops all sets, **keeps** the session, sends EVT_BUTTON.
- A failed stop request is retried once; if the controller may still be advertising the firmware resets
  the chip (guaranteed silence). The task watchdog resets the chip if the USB or core task hangs.

## 8. Version policy

`proto_major` changes on incompatible changes, `proto_minor` on additive ones (new optional flag bits,
new message types from the reserved ranges, longer response payloads at the end). Request layouts are
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
`test/host/test_codec.c` and `tools/tests/test_framing.py`.

| Frame | Bytes |
|-------|-------|
| HELLO seq 1, proto 1.0, keepalive 1000 | `B2 06 01 01 01 00 E8 03 D6 60` |
| PING seq 2 | `B2 02 02 02 DC E4` |
| SET_START seq 3: set 0, flags 0x01, interval 32, tx 127, mfr 0xFFF0, data 01..08 | `B2 12 10 03 00 01 20 00 7F F0 FF 08 01 02 03 04 05 06 07 08 F5 48` |
| SET_UPDATE seq 4: set 0, mfr 0xFFF0, data A1 A2 A3 A4 | `B2 0A 11 04 00 F0 FF 04 A1 A2 A3 A4 52 B1` |
| SET_STOP seq 5, set 0 | `B2 03 12 05 00 EA CD` |
| STOP_ALL seq 6 | `B2 02 13 06 1A 94` |
| GET_STATUS seq 7, summary | `B2 03 14 07 FF D8 07` |
| HELLO_RSP seq 1: proto 1.0, fw 0.1.0, 5 sets, 24/27, interval 32..16384, keepalive 1000, id 24:0A:BC:11:22:33 | `B2 1D 81 01 42 43 32 41 44 56 00 01 00 00 01 00 05 18 1B 20 00 00 40 E8 03 24 0A BC 11 22 33 40 AD` |
| ACK seq 3 for SET_START, OK, applied tx 127 | `B2 05 80 03 10 00 7F 06 38` |
| ACK seq 4 for SET_UPDATE, OK, 891 us | `B2 08 80 04 11 00 7B 03 00 00 D4 57` |
| PONG seq 2: advertising, set 0, 0 errors, uptime 10000 ms | `B2 0B 82 02 01 01 00 00 00 10 27 00 00 5C D9` |
| EVT_TIMEOUT, 1000 ms | `B2 04 E2 00 E8 03 F0 58` |

AD data example for the SET_START above (flags bit set):
`02 01 06 0B FF F0 FF 01 02 03 04 05 06 07 08` (15 bytes on air).

## 11. Notes for the BC2 host implementation (task 008)

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
