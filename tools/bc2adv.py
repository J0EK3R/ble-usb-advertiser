#!/usr/bin/env python3
"""Host test tool for the BC2 USB BLE advertiser firmware.

Implements the serial protocol of docs/protocol.md (framing, CRC, requests, responses) and a
small command line on top of it:

    ports   list candidate serial ports and probe them with HELLO
    hello   print the capabilities of the stick
    hold    start one advertising set, keep it alive, stop it afterwards
    shell   interactive prompt with a background keep-alive
    bench   measure update round-trip time and device processing time
    fuzz    send garbage / truncated / corrupted frames between valid ones
    scan, connect, discover, read, write, write-cmd, notify, disconnect, gatt-status, failsafe,
    gatt-bench, gatt-shell, connspike   GATT central tests (protocol 1.1, docs/protocol.md 13+)

The framing part (crc16, Frame, encode, Decoder) does not need pyserial so it can be unit tested
without hardware (tools/tests/test_framing.py).
"""
import argparse
import queue
import struct
import sys
import threading
import time

# ---------------------------------------------------------------------------
# Protocol constants (mirror components/bc2_protocol/include/bc2_protocol.h)
# ---------------------------------------------------------------------------
SOF = 0xB2
MIN_LEN, MAX_LEN, MAX_PAYLOAD = 2, 62, 60
FRAME_TIMEOUT_S = 0.2
PROTO_MAJOR, PROTO_MINOR = 1, 1
MAGIC = b"BC2ADV"
USB_VID, USB_PID = 0x303A, 0x1001  # fixed identity of the ESP32-S3 USB Serial/JTAG peripheral

MSG_HELLO, MSG_PING = 0x01, 0x02
MSG_SET_START, MSG_SET_UPDATE, MSG_SET_STOP, MSG_STOP_ALL, MSG_GET_STATUS = 0x10, 0x11, 0x12, 0x13, 0x14
MSG_ACK, MSG_HELLO_RSP, MSG_PONG, MSG_SET_STATUS_RSP = 0x80, 0x81, 0x82, 0x83
MSG_EVT_SET_STOPPED, MSG_EVT_BUTTON, MSG_EVT_TIMEOUT = 0xE0, 0xE1, 0xE2
# Protocol 1.1: scanning and GATT central
(MSG_SCAN_START, MSG_SCAN_STOP, MSG_CONNECT, MSG_DISCONNECT, MSG_DISCOVER, MSG_READ, MSG_WRITE,
 MSG_WRITE_CMD, MSG_SUBSCRIBE, MSG_GET_GATT_STATUS, MSG_SET_FAILSAFE) = range(0x30, 0x3B)
MSG_READ_RSP, MSG_GATT_STATUS_RSP = 0xA0, 0xA1
(MSG_EVT_NOTIFY, MSG_EVT_CONNECTED, MSG_EVT_DISCONNECTED, MSG_EVT_SCAN_REPORT, MSG_EVT_GATT_SERVICE,
 MSG_EVT_GATT_CHAR, MSG_EVT_DISCOVER_DONE, MSG_EVT_SCAN_STOPPED, MSG_EVT_DROPPED) = range(0xC0, 0xC9)
FIRST_EVENT_TYPE = 0xC0  # device->host types from here on carry SEQ 0 and are no responses
MAX_VALUE, MAX_SCAN_DATA, MAX_CONN_ID, FAILSAFE_SLOTS, FAILSAFE_MAX_DATA = 56, 31, 15, 4, 20
ADDR_AUTO = 0xFF

STATUS_NAMES = {0: "OK", 1: "UNKNOWN_CMD", 2: "BAD_LENGTH", 3: "BAD_PARAM", 4: "NO_SESSION",
                5: "VERSION_UNSUPPORTED", 6: "ALREADY_RUNNING", 7: "SET_NOT_FOUND",
                8: "PAYLOAD_TOO_LARGE", 9: "BLE_ERROR", 10: "NO_RESOURCES", 11: "BUSY",
                12: "NOT_CONNECTED", 13: "TIMEOUT", 14: "ATT_ERROR", 15: "DISCONNECTED",
                16: "NOT_SUPPORTED"}
DISCONNECT_REASONS = {0xF0: "connect timeout", 0xF1: "cancelled", 0xF2: "stick safety stop",
                      0xF3: "stick internal error"}
PDU_TYPES = {"ind": 0, "scan": 1, "nonconn": 2}


# ---------------------------------------------------------------------------
# Framing
# ---------------------------------------------------------------------------
def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0)."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Frame:
    """A protocol frame without SOF/LEN/CRC."""

    def __init__(self, type_, seq, payload=b""):
        self.type = type_
        self.seq = seq
        self.payload = bytes(payload)

    def __eq__(self, other):
        return (self.type, self.seq, self.payload) == (other.type, other.seq, other.payload)

    def __repr__(self):
        return "Frame(type=0x%02X, seq=%d, payload=%s)" % (self.type, self.seq, self.payload.hex())


def encode(frame):
    """Serialize a Frame to wire bytes: SOF LEN TYPE SEQ PAYLOAD CRC16(LE)."""
    if len(frame.payload) > MAX_PAYLOAD:
        raise ValueError("payload too large")
    body = bytes([len(frame.payload) + 2, frame.type, frame.seq]) + frame.payload
    return bytes([SOF]) + body + struct.pack("<H", crc16(body))


class Decoder:
    """Streaming parser with the same resync rules as the firmware.

    Bad LEN or bad CRC drops only the SOF byte and rescans the buffered bytes; a partial frame
    older than FRAME_TIMEOUT_S is dropped when the next byte arrives. Garbage before a SOF is
    skipped silently. `parse_errors` counts dropped frames.
    """

    def __init__(self):
        self.buf = bytearray()
        self.start = 0.0
        self.parse_errors = 0

    def feed(self, data, now=None):
        """Feed bytes (timestamp `now` in seconds, default: time.monotonic()); return new Frames."""
        now = time.monotonic() if now is None else now
        frames = []
        for byte in data:
            if self.buf and now - self.start > FRAME_TIMEOUT_S:
                self.buf.clear()  # stale partial frame must not swallow this one
                self.parse_errors += 1
            if not self.buf:
                self.start = now
            self.buf.append(byte)
            frames.extend(self._drain())
        return frames

    def _drain(self):
        """Extract every complete valid frame currently in the buffer."""
        out = []
        while self.buf:
            if self.buf[0] != SOF:
                nxt = self.buf.find(bytes([SOF]))
                del self.buf[: nxt if nxt > 0 else len(self.buf)]
                continue
            if len(self.buf) < 2:
                break
            length = self.buf[1]
            if length < MIN_LEN or length > MAX_LEN:
                del self.buf[0]
                self.parse_errors += 1
                continue
            total = length + 4
            if len(self.buf) < total:
                break
            crc_rx = self.buf[total - 2] | (self.buf[total - 1] << 8)
            if crc16(bytes(self.buf[1: total - 2])) != crc_rx:
                del self.buf[0]  # only the SOF: a frame may start inside the corrupted bytes
                self.parse_errors += 1
                continue
            out.append(Frame(self.buf[2], self.buf[3], self.buf[4: total - 2]))
            del self.buf[:total]
        return out


# ---------------------------------------------------------------------------
# Message builders / parsers
# ---------------------------------------------------------------------------
def hello_payload(keepalive_ms=0, minor=PROTO_MINOR):
    """HELLO request: proto_major(1) proto_minor(1) keepalive_ms(2)."""
    return struct.pack("<BBH", PROTO_MAJOR, minor, keepalive_ms)


def set_start_payload(set_id, mfr, data, interval=32, tx=127, add_flags=True, pdu="ind"):
    flags = (1 if add_flags else 0) | (PDU_TYPES[pdu] << 1)
    return struct.pack("<BBHbHB", set_id, flags, interval, tx, mfr, len(data)) + bytes(data)


def set_update_payload(set_id, mfr, data):
    return struct.pack("<BHB", set_id, mfr, len(data)) + bytes(data)


def addr_to_bytes(text):
    """'AA:BB:CC:DD:EE:FF' -> 6 bytes in display order (byte 0 = first octet), as on the wire."""
    raw = bytes.fromhex(text.replace(":", "").replace("-", ""))
    if len(raw) != 6:
        raise ValueError("address must have 6 octets")
    return raw


def addr_to_text(raw):
    """6 bytes in display order -> 'AA:BB:CC:DD:EE:FF'."""
    return bytes(raw).hex(":").upper()


def scan_start_payload(active=True, duplicates=False, interval=0, window=0, refresh_ms=0):
    """SCAN_START: flags(1) interval(2) window(2) refresh_ms(2); 0 values select the defaults."""
    flags = (1 if active else 0) | (2 if duplicates else 0)
    return struct.pack("<BHHH", flags, interval, window, refresh_ms)


def connect_payload(conn_id, addr, addr_type=ADDR_AUTO, background=False, timeout_ms=0, itvl_min=0,
                    itvl_max=0):
    """CONNECT: conn_id(1) addr(6) addr_type(1) flags(1) timeout_ms(2) itvl_min(2) itvl_max(2)."""
    return (bytes([conn_id]) + bytes(addr) +
            struct.pack("<BBHHH", addr_type, 1 if background else 0, timeout_ms, itvl_min, itvl_max))


def conn_id_payload(conn_id):
    """DISCONNECT and DISCOVER: conn_id(1)."""
    return bytes([conn_id])


def read_payload(conn_id, handle):
    """READ: conn_id(1) handle(2)."""
    return struct.pack("<BH", conn_id, handle)


def write_payload(conn_id, handle, data):
    """WRITE and WRITE_CMD: conn_id(1) handle(2) data(1..56)."""
    if not 1 <= len(data) <= MAX_VALUE:
        raise ValueError("data must have 1..%d bytes" % MAX_VALUE)
    return struct.pack("<BH", conn_id, handle) + bytes(data)


def subscribe_payload(conn_id, cccd_handle, mode):
    """SUBSCRIBE: conn_id(1) cccd_handle(2) mode(1: 0 off, 1 notify, 2 indicate)."""
    return struct.pack("<BHB", conn_id, cccd_handle, mode)


def set_failsafe_payload(conn_id, slot, handle, data):
    """SET_FAILSAFE: conn_id(1) slot(1) handle(2) data(0..20); empty data clears the slot."""
    if len(data) > FAILSAFE_MAX_DATA:
        raise ValueError("failsafe data is limited to %d bytes" % FAILSAFE_MAX_DATA)
    return struct.pack("<BBH", conn_id, slot, handle) + bytes(data)


def uuid_text(raw):
    """UUID bytes in ATT wire order (2 or 16 bytes) -> text (16 bit: '1624', 128 bit: canonical)."""
    raw = bytes(raw)
    if len(raw) == 2:
        return "%04x" % struct.unpack("<H", raw)
    be = raw[::-1].hex()
    return "%s-%s-%s-%s-%s" % (be[:8], be[8:12], be[12:16], be[16:20], be[20:])


def parse_adv_data(data):
    """Split raw AD structures into [(ad_type, value_bytes)]; stops at a zero length or truncation."""
    out, pos = [], 0
    while pos < len(data) and data[pos] != 0:
        length = data[pos]
        if pos + 1 + length > len(data):
            break
        out.append((data[pos + 1], bytes(data[pos + 2: pos + 1 + length])))
        pos += 1 + length
    return out


class AdvMerger:
    """Merges ADV and SCAN_RSP reports of one address (BC2 needs both: LEGO hubs put the name into the scan response)."""

    def __init__(self):
        self.devices = {}

    def update(self, report):
        """Add a parsed scan report (see parse_event); returns the merged record of its address."""
        rec = self.devices.setdefault(report["addr"], {"addr": report["addr"], "ad": {}})
        rec.update(addr_type=report["addr_type"], rssi=report["rssi"])
        if not report["scan_rsp"]:
            rec["connectable"] = report["connectable"]
        for ad_type, value in parse_adv_data(report["data"]):
            rec["ad"][ad_type] = value
        rec["name"] = rec["ad"].get(0x09, rec["ad"].get(0x08, b"")).decode("utf-8", "replace")
        rec["mfr_id"] = struct.unpack("<H", rec["ad"][0xFF][:2])[0] if len(rec["ad"].get(0xFF, b"")) >= 2 else None
        return rec


def parse_read_rsp(payload):
    conn_id, handle, flags = struct.unpack_from("<BHB", payload)
    return {"conn_id": conn_id, "handle": handle, "truncated": bool(flags & 1), "data": payload[4:]}


def parse_gatt_status(payload):
    scanning, connecting, connected, dropped_scan, dropped_notify = struct.unpack("<BHHHH", payload[:9])
    free_heap, min_free_heap = struct.unpack_from("<II", payload, 9) if len(payload) >= 17 else (0, 0)
    return {"scanning": bool(scanning), "connecting_mask": connecting, "connected_mask": connected,
            "dropped_scan": dropped_scan, "dropped_notify": dropped_notify,
            "free_heap": free_heap, "min_free_heap": min_free_heap}


def parse_event(frame):
    """Decode an event frame (0xC0..0xC8 and the 1.0 events) into a dict with a 'name' key."""
    p, t = frame.payload, frame.type
    if t == MSG_EVT_NOTIFY:
        conn_id, handle, flags = struct.unpack_from("<BHB", p)
        return {"name": "NOTIFY", "conn_id": conn_id, "handle": handle, "indication": bool(flags & 1),
                "truncated": bool(flags & 2), "data": p[4:]}
    if t == MSG_EVT_CONNECTED:
        conn_id, mtu, itvl = struct.unpack_from("<BHH", p)
        return {"name": "CONNECTED", "conn_id": conn_id, "mtu": mtu, "itvl": itvl, "addr": addr_to_text(p[5:11])}
    if t == MSG_EVT_DISCONNECTED:
        return {"name": "DISCONNECTED", "conn_id": p[0], "phase": p[1], "reason": p[2]}
    if t == MSG_EVT_SCAN_REPORT:
        flags, length = p[8], p[9]
        return {"name": "SCAN_REPORT", "addr": addr_to_text(p[0:6]), "addr_type": p[6],
                "rssi": struct.unpack("b", p[7:8])[0], "connectable": bool(flags & 1),
                "scan_rsp": bool(flags & 2), "directed": bool(flags & 4), "data": p[10:10 + length]}
    if t == MSG_EVT_GATT_SERVICE:
        conn_id, start, end, ulen = struct.unpack_from("<BHHB", p)
        return {"name": "SERVICE", "conn_id": conn_id, "start": start, "end": end, "uuid": uuid_text(p[6:6 + ulen])}
    if t == MSG_EVT_GATT_CHAR:
        conn_id, value, cccd, props, ulen = struct.unpack_from("<BHHBB", p)
        return {"name": "CHAR", "conn_id": conn_id, "value_handle": value, "cccd_handle": cccd,
                "props": props, "uuid": uuid_text(p[7:7 + ulen])}
    if t == MSG_EVT_DISCOVER_DONE:
        return {"name": "DISCOVER_DONE", "conn_id": p[0], "status": p[1], "n_services": p[2], "n_chars": p[3]}
    if t == MSG_EVT_SCAN_STOPPED:
        return {"name": "SCAN_STOPPED", "reason": p[0], "rc": struct.unpack_from("<H", p, 1)[0]}
    if t == MSG_EVT_DROPPED:
        return {"name": "DROPPED", "kind": "scan" if p[0] == 1 else "notify", "count": struct.unpack_from("<H", p, 1)[0]}
    if t == MSG_EVT_SET_STOPPED:
        return {"name": "SET_STOPPED", "set_id": p[0], "reason": p[1]}
    if t == MSG_EVT_BUTTON:
        return {"name": "BUTTON"}
    if t == MSG_EVT_TIMEOUT:
        return {"name": "TIMEOUT", "timeout_ms": struct.unpack_from("<H", p)[0]}
    return {"name": "UNKNOWN_0x%02X" % t, "payload": p.hex()}


def parse_hello_rsp(payload):
    """Decode a HELLO_RSP payload into a dict, or return None if it is not one."""
    if len(payload) < 27 or payload[:6] != MAGIC:
        return None
    (status, pmaj, pmin, fmaj, fmin, fpat, max_sets, dwf, dnf,
     imin, imax, keepalive) = struct.unpack_from("<BBBBBBBBBHHH", payload, 6)
    caps = {"status": status, "proto": "%d.%d" % (pmaj, pmin), "fw": "%d.%d.%d" % (fmaj, fmin, fpat),
            "max_sets": max_sets, "max_data_with_flags": dwf, "max_data_no_flags": dnf,
            "min_interval": imin, "max_interval": imax, "keepalive_ms": keepalive,
            "device_id": payload[21:27].hex(":")}
    # Protocol 1.1 appends the GATT capability block; a 1.0 device sends only 27 bytes.
    gatt_flags, max_conns, max_value, max_scan, failsafe_slots = (
        struct.unpack_from("<5B", payload, 27) if len(payload) >= 32 else (0, 0, 0, 0, 0))
    caps.update({"gatt_flags": gatt_flags, "max_conns": max_conns, "max_value_len": max_value,
                 "max_scan_data": max_scan, "failsafe_slots": failsafe_slots})
    return caps


def parse_pong(payload):
    state, mask, errors, uptime = struct.unpack("<BHHI", payload[:9])
    return {"state": state, "active_mask": mask, "parse_errors": errors, "uptime_ms": uptime}


# ---------------------------------------------------------------------------
# Serial link (needs pyserial)
# ---------------------------------------------------------------------------
class DeviceError(Exception):
    """The device answered with a non-OK status or did not answer.

    `status` is the protocol status code (None if there was no answer), `extra` the ACK extra bytes
    (for ATT_ERROR: the ATT error code, for BLE_ERROR: the NimBLE rc).
    """

    def __init__(self, message, status=None, extra=b""):
        super().__init__(message)
        self.status = status
        self.extra = extra


class Link:
    """Serial connection with a reader thread, request/response matching and event queue."""

    def __init__(self, port, verbose=False):
        import serial  # imported here so the framing code works without pyserial
        self.verbose = verbose
        self.decoder = Decoder()
        self.events = queue.Queue()
        self.responses = {}
        self.cond = threading.Condition()
        self.write_lock = threading.Lock()
        self.seq = 0
        self.running = True
        # DTR/RTS must stay de-asserted: toggling them can reset the chip (docs/protocol.md 9).
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = 115200
        self.ser.timeout = 0.05
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        self.ser.reset_input_buffer()
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()

    def _log(self, direction, data):
        if self.verbose:
            print("%s %s" % (direction, bytes(data).hex(" ")), file=sys.stderr)

    def _read_loop(self):
        """Decode incoming bytes; responses are stored by SEQ, events (type >= 0xC0) are queued."""
        while self.running:
            try:
                data = self.ser.read(256)
            except Exception:  # port vanished
                break
            if not data:
                continue
            self._log("<-", data)
            for frame in self.decoder.feed(data):
                if frame.type >= FIRST_EVENT_TYPE:
                    self.events.put(frame)
                else:
                    with self.cond:
                        self.responses[frame.seq] = frame
                        self.cond.notify_all()

    def request(self, type_, payload=b"", timeout=0.1, retries=0):
        """Send a request and wait for the frame with the same SEQ. Returns (Frame, rtt_seconds)."""
        for _ in range(retries + 1):
            with self.write_lock:
                self.seq = (self.seq % 255) + 1
                seq = self.seq
                wire = encode(Frame(type_, seq, payload))
                self._log("->", wire)
                t0 = time.perf_counter()
                self.ser.write(wire)
                self.ser.flush()
            with self.cond:
                if self.cond.wait_for(lambda: seq in self.responses, timeout):
                    return self.responses.pop(seq), time.perf_counter() - t0
        raise DeviceError("no response to request 0x%02X" % type_)

    def send_raw(self, data):
        """Write raw bytes (fuzzing)."""
        with self.write_lock:
            self._log("->", data)
            self.ser.write(data)
            self.ser.flush()

    def hello(self, keepalive_ms=1000, retries=3, timeout=0.2):
        """HELLO handshake; returns the capabilities dict or raises DeviceError."""
        frame, _ = self.request(MSG_HELLO, hello_payload(keepalive_ms), timeout, retries)
        caps = parse_hello_rsp(frame.payload) if frame.type == MSG_HELLO_RSP else None
        if caps is None:
            raise DeviceError("not a BC2 advertiser (response 0x%02X)" % frame.type)
        return caps

    def command(self, type_, payload=b"", timeout=0.1, retries=2):
        """Request that must be answered with ACK(OK); returns (extra bytes, rtt).

        Advertising requests are idempotent and may be retransmitted. GATT requests must NOT be
        (retries=0): a duplicate write would be executed twice (docs/protocol.md section 14).
        """
        frame, rtt = self.request(type_, payload, timeout, retries)
        if frame.type != MSG_ACK:
            return frame.payload, rtt
        status = frame.payload[1]
        if status != 0:
            raise DeviceError("%s (0x%02X): %s" % (STATUS_NAMES.get(status, status), type_,
                                                   frame.payload[2:].hex()), status=status,
                              extra=bytes(frame.payload[2:]))
        return frame.payload[2:], rtt

    def gatt(self, type_, payload=b"", timeout=6.0):
        """GATT request: no retransmission, waits up to 6 s (the stick answers within 5 s)."""
        return self.command(type_, payload, timeout, 0)

    def close(self):
        self.running = False
        self.reader.join(timeout=1)
        self.ser.close()


def candidate_ports():
    """Serial ports with the VID/PID of the ESP32-S3 USB Serial/JTAG peripheral."""
    from serial.tools import list_ports
    return [p for p in list_ports.comports() if (p.vid, p.pid) == (USB_VID, USB_PID)]


def find_port(explicit=None, verbose=False):
    """Return the port name: the explicit one, or the first candidate answering HELLO with the magic."""
    if explicit:
        return explicit
    for info in candidate_ports():
        try:
            link = Link(info.device, verbose)
            try:
                link.hello(retries=2)
                return info.device
            finally:
                link.close()
        except Exception:
            continue
    raise DeviceError("no BC2 advertiser found (try --port)")


def open_link(args):
    """Open a Link on the selected/auto-detected port and perform HELLO."""
    link = Link(find_port(args.port, args.verbose), args.verbose)
    caps = link.hello(args.keepalive)
    return link, caps


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------
def parse_int(text):
    return int(text, 0)


def keepalive_loop(link, caps, stop_event):
    """Send PING every keepalive/4 until stop_event is set (must be <= 250 ms for BC2)."""
    period = min(caps["keepalive_ms"] / 4000.0, 0.25)
    while not stop_event.wait(period):
        try:
            link.request(MSG_PING)
        except DeviceError:
            print("warning: PING not answered", file=sys.stderr)


def cmd_ports(args):
    from serial.tools import list_ports
    for info in list_ports.comports():
        probe = ""
        if (info.vid, info.pid) == (USB_VID, USB_PID):
            try:
                link = Link(info.device, args.verbose)
                try:
                    caps = link.hello(retries=2)
                    probe = "BC2 advertiser fw %s, %d sets" % (caps["fw"], caps["max_sets"])
                finally:
                    link.close()
            except Exception as exc:
                probe = "no BC2 answer (%s)" % exc
        print("%-12s VID:PID=%04X:%04X %s" % (info.device, info.vid or 0, info.pid or 0, probe))


def cmd_hello(args):
    link, caps = open_link(args)
    try:
        for key, value in caps.items():
            print("%-20s %s" % (key, value))
    finally:
        link.close()


def cmd_hold(args):
    link, caps = open_link(args)
    stop_event = threading.Event()
    try:
        extra, _ = link.command(MSG_SET_START, set_start_payload(
            args.set, args.mfr, bytes.fromhex(args.data), args.interval, args.tx,
            not args.no_flags, args.pdu))
        print("set %d started, applied tx power %d dBm" % (args.set, struct.unpack("b", extra[:1])[0]))
        ping = threading.Thread(target=keepalive_loop, args=(link, caps, stop_event), daemon=True)
        ping.start()
        deadline = time.monotonic() + args.seconds if args.seconds else None
        while deadline is None or time.monotonic() < deadline:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        stop_event.set()
        if not args.no_stop:
            try:
                link.command(MSG_STOP_ALL)
                print("stopped")
            except DeviceError as exc:
                print("stop failed: %s" % exc)
        else:
            print("--no-stop: leaving it to the device keep-alive timeout (%d ms)" % caps["keepalive_ms"])
        link.close()


def cmd_shell(args):
    link, caps = open_link(args)
    stop_event = threading.Event()
    threading.Thread(target=keepalive_loop, args=(link, caps, stop_event), daemon=True).start()
    print("commands: start <id> <mfr> <hex> | update <id> <mfr> <hex> | stop <id> | stopall | status [id] | quit")
    try:
        while True:
            try:
                parts = input("bc2> ").split()
            except EOFError:
                break
            if not parts:
                continue
            try:
                if parts[0] == "quit":
                    break
                elif parts[0] == "start":
                    link.command(MSG_SET_START, set_start_payload(
                        int(parts[1]), parse_int(parts[2]), bytes.fromhex(parts[3])))
                elif parts[0] == "update":
                    extra, rtt = link.command(MSG_SET_UPDATE, set_update_payload(
                        int(parts[1]), parse_int(parts[2]), bytes.fromhex(parts[3])))
                    print("device %d us, round trip %.1f ms" % (struct.unpack("<I", extra)[0], rtt * 1000))
                elif parts[0] == "stop":
                    link.command(MSG_SET_STOP, bytes([int(parts[1])]))
                elif parts[0] == "stopall":
                    link.command(MSG_STOP_ALL)
                elif parts[0] == "status":
                    sid = int(parts[1]) if len(parts) > 1 else 0xFF
                    frame, _ = link.request(MSG_GET_STATUS, bytes([sid]))
                    print(parse_pong(frame.payload) if frame.type == MSG_PONG else frame.payload.hex())
                else:
                    print("unknown command")
            except (DeviceError, IndexError, ValueError) as exc:
                print("error: %s" % exc)
            while not link.events.empty():
                print("event:", link.events.get())
    finally:
        stop_event.set()
        try:
            link.command(MSG_STOP_ALL)
        except DeviceError:
            pass
        link.close()


def cmd_bench(args):
    link, caps = open_link(args)
    stop_event = threading.Event()
    threading.Thread(target=keepalive_loop, args=(link, caps, stop_event), daemon=True).start()
    rtts, device_us = [], []
    try:
        link.command(MSG_SET_START, set_start_payload(args.set, args.mfr, bytes(8)))
        period = 1.0 / args.rate
        end = time.monotonic() + args.seconds
        payloads = (bytes([0x11] * 8), bytes([0x22] * 8))
        count = 0
        while time.monotonic() < end:
            t_next = time.monotonic() + period
            extra, rtt = link.command(MSG_SET_UPDATE, set_update_payload(
                args.set, args.mfr, payloads[count % 2]))
            rtts.append(rtt * 1000)
            device_us.append(struct.unpack("<I", extra)[0])
            count += 1
            time.sleep(max(0.0, t_next - time.monotonic()))
    finally:
        stop_event.set()
        try:
            link.command(MSG_STOP_ALL)
        except DeviceError:
            pass
        link.close()
    if rtts:
        print("%d updates; round trip ms min/avg/max = %.2f/%.2f/%.2f; device us min/avg/max = %d/%d/%d" % (
            len(rtts), min(rtts), sum(rtts) / len(rtts), max(rtts),
            min(device_us), sum(device_us) // len(device_us), max(device_us)))


def cmd_fuzz(args):
    """Interleave garbage, truncated and corrupted frames with valid PINGs; PINGs must still work."""
    link, caps = open_link(args)
    good = bad = errors = 0
    junk = [b"\x00\xff\x13garbage", encode(Frame(MSG_PING, 1))[:-1],
            bytes([SOF, 0x01, 0x02]), bytes([SOF, 0x3F]) + bytes(70),
            encode(Frame(MSG_PING, 2))[:-1] + b"\x00"]
    try:
        for i in range(args.rounds):
            link.send_raw(junk[i % len(junk)])
            time.sleep(0.25)  # lets the 200 ms inter-frame timeout drop a truncated frame
            try:
                frame, _ = link.request(MSG_PING, retries=1)
                good += frame.type == MSG_PONG
                errors = parse_pong(frame.payload)["parse_errors"]
            except DeviceError:
                bad += 1
        print("%d/%d PINGs answered after junk, device parse_errors=%d" % (good, args.rounds, errors))
    finally:
        link.close()
    return 0 if bad == 0 else 1


# ---------------------------------------------------------------------------
# GATT central session (protocol 1.1)
# ---------------------------------------------------------------------------
def latency_stats(values):
    """Summary of a list of latencies: count, min, mean, median, p95, max (same unit as the input)."""
    if not values:
        return {"count": 0}
    ordered = sorted(values)
    return {"count": len(ordered), "min": ordered[0], "mean": sum(ordered) / len(ordered),
            "p50": ordered[len(ordered) // 2], "p95": ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))],
            "max": ordered[-1]}


def format_stats(stats, unit="ms"):
    """One line text for latency_stats()."""
    if not stats["count"]:
        return "no samples"
    return "n=%d min/mean/p50/p95/max = %.2f/%.2f/%.2f/%.2f/%.2f %s" % (
        stats["count"], stats["min"], stats["mean"], stats["p50"], stats["p95"], stats["max"], unit)


def format_event(ev):
    """Human readable text of a parse_event() dict."""
    name = ev["name"]
    if name == "SCAN_REPORT":
        return "SCAN %s type %d rssi %d%s%s data %s" % (
            ev["addr"], ev["addr_type"], ev["rssi"], " conn" if ev["connectable"] else "",
            " rsp" if ev["scan_rsp"] else "", ev["data"].hex())
    if name == "NOTIFY":
        return "NOTIFY conn %d handle 0x%04X%s%s data %s" % (
            ev["conn_id"], ev["handle"], " (indication)" if ev["indication"] else "",
            " (truncated)" if ev["truncated"] else "", ev["data"].hex())
    if name == "DISCONNECTED":
        return "DISCONNECTED conn %d %s reason 0x%02X %s" % (
            ev["conn_id"], "after connect" if ev["phase"] else "while connecting", ev["reason"],
            DISCONNECT_REASONS.get(ev["reason"], ""))
    return "%s %s" % (name, {k: v for k, v in ev.items() if k != "name"})


class GattSession:
    """Event-driven GATT client on top of a Link: tracks scan results, connections, discovery
    results and notifications, and offers blocking helpers for every request of protocol 1.1."""

    def __init__(self, link, caps, echo=None):
        self.link, self.caps, self.echo = link, caps, echo
        self.cond = threading.Condition()
        self.adv = AdvMerger()
        self.connected, self.disconnected = {}, {}
        self.services, self.chars, self.discovery = {}, {}, {}
        self.notifications = {}          # conn_id -> number of EVT_NOTIFY seen
        self.dropped = {"scan": 0, "notify": 0}
        self.scan_stopped = None
        self.running = True
        self.thread = threading.Thread(target=self._pump, daemon=True)
        self.thread.start()
        if not caps.get("gatt_flags", 0) & 2:
            raise DeviceError("this firmware has no GATT central support (proto %s)" % caps["proto"])

    def close(self):
        self.running = False
        self.thread.join(timeout=1)

    def _pump(self):
        """Consume link events and update the session state."""
        while self.running:
            try:
                frame = self.link.events.get(timeout=0.1)
            except queue.Empty:
                continue
            ev = parse_event(frame)
            with self.cond:
                self._apply(ev)
                self.cond.notify_all()
            if self.echo:
                self.echo(ev)

    def _apply(self, ev):
        """Update the state from one event (caller holds the condition lock)."""
        name, conn = ev["name"], ev.get("conn_id")
        if name == "SCAN_REPORT":
            self.adv.update(ev)
        elif name == "CONNECTED":
            self.connected[conn] = ev
            self.disconnected.pop(conn, None)
        elif name == "DISCONNECTED":
            self.connected.pop(conn, None)
            self.disconnected[conn] = ev
        elif name == "SERVICE":
            self.services.setdefault(conn, []).append(ev)
        elif name == "CHAR":
            self.chars.setdefault(conn, []).append(ev)
        elif name == "DISCOVER_DONE":
            self.discovery[conn] = ev
        elif name == "NOTIFY":
            self.notifications[conn] = self.notifications.get(conn, 0) + 1
        elif name == "DROPPED":
            self.dropped[ev["kind"]] += ev["count"]
        elif name == "SCAN_STOPPED":
            self.scan_stopped = ev

    def wait_for(self, predicate, timeout):
        """Block until predicate() is true (evaluated under the lock) or the timeout expires."""
        with self.cond:
            return self.cond.wait_for(predicate, timeout)

    # ---- requests ----
    def scan_start(self, active=True, duplicates=False, interval=0, window=0, refresh_ms=0):
        self.link.gatt(MSG_SCAN_START, scan_start_payload(active, duplicates, interval, window, refresh_ms))

    def scan_stop(self):
        self.link.gatt(MSG_SCAN_STOP)

    def connect(self, conn_id, addr, addr_type=ADDR_AUTO, background=False, timeout_ms=0, wait=15.0,
                itvl_min=0, itvl_max=0):
        """Send CONNECT and wait for EVT_CONNECTED; raises DeviceError if the attempt ends otherwise."""
        with self.cond:
            self.connected.pop(conn_id, None)
            self.disconnected.pop(conn_id, None)
        self.link.gatt(MSG_CONNECT, connect_payload(conn_id, addr_to_bytes(addr), addr_type, background,
                                                    timeout_ms, itvl_min, itvl_max))
        if not self.wait_for(lambda: conn_id in self.connected or conn_id in self.disconnected, wait):
            raise DeviceError("connect to %s: no result within %.0f s" % (addr, wait))
        if conn_id in self.disconnected:
            raise DeviceError("connect to %s failed: %s" % (addr, format_event(self.disconnected[conn_id])))
        return self.connected[conn_id]

    def disconnect(self, conn_id, wait=3.0):
        self.link.gatt(MSG_DISCONNECT, conn_id_payload(conn_id))
        self.wait_for(lambda: conn_id not in self.connected, wait)

    def discover(self, conn_id, wait=20.0):
        """Run full discovery; returns (services, characteristics) as lists of event dicts."""
        with self.cond:
            self.services[conn_id], self.chars[conn_id] = [], []
            self.discovery.pop(conn_id, None)
        self.link.gatt(MSG_DISCOVER, conn_id_payload(conn_id))
        if not self.wait_for(lambda: conn_id in self.discovery, wait):
            raise DeviceError("discovery: no EVT_DISCOVER_DONE within %.0f s" % wait)
        done = self.discovery[conn_id]
        if done["status"] != 0:
            raise DeviceError("discovery failed: %s" % STATUS_NAMES.get(done["status"], done["status"]))
        return self.services[conn_id], self.chars[conn_id]

    def read(self, conn_id, handle):
        """READ; returns (parsed READ_RSP dict, round trip in seconds)."""
        payload, rtt = self.link.gatt(MSG_READ, read_payload(conn_id, handle))
        return parse_read_rsp(payload), rtt

    def write(self, conn_id, handle, data, response=True):
        """WRITE (response=True) or WRITE_CMD; returns the round trip to the ACK in seconds."""
        _, rtt = self.link.gatt(MSG_WRITE if response else MSG_WRITE_CMD, write_payload(conn_id, handle, data))
        return rtt

    def subscribe(self, conn_id, cccd_handle, mode=1):
        self.link.gatt(MSG_SUBSCRIBE, subscribe_payload(conn_id, cccd_handle, mode))

    def gatt_status(self):
        frame, _ = self.link.request(MSG_GET_GATT_STATUS, retries=0, timeout=1.0)
        return parse_gatt_status(frame.payload)

    def set_failsafe(self, conn_id, slot, handle, data):
        self.link.gatt(MSG_SET_FAILSAFE, set_failsafe_payload(conn_id, slot, handle, data))

    def ensure_seen(self, addr, timeout=8.0):
        """Scan until `addr` was reported (fills the stick's address type cache), then stop the scan."""
        self.scan_start(active=True)
        try:
            return self.wait_for(lambda: addr.upper() in self.adv.devices, timeout)
        finally:
            self.scan_stop()


def print_discovery(services, chars):
    """Print the service/characteristic tree of a discovery result."""
    for svc in services:
        print("service %s  handles 0x%04X-0x%04X" % (svc["uuid"], svc["start"], svc["end"]))
        for ch in chars:
            if svc["start"] <= ch["value_handle"] <= svc["end"]:
                props = [n for bit, n in ((0x02, "read"), (0x04, "write-no-rsp"), (0x08, "write"),
                                          (0x10, "notify"), (0x20, "indicate")) if ch["props"] & bit]
                print("  char %s  value 0x%04X  cccd %s  [%s]" % (
                    ch["uuid"], ch["value_handle"],
                    "0x%04X" % ch["cccd_handle"] if ch["cccd_handle"] else "-", ",".join(props)))


def open_gatt(args, echo=None):
    """Open the link, start the keep-alive thread and return (link, caps, session, stop_event)."""
    link, caps = open_link(args)
    stop_event = threading.Event()
    threading.Thread(target=keepalive_loop, args=(link, caps, stop_event), daemon=True).start()
    try:
        return link, caps, GattSession(link, caps, echo), stop_event
    except DeviceError:
        stop_event.set()
        link.close()
        raise


def close_gatt(link, session, stop_event):
    """Leave the stick clean: stop scan, drop links (HELLO of the next tool run would do it too)."""
    stop_event.set()
    session.running = False
    for conn_id in list(session.connected):
        try:
            session.disconnect(conn_id, wait=1.0)
        except DeviceError:
            pass
    try:
        link.command(MSG_STOP_ALL)
        link.gatt(MSG_SCAN_STOP)
    except DeviceError:
        pass
    link.close()


def connect_for(args, session):
    """Connect conn 0 to args.addr; runs a short scan first so the address type is known."""
    if args.addr_type == "auto" and not args.no_scan:
        if not session.ensure_seen(args.addr.upper()):
            print("warning: %s not seen in the scan, trying a public address" % args.addr, file=sys.stderr)
    types = {"auto": ADDR_AUTO, "public": 0, "random": 1}
    info = session.connect(0, args.addr, types[args.addr_type], args.background, args.timeout)
    print("connected: mtu %d, interval %.2f ms" % (info["mtu"], info["itvl"] * 1.25))
    return info


def cmd_scan(args):
    """Scan for advertisers and print the merged result (ADV + SCAN_RSP per address)."""
    link, caps, session, stop = open_gatt(args, echo=(lambda ev: print(format_event(ev))) if args.raw else None)
    try:
        session.scan_start(not args.passive, args.duplicates, args.interval, args.window, args.refresh)
        time.sleep(args.seconds)
        session.scan_stop()
        time.sleep(0.3)
        for rec in sorted(session.adv.devices.values(), key=lambda r: -r["rssi"]):
            if args.name and args.name.lower() not in rec["name"].lower():
                continue
            if args.mfr is not None and rec["mfr_id"] != args.mfr:
                continue
            services = [uuid_text(v[i:i + 16]) for t, v in rec["ad"].items() if t in (0x06, 0x07)
                        for i in range(0, len(v) - 15, 16)]
            print("%s type %d rssi %4d %s name %-16r mfr %s services %s" % (
                rec["addr"], rec["addr_type"], rec["rssi"], "conn" if rec.get("connectable") else "    ",
                rec["name"], "0x%04X" % rec["mfr_id"] if rec["mfr_id"] is not None else "-", services))
        print("dropped reports: %d, stick status: %s" % (session.dropped["scan"], session.gatt_status()))
    finally:
        close_gatt(link, session, stop)


def cmd_connect(args):
    """Connect, optionally discover / subscribe, then hold the link and print events."""
    link, caps, session, stop = open_gatt(args, echo=lambda ev: print(format_event(ev)) if ev["name"] == "NOTIFY" else None)
    try:
        connect_for(args, session)
        if args.discover:
            print_discovery(*session.discover(0))
        for cccd in args.notify or []:
            session.subscribe(0, cccd)
            print("subscribed to 0x%04X" % cccd)
        deadline = time.monotonic() + args.hold
        while time.monotonic() < deadline and 0 in session.connected:
            time.sleep(0.1)
        print("link %s" % ("still up" if 0 in session.connected else "lost: " + format_event(session.disconnected[0])))
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        close_gatt(link, session, stop)


def cmd_gatt_oneshot(args):
    """discover / read / write / write-cmd / notify against one address (connect, act, disconnect)."""
    link, caps, session, stop = open_gatt(args, echo=lambda ev: print(format_event(ev)) if ev["name"] == "NOTIFY" else None)
    try:
        connect_for(args, session)
        if args.command == "discover":
            print_discovery(*session.discover(0))
        elif args.command == "read":
            rsp, rtt = session.read(0, args.handle)
            print("%s%s  (%.1f ms)" % (rsp["data"].hex(), " (truncated)" if rsp["truncated"] else "", rtt * 1000))
        elif args.command in ("write", "write-cmd"):
            rtt = session.write(0, args.handle, bytes.fromhex(args.data), args.command == "write")
            print("acknowledged after %.1f ms" % (rtt * 1000))
        elif args.command == "notify":
            session.subscribe(0, args.cccd, 2 if args.indicate else 1)
            time.sleep(args.seconds)
            print("%d notifications in %.0f s" % (session.notifications.get(0, 0), args.seconds))
    finally:
        close_gatt(link, session, stop)


SHELL_HELP = """commands:
  scan [seconds] [passive]       connect <conn> <addr> [bg]      disconnect <conn>
  discover <conn>                read <conn> <handle>            write <conn> <handle> <hex>
  write-cmd <conn> <handle> <hex>   notify <conn> <cccd> [off|indicate]
  failsafe <conn> <slot> <handle> <hex>   gatt-status   status   wait <seconds>   help   quit
(numbers accept 0x.. hex; events are printed as they arrive)"""


def run_shell_command(session, parts):
    """Execute one gatt-shell command (list of words); returns False for quit."""
    cmd, a = parts[0], parts[1:]
    num = lambda text: int(text, 0)  # noqa: E731
    if cmd == "quit":
        return False
    if cmd == "help":
        print(SHELL_HELP)
    elif cmd == "scan":
        session.scan_start(active="passive" not in a)
        time.sleep(float(a[0]) if a and a[0] != "passive" else 3.0)
        session.scan_stop()
        for rec in session.adv.devices.values():
            print("%s rssi %d name %r mfr %s" % (rec["addr"], rec["rssi"], rec["name"],
                                                  "0x%04X" % rec["mfr_id"] if rec["mfr_id"] is not None else "-"))
    elif cmd == "connect":
        info = session.connect(num(a[0]), a[1], ADDR_AUTO, len(a) > 2 and a[2] == "bg")
        print("connected: mtu %d, interval %.2f ms" % (info["mtu"], info["itvl"] * 1.25))
    elif cmd == "disconnect":
        session.disconnect(num(a[0]))
    elif cmd == "discover":
        print_discovery(*session.discover(num(a[0])))
    elif cmd == "read":
        rsp, rtt = session.read(num(a[0]), num(a[1]))
        print("%s  (%.1f ms)" % (rsp["data"].hex(), rtt * 1000))
    elif cmd in ("write", "write-cmd"):
        print("ack after %.1f ms" % (session.write(num(a[0]), num(a[1]), bytes.fromhex(a[2]), cmd == "write") * 1000))
    elif cmd == "notify":
        mode = {"off": 0, "indicate": 2}.get(a[2] if len(a) > 2 else "", 1)
        session.subscribe(num(a[0]), num(a[1]), mode)
    elif cmd == "failsafe":
        session.set_failsafe(num(a[0]), num(a[1]), num(a[2]), bytes.fromhex(a[3]) if len(a) > 3 else b"")
    elif cmd in ("gatt-status", "status"):
        print(session.gatt_status())
    elif cmd == "wait":
        time.sleep(float(a[0]))
    else:
        print("unknown command (try help)")
    return True


def cmd_gatt_shell(args):
    """Interactive (or scripted with --script "cmd; cmd") GATT shell; connections live as long as the shell."""
    link, caps, session, stop = open_gatt(args, echo=lambda ev: print("  <", format_event(ev))
                                          if ev["name"] != "SCAN_REPORT" else None)
    try:
        if caps["max_conns"]:
            print("stick: up to %d connections, max value %d bytes" % (caps["max_conns"], caps["max_value_len"]))
        lines = [c.strip() for c in args.script.split(";")] if args.script else None
        while True:
            try:
                text = lines.pop(0) if lines else ("quit" if args.script else input("gatt> "))
            except EOFError:
                break
            parts = text.split()
            if not parts:
                continue
            try:
                if not run_shell_command(session, parts):
                    break
            except (DeviceError, IndexError, ValueError) as exc:
                print("error: %s" % exc)
    finally:
        close_gatt(link, session, stop)


def cmd_gatt_bench(args):
    """Write loop against one characteristic: ACK latency histogram, BUSY count, notification count."""
    link, caps, session, stop = open_gatt(args)
    lat, busy, errors = [], 0, 0
    try:
        connect_for(args, session)
        if args.cccd:
            session.subscribe(0, args.cccd)
        payload = bytes(range(args.size))
        period, end, count = 1.0 / args.rate, time.monotonic() + args.seconds, 0
        while time.monotonic() < end and 0 in session.connected:
            t_next = time.monotonic() + period
            try:
                lat.append(session.write(0, args.handle, payload, args.response) * 1000)
            except DeviceError as exc:
                busy += exc.status == 11
                errors += exc.status != 11
            count += 1
            time.sleep(max(0.0, t_next - time.monotonic()))
        print("%s writes: %d sent, %d acknowledged, %d BUSY, %d other errors, %.1f writes/s" % (
            "with-response" if args.response else "no-response", count, len(lat), busy, errors,
            count / args.seconds))
        print("ACK latency", format_stats(latency_stats(lat)))
        print("notifications received: %d, dropped (stick): %s" % (session.notifications.get(0, 0), session.dropped))
        print("link %s" % ("up" if 0 in session.connected else "LOST: " + format_event(session.disconnected.get(0, {"name": "?"}))))
    finally:
        close_gatt(link, session, stop)


def parse_spike_peer(text):
    """'AA:BB:CC:DD:EE:FF[=HANDLE[,CCCD]]' -> (addr, write_handle or None, cccd or None)."""
    addr, _, rest = text.partition("=")
    handle, _, cccd = rest.partition(",")
    return addr.upper(), int(handle, 0) if handle else None, int(cccd, 0) if cccd else None


def cmd_connspike(args):
    """Feasibility spike (task 009 step 1, run on real hardware by the product owner).

    Starts 5 advertising sets (20 ms) and an active scan, then connects the given peers one after
    the other. After every new link all links are driven (write loop at --rate per link, optional
    notify subscription) and observed for --settle seconds. The run ends at the first failing
    connect, lost link or --hold timeout; the last stable link count is the answer ("max_conns").
    """
    link, caps, session, stop = open_gatt(args)
    peers = [parse_spike_peer(p) for p in args.peers]
    stable, report, stop_drive = 0, [], threading.Event()
    stats = {"writes": 0, "busy": 0, "errors": 0}

    def driver(conn_id, handle):
        """Write loop of one link until the spike ends."""
        payload, period = bytes(range(args.size)), 1.0 / args.rate
        while not stop_drive.is_set():
            t_next = time.monotonic() + period
            try:
                session.write(conn_id, handle, payload, response=False)
                stats["writes"] += 1
            except DeviceError as exc:
                stats["busy" if exc.status == 11 else "errors"] += 1
            time.sleep(max(0.0, t_next - time.monotonic()))

    try:
        for set_id in range(min(5, caps["max_sets"])):
            link.command(MSG_SET_START, set_start_payload(set_id, 0xFFF0, bytes([set_id] * 8)))
        print("%d advertising sets running" % min(5, caps["max_sets"]))
        session.scan_start(active=True)
        for n, (addr, handle, cccd) in enumerate(peers, 1):
            print("--- connecting link %d to %s" % (n, addr))
            if not session.wait_for(lambda: addr in session.adv.devices, 10.0):
                print("warning: %s was not seen by the scan, connecting with a public address" % addr)
            try:
                info = session.connect(n - 1, addr, ADDR_AUTO, False, 15000, wait=20.0)
            except DeviceError as exc:
                print("link %d FAILED: %s" % (n, exc))
                report.append((n, "connect failed: %s" % exc))
                break
            print("link %d up: mtu %d, interval %.2f ms" % (n, info["mtu"], info["itvl"] * 1.25))
            if cccd:
                session.subscribe(n - 1, cccd)
            if handle:
                threading.Thread(target=driver, args=(n - 1, handle), daemon=True).start()
            settle_end = time.monotonic() + args.settle
            while time.monotonic() < settle_end and len(session.connected) == n:
                time.sleep(0.2)
            status = session.gatt_status()
            print("links connected: %d (mask 0x%04X), writes %d, BUSY %d, errors %d, notify %s, "
                  "free heap %d (min %d), dropped %s" % (
                      len(session.connected), status["connected_mask"], stats["writes"], stats["busy"],
                      stats["errors"], dict(session.notifications), status["free_heap"],
                      status["min_free_heap"], session.dropped))
            if len(session.connected) != n:
                lost = [format_event(e) for e in session.disconnected.values()]
                print("link lost -> %d links are NOT stable: %s" % (n, lost))
                report.append((n, "unstable: %s" % lost))
                break
            stable = n
            report.append((n, "stable for %.0f s" % args.settle))
        if stable == len(peers) and stable:
            print("all %d links up; holding %.0f s for the 10 min criterion" % (stable, args.hold))
            hold_end = time.monotonic() + args.hold
            while time.monotonic() < hold_end and len(session.connected) == stable:
                time.sleep(0.5)
            if len(session.connected) != stable:
                stable -= 1
                report.append((stable + 1, "lost during hold"))
            else:
                report.append((stable, "stable for hold time"))
        print("\nRESULT: highest stable link count = %d" % stable)
        for n, text in report:
            print("  %d links: %s" % (n, text))
        print("final stick status:", session.gatt_status())
    finally:
        stop_drive.set()
        close_gatt(link, session, stop)



def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", help="serial port (default: auto-detect by VID/PID + HELLO)")
    parser.add_argument("-v", "--verbose", action="store_true", help="hex dump of all frames to stderr")
    parser.add_argument("--keepalive", type=int, default=1000, help="requested keep-alive timeout in ms")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("ports").set_defaults(func=cmd_ports)
    sub.add_parser("hello").set_defaults(func=cmd_hello)
    hold = sub.add_parser("hold")
    hold.add_argument("--set", type=int, default=0)
    hold.add_argument("--mfr", type=parse_int, default=0xFFF0)
    hold.add_argument("--data", required=True, help="payload as hex string")
    hold.add_argument("--interval", type=int, default=32, help="0.625 ms units")
    hold.add_argument("--tx", type=int, default=127, help="dBm, 127 = max")
    hold.add_argument("--pdu", choices=sorted(PDU_TYPES), default="ind")
    hold.add_argument("--no-flags", action="store_true", help="do not add the AD flags field")
    hold.add_argument("--seconds", type=float, default=0, help="0 = until Ctrl-C")
    hold.add_argument("--no-stop", action="store_true", help="skip STOP_ALL, test the keep-alive timeout")
    hold.set_defaults(func=cmd_hold)
    sub.add_parser("shell").set_defaults(func=cmd_shell)
    bench = sub.add_parser("bench")
    bench.add_argument("--set", type=int, default=0)
    bench.add_argument("--mfr", type=parse_int, default=0xFFF0)
    bench.add_argument("--rate", type=float, default=50.0, help="updates per second")
    bench.add_argument("--seconds", type=float, default=10.0)
    bench.set_defaults(func=cmd_bench)
    fuzz = sub.add_parser("fuzz")
    fuzz.add_argument("--rounds", type=int, default=20)
    fuzz.set_defaults(func=cmd_fuzz)
    # ---- protocol 1.1: scanning and GATT central ----
    def target_args(p):
        """Options shared by the commands that connect to one device."""
        p.add_argument("addr", help="device address AA:BB:CC:DD:EE:FF")
        p.add_argument("--addr-type", choices=("auto", "public", "random"), default="auto")
        p.add_argument("--no-scan", action="store_true", help="skip the scan that fills the address type cache")
        p.add_argument("--background", action="store_true", help="background connect (no timeout)")
        p.add_argument("--timeout", type=int, default=0, help="connect timeout in ms (0 = 10 s default)")

    scan = sub.add_parser("scan", help="scan and print merged advertising data")
    scan.add_argument("--seconds", type=float, default=5.0)
    scan.add_argument("--passive", action="store_true", help="no scan requests (no scan response data)")
    scan.add_argument("--duplicates", action="store_true", help="report every PDU (no stick-side filter)")
    scan.add_argument("--interval", type=int, default=0, help="0.625 ms units, 0 = default")
    scan.add_argument("--window", type=int, default=0, help="0.625 ms units, 0 = default")
    scan.add_argument("--refresh", type=int, default=0, help="re-report period in ms, 0 = default")
    scan.add_argument("--name", help="only devices whose name contains this text")
    scan.add_argument("--mfr", type=parse_int, help="only devices with this manufacturer id")
    scan.add_argument("--raw", action="store_true", help="print every scan report event")
    scan.set_defaults(func=cmd_scan)
    conn = sub.add_parser("connect", help="connect, optionally discover/subscribe, hold the link")
    target_args(conn)
    conn.add_argument("--discover", action="store_true")
    conn.add_argument("--notify", type=parse_int, action="append", help="CCCD handle to subscribe (repeatable)")
    conn.add_argument("--hold", type=float, default=10.0, help="seconds to keep the link")
    conn.set_defaults(func=cmd_connect)
    for name in ("discover", "read", "write", "write-cmd", "notify"):
        one = sub.add_parser(name, help="connect, %s, disconnect" % name)
        target_args(one)
        if name in ("read", "write", "write-cmd"):
            one.add_argument("handle", type=parse_int)
        if name in ("write", "write-cmd"):
            one.add_argument("data", help="hex string, 1..56 bytes")
        if name == "notify":
            one.add_argument("cccd", type=parse_int, help="CCCD handle from discovery")
            one.add_argument("--indicate", action="store_true")
            one.add_argument("--seconds", type=float, default=10.0)
        one.set_defaults(func=cmd_gatt_oneshot)
    shell = sub.add_parser("gatt-shell", help="interactive GATT shell (all commands, several links)")
    shell.add_argument("--script", help='run commands separated by ";" instead of prompting')
    shell.set_defaults(func=cmd_gatt_shell)
    gbench = sub.add_parser("gatt-bench", help="write loop: ACK latency, BUSY count, notifications")
    target_args(gbench)
    gbench.add_argument("handle", type=parse_int, help="characteristic value handle to write")
    gbench.add_argument("--cccd", type=parse_int, help="subscribe to this CCCD handle first")
    gbench.add_argument("--size", type=int, default=8, help="bytes per write")
    gbench.add_argument("--rate", type=float, default=100.0, help="writes per second")
    gbench.add_argument("--seconds", type=float, default=10.0)
    gbench.add_argument("--response", action="store_true", help="write with response (default: no response)")
    gbench.set_defaults(func=cmd_gatt_bench)
    spike = sub.add_parser("connspike", help="maximum connection spike (5 adv sets + scan + N links)")
    spike.add_argument("peers", nargs="+", help="AA:BB:CC:DD:EE:FF[=WRITE_HANDLE[,CCCD_HANDLE]] per peripheral")
    spike.add_argument("--rate", type=float, default=100.0, help="write-no-response per second and link")
    spike.add_argument("--size", type=int, default=18, help="bytes per write")
    spike.add_argument("--settle", type=float, default=30.0, help="seconds each link count must stay stable")
    spike.add_argument("--hold", type=float, default=600.0, help="seconds to hold the final link count")
    spike.set_defaults(func=cmd_connspike)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        return args.func(args) or 0
    except DeviceError as exc:
        print("error: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
