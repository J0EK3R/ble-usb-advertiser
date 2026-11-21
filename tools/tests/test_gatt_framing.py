"""Unit tests for the protocol 1.1 (GATT) code of tools/bc2adv.py: builders, parsers, golden vectors
and the GattSession logic against a scripted fake link. No hardware and no pyserial needed.

The golden vectors are the same as in test/host/test_gatt_vectors.h and docs/protocol.md section 18.
"""
import os
import queue
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import bc2adv  # noqa: E402


def H(text):
    """Hex string with optional spaces to bytes."""
    return bytes.fromhex(text.replace(" ", ""))


# --- golden vectors (generated from hand-written payloads, see docs/protocol.md section 18) ---
GV_SCAN_START_DEFAULT = H("B2 09 30 08 01 00 00 00 00 00 00 8E 96")
GV_SCAN_START_EXPLICIT = H("B2 09 30 09 03 A0 00 50 00 E8 03 D5 8C")
GV_SCAN_STOP = H("B2 02 31 0A 12 35")
GV_CONNECT_BACKGROUND = H("B2 11 32 0B 00 90 84 2B 01 02 03 FF 01 00 00 00 00 00 00 D0 EB")
GV_CONNECT_EXPLICIT = H("B2 11 32 0C 02 AA BB CC DD EE FF 01 00 88 13 0C 00 18 00 D3 7C")
GV_DISCONNECT = H("B2 03 33 0D 01 94 E5")
GV_DISCOVER = H("B2 03 34 0E 01 57 35")
GV_READ = H("B2 05 35 0F 01 12 00 67 E4")
GV_WRITE_DUPLO = H("B2 0D 36 10 00 10 00 08 00 81 00 11 51 00 00 03 58")
GV_WRITE_CMD_MKDIY = H("B2 17 37 11 01 2A 00 CC AA BB 01 80 80 80 80 80 80 80 80 80 80 80 80 66 33 83 62")
GV_WRITE_CMD_MAX = H("B2 3D 37 12 01 2A 00 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F 20 21 22 23 24 25 26 27 28 29 2A 2B 2C 2D 2E 2F 30 31 32 33 34 35 36 37 23 F4")
GV_SUBSCRIBE = H("B2 06 38 13 00 11 00 01 DF FC")
GV_GET_GATT_STATUS = H("B2 02 39 14 44 4F")
GV_SET_FAILSAFE_MKDIY = H("B2 18 3A 15 01 00 2A 00 CC AA BB 01 80 80 80 80 80 80 80 80 80 80 80 80 66 33 34 1C")
GV_SET_FAILSAFE_CLEAR = H("B2 06 3A 16 01 01 2A 00 13 98")
GV_HELLO_RSP_V11 = H("B2 22 81 01 42 43 32 41 44 56 00 01 01 00 02 00 05 18 1B 20 00 00 40 E8 03 24 0A BC 11 22 33 07 04 38 1F 04 DA 6E")
GV_READ_RSP = H("B2 08 A0 0F 01 12 00 00 A1 B2 22 91")
GV_GATT_STATUS_RSP = H("B2 13 A1 14 01 04 00 03 00 02 00 00 00 00 80 02 00 00 40 01 00 56 F8")
GV_ACK_WRITE_ATT_ERROR = H("B2 05 80 10 36 0E 03 0F 10")
GV_ACK_CONNECT_OK = H("B2 04 80 0B 32 00 34 D6")
GV_EVT_NOTIFY = H("B2 0B C0 00 00 11 00 00 04 00 45 00 01 AC 42")
GV_EVT_CONNECTED = H("B2 0D C1 00 00 40 00 18 00 90 84 2B 01 02 03 B0 12")
GV_EVT_DISCONNECTED = H("B2 05 C2 00 00 01 08 13 88")
GV_EVT_SCAN_REPORT = H("B2 19 C3 00 90 84 2B 01 02 03 01 C4 01 0D 02 01 06 09 FF 97 03 00 41 20 00 00 00 B9 69")
GV_EVT_GATT_SERVICE_128 = H("B2 18 C4 00 00 10 00 20 00 10 23 D1 BC EA 5F 78 23 16 DE EF 12 12 23 16 00 00 FB 31")
GV_EVT_GATT_SERVICE_16 = H("B2 0A C4 00 00 30 00 3F 00 02 0A 18 21 CF")
GV_EVT_GATT_CHAR = H("B2 19 C5 00 00 11 00 12 00 1E 10 23 D1 BC EA 5F 78 23 16 DE EF 12 12 24 16 00 00 8F 18")
GV_EVT_DISCOVER_DONE = H("B2 06 C6 00 00 00 02 05 FD 55")
GV_EVT_SCAN_STOPPED = H("B2 05 C7 00 01 0F 00 73 3E")
GV_EVT_DROPPED = H("B2 05 C8 00 02 2A 00 C9 FB")

MK_DIY = H("CC AA BB 01 80 80 80 80 80 80 80 80 80 80 80 80 66 33")   # 18 byte MouldKing DIY frame
DUPLO = H("08 00 81 00 11 51 00 00")                                     # DUPLO motor frame
F = bc2adv.Frame
LEGO_UUID_WIRE = H("23 D1 BC EA 5F 78 23 16 DE EF 12 12 23 16 00 00")


def decode_one(wire):
    """Decode a golden vector with the streaming Decoder; returns the single Frame."""
    frames = bc2adv.Decoder().feed(wire, now=0.0)
    assert len(frames) == 1, frames
    return frames[0]


class RequestBuilderTests(unittest.TestCase):
    """Builders must produce exactly the golden request frames."""

    def check(self, vector, type_, seq, payload):
        self.assertEqual(vector, bc2adv.encode(F(type_, seq, payload)))

    def test_scan_start(self):
        self.check(GV_SCAN_START_DEFAULT, 0x30, 8, bc2adv.scan_start_payload())
        self.check(GV_SCAN_START_EXPLICIT, 0x30, 9, bc2adv.scan_start_payload(True, True, 160, 80, 1000))
        self.check(GV_SCAN_STOP, 0x31, 10, b"")

    def test_connect(self):
        self.check(GV_CONNECT_BACKGROUND, 0x32, 11,
                   bc2adv.connect_payload(0, bc2adv.addr_to_bytes("90:84:2B:01:02:03"), 0xFF, True))
        self.check(GV_CONNECT_EXPLICIT, 0x32, 12,
                   bc2adv.connect_payload(2, bc2adv.addr_to_bytes("AA:BB:CC:DD:EE:FF"), 1, False, 5000, 12, 24))

    def test_conn_requests(self):
        self.check(GV_DISCONNECT, 0x33, 13, bc2adv.conn_id_payload(1))
        self.check(GV_DISCOVER, 0x34, 14, bc2adv.conn_id_payload(1))
        self.check(GV_READ, 0x35, 15, bc2adv.read_payload(1, 0x12))
        self.check(GV_SUBSCRIBE, 0x38, 19, bc2adv.subscribe_payload(0, 0x11, 1))
        self.check(GV_GET_GATT_STATUS, 0x39, 20, b"")

    def test_writes(self):
        self.check(GV_WRITE_DUPLO, 0x36, 16, bc2adv.write_payload(0, 0x10, DUPLO))
        self.check(GV_WRITE_CMD_MKDIY, 0x37, 17, bc2adv.write_payload(1, 0x2A, MK_DIY))
        self.check(GV_WRITE_CMD_MAX, 0x37, 18, bc2adv.write_payload(1, 0x2A, bytes(range(56))))

    def test_write_limits(self):
        with self.assertRaises(ValueError):
            bc2adv.write_payload(0, 1, b"")
        with self.assertRaises(ValueError):
            bc2adv.write_payload(0, 1, bytes(57))

    def test_failsafe(self):
        self.check(GV_SET_FAILSAFE_MKDIY, 0x3A, 21, bc2adv.set_failsafe_payload(1, 0, 0x2A, MK_DIY))
        self.check(GV_SET_FAILSAFE_CLEAR, 0x3A, 22, bc2adv.set_failsafe_payload(1, 1, 0x2A, b""))
        with self.assertRaises(ValueError):
            bc2adv.set_failsafe_payload(1, 0, 0x2A, bytes(21))

    def test_address_helpers(self):
        self.assertEqual(H("90 84 2B 01 02 03"), bc2adv.addr_to_bytes("90:84:2b:01:02:03"))
        self.assertEqual("90:84:2B:01:02:03", bc2adv.addr_to_text(H("90 84 2B 01 02 03")))
        with self.assertRaises(ValueError):
            bc2adv.addr_to_bytes("90:84:2B")


class ResponseParserTests(unittest.TestCase):
    def test_hello_rsp_v11(self):
        caps = bc2adv.parse_hello_rsp(decode_one(GV_HELLO_RSP_V11).payload)
        self.assertEqual("1.1", caps["proto"])
        self.assertEqual("0.2.0", caps["fw"])
        self.assertEqual((7, 4, 56, 31, 4), (caps["gatt_flags"], caps["max_conns"], caps["max_value_len"],
                                              caps["max_scan_data"], caps["failsafe_slots"]))
        self.assertEqual("24:0a:bc:11:22:33", caps["device_id"])

    def test_hello_rsp_v10_still_parses(self):
        """A 27 byte HELLO_RSP of a 1.0 firmware has no GATT block."""
        old = decode_one(GV_HELLO_RSP_V11).payload[:27]
        caps = bc2adv.parse_hello_rsp(old)
        self.assertEqual(0, caps["gatt_flags"])
        self.assertEqual(0, caps["max_conns"])

    def test_read_rsp(self):
        rsp = bc2adv.parse_read_rsp(decode_one(GV_READ_RSP).payload)
        self.assertEqual({"conn_id": 1, "handle": 0x12, "truncated": False, "data": H("A1 B2")}, rsp)

    def test_gatt_status(self):
        st = bc2adv.parse_gatt_status(decode_one(GV_GATT_STATUS_RSP).payload)
        self.assertEqual((True, 4, 3, 2, 0), (st["scanning"], st["connecting_mask"], st["connected_mask"],
                                               st["dropped_scan"], st["dropped_notify"]))
        self.assertEqual((0x28000, 0x14000), (st["free_heap"], st["min_free_heap"]))

    def test_ack_with_att_error(self):
        frame = decode_one(GV_ACK_WRITE_ATT_ERROR)
        self.assertEqual((0x36, 14, 3), (frame.payload[0], frame.payload[1], frame.payload[2]))

    def test_events(self):
        ev = bc2adv.parse_event(decode_one(GV_EVT_NOTIFY))
        self.assertEqual(("NOTIFY", 0, 0x11, H("04 00 45 00 01")), (ev["name"], ev["conn_id"], ev["handle"], ev["data"]))
        ev = bc2adv.parse_event(decode_one(GV_EVT_CONNECTED))
        self.assertEqual((64, 24, "90:84:2B:01:02:03"), (ev["mtu"], ev["itvl"], ev["addr"]))
        ev = bc2adv.parse_event(decode_one(GV_EVT_DISCONNECTED))
        self.assertEqual((1, 8), (ev["phase"], ev["reason"]))
        ev = bc2adv.parse_event(decode_one(GV_EVT_SCAN_REPORT))
        self.assertEqual(("90:84:2B:01:02:03", 1, -60, True, False), (ev["addr"], ev["addr_type"], ev["rssi"],
                                                                      ev["connectable"], ev["scan_rsp"]))
        self.assertEqual(13, len(ev["data"]))
        ev = bc2adv.parse_event(decode_one(GV_EVT_GATT_SERVICE_128))
        self.assertEqual(("00001623-1212-efde-1623-785feabcd123", 0x10, 0x20),
                         (ev["uuid"], ev["start"], ev["end"]))
        ev = bc2adv.parse_event(decode_one(GV_EVT_GATT_SERVICE_16))
        self.assertEqual("180a", ev["uuid"])
        ev = bc2adv.parse_event(decode_one(GV_EVT_GATT_CHAR))
        self.assertEqual((0x11, 0x12, 0x1E, "00001624-1212-efde-1623-785feabcd123"),
                         (ev["value_handle"], ev["cccd_handle"], ev["props"], ev["uuid"]))
        ev = bc2adv.parse_event(decode_one(GV_EVT_DISCOVER_DONE))
        self.assertEqual((0, 2, 5), (ev["status"], ev["n_services"], ev["n_chars"]))
        ev = bc2adv.parse_event(decode_one(GV_EVT_SCAN_STOPPED))
        self.assertEqual((1, 15), (ev["reason"], ev["rc"]))
        ev = bc2adv.parse_event(decode_one(GV_EVT_DROPPED))
        self.assertEqual(("notify", 42), (ev["kind"], ev["count"]))

    def test_uuid_text(self):
        self.assertEqual("00001623-1212-efde-1623-785feabcd123", bc2adv.uuid_text(LEGO_UUID_WIRE))
        self.assertEqual("2a26", bc2adv.uuid_text(H("26 2A")))


class AdvertisingDataTests(unittest.TestCase):
    """Host-side merge of ADV and SCAN_RSP data, as BC2's device managers need it."""

    def report(self, data, scan_rsp, addr="90:84:2B:01:02:03", conn=True):
        return {"addr": addr, "addr_type": 1, "rssi": -50, "connectable": conn, "scan_rsp": scan_rsp, "data": data}

    def test_parse_ad_structures(self):
        ads = bc2adv.parse_adv_data(H("02 01 06 05 09 41 42 43 44 03 FF 97 03"))
        self.assertEqual([(1, H("06")), (9, b"ABCD"), (0xFF, H("97 03"))], ads)

    def test_parse_stops_at_truncated_structure(self):
        self.assertEqual([(1, H("06"))], bc2adv.parse_adv_data(H("02 01 06 09 FF 97")))
        self.assertEqual([], bc2adv.parse_adv_data(H("00 01 02")))

    def test_merge_name_from_scan_response(self):
        """LEGO DUPLO: manufacturer data (0x0397) in the ADV, name in the scan response."""
        merger = bc2adv.AdvMerger()
        merger.update(self.report(H("02 01 06 09 FF 97 03 00 41 20 00 00 00"), False))
        rec = merger.update(self.report(H("0B 09") + b"Train Base", True))
        self.assertEqual("Train Base", rec["name"])
        self.assertEqual(0x0397, rec["mfr_id"])
        self.assertTrue(rec["connectable"])

    def test_merge_service_uuid_wedo(self):
        merger = bc2adv.AdvMerger()
        rec = merger.update(self.report(H("11 07") + LEGO_UUID_WIRE.replace(H("23 16 00 00"), H("23 15 00 00")), False))
        self.assertEqual(16, len(rec["ad"][0x07]))

    def test_mouldking_manufacturer_id(self):
        merger = bc2adv.AdvMerger()
        rec = merger.update(self.report(H("05 FF 33 AC 01 02"), False, conn=False))
        self.assertEqual(0xAC33, rec["mfr_id"])
        self.assertEqual("", rec["name"])


class StatsTests(unittest.TestCase):
    def test_latency_stats(self):
        stats = bc2adv.latency_stats([float(i) for i in range(1, 101)])
        self.assertEqual((100, 1.0, 100.0), (stats["count"], stats["min"], stats["max"]))
        self.assertAlmostEqual(50.5, stats["mean"])
        self.assertEqual(51.0, stats["p50"])
        self.assertEqual(96.0, stats["p95"])

    def test_empty(self):
        self.assertEqual({"count": 0}, bc2adv.latency_stats([]))
        self.assertEqual("no samples", bc2adv.format_stats({"count": 0}))

    def test_single_value(self):
        stats = bc2adv.latency_stats([4.0])
        self.assertEqual((4.0, 4.0, 4.0), (stats["min"], stats["p95"], stats["max"]))

    def test_format_event(self):
        text = bc2adv.format_event(bc2adv.parse_event(decode_one(GV_EVT_DISCONNECTED)))
        self.assertIn("reason 0x08", text)
        stopped = bc2adv.format_event({"name": "DISCONNECTED", "conn_id": 0, "phase": 0, "reason": 0xF2})
        self.assertIn("stick safety stop", stopped)


class FakeLink:
    """Scripted stand-in for bc2adv.Link: records GATT requests and lets a test inject events."""

    def __init__(self):
        self.events = queue.Queue()
        self.sent = []                  # (type, payload) of every request
        self.on_request = None          # optional callback(type, payload) -> None / raises DeviceError / returns bytes

    def gatt(self, type_, payload=b"", timeout=6.0):
        self.sent.append((type_, bytes(payload)))
        extra = self.on_request(type_, bytes(payload)) if self.on_request else b""
        return (extra or b""), 0.002

    command = gatt

    def request(self, type_, payload=b"", timeout=0.1, retries=0):
        self.sent.append((type_, bytes(payload)))
        return F(bc2adv.MSG_GATT_STATUS_RSP, 1, STATUS_PAYLOAD), 0.001

    def push(self, type_, payload):
        self.events.put(F(type_, 0, payload))


STATUS_PAYLOAD = decode_one(GV_GATT_STATUS_RSP).payload
CAPS = {"gatt_flags": 7, "max_conns": 4, "proto": "1.1"}


class GattSessionTests(unittest.TestCase):
    def setUp(self):
        self.link = FakeLink()
        self.session = bc2adv.GattSession(self.link, CAPS)
        self.addCleanup(self.session.close)

    def test_refuses_firmware_without_gatt(self):
        with self.assertRaises(bc2adv.DeviceError):
            bc2adv.GattSession(FakeLink(), {"gatt_flags": 0, "proto": "1.0"})

    def test_connect_success_waits_for_event(self):
        threading.Timer(0.05, lambda: self.link.push(bc2adv.MSG_EVT_CONNECTED, H("00 40 00 18 00 90 84 2B 01 02 03"))).start()
        info = self.session.connect(0, "90:84:2B:01:02:03", wait=2.0)
        self.assertEqual(64, info["mtu"])
        type_, payload = self.link.sent[0]
        self.assertEqual(bc2adv.MSG_CONNECT, type_)
        self.assertEqual(H("00 90 84 2B 01 02 03 FF 00 00 00 00 00 00 00"), payload)

    def test_connect_failure_reports_reason(self):
        threading.Timer(0.05, lambda: self.link.push(bc2adv.MSG_EVT_DISCONNECTED, H("00 00 F0"))).start()
        with self.assertRaises(bc2adv.DeviceError) as ctx:
            self.session.connect(0, "90:84:2B:01:02:03", wait=2.0)
        self.assertIn("connect timeout", str(ctx.exception))

    def test_connect_timeout_without_event(self):
        with self.assertRaises(bc2adv.DeviceError):
            self.session.connect(0, "90:84:2B:01:02:03", wait=0.2)

    def test_discovery_collects_events_until_done(self):
        def answer(type_, payload):
            if type_ == bc2adv.MSG_DISCOVER:
                self.link.push(bc2adv.MSG_EVT_GATT_SERVICE, H("00 10 00 20 00 10") + LEGO_UUID_WIRE)
                self.link.push(bc2adv.MSG_EVT_GATT_CHAR, H("00 11 00 12 00 1E 10") + LEGO_UUID_WIRE)
                self.link.push(bc2adv.MSG_EVT_DISCOVER_DONE, H("00 00 01 01"))
        self.link.on_request = answer
        services, chars = self.session.discover(0, wait=2.0)
        self.assertEqual(1, len(services))
        self.assertEqual(0x12, chars[0]["cccd_handle"])

    def test_discovery_error_status(self):
        self.link.on_request = lambda t, p: self.link.push(bc2adv.MSG_EVT_DISCOVER_DONE, H("00 0D 00 00"))
        with self.assertRaises(bc2adv.DeviceError) as ctx:
            self.session.discover(0, wait=2.0)
        self.assertIn("TIMEOUT", str(ctx.exception))

    def test_notifications_are_counted_per_connection(self):
        for _ in range(3):
            self.link.push(bc2adv.MSG_EVT_NOTIFY, H("01 11 00 00 AA"))
        self.assertTrue(self.session.wait_for(lambda: self.session.notifications.get(1) == 3, 2.0))

    def test_dropped_counters_accumulate(self):
        self.link.push(bc2adv.MSG_EVT_DROPPED, H("01 05 00"))
        self.link.push(bc2adv.MSG_EVT_DROPPED, H("01 02 00"))
        self.assertTrue(self.session.wait_for(lambda: self.session.dropped["scan"] == 7, 2.0))

    def test_ensure_seen_uses_scan_and_stops_it(self):
        threading.Timer(0.05, lambda: self.link.push(
            bc2adv.MSG_EVT_SCAN_REPORT, H("90 84 2B 01 02 03 01 C4 01 03 02 01 06"))).start()
        self.assertTrue(self.session.ensure_seen("90:84:2B:01:02:03", timeout=2.0))
        self.assertEqual([bc2adv.MSG_SCAN_START, bc2adv.MSG_SCAN_STOP], [t for t, _ in self.link.sent])

    def test_write_and_read_helpers(self):
        self.link.on_request = lambda t, p: H("01 12 00 00 A1 B2") if t == bc2adv.MSG_READ else b""
        rsp, _ = self.session.read(1, 0x12)
        self.assertEqual(H("A1 B2"), rsp["data"])
        self.session.write(0, 0x10, DUPLO, response=True)
        self.session.write(0, 0x10, DUPLO, response=False)
        self.assertEqual([bc2adv.MSG_READ, bc2adv.MSG_WRITE, bc2adv.MSG_WRITE_CMD], [t for t, _ in self.link.sent])

    def test_shell_commands_build_requests(self):
        for words in (["failsafe", "1", "0", "0x2A", MK_DIY.hex()], ["failsafe", "1", "1", "0x2A"],
                      ["notify", "0", "0x11"], ["notify", "0", "0x11", "off"]):
            self.assertTrue(bc2adv.run_shell_command(self.session, words))
        self.assertFalse(bc2adv.run_shell_command(self.session, ["quit"]))
        self.assertEqual(GV_SET_FAILSAFE_MKDIY[4:-2], self.link.sent[0][1])
        self.assertEqual(H("01 01 2A 00"), self.link.sent[1][1])        # empty slot = clear
        self.assertEqual(H("00 11 00 01"), self.link.sent[2][1])
        self.assertEqual(H("00 11 00 00"), self.link.sent[3][1])

    def test_spike_peer_parsing(self):
        self.assertEqual(("AA:BB:CC:DD:EE:FF", 0x10, 0x11), bc2adv.parse_spike_peer("aa:bb:cc:dd:ee:ff=0x10,0x11"))
        self.assertEqual(("AA:BB:CC:DD:EE:FF", 16, None), bc2adv.parse_spike_peer("AA:BB:CC:DD:EE:FF=16"))
        self.assertEqual(("AA:BB:CC:DD:EE:FF", None, None), bc2adv.parse_spike_peer("AA:BB:CC:DD:EE:FF"))


class DecoderIntegrationTests(unittest.TestCase):
    def test_all_vectors_decode_with_valid_crc(self):
        """Every golden vector is a valid frame for the streaming decoder (CRC and LEN consistent)."""
        for name, wire in sorted(globals().items()):
            if name.startswith("GV_") and isinstance(wire, bytes):
                with self.subTest(vector=name):
                    dec = bc2adv.Decoder()
                    self.assertEqual(1, len(dec.feed(wire, now=0.0)))
                    self.assertEqual(0, dec.parse_errors)


if __name__ == "__main__":
    unittest.main()
