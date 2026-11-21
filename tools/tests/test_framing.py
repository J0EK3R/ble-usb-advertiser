"""Unit tests for the framing code of tools/bc2adv.py (no hardware, no pyserial needed).

The golden vectors are the same as in test/host/test_vectors.h and docs/protocol.md section 10.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import bc2adv  # noqa: E402

def H(text):
    """Hex string with optional spaces to bytes."""
    return bytes.fromhex(text.replace(" ", ""))


GV_HELLO = H("B2 06 01 01 01 00 E8 03 D6 60")
GV_PING = H("B2 02 02 02 DC E4")
GV_SET_START = H("B2 12 10 03 00 01 20 00 7F F0 FF 08 01 02 03 04 05 06 07 08 F5 48")
GV_SET_UPDATE = H("B2 0A 11 04 00 F0 FF 04 A1 A2 A3 A4 52 B1")
GV_SET_STOP = H("B2 03 12 05 00 EA CD")
GV_STOP_ALL = H("B2 02 13 06 1A 94")
GV_GET_STATUS = H("B2 03 14 07 FF D8 07")
GV_HELLO_RSP = H("B2 1D 81 01 42 43 32 41 44 56 00 01 00 00 01 00 05 18 1B 20 00 00 40 E8 03 "
                 "24 0A BC 11 22 33 40 AD")
GV_ACK_SET_START = H("B2 05 80 03 10 00 7F 06 38")
GV_ACK_SET_UPDATE = H("B2 08 80 04 11 00 7B 03 00 00 D4 57")
GV_PONG = H("B2 0B 82 02 01 01 00 00 00 10 27 00 00 5C D9")
GV_EVT_TIMEOUT = H("B2 04 E2 00 E8 03 F0 58")


class CrcTests(unittest.TestCase):
    def test_check_value(self):
        self.assertEqual(0x29B1, bc2adv.crc16(b"123456789"))

    def test_empty(self):
        self.assertEqual(0xFFFF, bc2adv.crc16(b""))

    def test_bit_flip(self):
        data = bytearray(b"\x01\x02\x03\x04")
        ref = bc2adv.crc16(bytes(data))
        data[2] ^= 0x10
        self.assertNotEqual(ref, bc2adv.crc16(bytes(data)))


class EncodeTests(unittest.TestCase):
    def test_golden_requests(self):
        F = bc2adv.Frame
        self.assertEqual(GV_HELLO, bc2adv.encode(F(0x01, 1, bc2adv.hello_payload(1000, minor=0))))
        self.assertEqual(GV_PING, bc2adv.encode(F(0x02, 2)))
        self.assertEqual(GV_SET_START, bc2adv.encode(F(0x10, 3, bc2adv.set_start_payload(
            0, 0xFFF0, bytes(range(1, 9)), interval=32, tx=127))))
        self.assertEqual(GV_SET_UPDATE, bc2adv.encode(F(0x11, 4, bc2adv.set_update_payload(
            0, 0xFFF0, b"\xA1\xA2\xA3\xA4"))))
        self.assertEqual(GV_SET_STOP, bc2adv.encode(F(0x12, 5, b"\x00")))
        self.assertEqual(GV_STOP_ALL, bc2adv.encode(F(0x13, 6)))
        self.assertEqual(GV_GET_STATUS, bc2adv.encode(F(0x14, 7, b"\xFF")))

    def test_payload_too_large(self):
        with self.assertRaises(ValueError):
            bc2adv.encode(bc2adv.Frame(0x80, 0, bytes(61)))

    def test_max_frame_roundtrip(self):
        frame = bc2adv.Frame(0x80, 77, bytes(range(60)))
        wire = bc2adv.encode(frame)
        self.assertEqual(66, len(wire))
        self.assertEqual([frame], bc2adv.Decoder().feed(wire, 0))


class DecoderTests(unittest.TestCase):
    def test_golden_responses(self):
        dec = bc2adv.Decoder()
        for wire, type_, seq in ((GV_HELLO_RSP, 0x81, 1), (GV_ACK_SET_START, 0x80, 3),
                                 (GV_ACK_SET_UPDATE, 0x80, 4), (GV_PONG, 0x82, 2),
                                 (GV_EVT_TIMEOUT, 0xE2, 0)):
            frames = dec.feed(wire, 0)
            self.assertEqual(1, len(frames), wire.hex())
            self.assertEqual((type_, seq), (frames[0].type, frames[0].seq))
        self.assertEqual(0, dec.parse_errors)

    def test_hello_rsp_fields(self):
        caps = bc2adv.parse_hello_rsp(bc2adv.Decoder().feed(GV_HELLO_RSP, 0)[0].payload)
        self.assertEqual(5, caps["max_sets"])
        self.assertEqual(1000, caps["keepalive_ms"])
        self.assertEqual("24:0a:bc:11:22:33", caps["device_id"])
        self.assertEqual((24, 27), (caps["max_data_with_flags"], caps["max_data_no_flags"]))

    def test_byte_by_byte(self):
        dec = bc2adv.Decoder()
        frames = []
        for i, b in enumerate(GV_SET_START):
            frames += dec.feed(bytes([b]), i * 0.001)
        self.assertEqual(1, len(frames))

    def test_back_to_back_and_garbage(self):
        dec = bc2adv.Decoder()
        frames = dec.feed(b"boot text\x00" + GV_PING + GV_STOP_ALL, 0)
        self.assertEqual([0x02, 0x13], [f.type for f in frames])
        self.assertEqual(0, dec.parse_errors)

    def test_bad_crc_counts_and_recovers(self):
        dec = bc2adv.Decoder()
        bad = GV_PING[:-1] + bytes([GV_PING[-1] ^ 1])
        self.assertEqual([], dec.feed(bad, 0))
        self.assertEqual(1, dec.parse_errors)
        self.assertEqual(1, len(dec.feed(GV_PING, 0.01)))

    def test_resync_inside_corrupted_frame(self):
        outer = bytearray(bc2adv.encode(bc2adv.Frame(0x11, 1, GV_PING)))
        outer[-1] ^= 0xFF
        dec = bc2adv.Decoder()
        frames = dec.feed(bytes(outer), 0)
        self.assertEqual([0x02], [f.type for f in frames])
        self.assertEqual(1, dec.parse_errors)

    def test_invalid_len(self):
        for length in (0, 1, 63, 255):
            dec = bc2adv.Decoder()
            self.assertEqual([], dec.feed(bytes([bc2adv.SOF, length, 2, 0]), 0))
            self.assertGreaterEqual(dec.parse_errors, 1, length)

    def test_interframe_timeout(self):
        dec = bc2adv.Decoder()
        self.assertEqual([], dec.feed(GV_SET_START[:10], 0.0))
        frames = dec.feed(GV_PING, 0.3)
        self.assertEqual([0x02], [f.type for f in frames])
        self.assertEqual(1, dec.parse_errors)

    def test_no_timeout_within_window(self):
        dec = bc2adv.Decoder()
        dec.feed(GV_SET_START[:10], 0.0)
        self.assertEqual(1, len(dec.feed(GV_SET_START[10:], 0.15)))


if __name__ == "__main__":
    unittest.main()
