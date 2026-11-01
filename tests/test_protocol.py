import math
import unittest
import struct

from drone_gcs.protocol import chunk_jpeg, FrameReassembler

from drone_gcs.model import DroneState, VelocityCommand
from drone_gcs.protocol import (
    encode_command,
    decode_command,
    encode_telemetry,
    decode_telemetry,
)


class TestProtocol(unittest.TestCase):

    def test_command_round_trip(self):
        original = VelocityCommand(
            vx=0.5,
            vy=-0.3,
            seq=12,
        )

        payload = encode_command(original)
        decoded = decode_command(payload)

        self.assertIsInstance(payload, bytes)
        self.assertEqual(decoded, original)

    def test_command_velocity_clamping(self):
        command = VelocityCommand(
            vx=2.5,
            vy=-3.0,
            seq=13,
        )

        decoded = decode_command(encode_command(command))

        self.assertEqual(decoded.vx, 1.0)
        self.assertEqual(decoded.vy, -1.0)

    def test_rejects_invalid_command_values(self):
        invalid_payloads = [
            b"not-json",
            b"[]",
            b'{"type":"command","vx":true,"vy":0.2,"seq":1}',
            b'{"type":"command","vx":NaN,"vy":0.2,"seq":1}',
            b'{"type":"command","vx":0.1,"seq":1}',
            b'{"type":"command","vx":0.1,"vy":0.2,"seq":true}',
        ]

        for payload in invalid_payloads:
            with self.subTest(payload=payload):
                with self.assertRaises(ValueError):
                    decode_command(payload)

    def test_rejects_non_finite_command(self):
        command = VelocityCommand(
            vx=math.inf,
            vy=0.0,
            seq=14,
        )

        with self.assertRaises(ValueError):
            encode_command(command)

    def test_telemetry_round_trip(self):
        state = DroneState(
            x=2.0,
            y=-1.0,
            vx=0.3,
            vy=0.4,
            battery_pct=87.5,
            last_seq=15,
        )

        payload = encode_telemetry(
            state,
            frame_id=42,
            timestamp_ns=123456789,
        )

        decoded = decode_telemetry(payload)

        self.assertIsInstance(payload, bytes)
        self.assertEqual(decoded["frame_id"], 42)
        self.assertEqual(decoded["timestamp_ns"], 123456789)
        self.assertEqual(decoded["x"], 2.0)
        self.assertEqual(decoded["y"], -1.0)
        self.assertEqual(decoded["battery_pct"], 87.5)
        self.assertEqual(decoded["last_seq"], 15)
        self.assertAlmostEqual(decoded["speed"], 0.5)

    def test_rejects_invalid_telemetry(self):
        invalid_payloads = [
            b"not-json",
            b"[]",
            b'{"type":"telemetry","frame_id":5}',
        ]

        for payload in invalid_payloads:
            with self.subTest(payload=payload):
                with self.assertRaises(ValueError):
                    decode_telemetry(payload)


class TestVideoProtocol(unittest.TestCase):

    def test_small_frame_uses_one_packet(self):
        jpeg = b"\xff\xd8hello\xff\xd9"

        packets = chunk_jpeg(
            jpeg,
            frame_id=1,
            timestamp_ns=100,
        )

        self.assertEqual(len(packets), 1)
        self.assertLessEqual(len(packets[0]), 1200)

        reassembler = FrameReassembler()
        result = reassembler.push(packets[0], now_ns=1000)

        self.assertEqual(result, (1, 100, jpeg))

    def test_large_frame_is_split_and_reassembled(self):
        jpeg = b"\xff\xd8" + bytes(range(256)) * 20 + b"\xff\xd9"

        packets = chunk_jpeg(
            jpeg,
            frame_id=42,
            timestamp_ns=123456,
        )

        self.assertGreater(len(packets), 1)

        for index, packet in enumerate(packets):
            self.assertLessEqual(len(packet), 1200)

            frame_id, chunk_index, chunk_count, timestamp = (
                struct.unpack("!IHHQ", packet[:16])
            )

            self.assertEqual(frame_id, 42)
            self.assertEqual(chunk_index, index)
            self.assertEqual(chunk_count, len(packets))
            self.assertEqual(timestamp, 123456)

        reassembler = FrameReassembler()
        result = None

        for packet in packets:
            completed = reassembler.push(packet, now_ns=1000)

            if completed is not None:
                result = completed

        self.assertEqual(result, (42, 123456, jpeg))

    def test_out_of_order_packets(self):
        jpeg = b"A" * 3000

        packets = chunk_jpeg(
            jpeg,
            frame_id=7,
            timestamp_ns=500,
        )

        reassembler = FrameReassembler()
        result = None

        for packet in reversed(packets):
            completed = reassembler.push(packet, now_ns=1000)

            if completed is not None:
                result = completed

        self.assertEqual(result, (7, 500, jpeg))

    def test_duplicate_packets_are_ignored(self):
        jpeg = b"B" * 3000

        packets = chunk_jpeg(
            jpeg,
            frame_id=8,
            timestamp_ns=600,
        )

        reassembler = FrameReassembler()

        self.assertIsNone(
            reassembler.push(packets[0], now_ns=1000)
        )

        self.assertIsNone(
            reassembler.push(packets[0], now_ns=1001)
        )

        result = None

        for packet in packets[1:]:
            completed = reassembler.push(packet, now_ns=1002)

            if completed is not None:
                result = completed

        self.assertEqual(result, (8, 600, jpeg))

    def test_missing_packet_expires(self):
        jpeg = b"C" * 3000

        packets = chunk_jpeg(
            jpeg,
            frame_id=9,
            timestamp_ns=700,
        )

        reassembler = FrameReassembler(timeout_ns=100)

        self.assertIsNone(
            reassembler.push(packets[0], now_ns=0)
        )

        reassembler.expire(now_ns=101)

        for packet in packets[1:]:
            self.assertIsNone(
                reassembler.push(packet, now_ns=102)
            )

    def test_malformed_packets_are_ignored(self):
        reassembler = FrameReassembler()

        invalid_packets = [
            b"",
            b"invalid",
            struct.pack("!IHHQ", 1, 0, 0, 100) + b"data",
            struct.pack("!IHHQ", 1, 2, 2, 100) + b"data",
        ]

        for packet in invalid_packets:
            with self.subTest(packet=packet):
                self.assertIsNone(
                    reassembler.push(packet, now_ns=1000)
                )

    def test_partial_frame_limit(self):
        reassembler = FrameReassembler(max_pending_frames=2)

        frames = [
            chunk_jpeg(
                bytes([65 + i]) * 3000,
                frame_id=i + 1,
                timestamp_ns=100 + i,
            )
            for i in range(3)
        ]

        for i, packets in enumerate(frames):
            self.assertIsNone(
                reassembler.push(packets[0], now_ns=i)
            )

        # The oldest incomplete frame should have been evicted.
        for packet in frames[0][1:]:
            self.assertIsNone(
                reassembler.push(packet, now_ns=10)
            )

    def test_rejects_invalid_chunk_size(self):
        with self.assertRaises(ValueError):
            chunk_jpeg(
                b"JPEG",
                frame_id=1,
                timestamp_ns=100,
                max_datagram=16,
            )


if __name__ == "__main__":
    unittest.main()