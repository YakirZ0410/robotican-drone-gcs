import socket
import unittest

from drone_gcs.gcs_client import (
    GCSClient,
    velocity_from_keys,
)
from drone_gcs.drone import receive_commands
from drone_gcs.model import DroneState, VelocityCommand
from drone_gcs.protocol import (
    chunk_jpeg,
    decode_command,
    encode_command,
    encode_telemetry,
)


class TestKeyboardCommands(unittest.TestCase):

    def test_forward_and_left_together(self):
        vx, vy = velocity_from_keys({"w", "a"})

        self.assertEqual(vx, 1.0)
        self.assertEqual(vy, 1.0)

    def test_backward_and_right_together(self):
        vx, vy = velocity_from_keys({"s", "d"})

        self.assertEqual(vx, -1.0)
        self.assertEqual(vy, -1.0)

    def test_releasing_keys_stops_movement(self):
        vx, vy = velocity_from_keys(set())

        self.assertEqual(vx, 0.0)
        self.assertEqual(vy, 0.0)

    def test_opposite_keys_cancel(self):
        vx, vy = velocity_from_keys({"w", "s", "a", "d"})

        self.assertEqual(vx, 0.0)
        self.assertEqual(vy, 0.0)


class TestCommandSending(unittest.TestCase):

    def test_commands_are_sent_with_increasing_sequence(self):
        with socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        ) as receiver:

            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(1.0)

            port = receiver.getsockname()[1]

            client = GCSClient(
                drone_host="127.0.0.1",
                command_port=port,
                bind_host="127.0.0.1",
                video_port=0,
                telemetry_port=0,
            )

            try:
                client.send_command(1.0, 0.0)
                client.send_command(0.0, 0.0)

                first = decode_command(receiver.recv(4096))
                second = decode_command(receiver.recv(4096))

                self.assertEqual(first.vx, 1.0)
                self.assertEqual(first.vy, 0.0)

                self.assertEqual(second.vx, 0.0)
                self.assertEqual(second.vy, 0.0)

                self.assertEqual(
                    second.seq,
                    first.seq + 1,
                )

            finally:
                client.close()


class TestCommandSequenceRecovery(unittest.TestCase):

    def setUp(self):
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(self.receiver.close)
        self.receiver.bind(("127.0.0.1", 0))
        self.receiver.settimeout(1.0)

    def new_client(self):
        client = GCSClient(
            command_port=self.receiver.getsockname()[1],
            bind_host="127.0.0.1",
            video_port=0,
            telemetry_port=0,
        )
        self.addCleanup(client.close)
        return client

    def send_telemetry(self, client, last_seq):
        payload = encode_telemetry(
            DroneState(last_seq=last_seq),
            frame_id=last_seq + 1,
            timestamp_ns=last_seq + 2,
        )
        self.receiver.sendto(
            payload, client._telemetry_socket.getsockname()
        )
        client.poll()

    def test_restarted_gcs_resumes_control_from_drone_telemetry(self):
        self.receiver.setblocking(False)
        original = self.new_client()
        for _ in range(3):
            original.send_command(0.5, 0.0)
        active, last_ns = receive_commands(
            self.receiver, VelocityCommand(0.0, 0.0, -1), None
        )
        self.assertEqual(active.seq, 2)
        original.close()

        restarted = self.new_client()
        restarted.send_command(0.1, -1.0)
        active, last_ns = receive_commands(self.receiver, active, last_ns)
        self.assertEqual(active.seq, 2)

        self.send_telemetry(restarted, last_seq=2)
        restarted.send_command(0.1, -1.0)
        active, last_ns = receive_commands(self.receiver, active, last_ns)
        self.assertEqual(active, VelocityCommand(0.1, -1.0, 3))

        # A delayed pre-restart datagram must not restore the old movement.
        self.receiver.sendto(
            encode_command(VelocityCommand(0.5, 0.0, 1)),
            self.receiver.getsockname(),
        )
        after_delay, after_ns = receive_commands(
            self.receiver, active, last_ns
        )
        self.assertEqual(after_delay, VelocityCommand(0.1, -1.0, 3))
        self.assertEqual(after_ns, last_ns)

    def test_delayed_telemetry_never_moves_command_sequence_backwards(self):
        client = self.new_client()
        self.send_telemetry(client, last_seq=100)
        client.send_command(1.0, 0.0)
        self.assertEqual(decode_command(self.receiver.recv(4096)).seq, 101)

        self.send_telemetry(client, last_seq=3)
        client.send_command(0.0, 1.0)
        self.assertEqual(decode_command(self.receiver.recv(4096)).seq, 102)

    def test_telemetry_does_not_reuse_unacknowledged_command_numbers(self):
        client = self.new_client()
        client.send_command(1.0, 0.0)
        client.send_command(0.0, 1.0)
        self.assertEqual(decode_command(self.receiver.recv(4096)).seq, 0)
        self.assertEqual(decode_command(self.receiver.recv(4096)).seq, 1)

        self.send_telemetry(client, last_seq=0)
        client.send_command(0.0, 0.0)
        self.assertEqual(decode_command(self.receiver.recv(4096)).seq, 2)

    def test_no_previous_drone_command_keeps_initial_sequence_zero(self):
        client = self.new_client()
        self.send_telemetry(client, last_seq=-1)
        client.send_command(1.0, 0.0)
        self.assertEqual(decode_command(self.receiver.recv(4096)).seq, 0)

    def test_invalid_telemetry_cannot_change_command_sequence(self):
        client = self.new_client()
        self.receiver.sendto(
            b'{"type":"telemetry","last_seq":100}',
            client._telemetry_socket.getsockname(),
        )
        client.poll()
        client.send_command(1.0, 0.0)
        self.assertEqual(decode_command(self.receiver.recv(4096)).seq, 0)

    def test_deeply_nested_json_is_ignored_and_next_telemetry_is_received(self):
        client = self.new_client()
        payload = b'{"type":"telemetry","x":' + b'[' * 1800 + b'0' + b']' * 1800 + b'}'
        self.assertLess(len(payload), 4096)
        self.receiver.sendto(payload, client._telemetry_socket.getsockname())

        self.send_telemetry(client, last_seq=8)
        client.send_command(1.0, 0.0)

        self.assertEqual(decode_command(self.receiver.recv(4096)).seq, 9)


class TestFrameTelemetryMatching(unittest.TestCase):
    def setUp(self):
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(self.sender.close)
        self.client = GCSClient(
            bind_host="127.0.0.1", video_port=0, telemetry_port=0,
        )
        self.addCleanup(self.client.close)

    def deliver_video(self, frame_id, timestamp_ns):
        # The networking client reassembles bytes; JPEG decoding belongs to Qt.
        for packet in chunk_jpeg(
            b"frame-bytes", frame_id=frame_id, timestamp_ns=timestamp_ns,
        ):
            self.sender.sendto(packet, self.client._video_socket.getsockname())
        self.client.poll()

    def deliver_telemetry(self, frame_id, timestamp_ns, x):
        self.sender.sendto(
            encode_telemetry(
                DroneState(x=x), frame_id=frame_id, timestamp_ns=timestamp_ns,
            ),
            self.client._telemetry_socket.getsockname(),
        )
        self.client.poll()

    def test_video_first_waits_until_matching_telemetry_arrives(self):
        self.deliver_video(42, 1000)
        self.assertIsNotNone(self.client.latest_frame())
        self.assertIsNone(self.client.telemetry_for(42))
        self.deliver_telemetry(42, 1000, x=12.0)
        self.assertEqual(self.client.telemetry_for(42)["x"], 12.0)

    def test_telemetry_first_is_available_when_video_arrives(self):
        self.deliver_telemetry(42, 1000, x=12.0)
        self.assertIsNone(self.client.latest_frame())
        self.assertIsNone(self.client.telemetry_for(42))
        self.deliver_video(42, 1000)
        self.assertEqual(self.client.telemetry_for(42)["x"], 12.0)

    def test_future_telemetry_does_not_replace_current_frame_data(self):
        self.deliver_video(42, 1000)
        self.deliver_telemetry(42, 1000, x=12.0)
        self.deliver_telemetry(43, 2000, x=99.0)
        self.assertEqual(self.client.telemetry_for(42)["x"], 12.0)
        self.assertIsNone(self.client.telemetry_for(43))
        self.deliver_video(43, 2000)
        self.assertEqual(self.client.telemetry_for(43)["x"], 99.0)

    def test_reused_frame_id_requires_matching_timestamp(self):
        self.deliver_video(42, 1000)
        self.deliver_telemetry(42, 1000, x=12.0)
        self.assertEqual(self.client.telemetry_for(42)["x"], 12.0)

        self.deliver_video(42, 2000)
        self.assertIsNone(self.client.telemetry_for(42))
        self.deliver_telemetry(42, 1000, x=-5.0)
        self.assertIsNone(self.client.telemetry_for(42))
        self.deliver_telemetry(42, 2000, x=99.0)
        self.assertEqual(self.client.telemetry_for(42)["x"], 99.0)

        # A late older image must not replace the current matching pair.
        self.deliver_video(42, 1000)
        self.assertEqual(self.client.latest_frame()[:2], (42, 2000))
        self.assertEqual(self.client.telemetry_for(42)["x"], 99.0)


if __name__ == "__main__":
    unittest.main()
