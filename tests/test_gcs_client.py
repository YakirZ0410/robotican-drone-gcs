import socket
import unittest

from drone_gcs.gcs_client import (
    GCSClient,
    velocity_from_keys,
)
from drone_gcs.protocol import decode_command


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


if __name__ == "__main__":
    unittest.main()