
import socket
import unittest

from drone_gcs.model import VelocityCommand
from drone_gcs.drone import (
    command_after_timeout,
    receive_commands,
    should_accept_command,
)
from drone_gcs.protocol import encode_command


class TestCommandTimeout(unittest.TestCase):

    def setUp(self):
        self.active_command = VelocityCommand(
            vx=0.8,
            vy=-0.4,
            seq=10,
        )

        self.stop_command = VelocityCommand(
            vx=0.0,
            vy=0.0,
            seq=10,
        )

    def test_recent_command_keeps_moving(self):
        result = command_after_timeout(
            last_ns=1_000_000_000,
            now_ns=1_200_000_000,
            timeout_ns=500_000_000,
            active=self.active_command,
        )

        self.assertEqual(result, self.active_command)

    def test_old_command_stops_movement(self):
        result = command_after_timeout(
            last_ns=1_000_000_000,
            now_ns=1_600_000_000,
            timeout_ns=500_000_000,
            active=self.active_command,
        )

        self.assertEqual(result, self.stop_command)

    def test_exact_timeout_boundary_stops(self):
        result = command_after_timeout(
            last_ns=1_000_000_000,
            now_ns=1_500_000_000,
            timeout_ns=500_000_000,
            active=self.active_command,
        )

        self.assertEqual(result, self.stop_command)

    def test_no_command_received_stops(self):
        result = command_after_timeout(
            last_ns=None,
            now_ns=1_000_000_000,
            timeout_ns=500_000_000,
            active=self.active_command,
        )

        self.assertEqual(result, self.stop_command)

    def test_backwards_clock_stops_safely(self):
        result = command_after_timeout(
            last_ns=2_000_000_000,
            now_ns=1_000_000_000,
            timeout_ns=500_000_000,
            active=self.active_command,
        )

        self.assertEqual(result, self.stop_command)

    def test_invalid_timeout_rejected(self):
        with self.assertRaises(ValueError):
            command_after_timeout(
                last_ns=1_000,
                now_ns=2_000,
                timeout_ns=0,
                active=self.active_command,
            )


class TestCommandOrdering(unittest.TestCase):

    def setUp(self):
        self.current = VelocityCommand(
            vx=0.5,
            vy=0.0,
            seq=10,
        )

    def test_accepts_newer_command(self):
        incoming = VelocityCommand(
            vx=0.0,
            vy=1.0,
            seq=11,
        )

        self.assertTrue(
            should_accept_command(incoming, self.current)
        )

    def test_rejects_older_command(self):
        incoming = VelocityCommand(
            vx=1.0,
            vy=0.0,
            seq=9,
        )

        self.assertFalse(
            should_accept_command(incoming, self.current)
        )

    def test_accepts_duplicate_as_keepalive(self):
        incoming = VelocityCommand(
            vx=0.5,
            vy=0.0,
            seq=10,
        )

        self.assertTrue(
            should_accept_command(incoming, self.current)
        )

    def test_rejects_conflicting_duplicate(self):
        incoming = VelocityCommand(
            vx=-1.0,
            vy=0.0,
            seq=10,
        )

        self.assertFalse(
            should_accept_command(incoming, self.current)
        )


class TestCommandReception(unittest.TestCase):
    def test_deeply_nested_json_is_ignored_and_next_command_is_received(self):
        with (
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver,
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender,
        ):
            receiver.bind(("127.0.0.1", 0))
            receiver.setblocking(False)
            payload = b'{"type":"command","x":' + b'[' * 1800 + b'0' + b']' * 1800 + b'}'
            self.assertLess(len(payload), 4096)
            sender.sendto(payload, receiver.getsockname())
            expected = VelocityCommand(0.5, -0.25, 1)
            sender.sendto(encode_command(expected), receiver.getsockname())

            active, last_ns = receive_commands(
                receiver, VelocityCommand(0.0, 0.0, -1), None,
            )

            self.assertEqual(active, expected)
            self.assertIsNotNone(last_ns)

    def test_oversized_datagram_is_ignored_and_next_command_is_received(self):
        with (
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver,
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender,
        ):
            receiver.bind(("127.0.0.1", 0))
            receiver.setblocking(False)
            sender.sendto(b"x" * 4097, receiver.getsockname())
            expected = VelocityCommand(0.5, -0.25, 1)
            sender.sendto(encode_command(expected), receiver.getsockname())

            active, last_ns = receive_commands(
                receiver, VelocityCommand(0.0, 0.0, -1), None,
            )

            self.assertEqual(active, expected)
            self.assertIsNotNone(last_ns)


if __name__ == "__main__":
    unittest.main()
