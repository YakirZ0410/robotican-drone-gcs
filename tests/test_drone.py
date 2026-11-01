
import unittest

from drone_gcs.model import VelocityCommand
from drone_gcs.drone import (
    command_after_timeout,
    should_accept_command,
)


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


if __name__ == "__main__":
    unittest.main()
