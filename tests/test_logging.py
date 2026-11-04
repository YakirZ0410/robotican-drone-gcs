"""Behavioral tests for simulator log output and bounded file retention."""

import io
import logging
from pathlib import Path
import socket
import tempfile
import unittest
from unittest.mock import patch

from drone_gcs import drone
from drone_gcs.log_config import configure_drone_logging, close_drone_logging
from drone_gcs.model import VelocityCommand
from drone_gcs.protocol import encode_command


class TestDroneLoggingConfiguration(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "logs" / "drone.log"
        self.console = io.StringIO()
        stderr = patch("sys.stderr", self.console)
        stderr.start()
        self.addCleanup(stderr.stop)

    def configure(self, **kwargs):
        self.addCleanup(close_drone_logging)
        return configure_drone_logging(self.path, **kwargs)

    def test_info_reaches_file_and_console_without_debug_noise(self):
        logger = self.configure()
        logger.info("Velocity changed: vx=1.0; שלום")
        logger.debug("Detailed keepalive")

        text = self.path.read_text(encoding="utf-8")
        self.assertIn("Velocity changed: vx=1.0; שלום", text)
        self.assertIn("INFO", text)
        self.assertRegex(text, r"\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}")
        self.assertIn("Velocity changed", self.console.getvalue())
        self.assertNotIn("Detailed keepalive", text)
        self.assertNotIn("Detailed keepalive", self.console.getvalue())

    def test_debug_can_be_enabled(self):
        logger = self.configure(level="DEBUG")
        logger.debug("Command accepted: seq=7")
        self.assertIn("Command accepted: seq=7", self.path.read_text(encoding="utf-8"))
        self.assertIn("DEBUG", self.console.getvalue())

    def test_rotation_retains_only_bounded_recent_history(self):
        logger = self.configure(max_bytes=200, backup_count=2)
        for index in range(20):
            logger.info("Event %02d: %s", index, "x" * 60)

        files = list(self.path.parent.glob("drone.log*"))
        self.assertEqual(len(files), 3)
        text = "\n".join(path.read_text(encoding="utf-8") for path in files)
        self.assertIn("Event 19:", self.path.read_text(encoding="utf-8"))
        self.assertNotIn("Event 00:", text)
        for path in files:
            self.assertLessEqual(path.stat().st_size, 200)

    def test_unwritable_log_path_falls_back_to_console(self):
        self.path.parent.write_text("Not a directory", encoding="utf-8")
        logger = self.configure()
        logger.info("Simulation continues")
        self.assertIn("Could not open log file", self.console.getvalue())
        self.assertIn("Simulation continues", self.console.getvalue())

    def test_reconfiguration_does_not_duplicate_messages_or_leave_files_open(self):
        logger = self.configure()
        logger.info("First event")
        logger = self.configure()
        logger.info("Second event")
        close_drone_logging()

        renamed = self.path.with_name("closed.log")
        self.path.rename(renamed)
        text = renamed.read_text(encoding="utf-8")
        self.assertEqual(text.count("First event"), 1)
        self.assertEqual(text.count("Second event"), 1)
        self.assertEqual(self.console.getvalue().count("Second event"), 1)


class TestDroneLogEvents(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "drone.log"
        self.addCleanup(close_drone_logging)

    def test_keyboard_interrupt_is_logged_and_file_is_closed(self):
        with patch.object(drone, "run_simulator", side_effect=KeyboardInterrupt):
            result = drone.main(["--log-file", str(self.path)])
        self.assertEqual(result, 0)
        self.assertIn("Simulator stopped by user", self.path.read_text(encoding="utf-8"))
        self.path.rename(self.path.with_name("closed.log"))

    def test_fatal_error_has_traceback_and_nonzero_exit_status(self):
        with patch.object(drone, "run_simulator", side_effect=RuntimeError("encoder unavailable")):
            result = drone.main(["--log-file", str(self.path), "--log-level", "DEBUG"])
        self.assertEqual(result, 1)
        text = self.path.read_text(encoding="utf-8")
        self.assertIn("ERROR", text)
        self.assertIn("Simulator failed", text)
        self.assertIn("Traceback", text)
        self.assertIn("encoder unavailable", text)

    def test_received_commands_log_changes_without_info_keepalive_spam(self):
        with (
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver,
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender,
        ):
            receiver.bind(("127.0.0.1", 0))
            receiver.setblocking(False)
            for command in (
                VelocityCommand(1.0, 0.0, 1),
                VelocityCommand(1.0, 0.0, 2),
                VelocityCommand(1.0, 0.0, 2),
                VelocityCommand(0.0, 0.0, 3),
                VelocityCommand(1.0, 0.0, 1),
            ):
                sender.sendto(encode_command(command), receiver.getsockname())
            sender.sendto(b"bad command", receiver.getsockname())

            with self.assertLogs("drone_gcs.drone", level="DEBUG") as captured:
                active, _ = drone.receive_commands(
                    receiver, VelocityCommand(0.0, 0.0, -1), None,
                )

        self.assertEqual(active, VelocityCommand(0.0, 0.0, 3))
        info = [record.getMessage() for record in captured.records if record.levelno == logging.INFO]
        self.assertEqual(len(info), 2)
        self.assertIn("seq=1 vx=1.000 vy=0.000", info[0])
        self.assertIn("seq=3 vx=0.000 vy=0.000", info[1])
        text = "\n".join(captured.output)
        self.assertEqual(text.count("Command accepted: seq=2"), 2)
        self.assertIn("Command rejected: seq=1", text)
        self.assertIn("Ignored malformed command", text)


if __name__ == "__main__":
    unittest.main()
