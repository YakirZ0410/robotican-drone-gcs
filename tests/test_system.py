"""Exercise the real simulator process and GCS client over loopback UDP."""

import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from drone_gcs.gcs_client import GCSClient


class TestSimulatorSystem(unittest.TestCase):
    def setUp(self):
        self.commands_sent = 0
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(("127.0.0.1", 0))
            self.command_port = reservation.getsockname()[1]

        self.client = GCSClient(
            command_port=self.command_port, bind_host="127.0.0.1",
            video_port=0, telemetry_port=0,
        )
        self.addCleanup(self.client.close)
        self.output = tempfile.TemporaryFile(mode="w+t", encoding="utf-8")
        self.addCleanup(self.output.close)
        self.log_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.log_directory.cleanup)
        self.log_path = Path(self.log_directory.name) / "drone.log"
        self.process = None
        self.addCleanup(self.stop_simulator)

    def start_simulator(self, fps="20"):
        executable = sys.executable
        environment = None
        if os.name == "nt" and sys.prefix != sys.base_prefix:
            # Match CPython multiprocessing's Windows venv launch strategy:
            # track the actual interpreter, not a redirector that spawns it.
            executable = sys._base_executable
            environment = os.environ.copy()
            environment["__PYVENV_LAUNCHER__"] = sys.executable
        self.process = subprocess.Popen(
            [
                executable, "-u", "-m", "drone_gcs.drone",
                "--bind-host", "127.0.0.1", "--gcs-host", "127.0.0.1",
                "--command-port", str(self.command_port),
                "--video-port", str(self.client._video_socket.getsockname()[1]),
                "--telemetry-port", str(self.client._telemetry_socket.getsockname()[1]),
                "--fps", fps, "--width", "160", "--height", "120",
                "--command-timeout-ms", "300",
                "--log-file", str(self.log_path),
            ],
            cwd=Path(__file__).resolve().parents[1],
            stdout=self.output, stderr=subprocess.STDOUT,
            env=environment,
            creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
        )

    def stop_simulator(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2.0)

    def wait_for_snapshot(self, predicate, command=None):
        deadline = time.monotonic() + 5.0
        next_command_at = 0.0
        last_telemetry = None
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                break
            now = time.monotonic()
            if command is not None and now >= next_command_at:
                self.client.send_command(*command)
                self.commands_sent += 1
                next_command_at = now + 0.05
            self.client.poll()
            latest = self.client.latest_frame()
            if latest is not None:
                telemetry = self.client.telemetry_for(latest[0])
                if telemetry is not None:
                    self.assertEqual(telemetry["frame_id"], latest[0])
                    self.assertEqual(telemetry["timestamp_ns"], latest[1])
                    last_telemetry = telemetry
                    if predicate(telemetry):
                        return latest, telemetry
            time.sleep(0.01)

        self.output.seek(0)
        self.fail(
            f"No matching snapshot satisfied the condition within 5 seconds; "
            f"last telemetry={last_telemetry}, exit={self.process.poll()}\n"
            f"Simulator output:\n{self.output.read()}"
        )

    def test_real_simulator_moves_and_watchdog_stops_after_commands_cease(self):
        self.start_simulator()
        initial_frame, initial = self.wait_for_snapshot(lambda state: True)
        pixels = cv2.imdecode(
            np.frombuffer(initial_frame[2], dtype=np.uint8), cv2.IMREAD_COLOR,
        )
        self.assertIsNotNone(pixels)
        self.assertEqual(pixels.shape, (120, 160, 3))
        self.assertEqual((initial["x"], initial["y"]), (0.0, 0.0))

        _, moving = self.wait_for_snapshot(
            lambda state: state["x"] > 0.1 and state["y"] < -0.05,
            command=(0.75, -0.5),
        )
        self.assertEqual((moving["vx"], moving["vy"]), (0.75, -0.5))
        self.assertLess(moving["battery_pct"], initial["battery_pct"])
        # This fresh client starts at zero; count sends independently of its
        # internal sequence counter, including a command still in flight.
        last_sent_sequence = self.commands_sent - 1

        # Send no zero command: stopping must come from the real watchdog.
        _, stopped = self.wait_for_snapshot(
            lambda state: state["vx"] == 0.0 and state["vy"] == 0.0
            and state["timestamp_ns"] > moving["timestamp_ns"],
        )
        self.assertEqual(stopped["last_seq"], last_sent_sequence)
        self.assertGreaterEqual(stopped["x"], moving["x"])
        self.assertLessEqual(stopped["y"], moving["y"])
        self.assertEqual(stopped["speed"], 0.0)

        _, later = self.wait_for_snapshot(
            lambda state: state["timestamp_ns"] > stopped["timestamp_ns"],
        )
        self.assertEqual(later["last_seq"], stopped["last_seq"])
        self.assertEqual((later["vx"], later["vy"]), (0.0, 0.0))
        self.assertEqual((later["x"], later["y"]), (stopped["x"], stopped["y"]))

        _, resumed = self.wait_for_snapshot(
            lambda state: state["last_seq"] > stopped["last_seq"],
            command=(0.0, 0.0),
        )
        text = self.log_path.read_text(encoding="utf-8")
        self.assertIn("Simulator started", text)
        self.assertIn("Velocity command received", text)
        self.assertEqual(text.count("Command timeout:"), 1)
        self.assertIn("Commands resumed", text)
        self.assertNotIn("Command accepted:", text)

        # A zero-velocity command can expire too; record that new transition
        # once rather than logging it on every idle control tick.
        self.wait_for_snapshot(
            lambda state: state["timestamp_ns"] >= resumed["timestamp_ns"] + 500_000_000,
        )
        self.assertEqual(
            self.log_path.read_text(encoding="utf-8").count("Command timeout:"), 2,
        )

    def test_low_video_rate_does_not_delay_command_handling_or_watchdog(self):
        self.start_simulator(fps="0.5")
        first_frame, _ = self.wait_for_snapshot(lambda state: True)
        # One command after the first frame, then no keepalives. Video frames
        # are two seconds apart, but control must honor the 300 ms watchdog.
        self.client.send_command(1.0, 0.0)
        _, telemetry = self.wait_for_snapshot(
            lambda state: state["timestamp_ns"] > first_frame[1],
        )
        self.assertEqual(telemetry["last_seq"], 0)
        self.assertEqual((telemetry["vx"], telemetry["vy"]), (0.0, 0.0))
        self.assertGreater(telemetry["x"], 0.1)
        self.assertLess(telemetry["x"], 0.7)
        self.assertEqual(telemetry["y"], 0.0)

    def test_opposite_arrow_keys_stop_motion_and_log_velocity_transitions(self):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtCore import QEvent, Qt
        from PySide6.QtGui import QKeyEvent
        from PySide6.QtWidgets import QApplication
        from drone_gcs.gcs import GCSWindow

        app = QApplication.instance() or QApplication([])
        window = GCSWindow(self.client)
        self.addCleanup(window.close)
        # Drive events/polling explicitly, as in the GUI control tests. The
        # separate simulator, UDP transport and log file remain real.
        window.stop_event.set()
        window.network_thread.join(timeout=1.0)
        self.assertFalse(window.network_thread.is_alive())
        window.command_timer.stop()
        window.display_timer.stop()
        self.start_simulator()
        self.wait_for_snapshot(lambda state: True)

        def send_and_observe(event_type=None, key=None):
            sequence = self.client._sequence
            if event_type is None:
                window._send_command()
            else:
                event = QKeyEvent(event_type, key, Qt.KeyboardModifier.NoModifier)
                app.sendEvent(window, event)
            _, state = self.wait_for_snapshot(
                lambda snapshot: snapshot["last_seq"] >= sequence,
            )
            return state

        expected_logs = []
        pairs = (
            (Qt.Key.Key_Up, Qt.Key.Key_Down, (1.0, 0.0)),
            (Qt.Key.Key_Down, Qt.Key.Key_Up, (-1.0, 0.0)),
            (Qt.Key.Key_Left, Qt.Key.Key_Right, (0.0, 1.0)),
            (Qt.Key.Key_Right, Qt.Key.Key_Left, (0.0, -1.0)),
        )
        with patch.object(window, "isActiveWindow", return_value=True):
            for first, second, first_velocity in pairs:
                for release_first in (True, False):
                    with self.subTest(first=first.name, release_first=release_first):
                        moving = send_and_observe(QEvent.Type.KeyPress, first)
                        self.assertEqual((moving["vx"], moving["vy"]), first_velocity)
                        stopped = send_and_observe(QEvent.Type.KeyPress, second)
                        self.assertEqual((stopped["vx"], stopped["vy"]), (0.0, 0.0))

                        # Send repeated commands while both keys stay held.
                        # Matching telemetry must retain the same position;
                        # command logs must not report either direction.
                        for _ in range(8):
                            held = send_and_observe()
                            self.assertEqual((held["vx"], held["vy"]), (0.0, 0.0))
                            self.assertEqual(held["speed"], 0.0)
                            self.assertEqual(
                                (held["x"], held["y"]), (stopped["x"], stopped["y"]),
                            )

                        released, remaining = (first, second) if release_first else (second, first)
                        remaining_velocity = (
                            tuple(-axis for axis in first_velocity)
                            if release_first else first_velocity
                        )
                        moving = send_and_observe(QEvent.Type.KeyRelease, released)
                        self.assertEqual((moving["vx"], moving["vy"]), remaining_velocity)
                        stopped = send_and_observe(QEvent.Type.KeyRelease, remaining)
                        self.assertEqual((stopped["vx"], stopped["vy"]), (0.0, 0.0))
                        self.assertEqual(window.pressed_keys, set())
                        expected_logs.extend((
                            first_velocity, (0.0, 0.0), remaining_velocity, (0.0, 0.0),
                        ))

        lines = self.log_path.read_text(encoding="utf-8").splitlines()
        actual_logs = [
            tuple(map(float, line.split(" vx=", 1)[1].split(" vy=")))
            for line in lines if "Velocity command received:" in line
        ]
        self.assertEqual(actual_logs, expected_logs)


if __name__ == "__main__":
    unittest.main()
