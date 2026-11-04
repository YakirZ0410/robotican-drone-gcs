"""Check keyboard and STOP/RESUME behavior through real UDP commands."""

import os
import socket
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QEvent, Qt
from PySide6.QtGui import QFocusEvent, QKeyEvent
from PySide6.QtWidgets import QApplication

from drone_gcs.gcs import GCSWindow
from drone_gcs.gcs_client import GCSClient
from drone_gcs.protocol import decode_command


class TestGCSControls(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(self.receiver.close)
        self.receiver.bind(("127.0.0.1", 0))
        self.receiver.settimeout(1.0)
        self.client = GCSClient(
            command_port=self.receiver.getsockname()[1],
            bind_host="127.0.0.1", video_port=0, telemetry_port=0,
        )
        self.addCleanup(self.client.close)
        self.window = GCSWindow(self.client)
        self.addCleanup(self.window.close)
        self.window.stop_event.set()
        self.window.network_thread.join(timeout=1.0)
        self.window.command_timer.stop()
        self.window.display_timer.stop()
        # Offscreen windows have no desktop activation; keep actual input,
        # velocity calculation, encoding and UDP transmission intact.
        active_window = patch.object(self.window, "isActiveWindow", return_value=True)
        active_window.start()
        self.addCleanup(active_window.stop)

    def key_event(self, event_type, key):
        event = QKeyEvent(event_type, key, Qt.KeyboardModifier.NoModifier)
        QApplication.sendEvent(self.window, event)

    def received_velocity(self):
        command = decode_command(self.receiver.recv(4096))
        return command.vx, command.vy

    def test_keys_combine_and_releasing_one_preserves_the_other_axis(self):
        self.key_event(QEvent.Type.KeyPress, Qt.Key.Key_W)
        self.assertEqual(self.received_velocity(), (1.0, 0.0))
        self.assertEqual(
            self.window.command_label.text(),
            "Command: MOVEMENT SENT | VX=1.0 | VY=0.0",
        )
        self.key_event(QEvent.Type.KeyPress, Qt.Key.Key_A)
        self.assertEqual(self.received_velocity(), (1.0, 1.0))
        self.key_event(QEvent.Type.KeyRelease, Qt.Key.Key_W)
        self.assertEqual(self.received_velocity(), (0.0, 1.0))
        self.key_event(QEvent.Type.KeyRelease, Qt.Key.Key_A)
        self.assertEqual(self.received_velocity(), (0.0, 0.0))
        self.assertEqual(
            self.window.command_label.text(),
            "Command: ZERO VELOCITY SENT | VX=0.0 | VY=0.0",
        )

    def test_stop_blocks_keys_and_resume_requires_fresh_input(self):
        self.key_event(QEvent.Type.KeyPress, Qt.Key.Key_W)
        self.assertEqual(self.received_velocity(), (1.0, 0.0))

        self.window.stop_button.click()
        self.assertEqual(self.received_velocity(), (0.0, 0.0))
        self.assertTrue(self.window.stop_latched)
        self.assertEqual(self.window.stop_button.text(), "RESUME")
        self.assertEqual(self.window.pressed_keys, set())
        self.assertEqual(
            self.window.command_label.text(),
            "Command: ZERO VELOCITY SENT | VX=0.0 | VY=0.0 | STOP ACTIVE",
        )

        self.key_event(QEvent.Type.KeyPress, Qt.Key.Key_A)
        self.assertEqual(self.received_velocity(), (0.0, 0.0))
        self.window._send_command()
        self.assertEqual(self.received_velocity(), (0.0, 0.0))

        self.window.stop_button.click()
        self.assertEqual(self.received_velocity(), (0.0, 0.0))
        self.assertFalse(self.window.stop_latched)
        self.assertEqual(self.window.stop_button.text(), "STOP")
        self.assertEqual(self.window.pressed_keys, set())
        self.assertEqual(
            self.window.command_label.text(),
            "Command: ZERO VELOCITY SENT | VX=0.0 | VY=0.0",
        )
        self.window._send_command()
        self.assertEqual(self.received_velocity(), (0.0, 0.0))

        self.key_event(QEvent.Type.KeyPress, Qt.Key.Key_W)
        self.assertEqual(self.received_velocity(), (1.0, 0.0))

    def test_focus_loss_clears_held_input_and_requests_zero(self):
        self.key_event(QEvent.Type.KeyPress, Qt.Key.Key_W)
        self.assertEqual(self.received_velocity(), (1.0, 0.0))
        QApplication.sendEvent(self.window, QFocusEvent(QEvent.Type.FocusOut))
        self.assertEqual(self.window.pressed_keys, set())

        # Even stale input must not move the drone while the window is inactive.
        self.window.pressed_keys.add("a")
        with patch.object(self.window, "isActiveWindow", return_value=False):
            self.window._send_command()
        self.assertEqual(self.received_velocity(), (0.0, 0.0))
        self.assertEqual(
            self.window.command_label.text(),
            "Command: ZERO VELOCITY SENT | VX=0.0 | VY=0.0 | WINDOW INACTIVE",
        )
        QApplication.sendEvent(self.window, QFocusEvent(QEvent.Type.FocusOut))
        self.window._send_command()
        self.assertEqual(self.received_velocity(), (0.0, 0.0))

    def test_failed_stop_send_keeps_stop_latched_and_reports_failure_until_retry(self):
        with patch.object(
            self.client, "send_command", side_effect=OSError("network unavailable")
        ):
            self.window.stop_button.click()

        self.assertTrue(self.window.stop_latched)
        self.assertEqual(self.window.stop_button.text(), "RESUME")
        self.assertEqual(self.window.command_label.text(), "Command: SEND FAILED")
        self.assertIn("Command send error", self.window.connection_label.text())

        self.window._send_command()
        self.assertEqual(self.received_velocity(), (0.0, 0.0))
        self.assertEqual(
            self.window.command_label.text(),
            "Command: ZERO VELOCITY SENT | VX=0.0 | VY=0.0 | STOP ACTIVE",
        )
        self.window._update_display()
        self.assertNotIn("Command send error", self.window.connection_label.text())


if __name__ == "__main__":
    unittest.main()
