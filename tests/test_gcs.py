"""Exercise stream-loss and recovery behavior through the real Qt widgets."""

import os
import socket
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import cv2
import numpy as np
from PySide6.QtWidgets import QApplication

from drone_gcs.gcs import GCSWindow
from drone_gcs.gcs_client import GCSClient
from drone_gcs.model import DroneState
from drone_gcs.protocol import chunk_jpeg, encode_telemetry


class TestVideoStreamStatus(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        ok, jpeg = cv2.imencode(
            ".jpg", np.full((48, 64, 3), (16, 64, 16), dtype=np.uint8)
        )
        assert ok
        cls.jpeg = jpeg.tobytes()

    def setUp(self):
        clock_patch = patch("time.monotonic", return_value=100.0)
        self.clock = clock_patch.start()
        self.addCleanup(clock_patch.stop)
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sender.bind(("127.0.0.1", 0))
        self.addCleanup(self.sender.close)
        self.client = GCSClient(
            command_port=self.sender.getsockname()[1],
            bind_host="127.0.0.1", video_port=0, telemetry_port=0,
        )
        self.addCleanup(self.client.close)
        self.window = GCSWindow(self.client)
        self.addCleanup(self.window.close)
        # Poll explicitly to make packet delivery and GUI updates deterministic.
        self.window.stop_event.set()
        self.window.network_thread.join(timeout=1.0)
        self.window.command_timer.stop()
        self.window.display_timer.stop()

    def deliver_telemetry(self, frame_id, battery_pct=87.0, state=None):
        if state is None:
            state = DroneState(x=12.0, battery_pct=battery_pct, last_seq=4)
        self.sender.sendto(
            encode_telemetry(
                state,
                frame_id=frame_id, timestamp_ns=frame_id * 1000,
            ),
            self.client._telemetry_socket.getsockname(),
        )

    def deliver_frame(
        self, frame_id, telemetry=True, jpeg=None, battery_pct=87.0, state=None
    ):
        for packet in chunk_jpeg(
            self.jpeg if jpeg is None else jpeg,
            frame_id=frame_id, timestamp_ns=frame_id * 1000,
        ):
            self.sender.sendto(
                packet, self.client._video_socket.getsockname()
            )
        if telemetry:
            self.deliver_telemetry(frame_id, battery_pct=battery_pct, state=state)
        self.client.poll()
        self.window._update_display()

    def test_video_loss_marks_retained_telemetry_stale_at_timeout(self):
        self.deliver_frame(1)
        self.assertIn("Receiving video", self.window.connection_label.text())
        self.assertIn("87.0%", self.window.telemetry_label.text())
        self.clock.return_value = 100.999
        self.window._update_display()
        self.assertIn("Receiving video", self.window.connection_label.text())

        self.clock.return_value = 101.0
        self.window._update_display()
        self.assertIn("Video stream lost", self.window.connection_label.text())
        self.assertIn("STALE", self.window.telemetry_label.text())
        self.assertIn("87.0%", self.window.telemetry_label.text())
        self.assertIsNotNone(self.window.video_label.pixmap())

    def test_startup_without_video_changes_from_waiting_to_no_video(self):
        self.window._update_display()
        self.assertIn("Waiting", self.window.connection_label.text())
        self.clock.return_value = 101.0
        self.window._update_display()
        self.assertIn("No video received", self.window.connection_label.text())
        self.assertNotIn("Receiving video", self.window.connection_label.text())

    def test_new_frame_recovers_status_without_releasing_stop(self):
        self.deliver_frame(1)
        self.clock.return_value = 101.0
        self.window._update_display()
        self.assertIn("Video stream lost", self.window.connection_label.text())
        self.window.stop_movement()

        self.clock.return_value = 101.1
        self.deliver_frame(2)
        self.assertIn("Receiving video", self.window.connection_label.text())
        self.assertIn("Frame ID: 2", self.window.connection_label.text())
        self.assertNotIn("STALE", self.window.telemetry_label.text())
        self.assertTrue(self.window.stop_latched)

    def test_duplicate_frame_does_not_keep_stream_alive(self):
        self.deliver_frame(1)
        self.clock.return_value = 100.8
        self.deliver_frame(1)
        self.clock.return_value = 101.0
        self.window._update_display()
        self.assertIn("Video stream lost", self.window.connection_label.text())

    def test_invalid_jpeg_does_not_refresh_video_or_show_new_telemetry(self):
        self.deliver_frame(1)
        self.clock.return_value = 100.8
        self.deliver_frame(2, jpeg=b"not a JPEG image", battery_pct=45.0)
        self.assertIn("Frame ID: 1", self.window.connection_label.text())
        self.assertIn("87.0%", self.window.telemetry_label.text())
        self.assertNotIn("45.0%", self.window.telemetry_label.text())
        self.clock.return_value = 101.0
        self.window._update_display()
        self.assertIn("Video stream lost", self.window.connection_label.text())
        self.assertIn("STALE", self.window.telemetry_label.text())

    def test_telemetry_alone_does_not_keep_video_alive(self):
        self.deliver_frame(1)
        self.clock.return_value = 100.8
        self.deliver_telemetry(1)
        self.client.poll()
        self.window._update_display()
        self.clock.return_value = 101.0
        self.window._update_display()
        self.assertIn("Video stream lost", self.window.connection_label.text())

    def test_new_video_without_telemetry_does_not_reuse_old_values(self):
        self.deliver_frame(1)
        self.clock.return_value = 100.5
        self.deliver_frame(2, telemetry=False)
        self.assertIn("Receiving video", self.window.connection_label.text())
        self.assertIn("waiting for matching telemetry", self.window.telemetry_label.text())
        self.assertNotIn("87.0%", self.window.telemetry_label.text())

    def test_delayed_telemetry_fills_the_already_displayed_frame(self):
        self.deliver_frame(1, telemetry=False)
        self.assertIn("waiting for matching telemetry", self.window.telemetry_label.text())
        self.deliver_telemetry(1)
        self.client.poll()
        self.window._update_display()
        self.assertEqual(self.window.last_displayed_frame, (1, 1000))
        self.assertIn("X=12.00", self.window.telemetry_label.text())
        self.assertNotIn("waiting for matching telemetry", self.window.telemetry_label.text())

    def test_command_send_failure_stays_visible_until_a_successful_send(self):
        self.deliver_frame(1)
        with patch.object(
            self.client, "send_command", side_effect=OSError("network unavailable")
        ):
            self.window._send_command()
        self.assertEqual(self.window.command_label.text(), "Command: SEND FAILED")
        self.window._update_display()
        self.assertIn("Command send error", self.window.connection_label.text())

        self.window._send_command()
        self.window._update_display()
        self.assertIn("ZERO VELOCITY SENT", self.window.command_label.text())
        self.assertIn("Receiving video", self.window.connection_label.text())
        self.assertNotIn("Command send error", self.window.connection_label.text())

    def test_dashboard_shows_reported_axes_instead_of_local_command(self):
        # An inactive window requests zero; the drone reports different axes.
        self.window._send_command()
        self.assertIn("VX=0.0 | VY=0.0", self.window.command_label.text())
        self.deliver_frame(1, state=DroneState(vx=0.1, vy=-1.0, last_seq=42))
        displayed = self.window.telemetry_label.text()
        self.assertIn("VX=0.10", displayed)
        self.assertIn("VY=-1.00", displayed)
        self.assertIn("Last command: 42", displayed)

        self.clock.return_value = 101.0
        self.window._update_display()
        self.assertIn("STALE", self.window.telemetry_label.text())
        self.assertIn("VY=-1.00", self.window.telemetry_label.text())

    def test_reported_zero_axes_replace_movement_with_same_command_number(self):
        self.deliver_frame(1, state=DroneState(vx=0.1, vy=-1.0, last_seq=42))
        # The watchdog can stop the drone without accepting a new command.
        self.deliver_frame(2, state=DroneState(vx=0.0, vy=0.0, last_seq=42))
        displayed = self.window.telemetry_label.text()
        self.assertIn("VX=0.00", displayed)
        self.assertIn("VY=0.00", displayed)
        self.assertIn("Last command: 42", displayed)
        self.assertNotIn("VY=-1.00", displayed)

    def test_reused_frame_id_during_decode_never_pairs_new_pose_with_old_image(self):
        self.deliver_frame(0)
        decode = cv2.imdecode

        def decode_while_new_frame_arrives(*args, **kwargs):
            image = decode(*args, **kwargs)
            for packet in chunk_jpeg(self.jpeg, frame_id=1, timestamp_ns=2000):
                self.sender.sendto(packet, self.client._video_socket.getsockname())
            self.sender.sendto(
                encode_telemetry(
                    DroneState(x=99.0, last_seq=4),
                    frame_id=1, timestamp_ns=2000,
                ),
                self.client._telemetry_socket.getsockname(),
            )
            self.client.poll()
            return image

        with patch("drone_gcs.gcs.cv2.imdecode", side_effect=decode_while_new_frame_arrives):
            self.deliver_frame(1, state=DroneState(x=12.0, last_seq=4))

        self.assertEqual(self.window.last_displayed_frame, (1, 1000))
        self.assertIn("X=12.00", self.window.telemetry_label.text())
        self.assertNotIn("waiting for matching telemetry", self.window.telemetry_label.text())
        self.assertNotIn("X=99.00", self.window.telemetry_label.text())
        self.window._update_display()
        self.assertEqual(self.window.last_displayed_frame, (1, 2000))
        self.assertIn("X=99.00", self.window.telemetry_label.text())


if __name__ == "__main__":
    unittest.main()
