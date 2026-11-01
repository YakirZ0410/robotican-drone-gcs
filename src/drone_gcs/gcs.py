
"""PySide6 Ground Control Station for the drone simulator."""

import sys
import argparse
import threading

import cv2
import numpy as np

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from drone_gcs.gcs_client import (
    GCSClient,
    velocity_from_keys,
)


KEY_MAP = {
    Qt.Key.Key_W: "w",
    Qt.Key.Key_S: "s",
    Qt.Key.Key_A: "a",
    Qt.Key.Key_D: "d",
    Qt.Key.Key_Up: "up",
    Qt.Key.Key_Down: "down",
    Qt.Key.Key_Left: "left",
    Qt.Key.Key_Right: "right",
}

# Windows physical scan codes for WASD.
WINDOWS_SCAN_CODES = {
    0x11: "w",
    0x1F: "s",
    0x1E: "a",
    0x20: "d",
}

STOP_BUTTON_STYLE = (
    "background-color: #c0392b;"
    "color: white;"
    "font-size: 16px;"
    "font-weight: bold;"
    "padding: 10px;"
)

RESUME_BUTTON_STYLE = (
    "background-color: #218c4a;"
    "color: white;"
    "font-size: 16px;"
    "font-weight: bold;"
    "padding: 10px;"
)


def movement_key(event) -> str | None:
    """Identify movement keys independently of keyboard layout."""

    if sys.platform == "win32":
        physical_key = WINDOWS_SCAN_CODES.get(
            event.nativeScanCode()
        )

        if physical_key is not None:
            return physical_key

    return KEY_MAP.get(event.key())


class GCSWindow(QMainWindow):

    def __init__(self, client: GCSClient):
        super().__init__()

        self.client = client
        self.pressed_keys: set[str] = set()

        # Emergency stop remains active until RESUME is clicked.
        self.stop_latched = False

        self.last_displayed_frame = None
        self.original_pixmap = None

        self.stop_event = threading.Event()

        self.setWindowTitle("Robotican - Ground Control Station")
        self.resize(900, 750)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self._setup_ui()

        # Receive UDP packets on a background thread.
        self.network_thread = threading.Thread(
            target=self._network_loop,
            daemon=True,
        )
        self.network_thread.start()

        # Send the active command every 100 ms.
        self.command_timer = QTimer(self)
        self.command_timer.timeout.connect(self._send_command)
        self.command_timer.start(100)

        # Refresh video and telemetry.
        self.display_timer = QTimer(self)
        self.display_timer.timeout.connect(self._update_display)
        self.display_timer.start(50)

        self.setFocus()

    def _setup_ui(self):
        central = QWidget(self)
        layout = QVBoxLayout(central)

        self.video_label = QLabel("Waiting for UDP video...")
        self.video_label.setAlignment(
            Qt.AlignmentFlag.AlignCenter
        )
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setStyleSheet(
            "background-color: #111a16;"
            "color: white;"
            "border: 2px solid #3b5446;"
            "font-size: 18px;"
        )

        self.connection_label = QLabel("Waiting for drone...")
        self.connection_label.setStyleSheet(
            "font-size: 16px; font-weight: bold;"
        )

        self.telemetry_label = QLabel(
            "Waiting for matching telemetry..."
        )
        self.telemetry_label.setStyleSheet(
            "font-size: 15px;"
        )

        self.command_label = QLabel(
            "Command: VX=0.0 | VY=0.0"
        )

        self.instructions_label = QLabel(
            "Controls: W/S - Forward/Backward | "
            "A/D - Left/Right | Arrow keys also supported"
        )

        self.stop_button = QPushButton("STOP")
        self.stop_button.setStyleSheet(STOP_BUTTON_STYLE)
        self.stop_button.setFocusPolicy(
            Qt.FocusPolicy.NoFocus
        )
        self.stop_button.clicked.connect(self.toggle_stop)

        layout.addWidget(self.video_label, stretch=1)
        layout.addWidget(self.connection_label)
        layout.addWidget(self.telemetry_label)
        layout.addWidget(self.command_label)
        layout.addWidget(self.instructions_label)
        layout.addWidget(self.stop_button)

        self.setCentralWidget(central)

    def _network_loop(self):
        """Receive UDP packets without blocking the GUI thread."""

        while not self.stop_event.is_set():
            try:
                self.client.poll()

            except OSError as exc:
                if not self.stop_event.is_set():
                    print(f"Network error: {exc}")
                break

            self.stop_event.wait(0.01)

    def _send_command(self):
        """Send velocity commands, respecting emergency stop."""

        if self.stop_latched or not self.isActiveWindow():
            vx, vy = 0.0, 0.0
        else:
            vx, vy = velocity_from_keys(self.pressed_keys)

        try:
            self.client.send_command(vx, vy)

        except (OSError, RuntimeError) as exc:
            self.connection_label.setText(
                f"Command send error: {exc}"
            )
            return

        if self.stop_latched:
            self.command_label.setText(
                "Command: VX=0.0 | VY=0.0 | STOP ACTIVE"
            )
        else:
            self.command_label.setText(
                f"Command: VX={vx:.1f} | VY={vy:.1f}"
            )

    def _update_display(self):
        """Display the newest frame and matching telemetry."""

        latest = self.client.latest_frame()

        if latest is None:
            return

        frame_id, timestamp_ns, jpeg = latest
        frame_key = (frame_id, timestamp_ns)

        if frame_key != self.last_displayed_frame:
            encoded = np.frombuffer(jpeg, dtype=np.uint8)

            frame = cv2.imdecode(
                encoded,
                cv2.IMREAD_COLOR,
            )

            if frame is not None:
                rgb = cv2.cvtColor(
                    frame,
                    cv2.COLOR_BGR2RGB,
                )

                height, width, channels = rgb.shape

                qimage = QImage(
                    rgb.data,
                    width,
                    height,
                    channels * width,
                    QImage.Format.Format_RGB888,
                ).copy()

                self.original_pixmap = QPixmap.fromImage(
                    qimage
                )

                self._scale_video()

                self.last_displayed_frame = frame_key

                self.connection_label.setText(
                    f"Receiving video | Frame ID: {frame_id}"
                )

        telemetry = self.client.telemetry_for(frame_id)

        if telemetry is None:
            self.telemetry_label.setText(
                f"Frame {frame_id}: waiting for matching telemetry"
            )
            return

        self.telemetry_label.setText(
            f"Position: X={telemetry['x']:.2f}, "
            f"Y={telemetry['y']:.2f}  |  "
            f"Speed: {telemetry['speed']:.2f}  |  "
            f"Battery: {telemetry['battery_pct']:.1f}%  |  "
            f"Last command: {telemetry['last_seq']}"
        )

    def _scale_video(self):
        """Fit the video inside the label without distortion."""

        if self.original_pixmap is None:
            return

        scaled = self.original_pixmap.scaled(
            self.video_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )

        self.video_label.setPixmap(scaled)

    def keyPressEvent(self, event):
        """Track pressed movement keys."""

        if event.isAutoRepeat():
            return

        key = movement_key(event)

        if key is not None:
            # Ignore movement input while emergency stop is active.
            if not self.stop_latched:
                self.pressed_keys.add(key)

            self._send_command()
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        """Remove released movement keys."""

        if event.isAutoRepeat():
            return

        key = movement_key(event)

        if key is not None:
            self.pressed_keys.discard(key)
            self._send_command()
        else:
            super().keyReleaseEvent(event)

    def focusOutEvent(self, event):
        """Prevent stuck keys when the window loses focus."""

        self.pressed_keys.clear()
        super().focusOutEvent(event)

    def stop_movement(self):
        """Engage emergency stop and request zero velocity."""

        self.stop_latched = True
        self.pressed_keys.clear()

        try:
            self.client.send_command(0.0, 0.0)

        except (OSError, RuntimeError) as exc:
            print(f"STOP command failed: {exc}", flush=True)

        self.command_label.setText(
            "Command: VX=0.0 | VY=0.0 | STOP ACTIVE"
        )

        self.stop_button.setText("RESUME")
        self.stop_button.setStyleSheet(RESUME_BUTTON_STYLE)

    def resume_movement(self):
        """Release emergency stop without restoring old key states."""

        self.stop_latched = False
        self.pressed_keys.clear()

        self.stop_button.setText("STOP")
        self.stop_button.setStyleSheet(STOP_BUTTON_STYLE)

        # Send zero until the user presses a new movement key.
        self._send_command()
        self.setFocus()

    def toggle_stop(self):
        """Toggle emergency stop using the STOP/RESUME button."""

        if self.stop_latched:
            self.resume_movement()
        else:
            self.stop_movement()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._scale_video()

    def closeEvent(self, event):
        """Stop movement and release networking resources."""

        self.command_timer.stop()
        self.display_timer.stop()

        # Always engage STOP when closing, never RESUME.
        self.stop_movement()

        self.stop_event.set()
        self.network_thread.join(timeout=1.0)

        self.client.close()

        event.accept()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Robotican Ground Control Station"
    )

    parser.add_argument(
        "--drone-host",
        default="127.0.0.1",
    )

    parser.add_argument(
        "--bind-host",
        default="0.0.0.0",
    )

    parser.add_argument(
        "--command-port",
        type=int,
        default=5000,
    )

    parser.add_argument(
        "--video-port",
        type=int,
        default=5001,
    )

    parser.add_argument(
        "--telemetry-port",
        type=int,
        default=5002,
    )

    args = parser.parse_args(argv)

    app = QApplication.instance()

    if app is None:
        app = QApplication([])

    try:
        client = GCSClient(
            drone_host=args.drone_host,
            command_port=args.command_port,
            bind_host=args.bind_host,
            video_port=args.video_port,
            telemetry_port=args.telemetry_port,
        )

    except OSError as exc:
        print(f"Failed to start GCS networking: {exc}")
        return 1

    window = GCSWindow(client)
    window.show()

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
