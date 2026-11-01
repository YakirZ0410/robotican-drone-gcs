
"""UDP networking client for the Ground Control Station."""

import socket
import threading
import time

from drone_gcs.model import VelocityCommand
from drone_gcs.protocol import (
    FrameReassembler,
    decode_telemetry,
    encode_command,
)


MAX_VIDEO_PACKETS_PER_POLL = 512
MAX_TELEMETRY_PACKETS_PER_POLL = 64
MAX_CACHED_TELEMETRY = 128


def velocity_from_keys(keys: set[str]) -> tuple[float, float]:
    """Convert pressed keys into normalized drone velocities."""

    pressed = {key.lower() for key in keys}

    forward = "w" in pressed or "up" in pressed
    backward = "s" in pressed or "down" in pressed

    left = "a" in pressed or "left" in pressed
    right = "d" in pressed or "right" in pressed

    vx = float(forward) - float(backward)
    vy = float(left) - float(right)

    return vx, vy


class GCSClient:
    """Send commands and receive UDP video and telemetry."""

    def __init__(
        self,
        drone_host: str = "127.0.0.1",
        command_port: int = 5000,
        bind_host: str = "0.0.0.0",
        video_port: int = 5001,
        telemetry_port: int = 5002,
    ):
        self.drone_target = (drone_host, command_port)

        self._sequence = 0
        self._closed = False

        self._lock = threading.Lock()

        self._latest_frame: tuple[int, int, bytes] | None = None

        self._telemetry: dict[
            tuple[int, int], dict
        ] = {}

        self._reassembler = FrameReassembler()

        # Socket for sending commands.
        self._command_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )

        # Socket for receiving video.
        self._video_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )

        # Socket for receiving telemetry.
        self._telemetry_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )

        try:
            self._video_socket.setsockopt(
                socket.SOL_SOCKET,
                socket.SO_RCVBUF,
                2_000_000,
            )

            self._video_socket.bind(
                (bind_host, video_port)
            )

            self._telemetry_socket.bind(
                (bind_host, telemetry_port)
            )

            self._video_socket.setblocking(False)
            self._telemetry_socket.setblocking(False)

        except OSError:
            self.close()
            raise

    def send_command(self, vx: float, vy: float) -> None:
        """Send a velocity command with an increasing sequence number."""

        with self._lock:
            if self._closed:
                raise RuntimeError("GCSClient is closed")

            command = VelocityCommand(
                vx=vx,
                vy=vy,
                seq=self._sequence,
            )

            payload = encode_command(command)

            self._command_socket.sendto(
                payload,
                self.drone_target,
            )

            self._sequence += 1

    def poll(self) -> None:
        """Read available video and telemetry packets without blocking."""

        if self._closed:
            return

        now_ns = time.monotonic_ns()
        self._reassembler.expire(now_ns)

        # Receive video packets.
        for _ in range(MAX_VIDEO_PACKETS_PER_POLL):
            try:
                packet, _address = self._video_socket.recvfrom(65535)

            except BlockingIOError:
                break

            except ConnectionResetError:
                break

            except OSError:
                if self._closed:
                    return
                raise

            completed = self._reassembler.push(
                packet,
                now_ns=now_ns,
            )

            if completed is None:
                continue

            frame_id, timestamp_ns, jpeg = completed

            with self._lock:
                if (
                    self._latest_frame is None
                    or timestamp_ns > self._latest_frame[1]
                ):
                    self._latest_frame = (
                        frame_id,
                        timestamp_ns,
                        jpeg,
                    )

        # Receive telemetry packets.
        for _ in range(MAX_TELEMETRY_PACKETS_PER_POLL):
            try:
                packet, _address = self._telemetry_socket.recvfrom(65535)

            except BlockingIOError:
                break

            except ConnectionResetError:
                break

            except OSError:
                if self._closed:
                    return
                raise

            try:
                telemetry = decode_telemetry(packet)

            except ValueError:
                continue

            key = (
                telemetry["frame_id"],
                telemetry["timestamp_ns"],
            )

            with self._lock:
                self._telemetry[key] = telemetry

                # Keep telemetry memory bounded.
                if len(self._telemetry) > MAX_CACHED_TELEMETRY:
                    oldest_key = next(iter(self._telemetry))
                    del self._telemetry[oldest_key]

    def latest_frame(self) -> tuple[int, int, bytes] | None:
        """Return the newest complete JPEG frame."""

        with self._lock:
            return self._latest_frame

    def telemetry_for(self, frame_id: int) -> dict | None:
        """Return telemetry matching the latest frame and timestamp."""

        with self._lock:
            if self._latest_frame is None:
                return None

            latest_id, timestamp_ns, _jpeg = self._latest_frame

            if frame_id != latest_id:
                return None

            return self._telemetry.get(
                (frame_id, timestamp_ns)
            )

    def close(self) -> None:
        """Close all networking sockets."""

        with self._lock:
            if self._closed:
                return

            self._closed = True

        self._command_socket.close()
        self._video_socket.close()
        self._telemetry_socket.close()
