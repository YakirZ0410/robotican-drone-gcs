
"""JSON protocols for commands and telemetry."""

import json
import math
import struct

from dataclasses import dataclass, field

from drone_gcs.model import (
    DroneState,
    VelocityCommand,
    current_speed,
)


MAX_JSON_BYTES = 4096
MAX_FRAME_ID = (1 << 32) - 1
MAX_TIMESTAMP_NS = (1 << 64) - 1


def _finite_number(value, name: str) -> float:
    """Validate and convert a value to a finite float."""

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")

    try:
        number = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{name} is invalid") from exc

    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")

    return number


def _integer(
    value,
    name: str,
    minimum: int = 0,
    maximum: int | None = None,
) -> int:
    """Validate an integer within an allowed range."""

    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")

    if value < minimum:
        raise ValueError(f"{name} is below the allowed minimum")

    if maximum is not None and value > maximum:
        raise ValueError(f"{name} exceeds the allowed maximum")

    return value


def _decode_message(payload: bytes, expected_type: str) -> dict:
    """Decode JSON and validate its message type."""

    if not isinstance(payload, bytes):
        raise ValueError("Payload must be bytes")

    if len(payload) > MAX_JSON_BYTES:
        raise ValueError("JSON payload is too large")

    try:
        message = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError, RecursionError) as exc:
        # A small datagram can still exceed the JSON parser's nesting limit.
        raise ValueError("Invalid JSON payload") from exc

    if not isinstance(message, dict):
        raise ValueError("JSON message must be an object")

    if message.get("type") != expected_type:
        raise ValueError("Unexpected message type")

    return message


def encode_command(command: VelocityCommand) -> bytes:
    """Serialize a velocity command to JSON bytes."""

    if not isinstance(command, VelocityCommand):
        raise ValueError("Expected VelocityCommand")

    vx = _finite_number(command.vx, "vx")
    vy = _finite_number(command.vy, "vy")
    seq = _integer(command.seq, "seq")

    message = {
        "type": "command",
        "vx": max(-1.0, min(1.0, vx)),
        "vy": max(-1.0, min(1.0, vy)),
        "seq": seq,
    }

    return json.dumps(
        message,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def decode_command(payload: bytes) -> VelocityCommand:
    """Deserialize and validate a velocity command."""

    message = _decode_message(payload, "command")

    vx = _finite_number(message.get("vx"), "vx")
    vy = _finite_number(message.get("vy"), "vy")
    seq = _integer(message.get("seq"), "seq")

    return VelocityCommand(
        vx=max(-1.0, min(1.0, vx)),
        vy=max(-1.0, min(1.0, vy)),
        seq=seq,
    )


def encode_telemetry(
    state: DroneState,
    frame_id: int,
    timestamp_ns: int,
) -> bytes:
    """Serialize a drone-state snapshot to JSON bytes."""

    if not isinstance(state, DroneState):
        raise ValueError("Expected DroneState")

    frame_id = _integer(frame_id, "frame_id", maximum=MAX_FRAME_ID)
    timestamp_ns = _integer(
        timestamp_ns,
        "timestamp_ns",
        maximum=MAX_TIMESTAMP_NS,
    )

    x = _finite_number(state.x, "x")
    y = _finite_number(state.y, "y")
    vx = _finite_number(state.vx, "vx")
    vy = _finite_number(state.vy, "vy")
    battery_pct = _finite_number(state.battery_pct, "battery_pct")
    last_seq = _integer(state.last_seq, "last_seq", minimum=-1)

    if not 0.0 <= battery_pct <= 100.0:
        raise ValueError("Battery percentage must be between 0 and 100")

    speed = current_speed(state)

    if not math.isfinite(speed):
        raise ValueError("Speed must be finite")

    message = {
        "type": "telemetry",
        "frame_id": frame_id,
        "timestamp_ns": timestamp_ns,
        "x": x,
        "y": y,
        "vx": vx,
        "vy": vy,
        "speed": speed,
        "battery_pct": battery_pct,
        "last_seq": last_seq,
    }

    return json.dumps(
        message,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def decode_telemetry(payload: bytes) -> dict:
    """Deserialize and validate a telemetry message."""

    message = _decode_message(payload, "telemetry")

    frame_id = _integer(
        message.get("frame_id"),
        "frame_id",
        maximum=MAX_FRAME_ID,
    )

    timestamp_ns = _integer(
        message.get("timestamp_ns"),
        "timestamp_ns",
        maximum=MAX_TIMESTAMP_NS,
    )

    x = _finite_number(message.get("x"), "x")
    y = _finite_number(message.get("y"), "y")
    vx = _finite_number(message.get("vx"), "vx")
    vy = _finite_number(message.get("vy"), "vy")
    speed = _finite_number(message.get("speed"), "speed")
    battery_pct = _finite_number(
        message.get("battery_pct"),
        "battery_pct",
    )
    last_seq = _integer(
        message.get("last_seq"),
        "last_seq",
        minimum=-1,
    )

    if speed < 0:
        raise ValueError("Speed cannot be negative")

    if not 0.0 <= battery_pct <= 100.0:
        raise ValueError("Invalid battery percentage")

    return {
        "frame_id": frame_id,
        "timestamp_ns": timestamp_ns,
        "x": x,
        "y": y,
        "vx": vx,
        "vy": vy,
        "speed": speed,
        "battery_pct": battery_pct,
        "last_seq": last_seq,
    }


# UDP video packet format:
# frame_id (4), chunk_index (2), chunk_count (2), timestamp_ns (8)

_VIDEO_HEADER = struct.Struct("!IHHQ")

MAX_DATAGRAM_BYTES = 1200
MAX_JPEG_BYTES = 2_000_000
MAX_CHUNKS_PER_FRAME = 2048


def chunk_jpeg(
    jpeg: bytes,
    frame_id: int,
    timestamp_ns: int,
    max_datagram: int = MAX_DATAGRAM_BYTES,
) -> list[bytes]:
    """Split JPEG bytes into UDP-sized packets."""

    if not isinstance(jpeg, bytes) or not jpeg:
        raise ValueError("JPEG must be non-empty bytes")

    if len(jpeg) > MAX_JPEG_BYTES:
        raise ValueError("JPEG frame is too large")

    frame_id = _integer(
        frame_id,
        "frame_id",
        maximum=MAX_FRAME_ID,
    )

    timestamp_ns = _integer(
        timestamp_ns,
        "timestamp_ns",
        maximum=MAX_TIMESTAMP_NS,
    )

    max_datagram = _integer(
        max_datagram,
        "max_datagram",
        minimum=_VIDEO_HEADER.size + 1,
        maximum=65507,
    )

    payload_size = max_datagram - _VIDEO_HEADER.size

    chunk_count = (
        len(jpeg) + payload_size - 1
    ) // payload_size

    if chunk_count > MAX_CHUNKS_PER_FRAME:
        raise ValueError("Too many JPEG chunks")

    packets = []

    for chunk_index in range(chunk_count):
        start = chunk_index * payload_size
        end = start + payload_size

        payload = jpeg[start:end]

        header = _VIDEO_HEADER.pack(
            frame_id,
            chunk_index,
            chunk_count,
            timestamp_ns,
        )

        packets.append(header + payload)

    return packets


@dataclass
class _PartialFrame:
    """Temporary storage for an incomplete JPEG frame."""

    created_ns: int
    chunk_count: int
    chunks: dict[int, bytes] = field(default_factory=dict)
    received_bytes: int = 0


class FrameReassembler:
    """Reassemble UDP video packets into complete JPEG frames."""

    def __init__(
        self,
        timeout_ns: int = 500_000_000,
        max_pending_frames: int = 8,
        max_frame_bytes: int = MAX_JPEG_BYTES,
        max_datagram: int = MAX_DATAGRAM_BYTES,
    ):
        self.timeout_ns = _integer(
            timeout_ns,
            "timeout_ns",
            minimum=1,
        )

        self.max_pending_frames = _integer(
            max_pending_frames,
            "max_pending_frames",
            minimum=1,
        )

        self.max_frame_bytes = _integer(
            max_frame_bytes,
            "max_frame_bytes",
            minimum=1,
        )

        self.max_datagram = _integer(
            max_datagram,
            "max_datagram",
            minimum=_VIDEO_HEADER.size + 1,
            maximum=65507,
        )

        self._pending: dict[
            tuple[int, int], _PartialFrame
        ] = {}

    def expire(self, now_ns: int) -> None:
        """Discard incomplete frames that have timed out."""

        now_ns = _integer(now_ns, "now_ns")

        expired_keys = [
            key
            for key, partial in self._pending.items()
            if now_ns - partial.created_ns > self.timeout_ns
        ]

        for key in expired_keys:
            del self._pending[key]

    def push(
        self,
        packet: bytes,
        now_ns: int,
    ) -> tuple[int, int, bytes] | None:
        """Accept a packet; return a frame when all chunks arrive."""

        now_ns = _integer(now_ns, "now_ns")

        self.expire(now_ns)

        if not isinstance(packet, bytes):
            return None

        if not (
            _VIDEO_HEADER.size < len(packet) <= self.max_datagram
        ):
            return None

        try:
            frame_id, chunk_index, chunk_count, timestamp_ns = (
                _VIDEO_HEADER.unpack(
                    packet[:_VIDEO_HEADER.size]
                )
            )
        except struct.error:
            return None

        if (
            chunk_count == 0
            or chunk_count > MAX_CHUNKS_PER_FRAME
            or chunk_index >= chunk_count
        ):
            return None

        payload = packet[_VIDEO_HEADER.size:]

        # Pair frame ID with timestamp to avoid mixing frames.
        key = (frame_id, timestamp_ns)

        partial = self._pending.get(key)

        if partial is None:
            # Keep only a bounded number of incomplete frames.
            if len(self._pending) >= self.max_pending_frames:
                oldest_key = min(
                    self._pending,
                    key=lambda existing_key:
                        self._pending[existing_key].created_ns,
                )
                del self._pending[oldest_key]

            partial = _PartialFrame(
                created_ns=now_ns,
                chunk_count=chunk_count,
            )

            self._pending[key] = partial

        elif partial.chunk_count != chunk_count:
            return None

        # Ignore duplicates.
        if chunk_index in partial.chunks:
            return None

        new_size = partial.received_bytes + len(payload)

        if new_size > self.max_frame_bytes:
            del self._pending[key]
            return None

        partial.chunks[chunk_index] = payload
        partial.received_bytes = new_size

        # The frame is still incomplete.
        if len(partial.chunks) < partial.chunk_count:
            return None

        # Reassemble in chunk-index order, not arrival order.
        jpeg = b"".join(
            partial.chunks[index]
            for index in range(partial.chunk_count)
        )

        del self._pending[key]

        return frame_id, timestamp_ns, jpeg
