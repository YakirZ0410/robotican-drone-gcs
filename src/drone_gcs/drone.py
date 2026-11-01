
"""Headless drone simulator with UDP video and telemetry."""

import argparse
import math
import socket
import time

import cv2

from drone_gcs.model import (
    DroneState,
    VelocityCommand,
    apply_command,
    advance_state,
)
from drone_gcs.protocol import (
    chunk_jpeg,
    decode_command,
    encode_telemetry,
)
from drone_gcs.render import render_frame


def command_after_timeout(
    last_ns: int | None,
    now_ns: int,
    timeout_ns: int,
    active: VelocityCommand,
) -> VelocityCommand:
    """Stop movement when the latest command has expired."""

    if (
        isinstance(timeout_ns, bool)
        or not isinstance(timeout_ns, int)
        or timeout_ns <= 0
    ):
        raise ValueError("timeout_ns must be a positive integer")

    if (
        isinstance(now_ns, bool)
        or not isinstance(now_ns, int)
        or now_ns < 0
    ):
        raise ValueError("now_ns must be a nonnegative integer")

    if last_ns is not None and (
        isinstance(last_ns, bool)
        or not isinstance(last_ns, int)
        or last_ns < 0
    ):
        raise ValueError("last_ns must be nonnegative or None")

    if not isinstance(active, VelocityCommand):
        raise ValueError("active must be a VelocityCommand")

    expired = (
        last_ns is None
        or now_ns < last_ns
        or now_ns - last_ns >= timeout_ns
    )

    if expired:
        return VelocityCommand(
            vx=0.0,
            vy=0.0,
            seq=active.seq,
        )

    return active


def should_accept_command(
    incoming: VelocityCommand,
    current: VelocityCommand,
) -> bool:
    """Accept new commands and identical keepalive commands."""

    if not isinstance(incoming, VelocityCommand):
        raise ValueError("incoming must be a VelocityCommand")

    if not isinstance(current, VelocityCommand):
        raise ValueError("current must be a VelocityCommand")

    if incoming.seq > current.seq:
        return True

    if incoming.seq < current.seq:
        return False

    return (
        incoming.vx == current.vx
        and incoming.vy == current.vy
    )


def receive_commands(
    command_socket: socket.socket,
    active_command: VelocityCommand,
    last_command_ns: int | None,
) -> tuple[VelocityCommand, int | None]:
    """Read available UDP commands without blocking."""

    # Limit work per frame so incoming traffic cannot
    # prevent the simulator from rendering.
    for _ in range(64):
        try:
            payload, _address = command_socket.recvfrom(4096)

        except BlockingIOError:
            break

        except ConnectionResetError:
            # Can occur with UDP on Windows.
            break

        try:
            incoming = decode_command(payload)

        except ValueError:
            # Ignore malformed commands.
            continue

        if not should_accept_command(incoming, active_command):
            continue

        if incoming.seq > active_command.seq:
            active_command = incoming

        # A valid duplicate also refreshes the watchdog.
        last_command_ns = time.monotonic_ns()

    return active_command, last_command_ns


def run_simulator(args: argparse.Namespace) -> None:
    """Run the drone simulation and UDP transmission loop."""

    state = DroneState()

    active_command = VelocityCommand(
        vx=0.0,
        vy=0.0,
        seq=-1,
    )

    last_command_ns = None
    timeout_ns = args.command_timeout_ms * 1_000_000

    frame_interval_s = 1.0 / args.fps
    frame_id = 0

    video_target = (args.gcs_host, args.video_port)
    telemetry_target = (args.gcs_host, args.telemetry_port)

    command_socket = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM,
    )

    sender_socket = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM,
    )

    try:
        command_socket.bind(
            (args.bind_host, args.command_port)
        )

        command_socket.setblocking(False)

        print(
            f"Drone listening for commands on "
            f"{args.bind_host}:{args.command_port}"
        )

        print(
            f"Video target: {args.gcs_host}:{args.video_port}"
        )

        print(
            f"Telemetry target: "
            f"{args.gcs_host}:{args.telemetry_port}"
        )

        print(f"Video: {args.width}x{args.height} @ {args.fps} FPS")
        print("Press Ctrl+C to stop.")

        last_update_ns = time.monotonic_ns()
        next_frame_at = time.monotonic()
        last_warning_ns = 0

        while True:

            # 1. Receive pending commands.
            active_command, last_command_ns = receive_commands(
                command_socket,
                active_command,
                last_command_ns,
            )

            # 2. Calculate elapsed time.
            now_ns = time.monotonic_ns()

            dt_s = (now_ns - last_update_ns) / 1_000_000_000
            last_update_ns = now_ns

            # 3. Apply the command watchdog.
            effective_command = command_after_timeout(
                last_ns=last_command_ns,
                now_ns=now_ns,
                timeout_ns=timeout_ns,
                active=active_command,
            )

            # 4. Update the drone state.
            state = apply_command(state, effective_command)
            state = advance_state(state, dt_s)

            # 5. Render the camera image.
            frame = render_frame(
                state,
                width=args.width,
                height=args.height,
            )

            # 6. Encode the image as JPEG.
            success, encoded = cv2.imencode(
                ".jpg",
                frame,
                [
                    cv2.IMWRITE_JPEG_QUALITY,
                    args.jpeg_quality,
                ],
            )

            if not success:
                raise RuntimeError("Failed to encode JPEG frame")

            jpeg = encoded.tobytes()

            # 7. Create packets and matching telemetry.
            packets = chunk_jpeg(
                jpeg,
                frame_id=frame_id,
                timestamp_ns=now_ns,
            )

            telemetry = encode_telemetry(
                state,
                frame_id=frame_id,
                timestamp_ns=now_ns,
            )

            # 8. Transmit video and telemetry.
            try:
                for packet in packets:
                    sender_socket.sendto(
                        packet,
                        video_target,
                    )

                sender_socket.sendto(
                    telemetry,
                    telemetry_target,
                )

            except OSError as exc:
                # A receiver may not be running yet.
                if now_ns - last_warning_ns >= 1_000_000_000:
                    print(f"UDP send warning: {exc}")
                    last_warning_ns = now_ns

            # 9. Move to the next frame ID.
            frame_id = (frame_id + 1) & 0xFFFFFFFF

            # 10. Maintain the requested frame rate.
            next_frame_at += frame_interval_s

            remaining_s = next_frame_at - time.monotonic()

            if remaining_s > 0:
                time.sleep(remaining_s)
            else:
                # Avoid accumulating delays if rendering is slow.
                next_frame_at = time.monotonic()

    finally:
        command_socket.close()
        sender_socket.close()


def main(argv: list[str] | None = None) -> int:
    """Parse arguments and start the drone simulator."""

    parser = argparse.ArgumentParser(
        description="Robotican UDP drone simulator"
    )

    parser.add_argument(
        "--bind-host",
        default="0.0.0.0",
    )

    parser.add_argument(
        "--gcs-host",
        default="127.0.0.1",
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

    parser.add_argument(
        "--fps",
        type=float,
        default=10.0,
    )

    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=65,
    )

    parser.add_argument(
        "--width",
        type=int,
        default=640,
    )

    parser.add_argument(
        "--height",
        type=int,
        default=480,
    )

    parser.add_argument(
        "--command-timeout-ms",
        type=int,
        default=500,
    )

    args = parser.parse_args(argv)

    if not math.isfinite(args.fps) or not 0 < args.fps <= 60:
        parser.error("--fps must be between 0 and 60")

    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be between 1 and 100")

    if args.width <= 0 or args.height <= 0:
        parser.error("Image dimensions must be positive")

    if args.command_timeout_ms <= 0:
        parser.error("--command-timeout-ms must be positive")

    for port in (
        args.command_port,
        args.video_port,
        args.telemetry_port,
    ):
        if not 1 <= port <= 65535:
            parser.error("Ports must be between 1 and 65535")

    try:
        run_simulator(args)

    except KeyboardInterrupt:
        print("\nDrone simulator stopped.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
