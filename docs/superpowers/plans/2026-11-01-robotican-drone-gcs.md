# Robotican Drone & GCS Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver an explainable two-process Python drone/GCS simulation with UDP live video, interactive commands, telemetry and submission documentation.

**Architecture:** The simulator owns world state and produces moving 2D JPEG video and matching telemetry. A PySide6 GCS sends periodic velocity commands, reassembles recent UDP frames, and renders the video/dashboard without blocking the GUI event loop. Three UDP ports keep commands, video and telemetry independent.

**Tech Stack:** Python 3.11+, OpenCV, NumPy, PySide6, built-in socket/threading/json/struct, pytest.

**Spec:** `docs/superpowers/specs/2026-11-01-robotican-drone-gcs-design.md`

## Global Constraints

- Deadline Thursday 2026-11-05; 12–15 working hours, prioritize a runnable core by Wednesday.
- Two independent executables over configurable local-network addresses. UDP mandatory for video.
- Normalized per-axis `vx`, `vy` in [-1.0,1.0] and simultaneous input; robot-aligned +X forward, +Y left (ROS convention); timeout-to-stop behavior.
- Video frames and telemetry share frame IDs and timestamps; no infinite blocking/buffering.
- Car-tracking challenge requires at least a detailed design proposal. Do not silently assume complete implementation is optional or mandatory; clarify with Robotican.
- Public GitHub publication happens only following explicit user review.

## Review Focus

1. Malformed/NaN/infinite/out-of-range velocity: validate without crashing (Task 1, 3).
2. Diagonal commands and irregular `dt`: move both axes using elapsed time, not frame count (Task 1).
3. Missing, duplicated, reordered and malicious UDP chunks: expire/reject safely with bounded memory (Task 3).
4. Client disappears/keys released: send zero or stop by watchdog (Task 4, 5).
5. Telemetry ahead of/behind video: never silently pair a frame to unrelated state (Task 5).

---

## File Structure

```text
pyproject.toml                   # dependencies, package, tests
README.md                        # installation and demo
src/drone_gcs/__init__.py
src/drone_gcs/model.py          # movement and battery
src/drone_gcs/render.py         # world -> OpenCV image
src/drone_gcs/protocol.py       # JSON + UDP frame chunks
src/drone_gcs/drone.py          # headless simulator runtime
src/drone_gcs/gcs_client.py     # network reception and frame buffers
src/drone_gcs/gcs.py            # PySide6 UI + keyboard
tests/test_model.py
tests/test_render.py
tests/test_protocol.py
tests/test_drone.py
tests/test_gcs_client.py
docs/car_tracking_proposal.md
```

### Task 1: Deterministic Drone State (~2 hours)

**Files:** Create `pyproject.toml`, `src/drone_gcs/__init__.py`, `src/drone_gcs/model.py`, `tests/test_model.py`.

**Interfaces:** `VelocityCommand(vx: float, vy: float, seq: int)` and `DroneState(x: float, y: float, vx: float, vy: float, battery_pct: float, last_seq: int)` dataclasses; `apply_command(state: DroneState, command: VelocityCommand) -> DroneState`, `advance_state(state: DroneState, dt_s: float) -> DroneState`, `current_speed(state: DroneState) -> float`. Define positive vx = forward, positive vy = left; update same-named state coordinates without swapping axes.

- [ ] Write `test_diagonal_motion_uses_both_axes` asserting that `vx=0.1, vy=1.0` advances both coordinates in one second; `test_time_scaling` comparing one 2-second step to two 1-second steps; `test_clamps_finite_velocity` asserting that `vx=2.0` becomes `1.0`; `test_rejects_nan_velocity` and `test_rejects_invalid_dt` expecting `ValueError`; `test_battery_never_negative`.
- [ ] Run `python -m pytest tests/test_model.py -q` and verify expected failures.
- [ ] Implement only deterministic state updates, velocity validation and battery consumption; no sockets/rendering yet.
- [ ] Run focused and full tests until green; review with the user.
- [ ] When a Git repository is explicitly set up, commit as `feat: add drone movement state`.

### Task 2: Animated Top-Down World (~1.5 hours)

**Files:** Create `src/drone_gcs/render.py`, `tests/test_render.py`.

**Interfaces:** `render_frame(state: DroneState, width: int = 640, height: int = 480) -> numpy.ndarray` yields BGR uint8 image.

- [ ] Test expected shape/type, invalid dimensions, deterministic same-state render, changed image after x/y motion.
- [ ] Run `python -m pytest tests/test_render.py -q` to confirm failures.
- [ ] Draw camera-relative grid and fixed colored landmarks with OpenCV. Convert world +X (forward) to screen up and world +Y (left) to screen left inside the renderer only.
- [ ] Run tests and visually check movement direction; review with the user.
- [ ] Commit `feat: add top-down moving environment` when Git is configured.

### Task 3: Loss-Tolerant UDP Protocol (~3 hours)

**Files:** Create `src/drone_gcs/protocol.py`, `tests/test_protocol.py`.

**Interfaces:** `encode_command(command: VelocityCommand) -> bytes`, `decode_command(payload: bytes) -> VelocityCommand`; `encode_telemetry(state: DroneState, frame_id: int, timestamp_ns: int) -> bytes`, `decode_telemetry(payload: bytes) -> dict`; `chunk_jpeg(jpeg: bytes, frame_id: int, timestamp_ns: int, max_datagram: int = 1200) -> list[bytes]`; `FrameReassembler.push(packet: bytes, now_ns: int) -> tuple[int,int,bytes] | None` and `expire(now_ns: int) -> None`. Header format `!IHHQ` (frame ID, index, count, timestamp); payload up to 1184 bytes.

- [ ] Test valid command/telemetry round trips and rejected invalid JSON/values.
- [ ] Test single/multiple chunk reassembly, out-of-order, duplicates, missing and expired chunks, malformed headers and bounded partial-frame memory.
- [ ] Run `python -m pytest tests/test_protocol.py -q` and verify failures.
- [ ] Implement JSON control/telemetry and binary UDP JPEG fragmentation/reassembly; never retransmit incomplete frames.
- [ ] Run focused and full tests; review with the user.
- [ ] Commit `feat: add udp video packet framing` when Git is configured.

### Task 4: Drone Network Runtime (~2 hours)

**Files:** Create `src/drone_gcs/drone.py`, `tests/test_drone.py`.

**Interfaces:** `main(argv: list[str] | None = None) -> int` via `python -m drone_gcs.drone`; configurable target host, command listen port 5000, video destination port 5001, telemetry destination port 5002, FPS, JPEG quality. `command_after_timeout(last_ns: int, now_ns: int, timeout_ns: int, active: VelocityCommand) -> VelocityCommand` must be independently testable.

- [ ] Test watchdog stop, stale/invalid command handling and clean shutdown with bounded socket waits.
- [ ] Run `python -m pytest tests/test_drone.py -q` to confirm failures.
- [ ] Receive periodic command state; advance model with monotonic dt; render, compress, packetize and transmit JPEG and matching telemetry snapshots.
- [ ] Run focused/full tests; manually verify headless startup and clean exit without a client.
- [ ] Commit `feat: run drone simulator` when Git is configured.

### Task 5: GCS Client and PySide6 UI (~3 hours)

**Files:** Create `src/drone_gcs/gcs_client.py`, `src/drone_gcs/gcs.py`, `tests/test_gcs_client.py`.

**Interfaces:** `GCSClient.send_command(vx: float, vy: float) -> None`, `poll() -> None`, `latest_frame() -> tuple[int,int,bytes] | None`, `telemetry_for(frame_id: int) -> dict | None`, `close() -> None`; `main(argv: list[str] | None = None) -> int` via `python -m drone_gcs.gcs`.

- [ ] Test combined key states, key release-to-zero, command sequence, newest-complete frame preference, mismatched telemetry handling, shutdown.
- [ ] Run `python -m pytest tests/test_gcs_client.py -q` to confirm failures.
- [ ] Implement PySide6 video panel, directional keys/WASD and speed/battery/last-command text, with off-UI-thread network polling and bounded queues.
- [ ] Run tests and manual two-process localhost demo; review with the user.
- [ ] Commit `feat: add ground control station` when Git is configured.

### Task 6: Integration and Submission (~2.5 hours)

**Files:** Create `README.md`, `docs/car_tracking_proposal.md`; extend focused tests as needed.

**Interfaces:** Document `python -m pip install -e .`, `python -m drone_gcs.drone`, `python -m drone_gcs.gcs`, ports/configuration, controls, expected output, video loss handling and `python -m pytest -q`.

- [ ] Run entire test suite from documented environment; resolve failures.
- [ ] Test combined movement, loss tolerance, telemetry sync, battery, command timeout and clean exit across both processes.
- [ ] Document periodic cars, image-based detection choices, association by IoU/centroid and Hungarian matching, stable ID lifecycle, occlusion/lost-track policy and performance trade-offs; clarify challenge scope.
- [ ] Prepare reproducible README and a 1–2 minute recording checklist; user records actual running application.
- [ ] Review for passwords/personal paths before explicit user approval to create/push public GitHub repository.

## Execution Handoff

Native, one task at a time with the user. No subagents. The user approved the high-level architecture and will review this detailed plan before implementation. Prioritize a working integration before optional enhancements and keep Thursday as contingency.
