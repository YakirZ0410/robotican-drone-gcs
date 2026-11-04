# Robotican Drone & GCS — Approved Design

Date: 2026-11-01. Submission: Thursday, 2026-11-05. Time budget: 12–15 focused hours.

## Intent and chosen approach

Build a demonstrable, explainable two-process drone simulator and Ground Control Station (GCS) over a local network. The implementation uses Python for both programs, OpenCV/NumPy for 2D image generation and JPEG compression, PySide6 for desktop controls and dashboard, standard UDP sockets, and pytest. Reliability of an end-to-end demo takes priority over visual polish. The candidate reviews the code step by step and must be able to explain the design in an interview.

## Components

- **Drone Simulator:** authoritative world state (2D position, velocity command, speed, battery, last command); receives commands; updates state with elapsed monotonic time; renders a world-anchored scrolling grid and simple landmarks; sends image frames and associated telemetry.
- **GCS:** keyboard control (simultaneous axes), live decoded video, current speed, battery and last command. Network reads never block the UI.
- **Networking:** separate UDP channels for commands, video and telemetry; video over UDP is mandatory, the others are design decisions. Both endpoints use configurable addresses and ports to work on localhost or two LAN computers.

## Protocols and defaults

- `vx` is forward/backward and `vy` left/right, independently bounded to [-1.0, 1.0]. The coordinate convention follows the ROS robot frame: +X is forward and +Y is left, so world state updates as `x += vx * dt` and `y += vy * dt`. The renderer alone converts these world coordinates to screen coordinates (forward appears up, left appears left). Finite out-of-range values are clamped to the nearest limit; non-numeric or non-finite values are rejected. Each command contains both axes and a sequence identifier. GCS repeats the latest command periodically, and simulator stops moving on command timeout.
- Video demonstration defaults: 640×480, 10 FPS, JPEG quality 65, all configurable. JPEG frames split into <=1200-byte UDP datagrams carrying frame ID, chunk number/count, and timestamp. Out-of-order chunks are reassembled, duplicates ignored, incomplete/expired frames dropped; no retransmission or unbounded queues.
- Telemetry includes speed, battery percentage, last command, frame ID and timestamp for the matching simulator snapshot. Dashboard shows matching data where present and flags stale/missing data; it must not silently pair unrelated state. Battery decreases over time and with movement and stays within [0,100].

## Acceptance criteria

1. Start simulator and GCS separately on localhost or configurable LAN hosts.
2. Combined forward/back and right/left commands visibly change the rendered scenery in real time.
3. Live JPEG video travels via UDP; loss of some packets does not freeze subsequent complete frames.
4. GCS shows speed, battery and last command with explicit frame/timestamp association.
5. Command timeout stops movement, and both programs exit cleanly.
6. Final deliverables: public GitHub source, install/run README and dependencies, 1–2 minute real-system screen recording.
7. A detailed architectural proposal covers the moving-car detection/tracking challenge and persistent IDs. Full implementation scope is ambiguous; seek clarification before increasing scope.

## Non-goals and alternatives

No 3D, photorealism, Android UI, real drone, heavyweight ML model, forward-error correction or unnecessary abstractions. C++ throughout would demonstrate low-level skills but cost more development time; mixed C++/Python adds integration burden. Python for both was chosen for a reliable demo under the time constraint. Finish core by Wednesday evening, reserve Thursday for contingency and submission. Public repository publication is a distinct user-reviewed step.
