# Simulated Drone & Ground Control Station

A Python implementation of the Robotican take-home assignment. Two independent
processes communicate over UDP on the same computer or across a local network:

- **Drone simulator:** receives velocity commands, moves through a generated 2D
  world, and sends JPEG video frames with synchronized telemetry.
- **Ground Control Station (GCS):** displays live video, position, speed, battery,
  applied velocity axes, and the last accepted command sequence number. Keyboard input controls movement;
  a STOP/RESUME button latches and releases a movement stop.

The simulator generates the camera feed programmatically using a scrolling grid
and fixed landmarks. No camera, drone hardware, video file, or model download is
required. Vehicle detection and tracking are not implemented; the
[architectural proposal](docs/car_tracking_proposal.md) describes the planned
extension, its trade-offs, and its validation strategy.

## Requirements

- Python **3.11 or newer**; Windows verification uses Python 3.12.
- A graphical desktop for the GCS. The simulator can run without a display.
- Git if cloning the repository, and an internet connection for dependency installation.

Dependencies are declared in [pyproject.toml](pyproject.toml): NumPy,
OpenCV (`opencv-python`), and PySide6. The optional `dev` extra installs pytest.
There is no separate compilation step for the application.

## Install

Clone or extract the project, then open a terminal in its root directory. If you
already have the project locally, skip the clone step.

```text
git clone https://github.com/YakirZ0410/robotican-drone-gcs.git
cd robotican-drone-gcs
```

### Windows PowerShell

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e "."
```

If Python 3.12 is not installed, use another installed Python version >=3.11 to
create the environment, for example `py -3.11 -m venv .venv`.
The commands use the environment's interpreter directly, so PowerShell activation
and changes to its execution policy are unnecessary.

### Linux / macOS

Check that `python3 --version` is >=3.11, then run:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e "."
```

On Linux, the Python venv package and Qt's system display libraries may need to
be installed through your distribution's package manager. Use a desktop session
to launch the GCS. Linux and macOS have not yet been verified for this submission.

## Run on one computer

Open **two terminals in the project root**. Start the GCS first so it is ready to
receive video, then start the simulator. Both use `127.0.0.1` by default.

### Windows PowerShell

Terminal 1 — GCS:

```powershell
.\.venv\Scripts\python.exe -m drone_gcs.gcs
```

Terminal 2 — simulator:

```powershell
.\.venv\Scripts\python.exe -m drone_gcs.drone
```

### Linux / macOS

Terminal 1 — GCS:

```bash
.venv/bin/python -m drone_gcs.gcs
```

Terminal 2 — simulator:

```bash
.venv/bin/python -m drone_gcs.drone
```

The GCS should display a grid, three colored landmarks when they are in view,
and a centered arrow representing the drone. Movement shifts the scenery relative
to the arrow. Telemetry shows position, speed, battery percentage, applied
velocity (`VX` and `VY`, to two decimal places), and the last accepted command
sequence number. These axes come from the simulator's telemetry for the displayed
frame. The separate command label shows the velocity currently requested by the
GCS. It shows `MOVEMENT SENT` or `ZERO VELOCITY SENT` after a successful local
UDP send, with `STOP ACTIVE` or `WINDOW INACTIVE` when applicable. A failed send
shows `SEND FAILED`, with details in the status line. `SENT` is not a delivery
acknowledgement from the simulator; reported applied velocity remains separate.
Before the first send, the label shows `WAITING TO SEND`.
If the simulator's command watchdog stops movement, the reported applied
axes become zero while the last accepted command number remains unchanged.

If no new, decodable video frame is displayed for one second, a red
`Video stream lost` warning appears and retained telemetry is marked `STALE`.
The last image remains visible for reference. Before the first video frame, the
GCS initially waits, then reports `No video received` after one second. A new
valid frame clears the warning automatically; it does not release an active STOP.
The timeout uses the GCS's local monotonic clock, so the two computers do not need
synchronized clocks. A video timeout indicates missing recent frames, not proof
that the command connection has failed.

Close the GCS window to stop its process; closing sends a zero-velocity command.
Press **Ctrl+C** in the simulator terminal to stop the simulator.

## Controls

Click the GCS window to make it active before using the keyboard.

| Key | Action | Command axis |
| --- | --- | --- |
| W / Up arrow | Forward | `vx = +1` |
| S / Down arrow | Backward | `vx = -1` |
| A / Left arrow | Left | `vy = +1` |
| D / Right arrow | Right | `vy = -1` |
| Release movement keys | Stop the released axes | `0` |
| STOP button | Latch zero velocity; ignore movement keys | Both axes `0` |
| RESUME button | Release STOP; press movement keys again | Initially `0` |

Hold keys on different axes together, such as W+D, to move diagonally. Opposite
directions on one axis cancel. The convention is **+X forward, +Y left**.
Each axis is independently limited to `[-1.0, 1.0]`; diagonal speed can therefore
reach `sqrt(2)` simulation units per second.

The protocol accepts fractional velocities, but the keyboard interface currently
selects only `-1`, `0`, or `+1` per axis. The GCS sends commands every 100 ms and
requests zero velocity while its window is inactive. The simulator stops movement
after 500 ms without an accepted command by default.
Between video frames, the simulator continues processing commands and updating
the motion/watchdog state, with idle sleeps capped at 10 ms. Lowering video FPS
therefore does not suspend control until the next frame. Rendering and sending
still run in the same loop, so this is best-effort timing rather than a hard
real-time guarantee.

## Run on two computers over a LAN

Install the project on both computers. Identify each computer's LAN IPv4 address
(`ipconfig` on Windows). Both machines must be able to reach each other.

For example, if the simulator is at `192.168.1.10` and the GCS is at
`192.168.1.20`, run these commands in each machine's project directory:

On the **GCS computer**, Windows PowerShell:

```powershell
.\.venv\Scripts\python.exe -m drone_gcs.gcs --drone-host 192.168.1.10
```

On the **simulator computer**, Windows PowerShell:

```powershell
.\.venv\Scripts\python.exe -m drone_gcs.drone --gcs-host 192.168.1.20
```

Replace the example addresses with your actual addresses. On Linux/macOS, replace
`.\.venv\Scripts\python.exe` with `.venv/bin/python`.

Both nodes listen on `0.0.0.0` by default. The `--drone-host` and `--gcs-host`
options specify the remote destination; `--bind-host` specifies the local
listening address. Use the remote LAN address for destinations when running on
separate machines.

| Channel | Direction | Default destination port | Payload |
| --- | --- | --- | --- |
| Commands | GCS -> simulator | UDP 5000 | JSON: `vx`, `vy`, `seq` |
| Video | Simulator -> GCS | UDP 5001 | Chunked JPEG with frame metadata |
| Telemetry | Simulator -> GCS | UDP 5002 | JSON snapshot with frame metadata |

Allow inbound UDP 5000 on the simulator computer and UDP 5001/5002 on the GCS
computer through the local firewall. On Windows, allow Python on the private
network when prompted. Wi-Fi client isolation can prevent devices from reaching
each other even when they share a network name.

Override ports with `--command-port`, `--video-port`, and `--telemetry-port` on
**both nodes**, using matching values. Two-computer LAN operation still needs a
manual verification; automated integration verification uses localhost.

## Video settings and network behavior

The default stream is **640x480 at 10 FPS**, JPEG quality **65**. To reduce
bandwidth, lower resolution, frame rate, or JPEG quality on the simulator:

```powershell
.\.venv\Scripts\python.exe -m drone_gcs.drone --width 320 --height 240 --fps 5 --jpeg-quality 45
```

On Linux/macOS, use `.venv/bin/python` in place of the Windows interpreter path.
All command-line options are available with:

```powershell
.\.venv\Scripts\python.exe -m drone_gcs.drone --help
.\.venv\Scripts\python.exe -m drone_gcs.gcs --help
```

- JPEG frames are split into UDP datagrams of at most 1200 bytes, including a
  16-byte header carrying frame ID, chunk index/count, and timestamp.
- The receiver handles out-of-order chunks, ignores duplicate chunks within a
  pending frame, and discards incomplete frames after 500 ms. It holds at most
  eight incomplete frames and displays the newest completed frame.
- No retransmission is used: losing a chunk can lose that frame, but later
  complete frames can still be displayed. This favors low latency over delivery
  of every frame. JPEG frames are independently decodable.
- Resolution, FPS, and quality provide manual bandwidth controls. There is no
  automatic bitrate adaptation, congestion control, or pacing between chunks of
  the same frame.
- Each frame and its telemetry snapshot share a frame ID and simulator monotonic
  timestamp. The GCS matches both values; it does not compare clocks on different
  computers. Missing matching telemetry is reported rather than replaced with
  data from an unrelated frame.
- Telemetry uses UDP to avoid waiting for old snapshots to be retransmitted.
  Lost telemetry is not recovered; subsequent frames provide new snapshots.
- Command sequence numbers reject older commands. Repeated commands and the
  simulator watchdog limit the effect of command loss. Change the watchdog using
  `--command-timeout-ms` on the simulator.
- After a GCS restart, validated telemetry advances its next command number past
  the simulator's last accepted number. Delayed telemetry never lowers the local
  counter, and the simulator continues rejecting older commands. This assumes
  one controlling GCS on a trusted LAN; it is not a controller-arbitration or
  authentication mechanism.

Battery drain includes a time-based component and a component proportional to
distance traveled. It is illustrative mock telemetry, clamped at zero. Speed and
position use simulation units, not calibrated physical measurements.

## Simulator logs

The simulator writes timestamped logs to the terminal and to `logs/drone.log`
by default. The directory is created automatically. Relative paths are resolved
from the working directory in which the simulator is started.

Normal `INFO` logs include startup addresses/settings, the first accepted command
and later velocity changes, watchdog timeout/recovery transitions, UDP send
warnings, shutdown and fatal errors with tracebacks. A timeout is logged once
when it starts, even if the requested velocity was already zero. Before the first
command the simulator reports waiting, rather than reporting a command timeout.
Repeated commands with unchanged velocities do not produce repeated INFO entries.

For detailed command diagnostics or a different output path:

```powershell
.\.venv\Scripts\python.exe -m drone_gcs.drone --log-level DEBUG --log-file logs/drone-debug.log
```

`DEBUG` additionally records every accepted command (including keepalives),
rejected old/conflicting commands and malformed-command diagnostics. JPEG bytes
and per-control-tick state are not written to the log. File output uses UTF-8;
timestamps use the simulator computer's local wall clock, while watchdog timing
continues to use its monotonic clock.

Illustrative output:

```text
2026-11-04 12:00:00,100 INFO    [1234] Simulator started: command_bind=0.0.0.0:5000 video_target=127.0.0.1:5001 telemetry_target=127.0.0.1:5002
2026-11-04 12:00:03,100 INFO    [1234] Velocity command received: seq=20 vx=1.000 vy=0.000
2026-11-04 12:00:05,600 WARNING [1234] Command timeout: last_seq=40; velocity set to zero
2026-11-04 12:00:07,100 INFO    [1234] Commands resumed: seq=41
```

The file rotates at 5,000,000 bytes, keeping three backups (`drone.log.1` through
`drone.log.3`) plus the current file. Use a separate log path for each concurrent
simulator process. If the log file cannot be opened, a warning is printed and
the simulator continues with console-only logging for that run. Logging uses
synchronous handlers; very verbose DEBUG output or a slow disk can add latency.

To watch the file in PowerShell:

```powershell
Get-Content .\logs\drone.log -Tail 30 -Wait
```

The simulator logs a received zero-velocity command; it cannot infer whether it
was caused by STOP, released keys or an inactive GCS window. STOP is a local GCS
latch. Logging does not add a timeout flag to the telemetry or GUI. Generated
log files are excluded from Git; the implementation and this example are tracked.

## Tests

Install the optional test dependency, then run from the project root:

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
.\.venv\Scripts\python.exe -m pytest -q
```

On Linux/macOS, replace the interpreter path with `.venv/bin/python`.
The tests can also run without pytest after the normal installation:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

Tests cover motion, battery drain, input validation, rendering, command ordering,
GCS restart recovery and timeouts, protocol serialization, and video chunk
reassembly with missing, duplicate, and out-of-order packets. GUI tests run with
Qt's offscreen platform and cover video-loss warnings, stale telemetry, and
recovery. Keyboard tests inspect UDP commands for combined keys, releases,
STOP/RESUME, and focus loss. Matching tests exercise both arrival orders and
reused frame IDs with different timestamps. A subprocess integration test starts
the real simulator, receives a decodable frame and matching telemetry, checks
movement, and verifies watchdog stopping after commands cease. These tests do
not replace a manual GUI or LAN test. An additional integration test drives Qt
arrow-key events through real UDP to the simulator, checking that opposite keys
cancel, position stays fixed while both are held, either release order restores
the remaining direction, and INFO logs match the velocity transitions. Timers
and polling are driven explicitly for this test. Logging tests cover level filtering,
console/file output, UTF-8, bounded rotation, inaccessible-file fallback,
handler cleanup, fatal tracebacks and watchdog transition events.

## Troubleshooting and current limitations

| Symptom | Check / current behavior |
| --- | --- |
| `No module named drone_gcs` | Run the installation command from the project root using the same virtual-environment interpreter used to launch the nodes. |
| Missing `cv2`, `numpy`, or `PySide6` | Install with `python -m pip install -e "."` using the virtual-environment interpreter. |
| No video | Confirm the simulator is running, the destination addresses and ports match, and the firewall allows the UDP traffic. |
| Port already in use | Close any previous node processes, or choose matching alternative ports on both nodes. |
| Keys do not move the scenery | Activate the GCS window, release STOP with RESUME if necessary, and check that command UDP 5000 reaches the simulator. Arrow keys are also supported. |
| Commands briefly fail after restarting the GCS | Recovery occurs after valid telemetry arrives. Check that telemetry UDP 5002 reaches the GCS; if telemetry is lost, recovery can be delayed. |
| Video does not recover after rebooting the simulator computer | Restart the GCS too. A host reboot resets the simulator's monotonic clock; the current protocol has no stream-session ID. Restarting only the simulator process on the same running computer does not reset that clock. |
| `Video stream lost` / `STALE` | No new decodable frame was displayed for one second. The retained image and values are old. Check the simulator and video UDP path. Receiving telemetry alone does not clear this video warning. |

## Vehicle detection and tracking proposal

The [architectural proposal](docs/car_tracking_proposal.md) covers regular car
spawning, detection from received JPEG frames, camera-motion compensation,
Kalman-based tracking with global assignment, persistent IDs, occlusion handling,
and performance/fault-injection evaluation. It compares the simulation baseline
with an extension for real imagery and includes technical interview discussion
points. This is a design deliverable; the runtime does not yet spawn or track cars.

## Demo video

[Watch the Drone & GCS demo](https://drive.google.com/file/d/1CCD06Kh_WDTwUi205GyzLqxWUsUECqqH/view?usp=sharing)

The screen recording is hosted on Google Drive.

## Source layout

```text
src/drone_gcs/
  model.py       Deterministic movement and battery state
  render.py      Generated top-down camera view
  log_config.py  Console and rotating-file logging configuration
  protocol.py    Command/telemetry encoding and video packet reassembly
  drone.py       Simulator process and UDP transmission loop
  gcs_client.py  GCS UDP networking and keyboard-to-velocity mapping
  gcs.py         PySide6 video display, controls, and telemetry dashboard
tests/           Automated behavioral tests
docs/            Design notes and vehicle-tracking architectural proposal
```

Before submission, ensure that the repository is publicly accessible, the
proposal is tracked in Git, and the demo video opens without requesting access.
