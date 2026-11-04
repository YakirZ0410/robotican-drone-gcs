# Control and synchronization test plan

> **For agentic workers:** Use superpowers:executing-plans for this test-only change.

**Goal:** Cover the three test gaps accepted in the conversation: keyboard/STOP/RESUME, exact frame/telemetry matching, and a real simulator/client exchange including watchdog stop.

**Architecture:** Extend the existing unittest suite. Exercise Qt events with offscreen widgets and inspect real UDP commands; exercise matching through real loopback sockets; start the existing simulator in a separate Python process for the system test. No production behavior changes or new dependencies are planned.

**Tech Stack:** unittest, pytest, PySide6, OpenCV, Python sockets/subprocess.

**Spec:** The user's accepted three-part testing proposal in this conversation.

## Constraints and decisions

- Keep the user's existing uncommitted work intact; work in the authorized checkout because the tests exercise those uncommitted changes.
- No automatic commits or publishing as part of this test addition.
- Use ephemeral loopback ports and bounded waits; clean up sockets, Qt windows and child processes even on failure.
- Test additions characterize existing behavior. A passing first run is expected; demonstrate sensitivity using temporary in-memory mutations, without changing production files.
- Simulate the active-window state for offscreen keyboard tests; keep Qt events, command encoding and UDP transmission real.

## Tasks

- [x] Add `tests/test_gcs_controls.py`: simultaneous keys and releases; STOP blocks movement and RESUME requires fresh input; focus loss clears held input and requests zero.
- [x] Extend `tests/test_gcs_client.py`: video-first, telemetry-first, future telemetry, and reused frame IDs with distinct timestamps.
- [x] Add `tests/test_system.py`: launch `python -m drone_gcs.drone`, receive a decodable JPEG and matching telemetry, observe commanded motion, stop sending commands, observe watchdog stop with unchanged command number and stable position.
- [x] Run the targeted tests. Check representative intentional behavioral mutations are caught, with mutations isolated to their own Python process.
- [x] Run the full suite and inspect the final diff.

## Review focus

- Keyboard release must remove only the released direction.
- RESUME must not restore keys held before or during STOP.
- Matching must use timestamp as well as frame ID and tolerate either arrival order.
- The system test must assert observed drone movement and watchdog results, not just process startup.
- Every child process and socket must be released on assertion failure; timing checks must use deadlines rather than fixed readiness sleeps.

## Progress

- Baseline: `.venv/Scripts/python.exe -m pytest -q` -> 58 passed, 19 subtests passed.
- Targeted run: 18 passed.
- Mutation checks caught unsafe RESUME, frame-ID-only matching, and a disabled watchdog.
- Fresh review found no critical or important issues. Strengthened the watchdog sequence assertion to compare against an independently counted final transmitted command, including commands still in flight.
- Review scope: only this test addition; existing production changes remain outside this task.
- The strengthened system test also caught a watchdog mutation that incorrectly increments the command sequence.
- Final verification: `.venv/Scripts/python.exe -m pytest -q` -> 66 passed, 19 subtests passed in 3.45s.
- `git -c core.autocrlf=false diff --check` passed. README test documentation updated.
