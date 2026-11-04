# Drone logging implementation plan

**Goal:** Implement the simulator logging design approved in the conversation.

**Scope:** Standard-library logging to console and `logs/drone.log` (UTF-8),
5,000,000 bytes per file, three rotating backups. Default INFO; optional DEBUG.
Log startup/configuration, received velocity changes, watchdog transitions,
recovery, throttled UDP send warnings, shutdown and fatal exceptions. Do not
claim a received zero-velocity command proves the GCS STOP button was pressed.

**Files:** new `src/drone_gcs/log_config.py`, new `tests/test_logging.py`,
existing `drone.py`, `tests/test_system.py`, README and `.gitignore`.

**Execution:** Continue directly in the existing authorized checkout, preserving
the user's uncommitted work. No commit or push requested. No new dependencies.

- [x] Write failing configuration tests: file/console output and level filtering,
      DEBUG detail, bounded rotation, file-open fallback, duplicate-free setup.
- [x] Implement isolated logger configuration and explicit handler cleanup.
- [x] Add failing event/CLI tests and extend real simulator integration assertions.
- [x] Instrument command receipt, watchdog transitions and runtime lifecycle.
      Add `--log-file` and `--log-level`; avoid INFO spam from keepalives/ticks.
- [x] Document usage, limitations and example output; ignore generated log files.
- [x] Run full tests, inspect changes, and perform a focused logging review.

## Review focus

Console-only fallback on inaccessible paths; handlers closed for Windows file
rotation and repeated in-process startup; timeout logged once per transition,
including zero-velocity timeouts; initial no-command state is waiting; normal
keepalives do not flood INFO; an actual fatal exception includes a traceback;
tests use isolated log paths and do not overwrite a running user's logs.

## Validation

- Full suite: `python -m pytest -q` using the project virtual environment:
  78 passed, 19 subtests passed (9.88 seconds).
- `git -c core.autocrlf=false diff --check` passed; CLI help exposes both
  logging options; generated default and custom rotated log paths are ignored.
- Focused independent review found no critical or important issues.
- Windows integration cleanup launches the base interpreter with
  `__PYVENV_LAUNCHER__`, following CPython's multiprocessing implementation,
  so the test tracks the actual simulator process and closes its log handles.
- No commit or push performed. Existing unrelated changes preserved.
