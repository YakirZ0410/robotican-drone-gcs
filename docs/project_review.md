# Project review

## Final pre-publication review - 2026-11-04

Reviewed the current source, modified/untracked tests and documentation against
`SW_Assigment_2026.pdf`. An independent read-only code review covered command
ordering/restart, watchdog scheduling, frame/telemetry matching, Qt/network
lifecycle, logging and boundary inputs.

### Issue found and fixed

A JSON datagram below the 4096-byte limit could contain enough nested arrays to
raise `RecursionError`. The simulator would terminate, or the GCS network thread
would stop receiving. The shared decoder now converts that parser error into
`ValueError`, which both receivers already handle by ignoring the packet.

Two new real-UDP regression tests reproduced the failures before the fix. Both
pass after it and verify that subsequent valid commands/telemetry are processed.
The follow-up independent review found no remaining critical or important issues.

### Fresh verification

- Source checkout: project virtual-environment `python -m pytest -q` ->
  **81 passed, 27 subtests passed** (19.24 seconds).
- Built the current wheel using the declared isolated build system, then
  installed it into `.venv/submission-review/installed`.
- Verified imports use that installed package, including the new logging module,
  and reran the full suite with the installed package on the subprocess import
  path -> **81 passed, 27 subtests passed** (12.87 seconds). Runtime and test
  dependencies were reused from the existing environment; they were not freshly
  installed into a separate clean environment.
- `python -m pip check` -> no broken requirements.
- Both CLI help commands succeeded; local README/proposal links resolve.
- `git -c core.autocrlf=false diff --check` passed. Runtime logs, rotated backups
  and temporary packaging artifacts are ignored by Git.

### Submission status and limits

- The user reports that the normal desktop manual check passed.
- The demo link is in README; the user reports verifying anonymous access.
- The architecture proposal is present and explicitly identifies detection and
  tracking as proposed rather than implemented.
- Two-computer LAN operation remains untested because a second computer is not
  available. Automated integration evidence uses real UDP over localhost.
- No commit or push was performed during this review. The modified and untracked
  deliverables still need to be committed/published, and public GitHub access
  verified before submission.

---

## Historical review - 2026-11-03

The following records the earlier review; its test counts and submission status
are historical. The final review above describes the current state.

Reviewed the current working tree against `SW_Assigment_2026.pdf`, the README,
runtime code, automated tests and the vehicle-tracking architecture proposal.

## Correctness fixes

Three runtime defects were reproduced with failing regression tests and fixed:

1. **Low video FPS delayed control and watchdog checks.** The simulator now
   processes commands and updates state between video frames, with idle sleeps
   capped at 10 ms. Position integrates the previously active velocity before a
   newly received command changes it. Rendering/sending remain synchronous;
   timing is best effort rather than a hard real-time guarantee.
2. **Oversized UDP commands crashed the simulator on Windows.** The command
   socket now receives the whole datagram; the existing JSON size validator
   rejects messages exceeding 4096 bytes. A following valid command still works.
3. **Reused frame IDs could produce a mismatched GUI snapshot.** If a newer
   image arrived while an older image was being decoded, the GUI could retrieve
   telemetry for the newer timestamp. The display now checks the returned full
   `(frame_id, timestamp_ns)` against its captured image before showing values.

Each reproducing test failed before its production fix and passed afterward.
A separate code review identified these issues and found no critical defect.

## Verification evidence

- Source checkout: `python -m pytest -q` -> **69 passed, 19 subtests passed**.
- Built `robotican_drone_gcs-0.1.0-py3-none-any.whl` using the declared isolated
  build system and installed it into a separate test target directory.
- Ran the suite against the wheel-installed package, also setting the subprocess
  import path to that package -> **69 passed, 19 subtests passed**. This reused
  the environment's installed runtime/test dependencies, rather than reinstalling
  them in a new environment.
- `python -m pip check` -> no broken requirements.
- Both node entry points returned command-line help successfully.
- Live offscreen Qt smoke check kept the network thread and both timers active:
  movement, STOP/RESUME, video loss, simulator process restart, matching telemetry,
  preservation of STOP, and shutdown all passed over loopback UDP.
- `git diff --check` passed.

The offscreen platform did not discover system fonts automatically. A system
font was loaded explicitly for a visual inspection only; production font
configuration was not changed. Desktop appearance and physical focus/keyboard
behavior still need a normal GUI check.

## Assignment and submission status

The runtime implements two nodes, simultaneous normalized velocity commands,
dynamic generated visuals, JPEG/UDP transport with bounded chunk reassembly,
matching mock telemetry and a GCS dashboard. The README covers dependencies,
installation, both-node startup, LAN addresses/ports, controls and limitations.

Vehicle detection/tracking remains a **design proposal**, as declared in
`car_tracking_proposal.md`. No detector, tracker, car simulation or benchmark
result is claimed as implemented. Region boundaries and detector tolerances
are still proposed configuration choices to settle during that implementation.

Before submission:

- Perform a normal visible GUI check and a two-computer LAN test. Current
  automated communication evidence uses localhost on Windows.
- Add the required 1-2 minute screen recording or an accessible link. None is
  currently present in this project/README.
- Commit and publish the current changes, including currently untracked docs and
  test files. This review did not commit or push anything.
- Verify the repository is publicly accessible. The connected GitHub metadata
  request returned 404 and the anonymous web fetch could not load it. This does
  not establish whether the repository is private, unavailable or inaccessible
  through the current connection.

## Documented limitation

Restarting the simulator process preserves the host monotonic clock and was
verified. Rebooting the simulator computer resets that clock and requires
restarting the GCS too. The README now documents this; stream-session IDs remain
a future extension in the tracking proposal.
