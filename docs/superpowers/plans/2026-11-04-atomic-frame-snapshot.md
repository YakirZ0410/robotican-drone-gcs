# Atomic frame and telemetry snapshot

Approved design: return the latest complete frame and its currently available
matching telemetry in one short critical section. Decode and Qt work happen
after releasing the lock. Missing telemetry remains explicit and may be filled
by a subsequent snapshot; no waiting, new buffering or network protocol changes.

Interface: `GCSClient.latest_snapshot() -> tuple[int, int, bytes, dict | None] | None`.
The tuple contains frame ID, timestamp, immutable JPEG bytes and a copy of the
flat telemetry dictionary. Existing `latest_frame`/`telemetry_for` remain usable.

Files: `gcs_client.py` implements the snapshot; `gcs.py` consumes it;
`tests/test_gcs_client.py` checks arrival order, full-key matching, snapshot
isolation and caller mutation; `tests/test_gcs.py` checks concurrent arrival
during decode and delayed telemetry. README and tracking proposal describe the
current API accurately.

- [x] Write failing client and GUI regressions; verify expected failures.
- [x] Implement snapshot and use captured telemetry throughout the GUI update.
- [x] Update documentation; run full tests and focused independent review.

Review focus: no cached metadata mutation via returned dictionaries, no lock
held during decode/display, missing pose stays missing, late pose fills an
existing displayed frame, reused frame IDs still require matching timestamps.

Execution: user requested implementation in the existing checkout. No commit
or push requested; retain current workflow and unrelated files.

Before implementation, three snapshot tests failed because the API was absent;
the GUI race regression failed because X=12.00 was replaced by a waiting message.
The existing delayed-telemetry behavior remained covered by a passing test.
After implementation, all 30 client/GUI tests pass.

Final validation: 89 passed, 36 subtests passed (14.10 seconds). Diff check is
clean. Independent focused review found no actionable issues. Snapshot copying
is sufficient for flat scalar metadata; old-frame lookups and legacy API
mutability remain unchanged. No commit or push performed.
