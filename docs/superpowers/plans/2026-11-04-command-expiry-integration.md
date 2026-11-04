# Integrate motion only while the previous command is valid

Approved scope: the user selected the explained policy and requested its
implementation in the current checkout. No commit or push is requested.

For each elapsed interval, retain the command receipt time from before polling
new commands. Integrate the previously applied velocity only until that command's
deadline; integrate the remaining time with zero velocity. Idle battery drain
uses the entire elapsed interval, motion drain only the moving portion. Apply
newly received commands only for future intervals. No stall threshold, new UI,
protocol, queued-packet age guarantee or separate watchdog thread is introduced.

## Tasks

- [x] Add deterministic regression tests for a two-second stall with no new
  command, with a new opposite command, and with an identical keepalive. Verify
  remaining validity (not an unconditional 500 ms), zero/expired command cases,
  normal updates, exact expiry and battery drain.
- [x] Implement a small pure helper in `drone.py` and call it from the existing
  loop with the previous receipt timestamp; keep new commands prospective.
- [x] Document the policy and its same-loop timing limitation in README.
- [x] Run the full suite, diff checks, and an independent focused review.

## Review focus

Fresh commands cannot extend old motion retroactively; command sequence is
preserved during expiry; sub-second remainder is exact; stationary battery drain
covers the full gap; ordinary control and existing watchdog/logging remain valid.

## Execution record

Continue in the user's existing workspace, preserving their established workflow.
Write and observe failing regressions before production edits.

The loop regressions failed with x=2.0 instead of x=0.5 before the fix for all
three post-stall command cases. Helper tests also failed before implementation.
Focused validation after implementation: 4 passed, 9 subtests passed.

Final validation: full suite 85 passed, 36 subtests passed (12.05 seconds).
Diff check passed. Independent focused review found no critical or important
issues. Same-loop execution, packet age before processing and hard real-time
guarantees remain the stated scope limits; no further runtime changes needed.
All changes remain local; no commit or push performed.
