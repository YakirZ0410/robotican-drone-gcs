# SDD ledger — plan: docs/superpowers/plans/2026-11-01-robotican-drone-gcs.md

Pre-flight: Task 1 state/command interfaces feed Task 2 renderer, Task 3 command encoding, Task 4 simulator and Task 5 dashboard; signatures are consistent in the plan. Tasks 3–5 share a `frame_id` and `timestamp_ns` for video/telemetry association.

Task 1: Ruling: The current workspace is not a Git repository. Work remains isolated here, and Git commits/publication are deferred until the user explicitly chooses the repository — cost: there is no Git history yet.

Task 1: Ruling: `pytest` is not installed in the selected runtime. Write test cases using standard-library `unittest` so they run immediately, keep `pytest` in the optional developer dependencies for the final documented environment — cost: no pytest-run evidence yet, though the same unittest test cases are discoverable by pytest.

Task 1: complete — wrote 8 behavioral tests before implementation, observed missing-module failure, implemented `model.py`, ran `PYTHONPATH=src python3 -m unittest discover -s tests -v` and observed 8 tests passed, 0 failures.

Task 1: Ruling (user-approved refinement): Adopt the ROS robot-frame convention (+X forward, +Y left) in the world model, with screen coordinate conversion deferred to the renderer. Previously, position axes were swapped to mimic screen coordinates — cost: any future renderer must respect the documented conversion.

Task 1: coordinate correction — changed the tests first and observed 4 failing cases for swapped axes, changed `advance_state` and documentation, and reran the full suite with 10 passing tests.

Next: Task 2 renderer, after the candidate reviews Task 1 code.
