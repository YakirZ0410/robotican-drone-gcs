# Simulated Drone & Ground Control Station

Robotican take-home assignment. This project is being implemented incrementally.

## Current status

- **Implemented:** deterministic drone movement, velocity validation and battery simulation (`model.py`).
- **Not implemented yet:** 2D rendering, UDP video transport, simulator runtime, GCS interface, telemetry transport and car-tracking proposal.

The currently planned structure and interfaces are documented in
`docs/superpowers/specs/` and `docs/superpowers/plans/`.

## Run the existing tests

From the project root:

```bash
PYTHONPATH=src python -m unittest discover -s tests -v
```

On Windows PowerShell:

```powershell
$env:PYTHONPATH = "src"
python -m unittest discover -s tests -v
```

Installation, dependency setup, simulator/GCS commands and recording instructions
will be completed as the corresponding features are implemented.
