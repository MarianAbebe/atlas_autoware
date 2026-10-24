# Atlas documentation

The current operational entry point is the
[Saturday team handoff package](handoff/README.md). It contains the stationary
quick start, architecture, validation evidence, troubleshooting, continuation
plan and repository-readiness audit for the integrated simulation.

This directory also contains:

- Architecture and package responsibilities.
- TF conventions, robot/sensor frame definitions, and verified extrinsics.
- Sensor interfaces, message contracts, and topic remapping.
- Development and robot setup instructions.
- Atlas and upstream Autoware integration notes.

Robot-specific hardware frames, mounting, calibration and interfaces remain
undefined. Simulation-only frames and assumptions are documented in the handoff;
they must not be treated as physical calibration.
