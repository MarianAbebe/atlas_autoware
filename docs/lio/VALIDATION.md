# Validation evidence

## Current result: BLOCKED for genuine LIO runtime

The selected source was shallow-cloned at `2659eb81addb01ee0d6b8998f6e73154b684b8ab`. Its `livox_interfaces` package built successfully with one job. FAST-LIO wrapper CMake configuration then stopped because `nlohmann_jsonConfig.cmake` (the `nlohmann-json3-dev` package) is absent. A narrowly scoped install was attempted but this VM requires an unavailable sudo password. PCL was found correctly by CMake; its missing pkg-config entry is not a blocker. No substitute estimator was added, no estimated odometry was fabricated, and therefore there are no trajectory-error numbers to report.

Passed locally:

```bash
PYTHONPATH=src/atlas_sensor_sim:src/atlas_lio_integration python3 -m pytest -q src/atlas_sensor_sim/test src/atlas_lio_integration/test
python3 -m py_compile src/atlas_sensor_sim/atlas_sensor_sim/*.py src/atlas_lio_integration/atlas_lio_integration/*.py
```

These check deterministic static-scene cloud geometry, ordered finite measurements, calibration helper behavior and independent error metric math. They do **not** validate FAST-LIO.

The live adapter check also passed: `/tmp/atlas_lio_adapter.yaml` recorded `frame_id: lidar_link`, 597 points, and increasing Livox `offset_time` values (`0`, `167785`, `335570` ns …). This proves input-format conversion only, not estimator odometry.

After installing the focused dependency chain, run the launch/evaluator commands in [README.md](README.md). A valid acceptance run must record nonzero motion; finite translation/orientation RMSE, drift, output rate and initialization time; and an independently launched LiDAR dropout health ERROR. Save evaluator JSON and health output under `/tmp/atlas_lio_*`.

| Acceptance | Status | Evidence |
|---|---|---|
| External estimator source pinned | PASS | clone revision above |
| Simulator provides scan offsets / IMU rate | PASS | source + unit tests |
| PointCloud2 to FAST-LIO Livox CustomMsg adapter | PASS | `/tmp/atlas_lio_adapter.yaml` |
| External estimator builds/starts | BLOCKED | `nlohmann-json3-dev` requires unavailable sudo authorization |
| LiDAR/IMU accepted, odometry, metrics | BLOCKED | estimator cannot run |
| Ground-truth isolation | PASS (design/static) | only evaluator subscribes to evaluation topic |
| Dropout degradation diagnostics | BLOCKED runtime | `atlas_lio_health` implementation ready |
| Existing Nav2 unaffected | NOT RUN | no Nav2 files changed |
