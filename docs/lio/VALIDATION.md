# Validation evidence

## Current status

The synthetic FAST-LIO2 runtime and pose-only adapter have prior host-side
validation evidence. No live ROS graph was started during the current work
session. The evidence was produced from the verified integration checkout and
must not be treated as a runtime test of today's uncommitted active-checkout
changes.

### Pose-only forwarding evidence

Original run: `/tmp/atlas_host_fastlio_pose_shadow_20261025_GNtA5j`.
The original analyzer returned FAIL (14/15) because it compared only messages
inside the receipt-time overlap and rejected one edge sample; the complete raw
recording contained the matching source pose. The corrected analyzer was run
against that preserved directory and wrote a separate report to
`/tmp/atlas_pose_shadow_analysis_recheck_20261026_01`.

Corrected result: 20/20 PASS. It found 900 raw callbacks, 304 unique raw
timestamps, 304 forwarded poses, no missing unique timestamps, and no pose
mismatches. All 304 forwarded pose values exactly matched the raw estimator
pose at the same ROS header timestamp. The analyzer uses monotonic receipts for
rate and capture-window measurements only; they do not determine forwarding
correctness. This verifies the recorded shadow message contract, not hardware
accuracy.

The corrected evaluator implementation was also replayed offline against the
preserved raw and ground-truth streams from that run. It matched 303 estimator
samples, rejected one unique sample outside truth coverage, measured 0.01575 m
aligned XY RMSE, 0.10703 m Z RMSE, 0.10818 m 3D RMSE, 0.00730 rad yaw RMSE,
0.00784 m relative translation RMSE, 0.00505 rad relative rotation RMSE, and
9.90 Hz unique output. The trajectory traveled 10.67 m. These are offline
recomputed synthetic-run results, not a new estimator execution. Initial SE(3)
alignment removes the initial pose/frame offset; these values do not establish
absolute map localization.

The earlier standalone FAST-LIO2 run is preserved at
`/tmp/atlas_host_fastlio_validation_20261025_KUEKL8`. Its original report
recorded 273 matched samples and 9.29 mm XY RMSE. Its reported
`relative_translation_error_rmse_m` duplicated 3D absolute error and should not
be used; the implementation now computes consecutive relative-pose error.

## Reproduction and local checks

On the host, build the focused packages and follow the standalone procedure in
[README.md](README.md). For preserved evidence, rerun analysis without
overwriting the input directory:

```bash
PYTHONPATH=src/atlas_lio_integration python3 \
  src/atlas_lio_integration/scripts/analyze_pose_shadow \
  /path/to/pose-shadow-evidence --output-dir /tmp/atlas_pose_analysis_new
```

The current source test command is:

```bash
source src/atlas_full_stack_demo/scripts/environment.sh
python3 -m pytest -q src/atlas_lio_integration/test
```

The evaluator keeps the existing XY RMSE <= 0.50 m, yaw RMSE <= 0.15 rad,
minimum 100 matched samples, positive output rate, and 90% unique-estimator
match-rate limits. Relative translation/rotation metrics are informational;
they have no newly invented acceptance threshold. Duplicate callback receipts
are reported independently and are not treated as missing unique states.

| Acceptance | Status | Evidence |
|---|---|---|
| FAST-LIO2 source pinned | PASS | commit `2659eb81addb01ee0d6b8998f6e73154b684b8ab` |
| Synthetic scan acquisition offsets and 200 Hz IMU | PASS (unit-tested) | `atlas_sensor_sim` tests; no Mid-360 firing model |
| Livox CustomMsg conversion | PASS (unit-tested) | `atlas_lio_integration` adapter tests |
| FAST-LIO2 publishes odometry on synthetic inputs | PASS (prior host run) | preserved FAST-LIO2 evidence above |
| Pose-only adapter exact forwarding | PASS (prior host run, corrected offline analysis) | 304 unique stamps, zero mismatches |
| Corrected evaluator runs on live ROS data | NOT YET RUN | only unit tests and offline replay in current session |
| Pose-only output connected to EKF/NDT | BLOCKED BY DESIGN | no calibrated pose covariance or map-to-odom ownership contract |
| Physical Mid-360S/LIO validation | NOT RUN | hardware unavailable |

FAST-LIO2's pose-only output remains isolated. Do not feed it to EKF, NDT,
planning, or control until a justified uncertainty model and TF/frame ownership
design are reviewed. Its full Odometry twist/covariance are not accepted as a
calibrated Atlas measurement contract. No full-stack runtime campaign was run
in this session.
