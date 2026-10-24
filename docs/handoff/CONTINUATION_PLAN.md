# Prioritized continuation plan

No task may weaken safety checks, introduce truth-state fallback, or modify pinned
Autoware merely to obtain a pass.

## P0 — Reproduce the stationary test on another machine

Work:

- Transfer a reviewed repository revision, pinned Autoware revisions, map assets
  and a documented dependency-build procedure.
- Launch one graph, initialize with `initialize_localization.py`, remain disengaged,
  and collect synchronized rosbag plus scheduler telemetry.

Acceptance criteria:

- Map hashes and four Autoware revisions match the handoff record.
- All expected components and exclusive topic publishers are present exactly once.
- Compact LiDAR and NDT are 9–11 Hz; EKF is 45–55 Hz for at least 120 seconds.
- Both monitors arm, never latch or become unarmed, and report no stale inputs
  after arming.
- Zero simulator speed and displacement; no engagement message is published.
- No scan skips; preserve the complete bag, telemetry and launch log.

## P0 — Validate controlled clear-route engaged operation

Work only after an operator explicitly authorizes simulation engagement.

Acceptance criteria:

- Three consecutive identical clear-route runs pass with all failures retained.
- MPC consumes EKF odometry exclusively; no truth fallback exists.
- Goal error is at most 0.15 m and the vehicle holds stopped for at least 10 s.
- No watchdog latch, stale input, unbounded control or runtime error occurs.
- Record tracking error, NDT/EKF error against hidden truth, message freshness,
  scheduler telemetry and pass count.

## P0 — Validate obstacle stopping

Acceptance criteria:

- Three consecutive identical obstacle runs produce cluster-derived predicted
  objects and a stop inserted by the pinned planner.
- Vehicle stops before the synthetic obstacle with positive configured geometric
  clearance and holds for at least 10 s.
- A freshness latch, evaluator abort or route-end stop cannot count as success.
- Preserve object lineage, stop-plan timing, clearance and every failed run.

## P0 — Validate localization-loss stopping

Acceptance criteria:

- After at least 1 m of engaged motion, deactivate actual pinned NDT without
  stopping LiDAR or substituting truth.
- Base monitor latches on NDT freshness while EKF predictions may still publish.
- Gate commands a bounded stop; report reaction time and stopping distance.
- Repeat twice with no indefinite driving and no automatic rearm.

## P1 — Investigate CPU scheduler contention

Acceptance criteria:

- Capture at least one healthy and one fault/near-fault synchronized run.
- Attribute gaps using per-process runtime/wait deltas, PSI, runnable tasks,
  paging, steal time, watchdog timer gaps and ROS source/receipt timing.
- Include the rosbag recorder process or document its exact unmatched command.
- Establish or reject host pressure, guest run-queue contention, DDS delivery and
  executor delay separately before changing topology.

## P1 — Profile Atlas Python callbacks

Focus on scene generation, both safety monitors and static-object adaptation.

Acceptance criteria:

- Measure callback wall time, CPU time, allocation/RSS and frequency without
  high-rate logging.
- Quantify scene trajectory construction, point-cloud work, JSON status work and
  full-message deserialization separately.
- Propose changes only where measurements show material load.

## P1 — Reduce unnecessary point-cloud deserialization safely

Acceptance criteria:

- Preserve original acquisition timestamps, monitored stages, source/receipt
  freshness, QoS, 300 ms threshold, arming and latching semantics.
- Add deterministic equivalence tests for fresh, missing, stale, frozen and
  malformed input behavior.
- Demonstrate lower guard CPU/runtime cost and no missed delivered safety event.
- Rerun stationary and all intentional safety-fault cases.

## P2 — Physical sensor and vehicle preparation

Prerequisites and acceptance criteria:

- Establish measured LiDAR/IMU/vehicle extrinsics and calibrated time domains.
- Register the actual campus PCD to the routing/map frame defensibly.
- Validate real PointCloud2 conversion, deskew inputs, transforms and map
  initialization without fabricated values.
- Define hardware command, feedback, emergency-stop and operator procedures.
- Complete simulation, replay, bench and closed-course safety reviews before any
  autonomous hardware motion.
