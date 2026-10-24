# Timeboxed watchdog and command-observation follow-up

Investigation started 2026-10-22 03:05:24 UTC, with an approximately 20-minute
limit. Safety predicates, thresholds, shutdown cutoff, provenance assertions,
pinned Autoware and the original recording remain unchanged. No system packages
were installed. No localization work was undertaken.

## Fresh nominal latch: established boundary, unresolved initiating cause

The fresh nominal run latched at ROS time **1792638547.848974**. Its frozen
trajectory stamp was 1792638547.5117621: source age **337.138 ms**, receipt age
**330.444 ms**. The watchdog tick gap was **281.134 ms**. Other required inputs
were just below the 300 ms bound. The warning log appeared later, at
1792638547.914363686; use the frozen decision snapshot, not the warning's output
time, when correlating events.

The depth-10 validator's trajectory history jumps from source stamp
1792638547.5117621 to raw trajectory 1792638547.8692539, followed by smoothed
trajectory 1792638547.8829293. Those recovered source stamps follow the latch
decision. Multiple simulator/controller streams show a concurrent observer gap.
This establishes a substantial execution/publication interruption and an actual
configured freshness violation. It does **not** establish the host/VM resource
actor, the exact producer thread's scheduling state, or a defect warranting a
safety-policy change. Earlier independent failures had much smaller watchdog
tick gaps; this event does not prove one cause for every historical failure.

The passive observer now retains Humble RMW metadata, which the installed
rclpy executor normally discards. Its CSV separates:

- application header stamp to middleware source timestamp: upstream work before
  publication, including intentionally retained/forwarded header stamps;
- middleware source to middleware received timestamp: observed delivery delay;
- middleware received timestamp to callback wall time: observed reader/executor
  and callback-dispatch delay, including any process scheduling delay.

For the recovered trajectory stamped 1792638547.8829293, observed middleware
publication-to-receive delay was **0.221 ms**, and receive-to-callback delay was
**31.654 ms**. The preceding observed trajectory had **0.105 ms** and
**3.713 ms** respectively. These measurements describe this observer's reader;
they cannot be substituted for the watchdog's reader timings. KEEP_LAST depth 1
can overwrite intermediate samples, so absence from this CSV alone does not
prove absence of publication. Zero/unavailable metadata is stored as blank.
The measurements use system-clock timestamps and are not a remote-clock
synchronization claim. The probe's executor override targets the installed
Humble private API and needs revalidation when rclpy changes.

## Archived odometry-drop command: missed observation, not absent delivery

The unmatched delivered command has stamp **1792636724.064528**, receipt
**28.646158935 s**, and acceleration **0.08652011305093765**. The graph has one
raw publisher (the pinned controller), one delivery publisher (Atlas relay), and
the relay forwards command messages unchanged during an odometry-only drop.
The delivered message therefore establishes forwarding/delivery of that command;
the validator's independent raw observer lacks the corresponding stamp.

Shutdown is ruled out for this archived occurrence: raw and delivered recording
continue until **34.308407143 s** and **34.308993729 s** respectively, more than
5.6 seconds later. The validator's `capturing=False` cutoff does discard cleanup
callbacks, but that mechanism cannot explain this earlier omission. The raw
observer records stamps .034619 then .0943687 around the missing .064528 stamp;
this is not a final callback still waiting to be drained.

A median status-clock alignment estimates raw header ages near **283–284 ms**
while a depth-10 reader services a roughly 33 Hz command stream. It then drains
roughly 30 ms source increments in 1–4 ms callback intervals. These are strong
backlog/KEEP_LAST-eviction clues, not direct historical reader-loss metadata.
The exact archived loss mechanism remains unproven; transport loss and reader
queue eviction cannot be conclusively separated from that old capture.

`test_command_observation.py` is a controlled integration experiment using
actual ROS readers and the actual Atlas `Relay.forward` method. It deliberately
withholds raw-observer takes while servicing relay and delivery reads:
**30 publications → 30 relay deliveries → only the last 10 raw observations**.
No shutdown is involved. This deterministically demonstrates that the existing
provenance check can fail from bounded observer history even when commands were
successfully forwarded. It is diagnostic coverage of the mechanism, not a
claim to have reproduced the historical host interruption. No queue sizes or
assertions were changed merely to obtain a pass.

## Reproduction

From the Atlas root, after sourcing ROS and `.closed_loop_cache/install/setup.bash`:

```bash
# Isolated controlled observer-backlog experiment; no simulator.
ROS_DOMAIN_ID=81 PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 \
  scripts/test_command_observation.py
# Metadata-adapter tests without a ROS graph.
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 scripts/test_watchdog_topic_probe.py
# Actual watchdog decision boundary/freeze tests.
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 \
  src/atlas_obstacle_stopping_demo/scripts/test_watchdog_health.py
```

The complete serial suite ran with `run_suite.py --results-root
/tmp/atlas_watchdog_timebox_suite`. A passive probe in the same process/sandbox
and ROS domain 80 wrote `/tmp/atlas_watchdog_timebox_hops.csv` with duration
1050 seconds. No concurrent perception/build/load experiment ran during the
suite. Small diagnostic tests ran separately; the controlled reader experiment
used domain 81 and completed in 39 ms, after the nominal latch.

## Results and remaining work

The bounded suite and preservation results are recorded below.
The complete suite passed **10/11**:

| Scenario | Result |
|---|---|
| Nominal driving | FAIL: unexpected trajectory-freshness latch; route completion and terminal hold consequently failed |
| Ahead obstacle | PASS |
| Off-route obstacle | PASS |
| Stale perception drop / freeze | PASS / PASS |
| Odometry drop / freeze | PASS / PASS |
| Steering drop / freeze | PASS / PASS |
| Command drop / freeze | PASS / PASS |

All eight intentional fault scenarios retained their expected latch, braking,
rearm-refusal and no-automatic-restart checks. Fresh odometry-drop recorded
**971 raw and 971 delivered commands** and passed the unchanged provenance
assertion. No complete-suite offline PASS is claimed; the suite exited 1.

Preservation checks after the suite:

- Synthetic perception CTest: **PASS**, 0.41 s, including the existing three-scan
  converter/deskew/clustering test.
- Five recorded perception scans: **PASS**; filtered cluster counts
  74/47/77/103/98, bypass counts 67/44/70/93/86. This remains execution/contract
  validation, without physical ground-segmentation or localization accuracy claims.
- Actual watchdog decision tests: **4 PASS**.
- Middleware metadata adapter tests: **2 PASS**.
- Controlled reader-backlog integration experiment: **PASS**, 30 delivered and
  10 raw observations, 0.032 s. This is a separate controlled experiment, not a
  passing driving scenario or evidence of fault removal.

The investigation ended at approximately **03:20 UTC**, within the requested
20-minute limit. The suite/probe exited and no further replay was started.
The completion criterion remains unmet; no runtime fix was applied. Retained
changes are the observer metadata adapter, diagnostic tests, and this evidence.

Raw suite logs remain under `/tmp/atlas_watchdog_timebox_suite`; selected original
reports, source hashes, frozen first-latch records and hop windows are preserved
here. Historical evidence remains in the parent report.

No runtime fix is justified by the evidence so far. Next, instrument the actual
watchdog/validator readers' middleware receive and callback times in an opt-in
run, together with producer callback timing and host scheduling evidence. The
independent observer cannot establish their queue contents. A bounded recorder
history adjustment should be evaluated only against demonstrated recorder loss,
without changing watchdog histories, thresholds or physical measurement cutoff.

The probe was stopped with SIGINT after the suite; its original log ended in
Humble `ExternalShutdownException`. CSV windows were flushed. The retained probe
now handles that normal signal-shutdown exception and uses `try_shutdown`; this
diagnostic-only cleanup does not change validator shutdown or measurement logic.
