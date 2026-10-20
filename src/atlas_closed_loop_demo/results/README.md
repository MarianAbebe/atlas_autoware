# Evidence provenance

`results/safety_validation/` contains fresh 2026-10-20 executions. The final
headless drive and six final interruption cases are summarized in
`safety_validation/release_summary.json`; raw data, timestamps, graphs,
threshold copies, source hashes and complete logs are preserved per run.
`release_drive/` also preserves an explicitly failed earlier hold-accounting
attempt; the final summary identifies the exact successful recording.

`safety_validation/baseline`, `fixed`, `faults`, and `faults_final` retain fresh
development runs, including startup and shutdown failures. They are not
substituted for the final results. Operational shutdown warnings are reported
separately from measurement-phase safety checks, with bounded cleanup required.

The older top-level `drive`, `drop`, `freeze`, `gated_*`, `failed_*`, build logs
and `build_summary.json` describe previous sessions and temporary build caches.
They are preserved unchanged. Their historical success flags do not establish
the current stopping criteria. `current_audit/` contains the prior session's
build/environment audits. Its UDP-only preflight conclusion was corrected by
fresh two-process DDS delivery testing; see the current `../VALIDATION.md`.
The prior report is archived at `../VALIDATION_PRIOR.md`.

## Versioned evidence

Only this provenance note, the predeclared thresholds, final release summary,
pinned-source audit and evidence hash manifest are versioned. All raw recordings,
logs, generated map extracts, plots and build outputs remain local and ignored.
Paths to those files in validation reports identify historical evidence; they
are not files supplied by a fresh clone. `safety_validation/evidence_manifest.json`
records hashes of the final run files for later comparison with local evidence.
The summary is a historical measured result, not a claim that tests ran on clone.
