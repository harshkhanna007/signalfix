# SignalFix Module 1 — Fault State Tracking Edge Cases

## Overview
This document details the critical edge cases handled by the newly implemented Fault State Tracker in Stage S5 (Rate of Change) and output gating in Stage S7. The implementation categorizes the stream into five mutually exclusive state cases to ensure unambiguous operator feedback and strict determinism.

## Edge Case 1: Multiple Faults in Sequence
**Scenario:** A fast succession of different faults where a secondary fault occurs before the first has fully cleared its graceful recovery period.

**Trace:**
- `Sample 10`: SPIKE detected, `ROC_EXCEEDED`. First fault onset. `is_active_fault=true`, `onset=10`, `conf=1.00`.
- `Sample 15`: `NOMINAL`. Confidence decaying. Status is recovering. `is_recovering=true`, `conf=0.80`.
- `Sample 20`: `ROC_EXCEEDED` again (a different anomaly). `is_active_fault=true` (forces overwrite of pending recovery state), `onset=20`, `conf=0.85`.

**Result:**
The system correctly re-latches into an active fault state. Sequence ID for the onset is updated to 20, and all previous recovery state memory is cleanly wiped, ensuring the telemetry reflects the newest active anomaly transparently. This corresponds to transitions from Case 3 back to Case 1.

## Edge Case 2: Noise-Induced Re-detection During Decay
**Scenario:** Confidence reaches `0.0` but a minor micro-spike occurs immediately after, introducing noise into the decay window without crossing the `ROC_EXCEEDED` line.

**Trace:**
- `Sample 10`: SPIKE detected, `onset=10`.
- `Sample 15`: `NOMINAL`, recovering, `conf=0.10`.
- `Sample 16`: A micro-spike occurs; algorithm logic pumps confidence slightly (e.g., `0.15`).
- `Sample 17`: `NOMINAL`, confidence hits `0.00`.
- `Sample 18`: `NOMINAL`, confidence holds at `0.00`.

**Result:**
The implementation incorporates a multi-sample recovery grace period (requiring 2 consecutive samples at `0.0`). The noise at Sample 16 resets the potential path to zero. Recovery is initially staged at sample 17 (grace period = 1). The noise is successfully masked from erroneously terminating the state early. Recovery is strictly confirmed only on Sample 18 (`recovery_confirmed=true`).

## Edge Case 3: Confidence Anomaly (Non-Monotonic Decay)
**Scenario:** In an unforeseen or non-optimal situation, intermediate components may artificially pump the tracking confidence variable higher than what was logged at the original onset.

**Result:**
Although mathematically guarded against physically in the main algorithm paths, any anomalies causing `current_confidence > confidence_at_onset` post-initialization are strictly bounded. The initial bounds (onset) dictate the contextual frame. While confidence updates to whatever downstream computes, it stays gated behind either the `is_recovering` block or physical `ROC_EXCEEDED` gating in case the values invert. S7 Output gating leverages the recovery boolean to safely enforce floors (`CONFIDENCE_FLOOR_SPIKE`) specifically blocking noise floors.

## Edge Case 4: Sequence ID Wraparound
**Scenario:** The continuous pipeline runs effectively forever without rebooting. `sequence_id` hits `UINT64_MAX` and rolls over to `0`.

**Trace:**
- Fault onset logged at `seq_id = UINT64_MAX - 100`.
- Current arrive sample `seq_id = 50`.
- `delta_seq = 50 - (UINT64_MAX - 100)`. Naive arithmetic would underflow/blow up.

**Result:**
The tracker logic features a safe sequence delta calculation:
`const uint64_t delta_seq = (envelope.sequence_id >= fault_onset) ? (envelope.sequence_id - fault_onset) : UINT32_MAX;`
Safe saturation ensures that upon wrapping or disjoint sequences, the `samples_since_fault_onset` truncates smoothly to `UINT32_MAX` rather than creating dangerous integer underflows.

## Edge Case 5: Very Long Fault (> 4 Billion Samples)
**Scenario:** An active fault refuses to clear and holds the flag `is_active_fault=true` for over 4 billion samples without resolution.

**Trace:**
- Fault onset at `seq_id = 1000`.
- Current sequence `seq_id = 5,000,000,000`.
- Delta becomes roughly `5B`, which exceeds `UINT32_MAX`.

**Result:**
Safe casting limits the output sequence duration via:
`std::min(static_cast<uint32_t>(delta_seq), static_cast<uint32_t>(UINT32_MAX))`
Thus, `samples_since_fault_onset` saturates exactly at `UINT32_MAX`. The downstream telemetry effectively informs the user the fault has persisted for "a remarkably long time" without silently failing by rolling the count back down to 0, completely avoiding modulo tracking errors.
