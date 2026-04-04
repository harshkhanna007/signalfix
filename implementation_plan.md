# Harden Stage S5 (Rate of Change)

This plan addresses the requirement to harden the Stage S5 runtime execution while preserving its isolated bounds, ensuring numerical safety, and guaranteeing determinism without changing any of the overarching pipeline responsibilities. A new focused test suite will also be introduced.

## User Review Required

> [!WARNING]
> Please review the decisions taken with respect to edge cases (warm-up state bounds, dt_s ≈ 0 behavior, NaN initialization). The core S5 equations (Welford statistics) are kept intact but rigorously wrapped in invariant guards to guarantee deterministic operation in the broader SignalFix pipeline.

## Proposed Changes

### Core Stage S5 Implementation
We will modify the core S5 execution flow to explicitly guarantee initialized outputs on all paths, secure division bounds, and add numerical stability enhancements.

#### [MODIFY] [stage_s5_rate_of_change.cpp](file:///c:/signalfix-ai/src/stages/stage_s5_rate_of_change.cpp)
**1. Deterministic Fast-Path Initialization:**
When returning early (dt_s ≈ 0, invalid samples, or warm-up frame '0'), S5 currently leaves `envelope.roc`, `envelope.roc_adaptive_limit`, and `roc_threshold_used` uninitialized (NaN from struct init). 
*Change:* Ensure deterministic initialization for these fields (e.g. `0.0f` for roc, `fallback_limit` for thresholds) on *all* early-exit paths so downstream stages (S7) are completely insulated from random memory states or NaNs.

**2. NaN / Inf Safety on ROC Math:**
The calculation `raw_roc = delta_v / dt_s` and subsequent derivations can generate `Inf` if an edge case is breached despite the `dt_s` guard. Furthermore, if `calibrated_value` isn't `NaN` but jumps immensely, it can inflate standard derivation incorrectly.
*Change:* Explicitly compute bounded values with `std::isfinite` checks upfront for `raw_roc`. Clamp `delta_v` definitively to `PHYSICAL_MAX_ROC` right at derivation. Ensure all division operations (like `conf_range` derivations) are securely checked for zero denominators (e.g., `threshold > 0`). 

**3. Robust Feedback Loop (Snowblindness) Protection:**
The `is_outlier` check prevents learning when a massive spike is encountered. This is correct to prevent runaway threshold explosions.
*Change:* Keep the outlier rejection tight (`learning_limit * 3.0`), but ensure that repeated outliers don't permanently brick the channel context if a legitimate physical step-change occurred. We will add a clamp to prevent Welford `var` / `m2` from blowing up to `inf` during extreme noise bursts. 

**4. Ensure Safe SPIKE/DRIFT Upgrades:**
Currently, SPIKE and DRIFT confidences vary and are mapped to `suggested_conf`. 
*Change:* We will tighten the bounding `fmin`/`fmax` and keep the classification exactly as requested, mapping extreme ROC jumps directly to SPIKE and persistent jumps to DRIFT, while avoiding overlap. 

### Testing Suite

#### [NEW] [test_s5_rate_of_change.cpp](file:///c:/signalfix-ai/tests/test_s5_rate_of_change.cpp)
We will create a comprehensive standalone test harness modeling the S5 environment to validate properties explicitly:
- **Test 1: Steady State:** Perfectly stable signal doesn't trip false positives and `sigma` stays near minimums securely.
- **Test 2: Spike:** A single extreme step jump immediately triggers SPIKE classification.
- **Test 3: Drift:** A sustained but small exceedance just above the adaptive limit slowly accrues and triggers DRIFT.
- **Test 4: Noise Burst:** Rapid fluctuation tightly packed around the baseline avoids classification by properly rejecting noise (or scaling threshold correctly).
- **Test 5: Recovery:** Normal samples following a spike completely stabilize the stream, resetting counters and flag masks.

## Open Questions

- Should we treat `dt_s <= config_.epsilon_dt` as a `TIMING_ANOMALY` flag injection or passively ignore it? *Proposal:* Passively ignore it and emit as `CONTINUE` holding `envelope.roc = 0.0f`, matching current silent behavior, to not disrupt Stage 6.

## Verification Plan

### Automated Tests
- Build and execute the new `test_s5_rate_of_change.cpp`.
- Execute a suite run utilizing:
```bash
g++ -std=c++17 -Wall -Wextra -Iinclude tests/test_s5_rate_of_change.cpp src/stages/stage_s5_rate_of_change.cpp -o tmp/test_s5
./tmp/test_s5
```
- Print outputs to stdout explicitly matching the formatting demanded in the prompt.
