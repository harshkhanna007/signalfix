# Root Cause Analysis: Undetected Slow Degradation (Drift)

## 1. Root Cause Analysis
Slow degradation is not being detected because the S5 (`StageS5RateOfChange`) layer completely fails to extract a persistent drift signal from smoothly degrading data. The system structurally "absorbs" the drift mathematically by adapting to it, measures the wrong statistical property (variance instead of mean rate of change) for drift detection, and then ruthlessly punishes its own internal accumulators via an aggressive consecutive-nominal decay multiplier.

Consequently, S5 never reaches `DRIFT_EXCEEDED`, and S5.5 is starved of the precursor signals (`drift_gsigma`) it expects to build up its `persistence_time_s`. 

## 2. Verified Findings

* **S5 computes `g_step` using `sigma` (variance), ignoring `roc_mean`:** `drift_gsigma` only grows if the *standard deviation* of the rate of change (`sigma`) deviates from `roc_sigma_baseline`. Smooth, slow degradation shifts the average value but introduces very little new variance to the rate of change. Thus, `g_step` remains near zero.
* **Aggressive self-canceling decay in S5:** Because the smooth drift never causes a sudden spike, `envelope.status` remains `NOMINAL`. As a result, `roc_normal_streak` reliably surpasses 20. Once `roc_normal_streak > 20`, S5 executes `channel_state.drift_gsigma *= 0.98` on **every single sample**. This exponential decay mathematically guarantees `drift_gsigma` gets crushed to zero faster than any trace variance can accumulate.
* **S5 continuous learning normalizes the degraded baseline:** The condition to suspend learning is `(raw_roc_safe > (learning_limit * 3.0))`. Slow drift always falls well below this spike threshold. Therefore, the background learning variables (`roc_mean`, `roc_m2`, `roc_threshold_ema`) continuously update and "bake in" the slow degradation as the new normal.
* **S5.5 is entirely starved by S5:** S5.5 relies on `"weak"` precursor signals (`WEAK_GSIGMA_THRESHOLD = 0.35f`) to accumulate `persistence_time_s` and transition states. Since S5 forces `drift_gsigma` into the ground, S5.5's `gsigma_weak` and `gsigma_sub` flags never fire. `roc_partial` fires occasionally on noise, but not tightly enough to overcome S5.5's own decay.

## 3. Exact Failure Points

* **File:** `src/stages/stage_s5_rate_of_change.cpp`
* **Function:** `StageS5RateOfChange::process`
* **Failure Condition 1 (Wrong Metric):** Line 335. `const double g_step = std::abs(sigma - baseline) / baseline;` measures only scale (variance), ignoring the directional ROC.
* **Failure Condition 2 (Aggressive Decay):** Lines 349-354. `if (channel_state.roc_normal_streak > 20) { channel_state.drift_gsigma *= 0.98; }` aggressively destroys the accumulator.
* **Failure Condition 3 (Normalization):** Line 303. `if (!is_outlier && envelope.status == SampleStatus::NOMINAL)` continuously updates `roc_mean` and `roc_m2`, adapting to the drift seamlessly.
* **File:** `src/stages/stage_s5_5.cpp`
* **Function:** `StageS55DriftPersistence::process`
* **Failure Condition 4 (Dependent Starvation):** Lines 165-167. S5.5 fundamentally expects `gsigma_val` (which S5 has ruined) to drive its `WEAK_GSIGMA_THRESHOLD` gate.

## 4. System Behavior Explanation

**Is the system too insensitive?**
Yes, but structurally so. S5 is blind to *mean shifts* because it characterizes drift only through the *variance* of the rate of change (`g_step`). 

**Is the system over-adaptive? (Incorrectly normalized?)**
Yes. Since the slow drift never triggers the 3.0x outlier cutoff, S5 treats the degradation as valid nominal data. It continuously digests the degradation into its running EMAs (`roc_mean`, `var`), essentially moving the goalposts in real time. 

**Is it missing weak accumulation?**
Yes. The 0.98 multiplier acts as a black hole. Any tiny fractional buildup of `g_step` is vaporized every sample once a 20-sample streak is reached (which is trivial for slow drift).

**Dependency Breakdown:**
S5 completely blocks S5.5. S5.5 is designed to be an evidence-based temporal persistence layer, but it requires S5 to provide the `drift_gsigma` evidence. Because S5 masks and decays `drift_gsigma` before it reaches S5.5, S5.5 receives mostly 0.0 weights and behaves completely cleanly. The S5.5 persistence layer is ineffective because its supplier is broken.

## 5. Final Question: Why does slow degradation in the NASA dataset never reach DRIFT_EXCEEDED?

Slow degradation never reaches `DRIFT_EXCEEDED` because the system's definition of drift is fundamentally flawed and self-defeating. S5 looks for drift by checking if the *variance* of the rate-of-change increases, completely ignoring the *mean* value shifting. Compounding this, because slow degradation doesn't trigger sudden spike alarms, S5 considers the signal "NOMINAL," triggering an aggressive 0.98x exponential decay every sample that permanently traps the drift accumulator near zero. The system actively learns the degradation as normal behavior and mathematically suppresses any evidence of it.
