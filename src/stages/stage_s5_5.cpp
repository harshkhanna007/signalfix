// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : src/stages/stage_s55_drift_persistence.cpp
// Stage  : S5.5 — Drift Persistence & Arbitration  [Rev 2.1]
// Spec   : SFX-M1-TDS-001
// =============================================================================
//
// BEHAVIORAL CONTRACT
// -------------------
//  Rev 1.0 (sample-based):
//      DRIFT → NOMINAL → DRIFT → NOMINAL  ← flicker
//
//  Rev 2.0 (time-based, hysteresis, weak-signal detection):
//      t=0.0  signal weak  → NOISE       (no flag forced)
//      t=0.5  gsigma rises → NOISE       (pre-accumulating)
//      t=1.0  BUILDUP      → flag forced, no hint upgrade yet
//      t=2.5  CONFIRMED    → flag forced + confidence-weighted DRIFT hint
//      t=6.0  CRITICAL     → flag forced + high confidence hint
//      brief blip at t=7   → CRITICAL held (hysteresis: need 4.5 s clear to drop)
//
// THREE PHASES (preserved from Rev 1.0):
//   OBSERVE   — read S5 output + raw channel_state signals
//   UPDATE    — advance time accumulators, adaptive decay
//   ARBITRATE — classify level (hysteresis), compute confidence, enforce flags
//
// =============================================================================
#include "signalfix/module1/stages/stage_s5_5.hpp"
#include "signalfix/module1/channel_state.hpp"
#include "signalfix/module1/types.hpp"
#include <algorithm>   // std::max, std::min
#include <cmath>       // std::isfinite
#include "pipeline.hpp"
namespace signalfix {

// ---------------------------------------------------------------------------
const char* StageS55DriftPersistence::stage_name() const noexcept {
    return "S5.5-DRIFT-PERSIST";
}

void StageS55DriftPersistence::reset() noexcept {
    // All state lives in ChannelState — caller is responsible for reset.
}

// ---------------------------------------------------------------------------
// classify_with_hysteresis()
//
// Converts the raw persistence accumulator to a DriftLevel.
// Upgrade and downgrade use different thresholds so the level cannot
// oscillate when persistence_s sits near a boundary.
//
// Rules (applied in priority order — current level determines which
// threshold set is active for transitions):
//
//   Any level → higher: use UPGRADE thresholds (higher bar).
//   Any level → lower:  use DOWNGRADE thresholds (lower bar — must fall
//                        further before level drops).
//
// This means: once CONFIRMED, persistence must drop below T_CONFIRMED_DOWN
// (not just below T_CONFIRMED_UP) before we fall back to BUILDUP.
// ---------------------------------------------------------------------------
DriftLevel StageS55DriftPersistence::classify_with_hysteresis(
    float      persistence_s,
    DriftLevel current_level) noexcept
{
    // ── Attempt upgrade (current → higher) ───────────────────────────────────
    if (persistence_s >= T_CRITICAL_UP)  return DriftLevel::CRITICAL;
    if (persistence_s >= T_CONFIRMED_UP) return DriftLevel::CONFIRMED;
    if (persistence_s >= T_BUILDUP_UP)   return DriftLevel::BUILDUP;

    // ── Below all upgrade thresholds: apply downgrade with hysteresis ─────────
    // Only downgrade if persistence has fallen below the DOWNGRADE threshold
    // for the step below current level.

    if (current_level == DriftLevel::CRITICAL) {
        // CRITICAL → CONFIRMED requires persistence < T_CRITICAL_DOWN
        if (persistence_s >= T_CRITICAL_DOWN) return DriftLevel::CRITICAL;
        // Falls through to CONFIRMED path below.
    }

    if (current_level >= DriftLevel::CONFIRMED) {
        if (persistence_s >= T_CONFIRMED_DOWN) return DriftLevel::CONFIRMED;
        // Falls through to BUILDUP path below.
    }

    if (current_level >= DriftLevel::BUILDUP) {
        if (persistence_s >= T_BUILDUP_DOWN) return DriftLevel::BUILDUP;
    }

    return DriftLevel::NOISE;
}

// ---------------------------------------------------------------------------
// compute_confidence()
//
// Produces a [0.0, 1.0] confidence value from three independent signals:
//
//   persistence_s  — how long drift has been sustained (weight: 50%)
//   drift_gsigma   — S5's accumulated fractional sigma deviation (weight: 30%)
//   roc_severity   — normalized ROC vs threshold (weight: 20%)
//
// Each factor is individually clamped to [0, 1] before weighting so that
// a single dominant signal cannot overwhelm the others.
// ---------------------------------------------------------------------------
float StageS55DriftPersistence::compute_confidence(
    float persistence_s,
    float drift_gsigma,
    float roc_severity) noexcept
{
    // Normalize each factor to [0, 1].
    const float f_persist = std::min(1.0f, persistence_s / T_CRITICAL_UP);

    const float f_gsigma  = (CONF_GSIGMA_CAP > 0.0f)
        ? std::min(1.0f, std::max(0.0f, drift_gsigma / CONF_GSIGMA_CAP))
        : 0.0f;

    // roc_severity is already a 0–1 fraction (roc / threshold); cap at 1.
    const float f_roc = std::min(1.0f, std::max(0.0f, roc_severity));

    const float confidence = W_PERSISTENCE * f_persist
                           + W_GSIGMA      * f_gsigma
                           + W_ROC         * f_roc;

    return std::min(1.0f, std::max(0.0f, confidence));
}

// ---------------------------------------------------------------------------
// process()
// ---------------------------------------------------------------------------
StageResult StageS55DriftPersistence::process(
    MeasurementEnvelope& envelope,
    ChannelState&        channel_state) noexcept
{
    // =========================================================================
    // PHASE 1: OBSERVE  [Rev 2.1 — Sub-threshold Accumulation]
    //
    // Three-tier signal classification:
    //   Tier 1 (full):  S5 confirmed DRIFT_EXCEEDED      → weight 1.0
    //   Tier 2 (weak):  gsigma or ROC above weak thresh   → weight 0.4 (+0.15 combo)
    //   Tier 3 (sub):   gsigma or ROC above sub thresh    → weight 0.15
    //   Tier 0 (clean): nothing elevated                  → weight 0.0
    //
    // The sub-threshold tier prevents total evidence loss during gradual
    // degradation where signals sit between 20–40% of threshold.
    // The combined bonus rewards multi-indicator coincidence.
    // =========================================================================

    const float dt_s = static_cast<float>(envelope.delta_t_us) * 1.0e-6f;

    // Guard: skip accumulation for invalid/missing/stale samples.
    // We do not want absent samples to pollute the time accumulators.
    if (!std::isfinite(dt_s) || dt_s <= 0.0f ||
        has_flag(envelope.status, SampleStatus::HARD_INVALID) ||
        has_flag(envelope.status, SampleStatus::MISSING)      ||
        has_flag(envelope.status, SampleStatus::STALE))
    {
        // Pass through without modifying drift state.
        return StageResult::CONTINUE;
    }

    // Signal 1: S5 fully confirmed drift.
    const bool s5_drift = has_flag(envelope.status, SampleStatus::DRIFT_EXCEEDED);

    // Signal 2: Weak pre-signal from drift_gsigma.
    // drift_gsigma is an accumulator initialized to 0 in S5 — always finite.
    const float gsigma_val = static_cast<float>(channel_state.drift_gsigma);
    const bool gsigma_weak = (gsigma_val > WEAK_GSIGMA_THRESHOLD);
    const bool gsigma_sub  = (gsigma_val > SUB_THRESHOLD_GSIGMA) && !gsigma_weak;

    // Signal 3: ROC approaching its adaptive threshold (partial trigger).
    // roc and roc_threshold_used are set to NaN by S5 on early-return paths;
    // guard both before using.
    bool roc_partial = false;
    bool roc_sub     = false;
    float roc_severity = 0.0f;
    if (std::isfinite(envelope.roc) &&
        std::isfinite(envelope.roc_threshold_used) &&
        envelope.roc_threshold_used > 0.0f)
    {
        roc_severity = envelope.roc / envelope.roc_threshold_used;
        roc_partial  = (roc_severity > WEAK_ROC_FRACTION);
        roc_sub      = (roc_severity > SUB_THRESHOLD_ROC_FRACTION) && !roc_partial;
    }

    // ── Three-tier weight assignment ─────────────────────────────────────────
    // Priority: S5 confirmed > weak > sub-threshold > clean.
    const bool any_weak = gsigma_weak || roc_partial;
    const bool any_sub  = gsigma_sub  || roc_sub;

    float signal_weight = 0.0f;
    if (s5_drift) {
        signal_weight = 1.0f;
    } else if (any_weak) {
        signal_weight = WEAK_SIGNAL_WEIGHT;
        // Combined-signal bonus: both indicators elevated → stronger evidence.
        if (gsigma_weak && roc_partial) {
            signal_weight = std::min(signal_weight + COMBINED_WEAK_BONUS, 0.6f);
        }
    } else if (any_sub) {
        signal_weight = SUB_THRESHOLD_WEIGHT;
    }

    const bool drift_active = (signal_weight > 0.0f);

    // =========================================================================
    // PHASE 2: UPDATE — time accumulators + adaptive decay
    //
    // drift_persistence_time_s grows while any drift signal is present.
    // drift_clear_time_s counts consecutive clean time.
    //
    // Decay rules:
    //   - Decay only starts after clear streak exceeds RECOVERY_GATE_S.
    //   - Decay rate is modulated by drift_gsigma: stronger residual drift →
    //     slower decay, reflecting that a sensor not yet fully recovered should
    //     not clear its history quickly.
    //   - Signal weight also scales accumulation: weak signals build persistence
    //     more slowly than S5-confirmed signals.
    // =========================================================================

    if (drift_active) {
        // Accumulate persistence at signal_weight × dt.
        // Weak signals build up slowly; S5-confirmed signals at full rate.
        channel_state.drift_persistence_time_s += signal_weight * dt_s;
        channel_state.drift_clear_time_s        = 0.0f;
    } else {
        // Accumulate clear time.
        channel_state.drift_clear_time_s += dt_s;

        // Only begin decaying persistence once the recovery gate is satisfied.
        // This prevents a brief NOMINAL blip from eating into hard-won persistence.
        if (channel_state.drift_clear_time_s >= RECOVERY_GATE_S) {
            // Adaptive decay: divide by (1 + gsigma) so sensors with high
            // residual sigma decay very slowly even when flags are absent.
         const float gsigma_safe = std::max(0.0f, static_cast<float>(channel_state.drift_gsigma));
            const float decay_rate  = BASE_DECAY_RATE / (1.0f + gsigma_safe);

            channel_state.drift_persistence_time_s =
                std::max(0.0f,
                    channel_state.drift_persistence_time_s - decay_rate * dt_s);
        }
        // Else: brief clear, hold persistence steady.
    }

    // =========================================================================
    // PHASE 3: ARBITRATE
    //
    // 3a. Classify level using asymmetric hysteresis thresholds.
    // 3b. Compute dynamic confidence from three independent factors.
    // 3c. Enforce DRIFT_EXCEEDED flag and failure fields per level.
    // =========================================================================

    // 3a. Classify.
    const DriftLevel new_level = classify_with_hysteresis(
        channel_state.drift_persistence_time_s,
        channel_state.drift_level);

    channel_state.drift_level = new_level;

    // 3b. Confidence — computed regardless of level so it's always readable.
    const float confidence = compute_confidence(
        channel_state.drift_persistence_time_s,
        channel_state.drift_gsigma,
        roc_severity);

    channel_state.drift_confidence = confidence;

    // ── Continuous Severity Metric (drift_score) ─────────────────────────────
    // 1. Evidence-Based Accumulation
    const float norm_persist = std::min(1.0f, std::max(0.0f, channel_state.drift_persistence_time_s / T_CRITICAL_UP));
    const float norm_gsigma  = (CONF_GSIGMA_CAP > 0.0f) 
                                ? std::min(1.0f, std::max(0.0f, static_cast<float>(channel_state.drift_gsigma) / CONF_GSIGMA_CAP))
                                : 0.0f;
    const float norm_roc     = std::isfinite(roc_severity) ? std::min(1.0f, std::max(0.0f, roc_severity)) : 0.0f;
    
    // Using W_SCORE specific weights to ensure independence from drift_confidence
    const float evidence = W_SCORE_PERSIST * norm_persist + 
                           W_SCORE_GSIGMA  * norm_gsigma + 
                           W_SCORE_ROC     * norm_roc;

    // 2. Level-Based Scaling
    float level_multiplier = 0.5f;
    switch (new_level) {
        case DriftLevel::NOISE:     level_multiplier = 0.5f; break;
        case DriftLevel::BUILDUP:   level_multiplier = 1.0f; break;
        case DriftLevel::CONFIRMED: level_multiplier = 1.5f; break;
        case DriftLevel::CRITICAL:  level_multiplier = 2.0f; break;
    }

    // 3. Time-Based Accumulation
    const float ACCUM_RATE = 100.0f / T_CRITICAL_UP;
    const float BASE_DECAY = 50.0f  / T_CRITICAL_UP;
    
    if (signal_weight > 0.0f) {
        channel_state.drift_score += evidence * level_multiplier * ACCUM_RATE * dt_s;
    } else {
        // 4. Intelligent Decay
        if (channel_state.drift_clear_time_s >= RECOVERY_GATE_S) {
            const float gsigma_safe = std::max(0.0f, static_cast<float>(channel_state.drift_gsigma));
            const float decay_rate = BASE_DECAY / (1.0f + gsigma_safe);
            channel_state.drift_score -= decay_rate * dt_s;
        }
    }

    // 5. Hard Constraints
    if (!std::isfinite(channel_state.drift_score)) {
        channel_state.drift_score = 0.0f;
    }
    channel_state.drift_score = std::max(0.0f, std::min(100.0f, channel_state.drift_score));
    envelope.drift_score = channel_state.drift_score; // Export for telemetry

    // 3c. Enforce flags and failure fields per level.
    //
    // NOISE:
    //   Pass-through. If S5 set DRIFT_EXCEEDED for a transient, let it through
    //   unchanged so S6/S7 can see the raw signal. Do not force it on.
    //   Do not upgrade failure hint.
    //
    // BUILDUP:
    //   Force DRIFT_EXCEEDED on — trend is forming. No failure hint upgrade yet
    //   because we are not confident enough to declare a fault.
    //
    // CONFIRMED:
    //   Force DRIFT_EXCEEDED. Upgrade hint to DRIFT with dynamic confidence.
    //   Update failure_duration so S7's package_sample() reflects fault age.
    //
    // CRITICAL:
    //   Force DRIFT_EXCEEDED. Upgrade hint to DRIFT with high confidence.
    //   Continue incrementing failure_duration.
    //
    // Note on failure_duration:
    //   S7 reads envelope.failure_duration directly in package_sample().
    //   S5's fault_tracker manages fault_state.samples_since_fault_onset for
    //   ROC faults; we do NOT touch that field here to avoid conflict.
    //   We manage failure_duration independently for the drift fault lifecycle.
    //   Reset when level returns to NOISE/BUILDUP.

    switch (new_level) {

        case DriftLevel::NOISE:
            // No override. If S5 set the flag transiently, leave it as-is.
            // Clear failure_duration — no active drift fault at this level.
            envelope.failure_duration = 0u;
            break;

        case DriftLevel::BUILDUP:
            // Trend forming: force flag, no hint yet.
            envelope.status |= SampleStatus::DRIFT_EXCEEDED;
            // Reset failure_duration — not yet a fault, just a warning.
            envelope.failure_duration = 0u;
            break;

        case DriftLevel::CONFIRMED:
            // Drift is real.
            envelope.status |= SampleStatus::DRIFT_EXCEEDED;
            upgrade_failure_hint(envelope, FailureMode::DRIFT, confidence);
            // Increment failure_duration (saturate at max uint32).
            if (envelope.failure_duration < UINT32_MAX) {
                envelope.failure_duration++;
            }
            break;

        case DriftLevel::CRITICAL:
            // Severe drift. Confidence pushed toward 1.0 but still dynamic.
            // At T_CRITICAL_UP with full gsigma, confidence ≈ 0.93–0.99.
            envelope.status |= SampleStatus::DRIFT_EXCEEDED;
            upgrade_failure_hint(envelope, FailureMode::DRIFT, confidence);
            if (envelope.failure_duration < UINT32_MAX) {
                envelope.failure_duration++;
            }
            break;
    }

    return StageResult::CONTINUE;
}

} // namespace signalfix
