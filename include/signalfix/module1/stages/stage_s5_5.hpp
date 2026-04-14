// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : src/stages/stage_s55_drift_persistence.hpp
// Stage  : S5.5 — Drift Persistence & Arbitration  [Rev 2.0]
// Spec   : SFX-M1-TDS-001
//
// PIPELINE POSITION: S5 → [S5.5] → S6 → S7
//
// CHANGES FROM Rev 1.0
// ---------------------
//   - Thresholds converted from sample counts to wall-clock seconds.
//     System is now sampling-rate agnostic.
//   - Observation widened: weak drift pre-signals (drift_gsigma, partial ROC)
//     detected before S5 fully triggers DRIFT_EXCEEDED.
//   - Confidence computed dynamically from persistence_time, drift_gsigma,
//     ROC severity.  Fixed 0.85/0.97 constants removed.
//   - Decay made adaptive: slower when drift_gsigma is high, faster when low.
//   - Hysteresis: upgrade/downgrade use asymmetric time thresholds to prevent
//     oscillation at level boundaries.
//   - failure_duration updated when drift is CONFIRMED or CRITICAL so S7
//     package_sample() reflects correct fault duration.
//
// S6 COMPATIBILITY
//   S6 reads: status, delta_t_us, arrival_time_us, timing correction state.
//   S6 does NOT read DRIFT_EXCEEDED, drift_level, or drift confidence.
//   No S6-owned field is written by S5.5.
//
// S7 COMPATIBILITY
//   S7 drop-gates on: STALE, MISSING, HARD_INVALID, TIMING_ANOMALY, RATE_ANOMALY.
//   DRIFT_EXCEEDED is NOT a S7 drop trigger.
//   S7 package_sample() reads: failure_hint, failure_confidence, failure_duration.
//   All three are updated here when level >= CONFIRMED.
// =============================================================================

#pragma once
#include "signalfix/module1/types.hpp"
#include "signalfix/module1/stage_interface.hpp"
namespace signalfix {

// ---------------------------------------------------------------------------
// DriftLevel
//
//  Upgrade thresholds (seconds):
//    NOISE → BUILDUP    :  1.0 s
//    BUILDUP → CONFIRMED:  2.5 s
//    CONFIRMED → CRITICAL: 6.0 s
//
//  Downgrade thresholds (seconds, lower = hysteresis):
//    CRITICAL → CONFIRMED: 4.5 s  (1.5 s gap)
//    CONFIRMED → BUILDUP:  1.5 s  (1.0 s gap)
//    BUILDUP → NOISE:      0.5 s  (0.5 s gap)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// ChannelState fields — MODIFY pipeline_types.hpp:
//
//  ADD:
//    float      drift_persistence_time_s = 0.0f;  // Time-accumulated drift (seconds)
//    float      drift_clear_time_s       = 0.0f;  // Consecutive clear time (seconds)
//    DriftLevel drift_level              = DriftLevel::NOISE;
//    float      drift_confidence         = 0.0f;  // Last computed confidence [0,1]
//    float      drift_score              = 0.0f;  // Continuous severity metric [0, 100]
//
//  REMOVE (replaced):
//    int  drift_persistence_count;
//    int  drift_clear_count;
// ---------------------------------------------------------------------------

class StageS55DriftPersistence {
public:
    StageS55DriftPersistence() noexcept = default;

    const char* stage_name() const noexcept;
    void        reset()      noexcept;

    StageResult process(MeasurementEnvelope& envelope,
                        ChannelState&        channel_state) noexcept;

private:
    // ── Upgrade thresholds (seconds) ─────────────────────────────────────────
    static constexpr float T_BUILDUP_UP    = 1.0f;
    static constexpr float T_CONFIRMED_UP  = 2.5f;
    static constexpr float T_CRITICAL_UP   = 6.0f;

    // ── Downgrade thresholds (seconds) — asymmetric for hysteresis ───────────
    static constexpr float T_BUILDUP_DOWN   = 0.5f;
    static constexpr float T_CONFIRMED_DOWN = 1.5f;
    static constexpr float T_CRITICAL_DOWN  = 4.5f;

    // ── Recovery gate: clear time before decay is allowed (seconds) ──────────
    // Increased from 0.5 to 1.5 so brief clean windows during slow degradation
    // do not prematurely erode accumulated evidence.
    static constexpr float RECOVERY_GATE_S = 1.5f;

    // ── Adaptive decay base rate (persistence seconds lost per clear second) ──
    // Actual rate is divided by (1 + drift_gsigma) — stronger drift decays slower.
    // Reduced from 0.4 to 0.25 to slow evidence loss during gradual degradation.
    static constexpr float BASE_DECAY_RATE = 0.25f;

    // ── Weak-drift observation thresholds ────────────────────────────────────
    // drift_gsigma above this contributes a partial (0.4×) accumulation signal
    // even when S5 has not yet fired DRIFT_EXCEEDED.
    // Lowered from 0.5 to 0.35 to capture more gradual degradation signal.
    static constexpr float WEAK_GSIGMA_THRESHOLD = 0.35f;
    // ROC fraction: if roc > threshold × this fraction, counts as weak signal.
    // Lowered from 0.5 to 0.4 to widen the weak-drift capture window.
    static constexpr float WEAK_ROC_FRACTION = 0.4f;
    // Accumulation weight for a weak-only signal (vs 1.0 for S5-confirmed).
    static constexpr float WEAK_SIGNAL_WEIGHT = 0.4f;

    // ── Sub-threshold observation thresholds (Rev 2.1) ───────────────────────
    // Signals between sub-threshold and weak thresholds contribute a small but
    // non-zero weight, preventing total evidence loss during gradual degradation.
    static constexpr float SUB_THRESHOLD_ROC_FRACTION = 0.2f;
    static constexpr float SUB_THRESHOLD_GSIGMA       = 0.2f;
    static constexpr float SUB_THRESHOLD_WEIGHT       = 0.15f;
    // Extra weight bonus when both ROC partial AND gsigma weak fire together.
    // Multi-indicator coincidence is stronger evidence than either alone.
    static constexpr float COMBINED_WEAK_BONUS         = 0.15f;

    // ── Confidence computation weights (must sum to 1.0) ─────────────────────
    static constexpr float W_PERSISTENCE = 0.50f;
    static constexpr float W_GSIGMA      = 0.30f;
    static constexpr float W_ROC         = 0.20f;

    // ── Score computation weights (independent from confidence) ──────────────
    static constexpr float W_SCORE_PERSIST = 0.4f;
    static constexpr float W_SCORE_GSIGMA  = 0.4f;
    static constexpr float W_SCORE_ROC     = 0.2f;

    // ── Confidence normalization caps ─────────────────────────────────────────
    // persistence normalized against T_CRITICAL_UP; gsigma against empirical cap.
    static constexpr float CONF_GSIGMA_CAP = 10.0f;

    // ── Helpers ───────────────────────────────────────────────────────────────
    [[nodiscard]] static DriftLevel classify_with_hysteresis(
        float      persistence_s,
        DriftLevel current_level) noexcept;

    [[nodiscard]] static float compute_confidence(
        float persistence_s,
        float drift_gsigma,
        float roc_severity) noexcept;
};

} // namespace signalfix
