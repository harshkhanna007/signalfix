// =============================================================================
// SignalFix AI — Module 1: Drift Stabilizer
// File   : src/stages/drift_stabilizer.hpp
// Spec   : SFX-M1-TDS-001  Revision 4.0 (Orthogonal Decision Architecture)
// =============================================================================
//
// Implements the "Orthogonal Decision Architecture": a strict two-layer
// drift classification engine that decouples instantaneous severity (Space)
// from temporal validation (Time). This is NOT a detector — it is a decision
// authority.
//
// Principles:
//   * Silence by default. Speak only when it matters.
//   * Confidence is a hard gate, not a soft weight.
//   * Momentum is directional context, not a primary trigger.
//   * Alerts are edges, never per-frame state emissions.
//
// Layer 1 — Instantaneous Severity Score (space domain):
//   severity_score = (drift_gsigma - MIN_GSIGMA) * confidence * m_factor
//   where m_factor = 1 + K_MOMENTUM * clamp(momentum, 0, MAX_MOMENTUM)
//   Result: 0.0 if confidence < CONFIDENCE_MIN; bounded above by clamping.
//
// Layer 2 — Temporal State Machine (time domain):
//   NOMINAL: persist_timer counts up while severity >= THRESH_TRIGGER,
//            decays asymmetrically (-2 per frame) when signal drops.
//   DRIFT_CONFIRMED: recover_timer counts up only when severity < THRESH_RECOVER
//            AND momentum <= 0.  Any worsening resets recover_timer to 0.
//
// Alert events (never per-frame):
//   DRIFT_STARTED   — exactly once on NOMINAL -> DRIFT_CONFIRMED
//   DRIFT_ESCALATED — once per cooldown window when severity >= ESCALATION_THRESHOLD
//   DRIFT_RESOLVED  — exactly once on DRIFT_CONFIRMED -> NOMINAL
//
// =============================================================================

#pragma once
#include <algorithm>
#include <cstdint>
#include <cstring>  // memset
#include "signalfix/module1/threshold_calibrator.hpp"

namespace signalfix {

// =============================================================================
// Enumerations
// =============================================================================

/// Coarse output state published to downstream pipeline stages.
enum class DriftOutput : uint8_t {
    NOMINAL        = 0,  ///< No confirmed drift. System is healthy.
    DRIFT_CONFIRMED = 1,  ///< Sustained, high-confidence drift confirmed.
};

/// Discrete alert events. Emitted at state transition edges only.
/// Never emitted per-frame. NONE means "no user-visible event this frame".
enum class DriftEvent : uint8_t {
    NONE            = 0,  ///< No event this frame. Silent path.
    DRIFT_STARTED   = 1,  ///< First confirmation of sustained drift.
    DRIFT_ESCALATED = 2,  ///< Severity crossed critical threshold (post-cooldown).
    DRIFT_RESOLVED  = 3,  ///< Full recovery confirmed. System returned to nominal.
};

/// Internal state machine states.
enum class DriftState : uint8_t {
    NOMINAL         = 0,
    DRIFT_CONFIRMED = 1,
};

// =============================================================================
// Result Type
// =============================================================================

/// Complete per-frame output from DriftStabilizer::update().
/// Callers MUST check event for any user-visible alert action.
/// The output field drives downstream flag logic.
struct StabilizerResult {
    DriftOutput output;        ///< Current coarse output state.
    DriftEvent  event;         ///< One-shot alert event for this frame (usually NONE).
    float       severity;      ///< Instantaneous severity score [0.0, +inf). For tracing.
    int32_t     persist_timer; ///< Current persistence accumulator value. For tracing.
    int32_t     recover_timer; ///< Current recovery accumulator value. For tracing.
};

// =============================================================================
// Configuration
// =============================================================================

/// One breakpoint in the gain schedule table.
/// Thresholds are multiplied by the interpolated K when the
/// dominant process variable (e.g. RPM) equals pv_value.
struct GainSchedulePoint {
    float pv_value;    ///< Process-variable value at this breakpoint.
    float multiplier;  ///< Threshold scale factor at this PV (>= 1.0).
};

/// Maximum number of gain-schedule breakpoints.
static constexpr int kMaxGainSchedulePoints = 8;

/// Maximum rolling-intensity window depth for dynamic deadband.
static constexpr int kDeadbandIntensityWindow = 64;

/// All tunable parameters in one POD struct.
/// Default values target the NASA C-MAPSS T30 HPC temperature channel and
/// are calibrated for the current g%sigma accumulation scale.
///
/// Tuning guidance (see implementation_plan.md §8):
///   - Raise THRESH_TRIGGER / REQUIRED_PERSIST_FRAMES to reduce false positives.
///   - Raise CONFIDENCE_MIN to require higher signal quality before trigger.
///   - Lower K_MOMENTUM to rely less on rate; raise it for faster response.
///   - REQUIRED_RECOVER_FRAMES must always > REQUIRED_PERSIST_FRAMES.
struct DriftStabilizerConfig {
    // ── Severity Layer ────────────────────────────────────────────────────────
    float confidence_min    = 0.35f; ///< Hard gate. Below this, severity = 0.0.
    float min_gsigma        = 1.5f;  ///< Dead-band floor. Eliminates nominal wander.
    float k_momentum        = 0.30f; ///< Momentum amplification factor [0.1, 0.5].
    float max_momentum      = 4.0f;  ///< Upper bound on momentum input [2.0, 5.0].
    float severity_max      = 100.0f;///< Hard ceiling on severity_score output.

    // ── State Machine Thresholds ──────────────────────────────────────────────
    float    thresh_trigger  = 7.0f;  ///< Severity required to begin accumulating.
    float    thresh_recover  = 2.5f;  ///< Severity floor for recovery. Must be < thresh_trigger.
    int32_t  required_persist = 8;   ///< Frames at or above thresh_trigger to confirm.
    int32_t  required_recover = 25;  ///< Frames at or below thresh_recover to resolve. Must be > required_persist.
    int32_t  persist_decay   = 2;    ///< Asymmetric decay per off-threshold frame (noise immunity).

    // ── Alert / Escalation ────────────────────────────────────────────────────
    float    escalation_threshold = 18.0f; ///< Severity watermark for DRIFT_ESCALATED.
    int32_t  cooldown_frames      = 200;   ///< Min frames between any alert emissions.

    // ── Dynamic Deadband ─────────────────────────────────────────────────────
    // The deadband gates how much residual energy leaks into CUSUM.
    // When local noise intensity exceeds 2× baseline, the deadband expands
    // linearly up to the ceiling multiplier to prevent false accumulation.
    float deadband_base_pos       = 0.0f;  ///< 95th-pctile positive deadband (from ThresholdProfile).
    float deadband_base_neg       = 0.0f;  ///< 95th-pctile negative deadband (from ThresholdProfile).
    float baseline_intensity      = 0.0f;  ///< Nominal noise floor (from ThresholdProfile).
    float deadband_expand_max     = 2.0f;  ///< Max expansion multiplier when noise is 4× baseline.
    float deadband_expand_trigger = 2.0f;  ///< Intensity ratio above which expansion begins.
    bool  dynamic_deadband_enabled= false; ///< Enable adaptive deadband logic.

    // ── Gain Scheduling ──────────────────────────────────────────────────────
    // Thresholds scale with a dominant process variable (e.g., RPM).
    // K_schedule is computed by linear interpolation between breakpoints
    // and applied to thresh_trigger, thresh_recover, escalation_threshold.
    GainSchedulePoint gain_schedule[kMaxGainSchedulePoints] = {};
    int       gain_schedule_count = 0;    ///< Number of active breakpoints (0 = disabled).
    float     k_schedule_min     = 1.0f;  ///< Hard floor on multiplier (never more sensitive than baseline).
    float     k_schedule_max     = 3.0f;  ///< Hard ceiling on multiplier (never completely deaf).
    float     schedule_lpf_alpha = 0.02f; ///< Low-pass filter alpha for PV smoothing (0.02 ≈ 50-sample lag).

    // ── ThresholdProfile integration ─────────────────────────────────────────
    /// Wire a calibrated ThresholdProfile into this config.
    /// Call this after constructing DriftStabilizerConfig but before creating
    /// the DriftStabilizer instance.
    void apply_threshold_profile(const signalfix::ThresholdProfile& p) noexcept {
        if (!p.is_valid) return;
        deadband_base_pos    = p.deadband_pos;
        deadband_base_neg    = p.deadband_neg;
        baseline_intensity   = p.baseline_intensity;
        // Map calibrated CUSUM thresholds to severity-space thresholds.
        // The severity score is in the same scale as the raw g%σ accumulator,
        // so calibrated thresholds translate directly.
        thresh_trigger       = p.warning_threshold;
        thresh_recover       = p.recovery_threshold;
        escalation_threshold = p.critical_threshold;
        dynamic_deadband_enabled = (deadband_base_pos > 0.0f || deadband_base_neg > 0.0f);
    }
};


static_assert(sizeof(DriftStabilizerConfig) <= 384u, "DriftStabilizerConfig unexpectedly large.");


// String helpers (defined in drift_stabilizer.cpp)
const char* drift_state_to_string(DriftState s) noexcept;
const char* drift_output_to_string(DriftOutput o) noexcept;
const char* drift_event_to_string(DriftEvent e) noexcept;

// =============================================================================
// DriftStabilizer
// =============================================================================

/// Two-layer drift decision authority.
///
/// Usage:
///   DriftStabilizer stabilizer;                      // or with custom config
///   StabilizerResult r = stabilizer.update(gsigma, confidence, momentum);
///   if (r.event != DriftEvent::NONE) { /* emit alert using r.event */ }
///   if (r.output == DriftOutput::DRIFT_CONFIRMED)    { /* set env flag */ }
///
/// Thread safety: NOT thread-safe. Must be used from a single producer thread.
/// Lifetime:      One instance per channel. Call reset() on channel reinit.
class DriftStabilizer {
public:
    /// Construct with default configuration (C-MAPSS T30 tuned).
    DriftStabilizer() noexcept = default;

    /// Construct with explicit configuration for custom channels.
    explicit DriftStabilizer(DriftStabilizerConfig cfg) noexcept : cfg_(cfg) {}

    /// Process one frame. Returns coarse state + event + diagnostic scalars.
    ///
    /// @param drift_gsigma  Raw CUSUM g%sigma accumulator value from S5.5.
    /// @param confidence    Drift confidence from S5.5 [0.0, 1.0].
    /// @param momentum      EWMA directional rate from S5 (signed).
    [[nodiscard]] StabilizerResult update(float drift_gsigma,
                                          float confidence,
                                          float momentum) noexcept;

    /// Reset all transient state to initial values.
    /// Call this when a channel is re-registered or the pipeline restarts.
    void reset() noexcept;

    /// Inject the current dominant process variable (e.g. RPM) for gain scheduling.
    /// Call this once per sample BEFORE calling update() when gain scheduling is active.
    /// If gain_schedule_count == 0, this is a no-op.
    void set_process_variable(float pv_raw) noexcept {
        if (cfg_.gain_schedule_count > 0) {
            k_schedule_ = compute_gain_schedule(pv_raw);
        }
    }

    /// Read-only access to current state for external diagnostics.
    [[nodiscard]] DriftState  state()         const noexcept { return state_; }
    [[nodiscard]] int32_t     persist_timer() const noexcept { return persist_timer_; }
    [[nodiscard]] int32_t     recover_timer() const noexcept { return recover_timer_; }
    [[nodiscard]] float       last_severity() const noexcept { return last_severity_; }

private:
    DriftStabilizerConfig cfg_{};

    // ── State Machine ─────────────────────────────────────────────────────────
    DriftState state_         = DriftState::NOMINAL;
    int32_t    persist_timer_ = 0;   ///< Frames where severity >= thresh_trigger.
    int32_t    recover_timer_ = 0;   ///< Frames where severity < thresh_recover AND momentum <= 0.
    int32_t    cooldown_timer_= 0;   ///< Alert dead-time countdown [frames].

    // ── Dynamic Deadband Runtime State ───────────────────────────────────────
    float    intensity_window_[kDeadbandIntensityWindow] = {}; ///< Rolling residual intensity buffer.
    int      intensity_head_   = 0;   ///< Next write index (circular).
    int      intensity_count_  = 0;   ///< Samples currently in window.
    float    intensity_sum_    = 0.0f;///< Running sum for O(1) mean.
    float    active_deadband_pos_ = 0.0f;
    float    active_deadband_neg_ = 0.0f;

    // ── Gain Scheduling Runtime State ────────────────────────────────────────
    float    pv_filtered_      = 0.0f; ///< Low-pass filtered process variable.
    float    k_schedule_       = 1.0f; ///< Current effective threshold multiplier.

    // ── Diagnostics ───────────────────────────────────────────────────────────
    float      last_severity_ = 0.0f;

    // ── Internal helpers ──────────────────────────────────────────────────────
    [[nodiscard]] float compute_severity(float drift_gsigma,
                                         float confidence,
                                         float momentum) const noexcept;

    /// Update rolling intensity window and recompute active deadbands.
    void update_dynamic_deadband(float residual_abs) noexcept;

    /// Compute gain schedule multiplier from current process variable.
    [[nodiscard]] float compute_gain_schedule(float pv_raw) noexcept;

};

} // namespace signalfix
