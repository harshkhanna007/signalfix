// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/interpretation_layer.hpp
// =============================================================================
//
// Interpretation Layer — converts S7 persistent output into human-readable
// diagnostics suitable for field engineers under time pressure.
//
// DESIGN INVARIANTS:
//   - Header-only.  No .cpp file.  All functions are inline.
//   - Stateless.    interpret() is a pure function of its inputs.
//   - No heap.      All text is static string literals. No sprintf in hot path.
//   - No exceptions. Every path is noexcept.
//   - O(1).         interpret() runs in constant time regardless of duration.
//   - Safe inputs.  NaN, out-of-range, and contradictory states are handled
//                   explicitly. No undefined behavior is reachable.
//
// INTEGRATION:
//   Called once per sample, after s7_stage.process(), before any logging.
//
// =============================================================================

#pragma once

#include <cstdint>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <cstring>
#include "types.hpp"
// Forward-declare the decision type so interpretation_layer.hpp
// can use it without creating a circular include.
// The full definition is in stages/drift_stabilizer.hpp.
// SSOT Rule: Interpretation reads ONLY DecisionResult for drift state.
namespace signalfix { struct StabilizerResult; }

namespace signalfix {

// =============================================================================
// SeverityLevel — How serious is the active failure?
// =============================================================================

enum class SeverityLevel : uint8_t
{
    INFO     = 0u,  ///< Nominal. No action required.
    LOW      = 1u,  ///< Log and monitor. No immediate action.
    MEDIUM   = 2u,  ///< Investigate at next scheduled opportunity.
    HIGH     = 3u,  ///< Immediate attention required. Potential sensor degradation.
    CRITICAL = 4u,  ///< System safety may be affected. Escalate immediately.
};

// =============================================================================
// StabilityClass — How long has the failure been continuously present?
// =============================================================================

enum class StabilityClass : uint8_t
{
    NORMAL     = 0u,  ///< No active failure. Stream nominal.
    TRANSIENT  = 1u,  ///< Failure appeared recently. May self-resolve.
    PERSISTENT = 2u,  ///< Failure sustained past threshold. Investigation warranted.
};

// =============================================================================
// ConfidenceLevel — Human-scale mapping of S7's smoothed confidence float.
// =============================================================================

enum class ConfidenceLevel : uint8_t
{
    LOW    = 0u,  ///< raw < 0.40.  Statistically weak. Treat with skepticism.
    MEDIUM = 1u,  ///< raw 0.40–0.69. Moderate evidence. Monitor.
    HIGH   = 2u,  ///< raw ≥ 0.70.  Strong statistical evidence. Act.
};

// =============================================================================
// InterpretationConfig — Configurable thresholds.
// =============================================================================

struct InterpretationConfig
{
    uint32_t persistent_threshold_samples = 10u;
};

// =============================================================================
// InterpretationReport — Complete diagnostic output for one sample.
// =============================================================================

struct InterpretationReport
{
    FailureMode      failure_type;
    SeverityLevel    severity;
    StabilityClass   stability;
    ConfidenceLevel  confidence_level;

    const char*      title;
    const char*      summary;
    const char*      diagnostic;

    float            raw_confidence;
    uint32_t         duration_samples;

    bool             input_valid;
    bool             state_coherent;
    uint8_t          _pad[2]; // Padding for 48 byte alignment
};

static_assert(sizeof(InterpretationReport) == 48u,
    "InterpretationReport layout mismatch. Verify 64-bit pointer size and padding.");

namespace interp_detail {

[[nodiscard]] constexpr const char*
to_cstr(SeverityLevel s) noexcept
{
    switch (s)
    {
        case SeverityLevel::INFO:     return "INFO    ";
        case SeverityLevel::LOW:      return "LOW     ";
        case SeverityLevel::MEDIUM:   return "MEDIUM  ";
        case SeverityLevel::HIGH:     return "HIGH    ";
        case SeverityLevel::CRITICAL: return "CRITICAL";
        default:                      return "UNKNOWN ";
    }
}

[[nodiscard]] constexpr const char*
to_cstr(StabilityClass s) noexcept
{
    switch (s)
    {
        case StabilityClass::NORMAL:     return "NORMAL    ";
        case StabilityClass::TRANSIENT:  return "TRANSIENT ";
        case StabilityClass::PERSISTENT: return "PERSISTENT";
        default:                         return "UNKNOWN   ";
    }
}

[[nodiscard]] constexpr const char*
to_cstr(ConfidenceLevel c) noexcept
{
    switch (c)
    {
        case ConfidenceLevel::LOW:    return "LOW ";
        case ConfidenceLevel::MEDIUM: return "MED ";
        case ConfidenceLevel::HIGH:   return "HIGH";
        default:                      return "??? ";
    }
}

struct TextRecord
{
    const char* title;
    const char* summary;
    const char* diagnostic;
};

// compute_severity: For DRIFT, severity is passed in via the decision_severity
// parameter (already bounded by DriftStabilizer). raw gsigma is FORBIDDEN here.
// For all other failure modes, the existing duration/confidence logic applies.
[[nodiscard]] inline SeverityLevel
compute_severity(const FailureMode  mode,
                 const uint32_t     duration,
                 const float        confidence,
                 const float        decision_severity = 0.0f) noexcept
{
    switch (mode)
    {
        case FailureMode::NONE:
            return SeverityLevel::INFO;

        case FailureMode::INVALID:
            return SeverityLevel::CRITICAL;

        case FailureMode::SPIKE:
            if (duration >= 10u)    return SeverityLevel::CRITICAL;
            if (duration >=  3u)    return SeverityLevel::HIGH;
            if (confidence >= 0.7f) return SeverityLevel::HIGH;
            if (confidence >= 0.4f) return SeverityLevel::MEDIUM;
            return                         SeverityLevel::LOW;

        case FailureMode::DRIFT:
            // SSOT RULE: Severity for DRIFT is computed from the Decision Layer's
            // bounded severity score, NOT from raw env.drift_gsigma.
            // This guarantees only confirmed, stabilizer-gated drift can escalate.
            if (decision_severity >= 18.0f) return SeverityLevel::CRITICAL;
            if (decision_severity >= 10.0f) return SeverityLevel::HIGH;
            if (decision_severity >=  5.0f) return SeverityLevel::MEDIUM;
            return                                 SeverityLevel::LOW;

        case FailureMode::STALE:
            if (duration > 15u) return SeverityLevel::CRITICAL;
            if (duration >  3u) return SeverityLevel::HIGH;
            return                     SeverityLevel::MEDIUM;

        case FailureMode::GAP:
            if (duration > 5u) return SeverityLevel::MEDIUM;
            return                    SeverityLevel::LOW;

        default:
            return SeverityLevel::CRITICAL;
    }
}

[[nodiscard]] inline StabilityClass
compute_stability(const FailureMode mode,
                  const uint32_t    duration,
                  const uint32_t    threshold) noexcept
{
    if (mode == FailureMode::NONE)    return StabilityClass::NORMAL;
    if (mode == FailureMode::INVALID) return StabilityClass::PERSISTENT;

    // For TRANSIENT -> PERSISTENT transition
    if (duration >= threshold)        return StabilityClass::PERSISTENT;
    return                                   StabilityClass::TRANSIENT;
}

[[nodiscard]] inline ConfidenceLevel
map_confidence(const float c) noexcept
{
    if (c >= 0.7f) return ConfidenceLevel::HIGH;
    if (c >= 0.4f) return ConfidenceLevel::MEDIUM;
    return                ConfidenceLevel::LOW;
}

[[nodiscard]] inline TextRecord
lookup_text(const FailureMode  mode,
            const SeverityLevel severity) noexcept
{
    switch (mode)
    {
        case FailureMode::NONE:
            return { "Signal Nominal", "Sensor operating within all configured bounds.", "All stage checks passed. No anomaly detected." };

        case FailureMode::INVALID:
            return { "Invalid Sensor Reading", "Non-physical value received. All downstream estimates are unreliable.", "Value is mathematically or physically undefined. Pipeline is operating in safe mode." };

        case FailureMode::DRIFT:
            switch (severity)
            {
                case SeverityLevel::LOW:
                    return { "Early Drift Hint", "Minor deviation detected. Monitor for persistence.", "Slow deviation may indicate calibration bias, thermal expansion, or sensor aging." };
                case SeverityLevel::MEDIUM:
                    return { "Sensor Drift Detected", "Sustained deviation from baseline. Review sensor calibration.", "Slow deviation may indicate calibration bias, thermal expansion, or sensor aging." };
                case SeverityLevel::HIGH:
                    return { "Confirmed Sensor Drift", "Significant persistent drift. Recalibration likely required.", "Long-duration drift indicates sensor aging, bias accumulation, or calibration expiry." };
                case SeverityLevel::CRITICAL:
                    return { "Critical Sensor Drift", "Prolonged drift detected. Immediate inspection required. Sensor is likely degraded.", "Long-duration drift indicates sensor aging, bias accumulation, or calibration expiry." };
                default: break;
            }
            break;

        case FailureMode::SPIKE:
            switch (severity)
            {
                case SeverityLevel::LOW:
                    return { "Transient Signal Spike", "Brief non-physical impulse. Likely electrical noise.", "Rate-of-change exceeded physical limit. Likely EMI, connector bounce, or single-sample artifact." };
                case SeverityLevel::MEDIUM:
                    return { "Signal Spike Detected", "Rate-of-change exceeded physical threshold. Verify mechanical state.", "Rate-of-change exceeded physical limit. Likely EMI, connector bounce, or single-sample artifact." };
                case SeverityLevel::HIGH:
                    return { "Significant Signal Spike", "Strong non-physical impulse. Check for impact event or sensor fault.", "Sudden multi-sample ROC violation may indicate mechanical impact, sensor damage, or bus noise burst." };
                case SeverityLevel::CRITICAL:
                    return { "Sustained Signal Spike", "Extended spike event. Sensor stuck or major physical event. Halt and inspect.", "Spike persisting beyond expected duration suggests sensor saturation, stuck output, or severe mechanical event." };
                default: break;
            }
            break;

        case FailureMode::STALE:
            switch (severity)
            {
                case SeverityLevel::MEDIUM:
                    return { "Signal Gap Detected", "Short loss of sensor data. Communication link suspect.", "No new samples received within the expected window. Check sensor and communication bus." };
                case SeverityLevel::HIGH:
                    return { "Extended Signal Loss", "Sensor data absent for multiple frames. Check physical connection.", "No new samples received within the expected window. Check sensor and communication bus." };
                case SeverityLevel::CRITICAL:
                    return { "Sensor Offline", "Prolonged absence. Sensor presumed failed. Do not rely on filter predictions.", "Sustained data loss indicates sensor hardware failure or bus disconnection." };
                default: break;
            }
            break;

        case FailureMode::GAP:
            switch (severity)
            {
                case SeverityLevel::LOW:
                    return { "Inter-Arrival Gap", "Timing anomaly in sample delivery. Monitor for recurrence.", "Sample inter-arrival time exceeded the expected interval. Likely scheduling jitter or bus contention." };
                case SeverityLevel::MEDIUM:
                    return { "Repeated Timing Gaps", "Multiple inter-arrival anomalies. Communication bus is under stress.", "Sample inter-arrival time exceeded the expected interval. Likely scheduling jitter or bus contention." };
                default: break;
            }
            break;

        default:
            break;
    }

    return { "Unknown Failure Condition", "Unrecognised failure mode. Pipeline state may be corrupted.", "No diagnostic information available. Treat as CRITICAL. Inspect pipeline state and sensor hardware immediately." };
}

} // namespace interp_detail

// =============================================================================
// to_string() — Public string accessors.
// =============================================================================

[[nodiscard]] constexpr const char* to_string(SeverityLevel  s) noexcept { return interp_detail::to_cstr(s); }
[[nodiscard]] constexpr const char* to_string(StabilityClass s) noexcept { return interp_detail::to_cstr(s); }
[[nodiscard]] constexpr const char* to_string(ConfidenceLevel c) noexcept { return interp_detail::to_cstr(c); }

// =============================================================================
// interpret() — Primary API.
// =============================================================================

// =============================================================================
// interpret() — Hardware/Timing Faults only (SPIKE, STALE, GAP, INVALID)
// =============================================================================
// SSOT RULE: This overload is FORBIDDEN from classifying drift.
// It reads env for hardware failure modes only. Drift classification is handled
// exclusively by the decision-aware overload below.
[[nodiscard]] inline InterpretationReport
interpret(const MeasurementEnvelope& env,
          const InterpretationConfig& cfg = InterpretationConfig{}) noexcept
{
    InterpretationReport r{};

    // ── Duration Overflow Guard ──────────────────────────────────────────────
    const uint32_t kMaxSafeDuration = 999999u;
    uint32_t safe_duration = env.failure_duration;
    if (safe_duration > kMaxSafeDuration) { safe_duration = kMaxSafeDuration; }

    // ── Step 1: Failure Type — Hardware faults ONLY ──────────────────────────
    // SSOT: We NEVER read env.status DRIFT_EXCEEDED or env.drift_gsigma here.
    // Drift can only be injected by the decision-aware overload.
    // If the envelope's hint says DRIFT (from a previous pipeline pass or legacy
    // code), we silently suppress it. Hardware-only modes are passed through.
    if (env.failure_hint == FailureMode::DRIFT) {
        r.failure_type = FailureMode::NONE; // Suppressed — not our authority.
    } else {
        r.failure_type = env.failure_hint;
    }

    r.raw_confidence   = env.failure_confidence;
    r.duration_samples = safe_duration;
    r.input_valid      = true;
    r.state_coherent   = true;

    // ── Step 2: Input Trust Hardening (NaN / Inf / Range) ─────────────────────
    const bool conf_is_nan = std::isnan(env.failure_confidence);
    const bool conf_is_inf = std::isinf(env.failure_confidence);
    const bool out_of_range = (env.failure_confidence < 0.0f || env.failure_confidence > 1.0f);

    if (conf_is_nan || conf_is_inf || out_of_range)
    {
        r.input_valid      = false;
        r.raw_confidence   = 0.0f;
        r.confidence_level = ConfidenceLevel::LOW;
        if (conf_is_inf || out_of_range) { r.failure_type = FailureMode::INVALID; }
    }
    else
    {
        r.confidence_level = interp_detail::map_confidence(env.failure_confidence);
    }

    // ── Step 3: Invariant Enforcement ────────────────────────────────────────
    if (r.failure_type == FailureMode::NONE)
    {
        if (r.duration_samples > 0u || r.raw_confidence > 0.05f) {
            r.state_coherent = false;
            r.duration_samples = 0u;
            r.raw_confidence   = 0.0f;
        }
    }

    if (r.failure_type == FailureMode::INVALID)
    {
        r.severity = SeverityLevel::CRITICAL;
        if (r.raw_confidence < 0.99f && !conf_is_nan) { r.state_coherent = false; }
    }

    // ── Step 4: Severity (hardware fault modes only; DRIFT must not appear here)
    if (r.failure_type == FailureMode::INVALID)
    {
        r.severity = SeverityLevel::CRITICAL;
    }
    else
    {
        // decision_severity=0.0f: safe — DRIFT case is unreachable here.
        r.severity = interp_detail::compute_severity(
            r.failure_type, r.duration_samples, r.raw_confidence, 0.0f);
    }

    r.stability = interp_detail::compute_stability(
        r.failure_type, r.duration_samples, cfg.persistent_threshold_samples);

    if (!r.state_coherent)
    {
        r.title      = "State Anomaly Detected";
        r.summary    = "Failure state is internally inconsistent. Diagnostic accuracy is reduced.";
        r.diagnostic = "Pipeline stages may be emitting contradictory outputs.";
    }
    else
    {
        const interp_detail::TextRecord text =
            interp_detail::lookup_text(r.failure_type, r.severity);
        r.title      = text.title;
        r.summary    = text.summary;
        r.diagnostic = text.diagnostic;
    }

    return r;
}

// =============================================================================
// interpret() — Decision-Aware Overload (SSOT path for drift)
// =============================================================================
// This is the AUTHORITATIVE overload used in the main pipeline loop.
// It receives the DecisionResult from the DriftStabilizer and uses it
// as the SOLE source of drift classification.
// Hardware faults from env are still interpreted separately and merged.
//
// Usage:
//   auto interp = interpret(env, cfg, stabilizer_result);
//
// NOTE: The StabilizerResult forward declaration at the top of this header
// is sufficient. The full type is resolved at the call site in the .cpp.
// We must template on the Decision type to avoid a circular include.
template <typename TDecision>
[[nodiscard]] inline InterpretationReport
interpret(const MeasurementEnvelope& env,
          const InterpretationConfig& cfg,
          const TDecision& decision) noexcept
{
    // ── Step 1: Interpret hardware faults from env (non-drift only) ───────────
    InterpretationReport r = interpret(env, cfg);

    // ── Step 2: SSOT Drift Injection ─────────────────────────────────────────
    // If the Decision Authority confirms drift, we upgrade the report.
    // If not, we guarantee drift is suppressed regardless of what env says.
    // The decision.output field is the ONLY permitted source of drift truth.
    const bool authority_says_drift =
        (decision.output == decltype(decision.output)(1)); // 1 == DRIFT_CONFIRMED

    if (authority_says_drift)
    {
        // Override: the authority has spoken. This is now a drift report.
        r.failure_type    = FailureMode::DRIFT;
        r.raw_confidence  = decision.severity / 100.0f; // Normalise severity to [0,1] range
        if (r.raw_confidence > 1.0f) r.raw_confidence = 1.0f;
        r.confidence_level = interp_detail::map_confidence(r.raw_confidence);
        r.duration_samples = static_cast<uint32_t>(decision.persist_timer);
        r.input_valid      = true;
        r.state_coherent   = true;

        // Severity is driven by the stabilizer's bounded severity score.
        r.severity = interp_detail::compute_severity(
            FailureMode::DRIFT, 0u, 0.0f, decision.severity);
        r.stability = interp_detail::compute_stability(
            FailureMode::DRIFT, r.duration_samples, cfg.persistent_threshold_samples);

        const interp_detail::TextRecord text =
            interp_detail::lookup_text(FailureMode::DRIFT, r.severity);
        r.title      = text.title;
        r.summary    = text.summary;
        r.diagnostic = text.diagnostic;
    }
    // If !authority_says_drift: r already has the hardware-only result from above.
    // Any residual DRIFT in failure_hint was already silently suppressed in interpret(env).

    return r;
}

// =============================================================================
// ActionHint — Context-Aware Engineer Advisories
// =============================================================================

enum class ActionHint : uint8_t
{
    NONE                 = 0u,  ///< INFO     — System nominal.
    MONITOR              = 1u,  ///< LOW      — Watch; log to telemetry.
    INVESTIGATE          = 2u,  ///< MEDIUM   — Review at next opportunity.
    URGENT_CHECK         = 3u,  ///< HIGH     — Immediate attention required.
    RECALIBRATE_REQUIRED = 4u,  ///< DRIFT    — Escalated persistent drift.
    STOP_INSPECT         = 5u,  ///< CRITICAL — Halt operation. Inspect now.
};

[[nodiscard]] constexpr const char*
severity_marker(const SeverityLevel s) noexcept
{
    switch (s)
    {
        case SeverityLevel::INFO:     return "     ";
        case SeverityLevel::LOW:      return "[.]  ";
        case SeverityLevel::MEDIUM:   return "[!]  ";
        case SeverityLevel::HIGH:     return "[!!] ";
        case SeverityLevel::CRITICAL: return "[!!!]";
        default:                      return "[!!!]";
    }
}

[[nodiscard]] constexpr const char*
failure_mode_to_cstr(const FailureMode m) noexcept
{
    switch (m)
    {
        case FailureMode::NONE:    return "NOMINAL";
        case FailureMode::DRIFT:   return "DRIFT  ";
        case FailureMode::GAP:     return "GAP    ";
        case FailureMode::STALE:   return "STALE  ";
        case FailureMode::SPIKE:   return "SPIKE  ";
        case FailureMode::INVALID: return "INVALID";
        default:                   return "UNKNOWN";
    }
}

[[nodiscard]] constexpr ActionHint
derive_action_hint(const SeverityLevel   s,
                   const FailureMode     mode,
                   const uint32_t        duration,
                   const ConfidenceLevel conf) noexcept
{
    // ── Context-Aware Target Escalation ───────────────────────────────────────
    if (mode == FailureMode::DRIFT)
    {
        if (duration >= 20u) return ActionHint::RECALIBRATE_REQUIRED;
        if (duration >= 10u) return ActionHint::INVESTIGATE;
        return                      ActionHint::MONITOR;
    }
    if (mode == FailureMode::SPIKE && conf == ConfidenceLevel::HIGH)
    {
        return ActionHint::URGENT_CHECK;
    }

    // ── Standard Default Escalation ───────────────────────────────────────────
    switch (s)
    {
        case SeverityLevel::INFO:     return ActionHint::NONE;
        case SeverityLevel::LOW:      return ActionHint::MONITOR;
        case SeverityLevel::MEDIUM:   return ActionHint::INVESTIGATE;
        case SeverityLevel::HIGH:     return ActionHint::URGENT_CHECK;
        case SeverityLevel::CRITICAL: return ActionHint::STOP_INSPECT;
        default:                      return ActionHint::STOP_INSPECT;
    }
}

[[nodiscard]] constexpr const char*
to_string(const ActionHint a) noexcept
{
    switch (a)
    {
        case ActionHint::NONE:                 return "No action";
        case ActionHint::MONITOR:              return "Monitor";
        case ActionHint::INVESTIGATE:          return "Monitor";
        case ActionHint::URGENT_CHECK:         return "Schedule maintenance soon";
        case ActionHint::RECALIBRATE_REQUIRED: return "Schedule maintenance soon";
        case ActionHint::STOP_INSPECT:         return "Immediate action required";
        default:                               return "Immediate action required";
    }
}

// =============================================================================
// State Tracking & Deterministic Edge Detection
// =============================================================================

[[nodiscard]] inline bool
check_and_set_milestone(const uint32_t curr_dur, uint8_t& flags) noexcept
{
    // Prevent re-triggering if duration fluctuates via single-shot flags
    if (curr_dur >= 10u && !(flags & 0x01)) { flags |= 0x01; return true; }
    if (curr_dur >= 20u && !(flags & 0x02)) { flags |= 0x02; return true; }
    if (curr_dur >= 50u && !(flags & 0x04)) { flags |= 0x04; return true; }
    if (curr_dur >= 100u&& !(flags & 0x08)) { flags |= 0x08; return true; }
    if (curr_dur >= 200u&& !(flags & 0x10)) { flags |= 0x10; return true; }
    return false;
}

struct InterpretationPrintState
{
    FailureMode    last_failure_type  = FailureMode::NONE;
    SeverityLevel  last_severity      = SeverityLevel::INFO;
    StabilityClass last_stability     = StabilityClass::NORMAL;
    uint32_t       last_printed_dur   = 0u;  
    
    // Episode lifecycle flags
    uint8_t        milestone_flags    = 0u;        ///< bitmask [10, 20, 50, 100, 200]
    bool           persistent_reached = false;     ///< latched TRANSIENT->PERSISTENT edge
    bool           initialized        = false;
    
    // Safety peak-latching (Prevents Action/Severity downgrades within an episode)
    SeverityLevel  peak_severity      = SeverityLevel::INFO;
    ActionHint     peak_action        = ActionHint::NONE;

    uint8_t        _pad[1]            = {};
};

// =============================================================================
// log_interpretation_smart() — Change-driven monitoring output.
// =============================================================================

inline void
log_interpretation_smart(const int                   sample_index,
                         const InterpretationReport& r,
                         InterpretationPrintState&   state) noexcept
{
    // ── 1. Episode Start & Global Guards ──────────────────────────────────────
    const bool was_active_failure = state.initialized && (state.last_failure_type != FailureMode::NONE);

    // ── 2. Recovery Handling (Episode End) ────────────────────────────────────
    if (r.failure_type == FailureMode::NONE)
    {
        if (was_active_failure)
        {
            std::printf(
                "  [%3d]  [NOMINAL] Signal restored. Stream operating normally.\n",
                sample_index
            );
            
            // Formal Reset of Episode tracking state
            state.last_failure_type  = FailureMode::NONE;
            state.last_severity      = SeverityLevel::INFO;
            state.last_stability     = StabilityClass::NORMAL;
            state.last_printed_dur   = 0u;
            state.milestone_flags    = 0u;
            state.persistent_reached = false;
            state.peak_severity      = SeverityLevel::INFO;
            state.peak_action        = ActionHint::NONE;
        }
        else if (!state.initialized)
        {
            // Initial boot silent-guard or diagnostic block
            std::printf("  [%3d]  [NOMINAL] Monitoring engine initialised. Stream is healthy.\n", sample_index);
            state.initialized = true;
        }
        return; 
    }

    // ── 3. Escalation Tracking (Episode Progression) ──────────────────────────

    const bool type_changed = (r.failure_type != state.last_failure_type);
    
    // Severity must strictly increase to trigger an escalation log within an episode.
    const bool severity_increased = (static_cast<uint8_t>(r.severity) > static_cast<uint8_t>(state.last_severity));
    
    // Stability escalation triggers exactly once per episode.
    bool stability_escalated = (r.stability == StabilityClass::PERSISTENT && !state.persistent_reached);
    
    // Milestone bitmask ensures O(1) single-shot trigger.
    const bool at_milestone   = check_and_set_milestone(r.duration_samples, state.milestone_flags);

    // Context-Aware Escalation Message Selection
    const char* esc_msg = nullptr;
    if (type_changed)            esc_msg = "Failure mode changed.";
    else if (stability_escalated) esc_msg = "Stability escalated to PERSISTENT.";
    else if (severity_increased)  esc_msg = "Severity increased.";
    else if (at_milestone)        {
        if (r.duration_samples >= 50u)      esc_msg = "Duration reached 50 samples.";
        else if (r.duration_samples >= 20u) esc_msg = "Duration reached 20 samples.";
        else                                esc_msg = "Duration reached 10 samples.";
    }

    const bool print_full = !state.initialized || type_changed || stability_escalated || severity_increased || at_milestone;

    if (!print_full) {
        return; // Zero-Noise Guarantee
    }

    // ── 4. Peak Latching ──────────────────────────────────────────────────────
    if (static_cast<uint8_t>(r.severity) > static_cast<uint8_t>(state.peak_severity)) {
        state.peak_severity = r.severity;
    }
    
    ActionHint current_action = derive_action_hint(r.severity, r.failure_type, r.duration_samples, r.confidence_level);
    if (static_cast<uint8_t>(current_action) > static_cast<uint8_t>(state.peak_action)) {
        state.peak_action = current_action;
    }

    // ── 5. Output Commit ──────────────────────────────────────────────────────
    if (stability_escalated) state.persistent_reached = true;
    
    state.last_failure_type = r.failure_type;
    state.last_severity     = r.severity;
    state.last_stability    = r.stability;
    state.last_printed_dur  = r.duration_samples;
    state.initialized       = true;

    const char* flags_str = "";
    if (!r.input_valid && !r.state_coherent) flags_str = "  [!INPUT+STATE]";
    else if (!r.input_valid)                 flags_str = "  [!INPUT]";
    else if (!r.state_coherent)              flags_str = "  [!STATE]";

    std::printf(
        "  [%3d]  [%s][%-8s][%-10s] %s%s\n",
        sample_index, failure_mode_to_cstr(r.failure_type), interp_detail::to_cstr(r.severity),
        interp_detail::to_cstr(r.stability), severity_marker(r.severity), flags_str
    );

    if (esc_msg) {
        std::printf("          >> %s\n", esc_msg);
    }

    std::printf("          %s\n", r.summary);
    std::printf("          dur=%-3u | conf=%s(%.2f)\n", r.duration_samples, interp_detail::to_cstr(r.confidence_level), static_cast<double>(r.raw_confidence));
    std::printf("          Cause:  %s\n", r.diagnostic);
    std::printf("          Action: %s\n", to_string(state.peak_action));
}

// =============================================================================
// Trend Indicator — Qualitative trajectory of the anomaly
// =============================================================================

struct ChannelTrendState {
    static constexpr int kWindowSize = 5;
    float buffer[kWindowSize] = {0.0f};
    int head = 0;
    int count = 0;

    // Inertia/Persistence
    int pending_rising_count = 0;
    int pending_falling_count = 0;
    const char* confirmed_trend = "Stable";
};

inline void update_trend_state(ChannelTrendState& state, float gsigma) noexcept {
    state.buffer[state.head] = gsigma;
    state.head = (state.head + 1) % ChannelTrendState::kWindowSize;
    if (state.count < ChannelTrendState::kWindowSize) state.count++;
}

[[nodiscard]] inline const char* compute_trend_label(ChannelTrendState& state, bool in_drift, float current_gsigma) noexcept {
    // Rule 1: Not in drift -> Always Stable
    if (!in_drift) {
        state.confirmed_trend = "Stable";
        state.pending_rising_count = 0;
        state.pending_falling_count = 0;
        return "Stable";
    }

    // Rule 2: Minimum history required (buffer must be full)
    if (state.count < ChannelTrendState::kWindowSize) {
        return "Stable";
    }

    // Rule 3: Smoothing. Compare newest 2 vs oldest 2 in the 5-sample window.
    // indices relative to head: (head-1, head-2) vs (head-4, head-5)
    auto get_idx = [](int head, int offset) {
        return (head + ChannelTrendState::kWindowSize + offset) % ChannelTrendState::kWindowSize;
    };

    float avg_new = (state.buffer[get_idx(state.head, -1)] + state.buffer[get_idx(state.head, -2)]) / 2.0f;
    float avg_old = (state.buffer[get_idx(state.head, -4)] + state.buffer[get_idx(state.head, -5)]) / 2.0f;
    float delta = avg_new - avg_old;

    // Rule 4: Adaptive Deadband
    float threshold = std::max(0.01f, 0.02f * current_gsigma);

    // Rule 5: Inertia (3 consecutive confirmations)
    if (delta > threshold) {
        state.pending_rising_count++;
        state.pending_falling_count = 0;
        if (state.pending_rising_count >= 3) {
            state.confirmed_trend = "Rising";
        }
    } else if (delta < -threshold) {
        state.pending_falling_count++;
        state.pending_rising_count = 0;
        if (state.pending_falling_count >= 3) {
            state.confirmed_trend = "Falling";
        }
    } else {
        state.pending_rising_count = 0;
        state.pending_falling_count = 0;
        state.confirmed_trend = "Stable";
    }

    // Rule 6: Conservative Safety Mask (No falling during active drift)
    if (std::strcmp(state.confirmed_trend, "Falling") == 0) {
        return "Stable";
    }

    return state.confirmed_trend;
}

// =============================================================================
// Drift Severity Label — Maps internal SeverityLevel to customer-facing string
// =============================================================================

[[nodiscard]] constexpr const char*
drift_severity_label(const SeverityLevel s) noexcept
{
    switch (s)
    {
        case SeverityLevel::INFO:     return "Normal operation";
        case SeverityLevel::LOW:      return "Early deviation detected";
        case SeverityLevel::MEDIUM:   return "Significant drift detected";
        case SeverityLevel::HIGH:     return "Significant drift detected";
        case SeverityLevel::CRITICAL: return "Severe degradation detected";
        default:                      return "Status unknown";
    }
}

// ── Decision Matrix for Drift Recommendations ──────────────────────────────
[[nodiscard]] inline const char*
generate_drift_recommendation(SeverityLevel s,
                            const char*   trend,
                            float         confidence,
                            uint32_t      duration) noexcept
{
    const bool is_rising  = (std::strcmp(trend, "Rising") == 0);
    const bool is_falling = (std::strcmp(trend, "Falling") == 0);

    // 1. CONFIDENCE GATE: LOW (Strictly non-actionable)
    if (confidence < 0.15f) {
        if (s == SeverityLevel::CRITICAL) {
            return "Strong signal forming. Monitoring closely to confirm severity.";
        }
        return "Early signal detected. Monitoring to confirm persistence.";
    }

    // 2. TREND OVERRIDE: FALLING (Do not clear fault)
    if (is_falling) {
        return "Trend is falling but severity remains high. Continue monitoring to confirm recovery.";
    }

    // 3. DURATION GATE: MEDIUM CONFIDENCE (< 10 samples)
    if (confidence < 0.65f && duration < 10u) {
        return "Signal is developing. Continue monitoring to confirm progression.";
    }

    // 4. SEVERITY DOMINANCE: CRITICAL (High/Medium Confidence)
    if (s == SeverityLevel::CRITICAL) {
        return "Severe degradation confirmed. Immediate inspection required.";
    }

    // 5. HIGH SEVERITY MATRIX
    if (s >= SeverityLevel::MEDIUM) {
        if (confidence >= 0.65f) {
            if (is_rising) return "Significant drift is worsening. Maintenance should be scheduled soon.";
            return "Significant drift confirmed. Evaluate recalibration at next opportunity.";
        } else {
            // Medium confidence (Duration >= 10 handled by step 3)
            if (is_rising) return "Trend is actively worsening. Maintenance may be required if condition persists.";
            return "Condition sustained. Action may be required at next opportunity.";
        }
    }

    // 6. WARNING/INFO (Low tier)
    if (confidence >= 0.65f) return "Early deviation confirmed. Evaluate sensor health during next cycle.";
    return "Early deviation detected. Monitoring to confirm progression.";
}

// =============================================================================
// print_final_drift_report() — Mandated 34-dash diagnostic block output.
// Only prints when system_in_drift is true (DRIFT_EXCEEDED bitmask set).
// =============================================================================

// print_final_drift_report — Fires only on DRIFT_STARTED events.
// SSOT: The guard is now performed at the call site by checking
// decision.event == DRIFT_STARTED. This function no longer re-reads
// env.status to determine whether drift is active.
inline void
print_final_drift_report(const InterpretationReport& r,
                         const char*                 sensor_name,
                         const float                 drift_confidence,
                         const char*                 trend_label) noexcept
{
    // Guard: caller must ensure this is only called during a DRIFT_STARTED event.

    const char* severity_str = drift_severity_label(r.severity);
    const char* recommendation = generate_drift_recommendation(
        r.severity, trend_label, drift_confidence, r.duration_samples);

    const char* display_severity = severity_str;
    const char* display_trend    = trend_label;

    // ── Early-Phase Consistency Override ─────────────────────────────────────
    // During emergence (duration < 10), we communicate uncertainty and state-alignment.
    if (r.duration_samples < 10u) {
        display_severity = "Drift signal detected (developing)";
        display_trend    = "Developing";
    }

    // Enhanced Confidence Wording
    const char* conf_context = "N/A";
    if (drift_confidence < 0.15f)      conf_context = "Low (early signal)";
    else if (drift_confidence < 0.65f) conf_context = "Medium (confirmed trend)";
    else                               conf_context = "High (strong persistence)";

    std::printf("----------------------------------\n");
    std::printf("STATUS: DRIFT DETECTED\n");
    std::printf("Sensor: %s\n",        sensor_name);
    std::printf("Severity: %s\n",      display_severity);
    std::printf("Confidence: %s\n",    conf_context);
    std::printf("Trend: %s\n",         display_trend);
    std::printf("----------------------------------\n");
    std::printf("Recommendation:\n");
    std::printf("%s\n",                recommendation);
    std::printf("----------------------------------\n");
}

} // namespace signalfix
