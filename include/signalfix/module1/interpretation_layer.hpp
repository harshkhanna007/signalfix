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
#include "types.hpp"

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

[[nodiscard]] inline SeverityLevel
compute_severity(const FailureMode  mode,
                 const uint32_t     duration,
                 const float        confidence) noexcept
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
            if (duration > 60u)                          return SeverityLevel::CRITICAL;
            if (duration > 20u)                          return SeverityLevel::HIGH;
            if (duration >  5u && confidence >= 0.6f)    return SeverityLevel::HIGH;
            if (duration >  5u)                          return SeverityLevel::MEDIUM;
            return                                              SeverityLevel::LOW;

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

[[nodiscard]] inline InterpretationReport
interpret(const MeasurementEnvelope& env,
          const InterpretationConfig& cfg = InterpretationConfig{}) noexcept
{
    InterpretationReport r{};

    // ── Edge Case Guard: Duration Overflow (SFX-M1-TDS-023) ────────────────────
    // Clamps to 999,999 to prevent 32-bit wrap during long-running streams.
    const uint32_t kMaxSafeDuration = 999999u;
    uint32_t safe_duration = env.failure_duration;
    if (safe_duration > kMaxSafeDuration)
    {
        safe_duration = kMaxSafeDuration;
    }

    // ── Step 1: Copy raw fields with hardening ──────────────────────────────
    r.failure_type     = env.failure_hint;
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
        r.raw_confidence   = 0.0f; // Safe clamp
        r.confidence_level = ConfidenceLevel::LOW;
        if (conf_is_inf || out_of_range) {
            r.failure_type = FailureMode::INVALID; // Escalate non-physical confidence to INVALID
        }
    }
    else
    {
        r.confidence_level = interp_detail::map_confidence(env.failure_confidence);
    }

    // ── Step 3: Invariant Enforcement (Coherent State Resolver) ────────────────
    
    // Invariant: NONE mode MUST NOT have a non-zero duration or high confidence.
    if (r.failure_type == FailureMode::NONE)
    {
        if (r.duration_samples > 0u || r.raw_confidence > 0.05f) {
            r.state_coherent = false;
            r.duration_samples = 0u;
            r.raw_confidence   = 0.0f;
        }
    }

    // Invariant: INVALID mode MUST be CRITICAL regardless of other metadata.
    if (r.failure_type == FailureMode::INVALID)
    {
        r.severity = SeverityLevel::CRITICAL;
        if (r.raw_confidence < 0.99f && !conf_is_nan) {
            r.state_coherent = false; // INVALID implies 1.0 trust logically.
        }
    }

    // ── Step 4: Hierarchical Severity Layer (HARD > DATA > SIGNAL) ─────────────
    if (r.failure_type == FailureMode::INVALID)
    {
        r.severity = SeverityLevel::CRITICAL;
    }
    else if (r.failure_type == FailureMode::STALE || r.failure_type == FailureMode::GAP)
    {
        // Data absence/timing is higher priority than statistical signal logic.
        r.severity = interp_detail::compute_severity(r.failure_type, r.duration_samples, r.raw_confidence);
    }
    else
    {
        // Signal logic (DROOP/SPIKE/DRIFT)
        r.severity = interp_detail::compute_severity(r.failure_type, r.duration_samples, r.raw_confidence);
    }

    r.stability = interp_detail::compute_stability(
        r.failure_type, r.duration_samples, cfg.persistent_threshold_samples);

    if (!r.state_coherent)
    {
        r.title      = "State Anomaly Detected";
        r.summary    = "Failure state is internally inconsistent. Diagnostic accuracy is reduced.";
        r.diagnostic = "Pipeline stages may be emitting contradictory outputs. Possible causes: memory corruption, S7 state machine reset, or a pending recovery not yet confirmed.";
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
        case ActionHint::NONE:                 return "NONE";
        case ActionHint::MONITOR:              return "MONITOR";
        case ActionHint::INVESTIGATE:          return "INVESTIGATE";
        case ActionHint::URGENT_CHECK:         return "URGENT_CHECK";
        case ActionHint::RECALIBRATE_REQUIRED: return "RECALIBRATE_REQUIRED";
        case ActionHint::STOP_INSPECT:         return "STOP_AND_INSPECT";
        default:                               return "STOP_AND_INSPECT";
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

} // namespace signalfix
