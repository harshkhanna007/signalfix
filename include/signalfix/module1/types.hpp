// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : signalfix/include/signalfix/module1/types.hpp
// Spec   : SFX-M1-TDS-001  Revision 2.4 (Architectural Repair)
// =============================================================================
//
// Canonical data contract between Module 1 and all downstream modules.
// Revision 2.4: 16-bit status flags and dual timestamp/delta-t support.
//
// Enforcement:
//   static_assert(sizeof(MeasurementEnvelope) == 96)
//
// =============================================================================

#pragma once
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <limits>
#include <type_traits>

namespace signalfix {

// =============================================================================
// 1. ENUMS (MUST BE DECLARED FIRST)
// =============================================================================

// SampleStatus — Bitfield status flags (16-bit)
enum class SampleStatus : uint16_t
{
    NOMINAL         = 0x0000u,  ///< All checks passed. No anomaly detected.
    SOFT_SUSPECT    = 0x0001u,  ///< Calibrated value outside soft bounds.
    ROC_EXCEEDED    = 0x0002u,  ///< Adaptive rate-of-change threshold exceeded.
    TIMING_ANOMALY  = 0x0004u,  ///< Timestamp jitter or ordering anomaly.
    INTERPOLATED    = 0x0008u,  ///< Gap-fill applied. Module 2 MUST NOT update.
    MISSING         = 0x0010u,  ///< No measurement data. Module 2 MUST NOT update.
    HARD_INVALID    = 0x0020u,  ///< Physical bounds violated. Value unusable.
    FILTER_CLIPPED  = 0x0040u,  ///< Pre-filter output clipped to channel range.
    STALE           = 0x0080u,  ///< Watchdog timeout (paired with MISSING).
    RATE_ANOMALY    = 0x0100u,  ///< [Rev 2.2] Deterministic rate-of-change check failure (S5).
    PRE_FILTER_OK   = 0x0200u,  ///< [Rev 2.2] Median filter successfully applied (S5).
    DRIFT_EXCEEDED  = 0x0400u,  ///< [Rev 2.8.6] Slow drift deviation threshold exceeded (g%σ).
};

// MeasurementTrustTier — Ordinal measurement quality classification
enum class MeasurementTrustTier : uint8_t
{
    HIGH      = 0u,  ///< NOMINAL. Full measurement weight.
    TRUSTED   = 0u,  ///< Alias for HIGH (used by S7).
    DEGRADED  = 1u,  ///< Minor anomaly.
    LOW       = 2u,  ///< Multiple anomalies.
    REJECTED  = 3u,  ///< HARD_INVALID | MISSING | INTERPOLATED | STALE.
};

enum class DriftLevel : uint8_t {
    NOISE     = 0,
    BUILDUP   = 1,
    CONFIRMED = 2,
    CRITICAL  = 3,
};

// Classification — Semantic meaning of the measurement outcome
enum class Classification : uint8_t
{
    NORMAL        = 0u,
    SENSOR_GLITCH = 1u,
    REAL_EVENT    = 2u,
    NO_DATA       = 3u,
    DRIFT         = 4u,
};

// FailureMode — Unified failure classification (Rev 2.5)
enum class FailureMode : uint8_t
{
    NONE    = 0u,  ///< Nominal operation.
    DRIFT   = 1u,  ///< Slow, persistent deviation (Soft bounds or ROC crawl).
    GAP     = 2u,  ///< Detected inter-arrival gap (pre-stale).
    STALE   = 3u,  ///< Watchdog timeout/absence.
    SPIKE   = 4u,  ///< Sudden, non-physical impulse (ROC exceeded).
    INVALID = 5u,  ///< Physically impossible value (NaN, Inf, Hard bounds).
};

// =============================================================================
// 2. HELPER FUNCTIONS (Bitwise ops and status checkers)
// =============================================================================

// Bitwise operators — required for composing SampleStatus flags
[[nodiscard]] constexpr inline SampleStatus
operator|(SampleStatus lhs, SampleStatus rhs) noexcept
{
    return static_cast<SampleStatus>(
        static_cast<uint16_t>(lhs) | static_cast<uint16_t>(rhs));
}

[[nodiscard]] constexpr inline SampleStatus
operator&(SampleStatus lhs, SampleStatus rhs) noexcept
{
    return static_cast<SampleStatus>(
        static_cast<uint16_t>(lhs) & static_cast<uint16_t>(rhs));
}

[[nodiscard]] constexpr inline SampleStatus
operator~(SampleStatus s) noexcept
{
    return static_cast<SampleStatus>(~static_cast<uint16_t>(s));
}

inline SampleStatus&
operator|=(SampleStatus& lhs, SampleStatus rhs) noexcept
{
    lhs = lhs | rhs;
    return lhs;
}

inline SampleStatus&
operator&=(SampleStatus& lhs, SampleStatus rhs) noexcept
{
    lhs = lhs & rhs;
    return lhs;
}

[[nodiscard]] constexpr bool
has_flag(SampleStatus status, SampleStatus flag) noexcept
{
    return (static_cast<uint16_t>(status) & static_cast<uint16_t>(flag))
           == static_cast<uint16_t>(flag);
}

[[nodiscard]] constexpr SampleStatus
set_flag(SampleStatus status, SampleStatus flag) noexcept
{
    return status | flag;
}

[[nodiscard]] constexpr SampleStatus
clear_flag(SampleStatus status, SampleStatus flag) noexcept
{
    return status & ~flag;
}

[[nodiscard]] constexpr bool
is_nominal(SampleStatus status) noexcept
{
    return status == SampleStatus::NOMINAL;
}

[[nodiscard]] constexpr bool
requires_prediction_only(SampleStatus status) noexcept
{
    constexpr uint16_t reject_mask =
        static_cast<uint16_t>(SampleStatus::HARD_INVALID) |
        static_cast<uint16_t>(SampleStatus::MISSING)      |
        static_cast<uint16_t>(SampleStatus::INTERPOLATED) |
        static_cast<uint16_t>(SampleStatus::STALE);

    return (static_cast<uint16_t>(status) & reject_mask) != 0u;
}

[[nodiscard]] constexpr MeasurementTrustTier
derive_trust_tier(SampleStatus s) noexcept
{
    constexpr uint16_t reject_mask =
        static_cast<uint16_t>(SampleStatus::HARD_INVALID) |
        static_cast<uint16_t>(SampleStatus::MISSING)      |
        static_cast<uint16_t>(SampleStatus::INTERPOLATED) |
        static_cast<uint16_t>(SampleStatus::STALE);

    if ((static_cast<uint16_t>(s) & reject_mask) != 0u)
    {
        return MeasurementTrustTier::REJECTED;
    }

    if (has_flag(s, SampleStatus::ROC_EXCEEDED))
    {
        return MeasurementTrustTier::LOW;
    }

    // Heuristic: any two minor anomalies -> LOW. One -> DEGRADED.
    uint16_t status_raw = static_cast<uint16_t>(s);
    uint32_t count = 0;
    for (int i = 0; i < 16; ++i) {
        if ((status_raw >> i) & 1u) count++;
    }

    if (count >= 2u) return MeasurementTrustTier::LOW;
    if (count == 1u) return MeasurementTrustTier::DEGRADED;
    return MeasurementTrustTier::HIGH;
}

[[nodiscard]] inline bool
validate_status_flags(SampleStatus s) noexcept
{
    const uint16_t u = static_cast<uint16_t>(s);

    // 1. Categorical Mask Definitions
    // -------------------------------------------------------------------------
    // Absence: data did not arrive or is a placeholder.
    constexpr uint16_t M_ABSENT  = static_cast<uint16_t>(SampleStatus::MISSING) |
                                   static_cast<uint16_t>(SampleStatus::STALE);

    // Corruption: data is physically unusable.
    constexpr uint16_t M_INVALID = static_cast<uint16_t>(SampleStatus::HARD_INVALID);

    // Derived: data is synthetic or processed.
    constexpr uint16_t M_DERIVED = static_cast<uint16_t>(SampleStatus::INTERPOLATED) |
                                   static_cast<uint16_t>(SampleStatus::PRE_FILTER_OK);

    // Quality/Events: specific properties of a valid measurement value.
    constexpr uint16_t M_VALUE_PROPS = static_cast<uint16_t>(SampleStatus::SOFT_SUSPECT)   |
                                       static_cast<uint16_t>(SampleStatus::ROC_EXCEEDED)   |
                                       static_cast<uint16_t>(SampleStatus::FILTER_CLIPPED) |
                                       static_cast<uint16_t>(SampleStatus::RATE_ANOMALY)   |
                                       static_cast<uint16_t>(SampleStatus::DRIFT_EXCEEDED);

    // 2. Invariant Rules
    // -------------------------------------------------------------------------

    // Rule 1: Absence is exclusive with all Value Properties, Corruption, and Derived flags.
    // Metadata flags like TIMING_ANOMALY are permitted on placeholders.
    if ((u & M_ABSENT) != 0u) {
        if ((u & (M_INVALID | M_DERIVED | M_VALUE_PROPS)) != 0u) return false;
    }

    // Rule 2: Corruption (HARD_INVALID) is exclusive with Quality/Event and Derived flags.
    // If data is physically corrupt, its quality or derivative is meaningless.
    if ((u & M_INVALID) != 0u) {
        if ((u & (M_DERIVED | M_VALUE_PROPS)) != 0u) return false;
    }

    // Rule 3: Derived values (Interpolations) must be semantically clean.
    // We do not permit SOFT_SUSPECT or ROC_EXCEEDED on synthetic gap-fills.
    if (has_flag(s, SampleStatus::INTERPOLATED)) {
        constexpr uint16_t M_SUSPECT = static_cast<uint16_t>(SampleStatus::SOFT_SUSPECT) |
                                       static_cast<uint16_t>(SampleStatus::ROC_EXCEEDED);
        if ((u & M_SUSPECT) != 0u) return false;
    }

    return true;
}

// =============================================================================
// 3. STRUCTS
// =============================================================================

// FaultState — Active vs Recovered Tracking (Rev 2.8.5)
struct FaultState {
    bool is_active_fault;                       // true = fault currently active (status=ROC_EXCEEDED)
    bool is_recovering;                         // true = fault detected but now decaying (status=NOMINAL, conf > 0)
    bool recovery_confirmed;                    // true = confidence has decayed to 0.0 AND stayed there for 1+ sample
    uint8_t _padding;                           // alignment
    
    uint32_t samples_since_fault_onset;         // samples elapsed since fault first detected (saturates at UINT32_MAX)
    uint32_t fault_onset_sequence_id;           // sequence_id when fault was first detected
    uint32_t samples_in_recovery;               // how many samples have elapsed since recovery_confirmed=true
    
    float confidence_at_onset;                  // failure_confidence value when ROC_EXCEEDED was first set
    float confidence_current;                   // current failure_confidence (may be decaying)
};

// MeasurementEnvelope — Versioned wire contract (Rev 2.4)
struct MeasurementEnvelope
{
    // ── 64-byte block: eight 8-byte fields (no padding) ──────────────────────

    uint64_t  sequence_id;               ///< Monotonic per-channel counter.
    uint64_t  timestamp_us;              ///< Raw/PLL-estimated timestamp [μs].
    uint64_t  corrected_timestamp_us;    ///< S6 elite-corrected timestamp [μs].
    uint64_t  arrival_time_us;           ///< Raw system clock at ingestion [μs].

    uint64_t  delta_t_us;                ///< Raw/PLL-estimated interval [μs].
    uint64_t  corrected_delta_t_us;      ///< S6 elite-corrected interval [μs].
    double    raw_value;                 ///< Pre-calibration sensor value.
    double    calibrated_value;          ///< Post-calibration physical-unit value.

    // ── 16-byte block: four 4-byte fields ────────────────────────────────────

    uint32_t  channel_id;                ///< Channel identifier.
    int32_t   jitter_us;                 ///< Signed jitter [μs].
    uint32_t  gap_event_id;              ///< Non-zero if gap-filled.
    uint32_t  calibration_version;       ///< Calibration config version.

    // ── 12-byte block ─────────────────────────────────────────────────────────

    uint16_t  nominal_streak_count;      ///< Consecutive NOMINAL samples (S6).
    uint16_t  roc_window_n;              ///< Adaptive ROC window size (S3).
    float     roc;                       ///< Instantaneous rate-of-change.
    float     roc_adaptive_limit;        ///< Adaptive ROC threshold.
    float     roc_threshold_used;        ///< The actual limit used for classification (Fix 8).

    // ── 32-byte block: status, tier, and persistence (Rev 2.6) ──────────────

    SampleStatus          status;                 ///< 16-bit status flags.
    MeasurementTrustTier  measurement_trust_tier; ///< Authoritative R-tier.

    // PUBLIC OUTPUT: Sanitized for external consumers (e.g. Kalman Filter, CSV).
    // Suppressed (NONE/0.0) during recovery phase.
    FailureMode           failure_hint;           ///< Public failure classification.
    FailureMode           internal_failure_hint;  ///< Internal latched FailureMode.
    uint8_t               filter_window_n;        ///< Median window N.
    uint8_t               _pad_align_conf[1];     ///< Alignment padding.
    float                 failure_confidence;     ///< Public smoothed confidence [0.0, 1.0].
    uint32_t              failure_duration;       ///< Public consecutive sample count.
    uint64_t              last_failure_timestamp_us; ///< Time of last observed failure [μs].
    
    // INTERNAL STATE: Preserves raw latched dynamics (including recovery decay) for forensic audit.
    float                 internal_failure_confidence; ///< Latched confidence for S7 engine.
    uint32_t              internal_failure_duration;   ///< Latched duration for S7 engine.
    
    // CONTINUOUS METRICS
    float                 drift_score;                 ///< Continuous drift severity metric [0, 100].
    float                 drift_gsigma;                ///< Raw CUSUM gSigma exported from S5 for severity mapping.
    
    FaultState            fault_state;            ///< Advanced fault tracking (Rev 2.8.5)
};

// --- ABI Stability Guards (Architecture Invariants) ---
static_assert(sizeof(MeasurementEnvelope) <= 160u, "Envelope grew unexpectedly");
static_assert(alignof(MeasurementEnvelope) == 8u, "MeasurementEnvelope alignment check failed.");

// =============================================================================
// 4. FUNCTIONS DEPENDENT ON STRUCTS
// =============================================================================

[[nodiscard]] inline MeasurementEnvelope
make_nominal_envelope() noexcept
{
    MeasurementEnvelope env{};

    constexpr double kNaN_d = std::numeric_limits<double>::quiet_NaN();
    constexpr float  kNaN_f = std::numeric_limits<float>::quiet_NaN();

    env.sequence_id           = 0u;
    env.timestamp_us          = 0u;
    env.corrected_timestamp_us = 0u;
    env.arrival_time_us       = 0u;
    env.delta_t_us            = 0u;
    env.corrected_delta_t_us   = 0u;

    env.raw_value             = kNaN_d;
    env.calibrated_value      = kNaN_d;

    env.channel_id            = 0u;
    env.jitter_us             = 0;
    env.gap_event_id          = 0u;
    env.calibration_version   = 0u;

    env.nominal_streak_count  = 0u;
    env.roc_window_n          = 0u;
    env.roc                   = kNaN_f;
    env.roc_adaptive_limit    = kNaN_f;
    env.roc_threshold_used    = kNaN_f;

    env.status                = SampleStatus::NOMINAL;
    env.measurement_trust_tier= MeasurementTrustTier::HIGH;
    env.filter_window_n       = 0u;
    env.failure_hint          = FailureMode::NONE;
    env.failure_confidence    = 0.0f;
    env.failure_duration      = 0u;
    env.last_failure_timestamp_us = 0u;

    env.internal_failure_hint = FailureMode::NONE;
    env.internal_failure_confidence = 0.0f;
    env.internal_failure_duration = 0u;

    env.drift_score           = 0.0f;
    env.drift_gsigma          = 0.0f;

    env.fault_state.is_active_fault = false;
    env.fault_state.is_recovering = false;
    env.fault_state.recovery_confirmed = false;
    env.fault_state.samples_since_fault_onset = 0u;
    env.fault_state.fault_onset_sequence_id = 0u;
    env.fault_state.samples_in_recovery = 0u;
    env.fault_state.confidence_at_onset = 0.0f;
    env.fault_state.confidence_current = 0.0f;

    return env;
}

inline void upgrade_failure_hint(
    MeasurementEnvelope& env,
    const FailureMode    suggested_hint,
    const float         suggested_confidence) noexcept
{
    // Rule 4: INVALID override (Immediate, zero-hysteresis path)
    if (suggested_hint == FailureMode::INVALID)
    {
        env.failure_hint = FailureMode::INVALID;
        env.failure_confidence = 1.0f;
        return;
    }

    // Rule 2: Hierarchy enforcement
    if (static_cast<uint8_t>(suggested_hint) > static_cast<uint8_t>(env.failure_hint))
    {
        env.failure_hint = suggested_hint;
        env.failure_confidence = suggested_confidence;
    }
}

[[nodiscard]] inline Classification
derive_classification(const MeasurementEnvelope& env) noexcept
{
    if (has_flag(env.status, SampleStatus::HARD_INVALID))
    {
        return Classification::SENSOR_GLITCH;
    }

    if (has_flag(env.status, SampleStatus::STALE) ||
        has_flag(env.status, SampleStatus::MISSING))
    {
        return Classification::NO_DATA;
    }

    if (!std::isfinite(env.calibrated_value))
    {
        return Classification::NO_DATA;
    }

    if (has_flag(env.status, SampleStatus::ROC_EXCEEDED))
    {
        return Classification::REAL_EVENT;
    }

    if (has_flag(env.status, SampleStatus::DRIFT_EXCEEDED))
    {
        return Classification::DRIFT;
    }

    return Classification::NORMAL;
}

} // namespace signalfix
