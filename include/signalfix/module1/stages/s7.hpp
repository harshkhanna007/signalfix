// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/stage_s7_output.hpp
// Spec   : SFX-M1-TDS-001  Revision 1.3  Stage S7
// =============================================================================
//
// Stage S7 — Output Packaging & Integrity Layer
//
// PIPELINE POSITION: final stage; after S6 (Timestamp Correction).
//
// ─── ROLE ───────────────────────────────────────────────────────────────────
//
// S7 is the LAST LINE OF DEFENSE in Module 1.
//
// S7 operates under ONE foundational assumption:
//   UPSTREAM STAGES (S1–S6) MAY HAVE FAILED.
//
// S7 does NOT trust flags alone. S7 validates fields independently.
// S7 maintains its own cross-sample state to verify monotonic timestamp
// continuity independently of S6. S7 applies context-aware quality
// classification that considers field values, not just status flags.
//
// ─── PIPELINE CONTEXT ───────────────────────────────────────────────────────
//
// S7 runs as a registered stage inside the pipeline's stage loop
// (IngestionPipeline::process). After all registered stages complete
// normally, the pipeline calls run_output_bus(), which:
//   1. Calls validate_status_flags() — rejects invalid flag combinations.
//   2. Calls output_bus_fn_()        — writes to the audit ring buffer.
//
// Implications:
//   - S7 ABORT_DROP prevents run_output_bus from being called.
//   - S7 CONTINUE allows run_output_bus to run; it may still GATE_REJECT.
//   - Watchdog-path STALE envelopes bypass the stage loop entirely and go
//     directly to run_output_bus via process_watchdog(). S7 never sees them.
//
// ─── HARD SAFETY GATE — ABORT_DROP CONDITIONS ───────────────────────────────
//
// Each DROP condition indicates a structural invariant violation by an upstream
// stage, a hardware fault, or memory corruption. These are not data quality
// decisions — they are system integrity checks.
//
// DROP-1  Non-finite calibrated_value (non-STALE sample):
//   Trigger: !isfinite(calibrated_value) && !STALE
//   Reason:  S1 must produce finite values. Non-finite without HARD_INVALID
//            means a stage silently failed. Set HARD_INVALID; ABORT_DROP.
//
// DROP-2  Dual corruption: HARD_INVALID + TIMING_ANOMALY:
//   Trigger: HARD_INVALID && TIMING_ANOMALY
//   Reason:  Value AND timestamp are both corrupted. Passing this to a Kalman
//            filter corrupts state more severely than a missing observation.
//            ABORT_DROP. HARD_INVALID already set by upstream.
//
// DROP-3  Zero corrected_timestamp_us (S6 not initialized):
//   Trigger: corrected_timestamp_us == 0
//   Reason:  S6 seeds this on bootstrap. Zero means S6 did not run.
//            Set HARD_INVALID; ABORT_DROP.
//
// DROP-4  Zero corrected_delta_t_us (S6 guarantee violated):
//   Trigger: corrected_delta_t_us == 0
//   Reason:  S6 guarantees ≥ 1 μs. Zero means S6 did not run. dt = 0 causes
//            division-by-zero in the Kalman filter's F = expm(A × dt).
//            Set HARD_INVALID; ABORT_DROP.
//
// DROP-5  Timestamp monotonicity regression (S7 independent check):
//   Trigger: corrected_timestamp_us ≤ last_ts_us for this channel AND
//            last_ts_us < UINT64_MAX (UINT64_MAX saturation is legitimate).
//   Reason:  S7 maintains its own per-channel last-seen timestamp, independent
//            of S6's internal state. S6 guarantees monotonicity; if this check
//            fires, S6's guarantee was violated — either a bug in S6, a memory
//            fault overwriting corrected_timestamp_us, or channel state
//            corruption. This is the ONLY check that can catch S6 internal
//            failures that still produce non-zero timestamps.
//            Set HARD_INVALID; ABORT_DROP. Do NOT update last_ts_us.
//   Note:    last_ts_us is initialized to 0. The condition
//            corrected_timestamp_us ≤ 0 is already caught by DROP-3.
//            Therefore DROP-3 must be evaluated BEFORE DROP-5.
//
// DROP-6  Extreme dt upper bound (S6 contract violation):
//   Trigger: corrected_delta_t_us > kAbsoluteMaxDtMultiplier * nominal_delta_t_us
//   Reason:  S6's max_time_jump_us config default is 8 × nominal. The S7
//            threshold is 10 × nominal — a 25% safety margin above S6's hard
//            cap. A dt above this means S6's own maximum gap limit was violated.
//            This indicates either S6 misconfiguration, a bug in handle_long_gap,
//            or memory corruption of corrected_delta_t_us.
//            Set HARD_INVALID; ABORT_DROP.
//
// DROP-7  Gap + timing dual corruption: MISSING + TIMING_ANOMALY:
//   Trigger: MISSING && TIMING_ANOMALY
//   Reason:  MISSING means S4 confirmed the measurement slot is absent.
//            TIMING_ANOMALY means the timestamp for this absent slot is also
//            corrupted. The Kalman filter's pure-prediction step for a gap
//            requires knowing the gap duration. With corrupted timing, even
//            the prediction step cannot be performed safely.
//            ABORT_DROP. HARD_INVALID already set or being set here.
//
// ─── QUALITY CLASSIFICATION — CONTEXT-AWARE ──────────────────────────────────
//
// Quality is classified after the drop gate. It is a function of:
//   (SampleStatus flags, corrected_delta_t_us, nominal_delta_t_us)
//
// Rules are evaluated in strict priority order. First match wins.
//
//   Priority 1 → BAD
//     (a) STALE:   No real measurement data.
//     (b) MISSING: Gap confirmed; measurement absent.
//     (c) HARD_INVALID: Definitively unphysical value.
//     (d) TIMING_ANOMALY + dt > 4 × nominal:
//         Timestamp is uncertain AND the gap exceeds the normal operating
//         window. The time coordinate is unreliable at both ends; the sample
//         cannot be safely timestamped for the filter.
//     (e) RATE_ANOMALY + dt > 4 × nominal:
//         Rate anomaly combined with a large gap means we cannot distinguish
//         genuine fast signal change from the artifact of accumulated drift
//         over multiple missed samples.
//
//   Priority 2 → DEGRADED
//     (f) TIMING_ANOMALY (alone, dt ≤ 4 × nominal):
//         Timestamp uncertainty within the normal dt window. Value likely valid;
//         increase time covariance in Kalman filter.
//     (g) RATE_ANOMALY (alone, dt ≤ 4 × nominal):
//         Statistically suspicious rate; value may be real. Increase
//         measurement covariance. Do not discard: a genuine fast change
//         rejected here causes filter divergence.
//     (h) TIMING_ANOMALY + RATE_ANOMALY (both, dt ≤ 4 × nominal):
//         Both covariance adjustments apply; still delivers value to filter.
//     (i) dt < nominal / 4 (no bad flags): "GOOD but broken" defense.
//         S6 clamps output to [min_dt_us, max_dt_us]. If dt < nominal/4,
//         S6's lower bound configuration may be wrong, or the field was
//         written incorrectly. The value itself may be fine; the timing is
//         suspect. DEGRADED is the conservative response — value is delivered
//         but the downstream must inflate dt uncertainty.
//
//   Priority 3 → GOOD
//     No anomaly flags, dt within expected range, timing consistent.
//
// ─── SYSTEM HEALTH AWARENESS ────────────────────────────────────────────────
//
// After quality classification, S7 checks cumulative system health.
// If either condition is true:
//   drop_count_ / total_processed_samples > kDropRateThresholdPct  (5%)
//   bad_count_  / total_processed_samples > kBadRateThresholdPct   (50%)
// AND at least kHealthMinSamples have been processed,
// THEN quality GOOD is demoted to DEGRADED.
//
// Rationale:
//   A sustained drop rate > 5% means structural violations are occurring
//   frequently. Even samples that look clean should be treated with caution:
//   the pipeline is demonstrably malfunctioning.
//   A sustained bad rate > 50% means the sensor is delivering more unusable
//   than usable data. Nominally clean samples during a period of deep fault
//   may represent recovery artifacts rather than genuine good data.
//
// This check uses only existing cumulative counters and is O(1) with no
// additional state. It is applied only to GOOD (demote to DEGRADED), never
// to DEGRADED or BAD (no further demotion needed since the downstream already
// inflates covariance or ignores).
//
// ─── INTERNAL CROSS-SAMPLE STATE ─────────────────────────────────────────────
//
// Rev 1.1 adds per-channel timestamp tracking for DROP-5.
// This is the ONLY cross-sample state in S7.
//
//   ts_records_[kMaxTrackedChannels]: per-channel {channel_id, last_ts_us}
//   ts_record_count_: number of occupied slots
//
// S7 NEVER writes to ChannelState. Per-channel tracking is in S7's own table.
// reset() clears this table so that after a pipeline restart, the first sample
// from each channel is accepted regardless of pre-reset timestamps.
//
// ─── WHAT S7 WRITES TO THE ENVELOPE ─────────────────────────────────────────
//
//   Writes: envelope.measurement_trust_tier (based on quality classification).
//   Writes: envelope.status | HARD_INVALID   (on DROP-1, DROP-3, DROP-4,
//                                              DROP-5, DROP-6, DROP-7 where
//                                              HARD_INVALID not already set).
//   Never clears any SampleStatus flag set by a prior stage.
//   Never modifies any other envelope field.
//
// ─── STAGE RESULT POLICY ────────────────────────────────────────────────────
//
//   ABORT_DROP  — DROP-1..7. Pipeline records DROPPED; run_output_bus skipped.
//   CONTINUE    — GOOD, DEGRADED, BAD. run_output_bus is called.
//   ABORT_FAULT — never returned by S7.
//
// ─── GUARANTEES ─────────────────────────────────────────────────────────────
//
//   G1.  ProcessedOutputFn is NEVER called with a non-finite value.
//   G2.  ProcessedOutputFn is NEVER called with dt ≤ 0.0f.
//   G3.  ProcessedOutputFn is NEVER called with timestamp_us == 0.
//   G4.  ProcessedOutputFn is NEVER called with quality == BAD.
//   G5.  ABORT_DROP iff DROP-1..7 is detected.
//   G6.  S7 never clears any SampleStatus flag set by a prior stage.
//   G7.  S7 never modifies ChannelState.
//   G8.  S7 output is deterministic for identical (envelope, internal_state).
//   G9.  No heap allocation, no exceptions, no system calls.
//   G10. ProcessedOutputFn is NEVER called if corrected_timestamp_us ≤ any
//        prior corrected_timestamp_us for the same channel (per S7's own table).
//   G11. Quality classification uses dt context: TIMING_ANOMALY or RATE_ANOMALY
//        combined with dt > 4 × nominal always yields BAD, never DEGRADED.
//   G12. A cumulative drop rate > 5% or bad rate > 50% demotes GOOD → DEGRADED.
//
// =============================================================================

#pragma once

#include "signalfix/module1/stage_interface.hpp"
#include "signalfix/module1/channel_state.hpp"
#include "signalfix/module1/output_types.hpp"

#include <cstdint>
#include <limits>

namespace signalfix {

// ---------------------------------------------------------------------------
// Hard Gate Layer Definitions
// ---------------------------------------------------------------------------
enum class HardGateFailReason : uint8_t {
    NONE = 0,
    INVALID_DATA,
    TIMING_ANOMALY,
    UNSTABLE_CONTEXT,
    INSUFFICIENT_HISTORY,
    NUMERICAL_FAILURE
};

struct HardGateResult {
    bool pass;
    HardGateFailReason reason;
};

// ---------------------------------------------------------------------------
// StageS7Output — Output Packaging & Integrity Layer (Rev 1.3)
// ---------------------------------------------------------------------------
class StageS7Output final : public IStage
{
public:
    /// Construct S7 with a mandatory downstream callback and optional context.
    ///
    /// processed_fn:
    ///   Called for every sample that passes the hard safety gate AND has
    ///   quality ∈ {GOOD, DEGRADED}. Must not be null.
    ///   If null, S7 is non-operational: every process() call returns ABORT_DROP.
    ///
    /// processed_ctx:
    ///   Opaque pointer forwarded to processed_fn. May be null.
    explicit StageS7Output(
        ProcessedOutputFn processed_fn,
        void* processed_ctx) noexcept;

    ~StageS7Output() noexcept override = default;

    // ── IStage interface ─────────────────────────────────────────────────────

    [[nodiscard]] StageResult process(
        MeasurementEnvelope& envelope,
        ChannelState&        channel_state) noexcept override;

    [[nodiscard]] const char* stage_name() const noexcept override;

    /// Resets S7's per-channel timestamp tracking table and escalation streaks.
    /// Diagnostic counters are NOT reset (they are post-boot system health metrics).
    void reset() noexcept override;

    // ── Diagnostic accessors (non-hot-path) ──────────────────────────────────

    [[nodiscard]] uint64_t drop_count()            const noexcept;
    [[nodiscard]] uint64_t consumer_reject_count() const noexcept;
    [[nodiscard]] uint64_t good_count()            const noexcept;
    [[nodiscard]] uint64_t degraded_count()        const noexcept;
    [[nodiscard]] uint64_t bad_count()             const noexcept;

private:
    // ── Injected downstream callback ──────────────────────────────────────────

    ProcessedOutputFn processed_fn_;
    void* processed_ctx_;

    // ── Diagnostic & Escalation counters ──────────────────────────────────────

    uint64_t drop_count_;
    uint64_t consumer_reject_count_;
    uint64_t good_count_;
    uint64_t degraded_count_;
    uint64_t bad_count_;

    uint32_t consecutive_consumer_rejects_;
    uint32_t consecutive_drops_;

    static constexpr uint32_t kMaxConsecutiveRejects   = 100u;
    static constexpr uint32_t kMaxConsecutiveDrops     = 100u;
    static constexpr uint64_t kMaxNominalDtUs          = 100'000'000u; // 100 seconds max safe delta

    // ── Safe Math Helpers ─────────────────────────────────────────────────────
    
    static constexpr void sat_inc(uint64_t& val) noexcept {
        if (val < std::numeric_limits<uint64_t>::max()) { ++val; }
    }
    
    static constexpr void sat_inc_32(uint32_t& val) noexcept {
        if (val < std::numeric_limits<uint32_t>::max()) { ++val; }
    }
    
    static constexpr uint64_t sat_add_u64(uint64_t a, uint64_t b) noexcept {
        return (std::numeric_limits<uint64_t>::max() - a >= b) 
               ? (a + b) 
               : std::numeric_limits<uint64_t>::max();
    }
    
    static constexpr bool safe_mul_u64(uint64_t a, uint64_t b, uint64_t& out) noexcept {
        if (a == 0u || b == 0u) { out = 0u; return true; }
        if (a > std::numeric_limits<uint64_t>::max() / b) { return false; }
        out = a * b;
        return true;
    }

    // ── Per-channel timestamp tracking (DROP-5) ───────────────────────────────

    static constexpr uint32_t kMaxTrackedChannels = 64u;
    static constexpr uint32_t kEmptyChannelId     = 0xFFFF'FFFFu;
    static constexpr uint32_t kSlotNotFound       = 0xFFFF'FFFFu;

    struct PerChannelTsRecord
    {
        uint32_t channel_id;
        uint32_t _pad;
        uint64_t last_ts_us;
    };
    static_assert(sizeof(PerChannelTsRecord) == 16u,
        "PerChannelTsRecord layout changed.");

    PerChannelTsRecord ts_records_[kMaxTrackedChannels];
    uint32_t           ts_record_count_;

    [[nodiscard]] uint32_t find_or_register_channel(uint32_t channel_id) noexcept;

    // ── Internal logic ────────────────────────────────────────────────────────

    [[nodiscard]] bool evaluate_drop_gate(
        MeasurementEnvelope&      envelope,
        bool                      is_stale,
        uint64_t                  nominal_delta_t_us,
        const PerChannelTsRecord* ts_record) noexcept;

    [[nodiscard]] HardGateResult hard_gate_pass(
        const MeasurementEnvelope& envelope,
        const ChannelState&        channel_state) const noexcept;

    static void suppress_drift(
        MeasurementEnvelope& envelope,
        ChannelState&        channel_state,
        HardGateFailReason   reason) noexcept;

    [[nodiscard]] static SampleQuality classify_quality(
        SampleStatus status,
        uint64_t     corrected_delta_t_us,
        uint64_t     nominal_delta_t_us) noexcept;

    [[nodiscard]] SampleQuality apply_health_check(SampleQuality quality) const noexcept;

    [[nodiscard]] static MeasurementTrustTier quality_to_trust_tier(
        SampleQuality quality) noexcept;

    [[nodiscard]] static ProcessedSample package_sample(
        const MeasurementEnvelope& envelope,
        SampleQuality              quality) noexcept;

    static void set_hard_invalid(MeasurementEnvelope& envelope) noexcept;

    [[nodiscard]] static bool has_flag(
        SampleStatus status,
        SampleStatus flag) noexcept;
};

} // namespace signalfix