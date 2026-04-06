// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : src/module1/stage_s7_output.cpp
// Spec   : SFX-M1-TDS-001  Revision 1.3  Stage S7
// =============================================================================
//
// Stage S7 — Output Packaging & Integrity Layer: hardened implementation.
//
// COMPILER REQUIREMENTS:
//   -fno-exceptions  — no throw anywhere.
//   -fno-rtti        — no dynamic_cast.
//   IEEE 754 double  — std::isfinite() operates on exponent bits; no arithmetic.
//   C++17 or later   — [[nodiscard]], inline constexpr.
//
// ─── REVISION 1.1 CHANGES OVER 1.0 ──────────────────────────────────────────
//
// FIX-R1  [CRITICAL] Monotonic timestamp defense (DROP-5).
//         Rev 1.0 trusted S6's monotonicity guarantee blindly. S6 guarantees
//         corrected_timestamp_us is non-decreasing, but S7 had no independent
//         check. If S6 produced a non-monotonic timestamp (memory corruption,
//         S6 bug), Rev 1.0 would silently pass a regressed timestamp to the
//         Kalman filter, causing state estimate discontinuity.
//         Fix: S7 maintains ts_records_[], a per-channel {channel_id, last_ts_us}
//         table. DROP-5 fires if corrected_timestamp_us ≤ last_ts_us for the
//         same channel (with an exception for UINT64_MAX saturation, which is
//         S6's legitimate overflow behavior). DROP-3 (ts == 0) is checked BEFORE
//         DROP-5 to avoid comparing against an uninitialized last_ts_us.
//         reset() clears the table to prevent false positives after restart.
//
// FIX-R2  [SERIOUS] Extreme dt upper bound check (DROP-6).
//         Rev 1.0 only checked dt == 0. An absurdly large dt (e.g., due to
//         memory corruption writing UINT64_MAX to corrected_delta_t_us) would
//         produce a ProcessedSample.dt of ~5.85e8 seconds, causing the Kalman
//         filter's state propagation to produce NaN or ±Inf.
//         Fix: DROP-6 fires if corrected_delta_t_us > 10 × nominal_delta_t_us.
//         S6's default max_time_jump_us is 8 × nominal; 10 × is a 25% safety
//         margin. Any dt above this is outside S6's operating envelope.
//
// FIX-R3  [SERIOUS] Multi-condition drop: MISSING + TIMING_ANOMALY (DROP-7).
//         Rev 1.0 classified MISSING as BAD (CONTINUE). But MISSING +
//         TIMING_ANOMALY means the gap slot has uncertain timing — even the
//         Kalman filter's pure prediction step (which uses dt for state
//         propagation) cannot be performed reliably. This combination is
//         more dangerous than either flag alone.
//         Fix: DROP-7 rejects MISSING + TIMING_ANOMALY unconditionally.
//
// FIX-R4  [MODERATE] Context-aware quality classification.
//         Rev 1.0 classify_quality() was a pure function of SampleStatus only.
//         It classified TIMING_ANOMALY and RATE_ANOMALY as DEGRADED regardless
//         of dt. But TIMING_ANOMALY + large dt (> 4 × nominal) means: the
//         timestamp is uncertain AND we are in SHORT_GAP territory. The filter
//         cannot reliably determine WHEN this observation occurred. Similarly,
//         RATE_ANOMALY + large dt means the apparent rate may be a multi-sample
//         artifact rather than genuine fast signal change.
//         Fix: classify_quality now takes (status, corrected_dt_us, nominal_dt_us).
//         TIMING_ANOMALY or RATE_ANOMALY + dt > 4 × nominal → BAD, not DEGRADED.
//
// FIX-R5  [MODERATE] "GOOD but broken" defense.
//         Rev 1.0 had no check for a sample with no bad flags but physically
//         suspect timing. If S6's min_dt_us is misconfigured (e.g., set to 1 μs
//         when nominal is 10ms), S6 might clamp dt to 1 μs without setting any
//         flag. A Kalman filter receiving dt = 0.000001 s would propagate state
//         almost zero time per step, causing catastrophic lag.
//         Fix: if dt < nominal / 4 and no bad flags → DEGRADED. This is a
//         secondary S6-config sanity check: S6 should set min_dt_us = nominal/4
//         by the config factory default, so dt < nominal/4 is anomalous.
//
// FIX-R6  [MODERATE] System health awareness.
//         Rev 1.0 evaluated each sample in isolation. If the pipeline was in a
//         sustained fault state (repeated drops or bad samples), Rev 1.0 would
//         still classify nominally-clean samples as GOOD.
//         Fix: apply_health_check() demotes GOOD → DEGRADED if cumulative
//         drop_count_ / total > 5% OR bad_count_ / total > 50% (after ≥ 100
//         samples). This is O(1) using existing counters. Downstream consumers
//         receive a signal that even clean samples occurred during a fault epoch.
//
// FIX-R7  [STRUCTURAL] Strict no-trust-upstream assumption.
//         Rev 1.0 implicitly trusted S6's output format. All critical fields
//         are now independently validated by value, not by flag:
//           - corrected_timestamp_us: checked for zero (DROP-3) and regression
//             against S7's own record (DROP-5).
//           - corrected_delta_t_us: checked for zero (DROP-4) and extreme upper
//             bound (DROP-6).
//           - calibrated_value: checked for non-finite (DROP-1).
//           - dt context: used in quality classification (FIX-R4).
//         Flags are still checked (DROP-2, DROP-7) but are not the sole defense.
//
// =============================================================================

#include "signalfix/module1/stages/s7.hpp"
#include "signalfix/module1/types.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <cstring>

namespace signalfix {

// =============================================================================
// File-scope constants
// =============================================================================

namespace {

/// DROP-6: dt > this multiplier × nominal → S6 operating envelope violated.
/// S6 default max_time_jump_us = 8 × nominal. Using 10 × as a 25% safety margin.
constexpr uint64_t kAbsoluteMaxDtMultiplier = 10u;

/// FIX-R4: TIMING_ANOMALY or RATE_ANOMALY + dt above this threshold → BAD.
/// 4 × nominal matches S6's default max_dt_us boundary (above this = SHORT_GAP).
/// TIMING_ANOMALY at SHORT_GAP scale means the time coordinate is unreliable
/// in both the immediate neighborhood AND over a multi-sample window.
constexpr uint64_t kLargeDtThresholdMultiplier = 4u;

/// FIX-R5: dt below this fraction of nominal → DEGRADED (no flags required).
/// Matches S6's default min_dt_us = nominal / 4. dt < nominal/4 means S6's
/// lower clamp was bypassed (misconfiguration or memory fault).
constexpr uint64_t kMinDtFraction = 4u;

/// FIX-R6: minimum sample count before health check is applied.
/// Prevents false early alarms during pipeline startup.
constexpr uint64_t kHealthMinSamples = 100u;

/// FIX-R6: if drop_count_ / total > this percent → demote GOOD to DEGRADED.
constexpr uint64_t kDropRateThresholdPct = 5u;

/// FIX-R6: if bad_count_ / total > this percent → demote GOOD to DEGRADED.
/// High threshold (50%) because bad_count_ includes expected STALE events.
constexpr uint64_t kBadRateThresholdPct = 50u;

} // namespace


// =============================================================================
// Construction
// =============================================================================
//
// If processed_fn is null, S7 is in fail-safe non-operational mode.
// All process() calls will return ABORT_DROP with HARD_INVALID set.
// This is the correct behavior for a misconfigured safety boundary.
//
// =============================================================================

StageS7Output::StageS7Output(
    ProcessedOutputFn processed_fn,
    void* processed_ctx) noexcept
    : processed_fn_                (processed_fn)
    , processed_ctx_               (processed_ctx)
    , drop_count_                  (0u)
    , consumer_reject_count_       (0u)
    , good_count_                  (0u)
    , degraded_count_              (0u)
    , bad_count_                   (0u)
    , consecutive_consumer_rejects_(0u)
    , consecutive_drops_           (0u)
    , ts_record_count_             (0u)
{
    // Zero-initialize the per-channel timestamp table.
    // channel_id = kEmptyChannelId is a valid initialized empty state.
    //
    // last_ts_us = 0 for all entries. This is the correct initial state: 
    // any positive corrected_timestamp_us is accepted as the first sample.
    std::memset(ts_records_, 0xFF, sizeof(ts_records_));

    for (uint32_t i = 0u; i < kMaxTrackedChannels; ++i)
    {
        ts_records_[i].last_ts_us = 0u;
    }
}


// =============================================================================
// IStage interface
// =============================================================================

const char* StageS7Output::stage_name() const noexcept
{
    return "S7-OutputPackager";
}

void StageS7Output::reset() noexcept
{
    // FIX-R1: Clear the per-channel timestamp tracking table.
    std::memset(ts_records_, 0xFF, sizeof(ts_records_));
    for (uint32_t i = 0u; i < kMaxTrackedChannels; ++i)
    {
        ts_records_[i].last_ts_us = 0u;
    }
    ts_record_count_ = 0u;

    consecutive_consumer_rejects_ = 0u;
    consecutive_drops_            = 0u;
}

uint64_t StageS7Output::drop_count()            const noexcept { return drop_count_;            }
uint64_t StageS7Output::consumer_reject_count() const noexcept { return consumer_reject_count_; }
uint64_t StageS7Output::good_count()            const noexcept { return good_count_;            }
uint64_t StageS7Output::degraded_count()        const noexcept { return degraded_count_;        }
uint64_t StageS7Output::bad_count()             const noexcept { return bad_count_;             }


// =============================================================================
// process() — pipeline hot path
// =============================================================================

StageResult StageS7Output::process(
    MeasurementEnvelope& envelope,
    ChannelState&        channel_state) noexcept
{
    const Classification classification = signalfix::derive_classification(envelope);
    (void)classification;

    // ── Step 0: Null-callback guard ───────────────────────────────────────────

    if (processed_fn_ == nullptr)
    {
        set_hard_invalid(envelope);
        sat_inc(drop_count_);
        sat_inc_32(consecutive_drops_);
        return StageResult::ABORT_DROP;
    }

    // ── Channel ID and Configuration Validation ───────────────────────────────

    if (channel_state.channel_id == kEmptyChannelId ||
        channel_state.nominal_delta_t_us == 0u ||
        channel_state.nominal_delta_t_us > kMaxNominalDtUs)
    {
        set_hard_invalid(envelope);
        sat_inc(drop_count_);
        sat_inc_32(consecutive_drops_);
        return StageResult::ABORT_DROP;
    }

    // ── Consumer Failure Escalation Check ─────────────────────────────────────

    if (consecutive_consumer_rejects_ >= kMaxConsecutiveRejects)
    {
        set_hard_invalid(envelope);
        sat_inc(drop_count_);
        sat_inc_32(consecutive_drops_);
        return StageResult::ABORT_DROP;
    }

    // ── Step 1: Pre-compute is_stale ─────────────────────────────────────────

    const bool is_stale = has_flag(envelope.status, SampleStatus::STALE);

    // ── Hierarchy Enforcement (S7) ───────────────────────────────────────────
    if (is_stale)
    {
        upgrade_failure_hint(envelope, FailureMode::STALE, 1.0f);
    }

    // ── Step 2: Locate per-channel timestamp record ───────────────────────────

    const uint32_t ts_slot = find_or_register_channel(channel_state.channel_id);
    
    // Channel Table Overflow Defense: Monotonic protection must never be disabled.
    if (ts_slot == kSlotNotFound)
    {
        set_hard_invalid(envelope);
        sat_inc(drop_count_);
        sat_inc_32(consecutive_drops_);
        return StageResult::ABORT_DROP;
    }
    
    const PerChannelTsRecord* ts_record = &ts_records_[ts_slot];

    // ── Step 3: Hard safety gate ──────────────────────────────────────────────

    const uint64_t nominal_dt = channel_state.nominal_delta_t_us;

    if (evaluate_drop_gate(envelope, is_stale, nominal_dt, ts_record))
    {
        sat_inc_32(consecutive_drops_);
        if (consecutive_drops_ >= kMaxConsecutiveDrops)
        {
            // Drop-storm escalation: aggressively assert HARD_INVALID if not already.
            set_hard_invalid(envelope);
        }
        return StageResult::ABORT_DROP;
    }

    // Valid sample survived drop gate, reset drop streak
    consecutive_drops_ = 0u;

    // ── Step 4: Context-aware quality classification ──────────────────────────

    SampleQuality quality = classify_quality(
        envelope.status,
        envelope.corrected_delta_t_us,
        channel_state.nominal_delta_t_us);

    // ── Step 5: System health check ───────────────────────────────────────────

    quality = apply_health_check(quality);

    // ── Step 6: Set measurement_trust_tier ───────────────────────────────────

    envelope.measurement_trust_tier = quality_to_trust_tier(quality);

    // ── Step 6b: Semantic classification ──────────────────────────────────────
    // (Moved to top of process() function)

    // ── Step 7: Update per-channel timestamp record ───────────────────────────

    ts_records_[ts_slot].last_ts_us = envelope.corrected_timestamp_us;

    // ── HARD GATE LAYER (Safety Boundary) ─────────────────────────────────────
    
    const auto gate = hard_gate_pass(envelope, channel_state);

    if (!gate.pass) {
        // block drift logic entirely
        suppress_drift(envelope, channel_state, gate.reason);
        return StageResult::CONTINUE;
    }

    // ── Failure Persistence State Machine (Rev 2.7 — Production Grade) ─────────
    //
    // Design invariants:
    //   • One and only one code path owns every write to fs.* per frame.
    //   • No dynamic allocation, no exceptions, no non-determinism.
    //   • All counters are bounded (saturating arithmetic) to prevent overflow.
    //   • Severity ordering: NONE(0) < DRIFT(1) < GAP(2) < STALE(3) < SPIKE(4) < INVALID(5)
    //     (matches the uint8_t values of FailureMode enum — numerical comparison is valid).

    auto& fs = channel_state.failure_state;

    // Snapshot the hint/confidence written by upstream stages (S3–S6).
    // We must read BEFORE any write so CASE B's same-mode check is correct.
    const FailureMode suggested      = envelope.failure_hint;
    const float       suggested_conf = envelope.failure_confidence;

    // Hysteresis constants — tuned for a 1 kHz channel:
    //   ENTER_THRESHOLD = 3  → failure must persist 3 consecutive frames before latching
    //   EXIT_THRESHOLD  = 5  → recovery must persist 5 consecutive frames before clearing
    // Prevents single-frame glitches from toggling the output state.
    constexpr uint8_t kEnterThreshold = 3u;
    constexpr uint8_t kExitThreshold  = 5u;

    // Confidence smoothing weight for the incoming sample (EMA, α = 0.3).
    // Balance: 0.3 is responsive enough for rapid escalation but stable enough
    // to reject single-frame noise.
    constexpr float kAlpha = 0.3f;

    // Pre-compute hard-invalid conditions once; reused in CASE A guard.
    // We check both the status flag (set by S3/S4) AND the raw value (defensive
    // against a stage that wrote NaN/Inf but forgot to set HARD_INVALID).
    const bool is_hard_invalid  = has_flag(envelope.status, SampleStatus::HARD_INVALID);
    const bool is_invalid_value = !std::isfinite(envelope.calibrated_value);

    // ────────────────────────────────────────────────────────────────────────
    // CASE A  —  HARD FAILURE OVERRIDE (zero hysteresis, immediate latch)
    //
    // Triggered by: NaN/Inf value, HARD_INVALID flag, or explicit INVALID hint.
    // Any one condition is sufficient — physical data is unusable regardless.
    // Overrides ALL other logic; enter/exit counters are cleared so that as
    // soon as the underlying condition is gone, the normal hysteresis path
    // takes over cleanly.
    // ────────────────────────────────────────────────────────────────────────
    if (suggested == FailureMode::INVALID || is_invalid_value || is_hard_invalid)
    {
        fs.latched_hint       = FailureMode::INVALID;
        fs.latched_confidence = 1.0f;

        // Saturating increment — duration never wraps to 0.
        if (fs.duration_frames < 0xFFFFu)
            fs.duration_frames++;

        // Clear counters so neither path carries stale accumulation forward.
        fs.enter_counter = 0u;
        fs.exit_counter  = 0u;

        fs.last_failure_ts_us = envelope.arrival_time_us;
    }

    // ────────────────────────────────────────────────────────────────────────
    // CASE B  —  FAILURE DETECTED (suggested != NONE, not hard-invalid)
    // ────────────────────────────────────────────────────────────────────────
    else if (suggested != FailureMode::NONE)
    {
        if (suggested == fs.latched_hint)
        {
            // (B-1) Same failure mode continues.
            // Extend duration; reset exit counter so a brief NONE blip never
            // falsely starts the recovery countdown.
            if (fs.duration_frames < 0xFFFFu)
                fs.duration_frames++;

            fs.enter_counter = 0u;  // No entry transition needed.
            fs.exit_counter  = 0u;  // Active failure; cancel any recovery count.

            // EMA confidence smoothing: stable over time, responsive to change.
            fs.latched_confidence =
                (0.7f * fs.latched_confidence) + (kAlpha * suggested_conf);

            // Timestamp of most recent failure observation.
            fs.last_failure_ts_us = envelope.arrival_time_us;
        }
        else
        {
            // (B-2) Different failure mode detected.
            const bool higher_severity =
                static_cast<uint8_t>(suggested) > static_cast<uint8_t>(fs.latched_hint);

            if (higher_severity)
            {
                // Immediate switch for escalation — a more severe condition must
                // never be delayed by hysteresis (safety requirement).
                fs.latched_hint       = suggested;
                fs.latched_confidence = suggested_conf;
                fs.duration_frames    = 1u;

                fs.enter_counter = 0u;
                fs.exit_counter  = 0u;

                fs.last_failure_ts_us = envelope.arrival_time_us;
            }
            else
            {
                // Lower-severity candidate: require ENTER_THRESHOLD consecutive
                // frames before switching to prevent upward oscillation.
                // Saturating increment prevents uint8_t overflow.
                if (fs.enter_counter < 0xFFu)
                    fs.enter_counter++;

                if (fs.enter_counter >= kEnterThreshold)
                {
                    // Hysteresis satisfied — commit the transition.
                    fs.latched_hint       = suggested;
                    fs.latched_confidence = suggested_conf;
                    fs.duration_frames    = 1u;

                    fs.enter_counter = 0u;
                    fs.exit_counter  = 0u;

                    fs.last_failure_ts_us = envelope.arrival_time_us;
                }
                // else: accumulating — latched_hint, duration, confidence unchanged.
            }
        }
    }

    // ────────────────────────────────────────────────────────────────────────
    // CASE C  —  RECOVERY (suggested == NONE)
    //
    // A single NONE frame does NOT immediately clear the latched failure.
    // EXIT_THRESHOLD consecutive NONE frames are required. This is the key
    // mechanism that eliminates the DRIFT→NONE→DRIFT flickering pattern.
    // ────────────────────────────────────────────────────────────────────────
    else
    {
        if (fs.latched_hint != FailureMode::NONE)
        {
            // Still in a failure state — accumulate recovery evidence.
            // Saturating increment prevents uint8_t overflow.
            if (fs.exit_counter < 0xFFu)
                fs.exit_counter++;

            if (fs.exit_counter >= kExitThreshold)
            {
                // Recovery confirmed: exit the failure state cleanly.
                fs.latched_hint       = FailureMode::NONE;
                fs.latched_confidence = 0.0f;
                fs.duration_frames    = 0u;

                fs.enter_counter = 0u;
                fs.exit_counter  = 0u;
            }
            else
            {
                // Still within the hold window — failure remains latched.
                // Duration keeps incrementing (failure still "active").
                if (fs.duration_frames < 0xFFFFu)
                    fs.duration_frames++;

                // Confidence decays toward zero during recovery window so
                // downstream consumers see a gradual withdrawal of certainty,
                // not an abrupt flip.
                fs.latched_confidence *= 0.95f;
            }
        }
        else
        {
            // Already fully nominal — keep counters clean.
            // This branch executes on every non-failure frame after full recovery.
            fs.duration_frames    = 0u;
            fs.enter_counter      = 0u;
            fs.exit_counter       = 0u;
            fs.latched_confidence = 0.0f;
        }
    }

    // ────────────────────────────────────────────────────────────────────────
    // SAFETY CLAMPS
    // Guard against floating-point rounding drift over long-running streams.
    // ────────────────────────────────────────────────────────────────────────
    if (fs.latched_confidence > 1.0f) fs.latched_confidence = 1.0f;
    if (fs.latched_confidence < 0.0f) fs.latched_confidence = 0.0f;

    // ────────────────────────────────────────────────────────────────────────
    // OUTPUT — single authoritative write-back to the envelope
    // Downstream stages/modules read ONLY from the envelope, never from fs.
    // ────────────────────────────────────────────────────────────────────────
    envelope.failure_hint              = fs.latched_hint;
    envelope.failure_confidence        = fs.latched_confidence;
    envelope.failure_duration          = static_cast<uint32_t>(fs.duration_frames);
    envelope.last_failure_timestamp_us = fs.last_failure_ts_us;

    // =========================================================================
    // STRICT CONFIDENCE GATING: Final safety boundary against noise
    // =========================================================================

    // Floating-point drift clamp (defensive boundary)
    if (!std::isfinite(envelope.failure_confidence)) {
        envelope.failure_confidence = 0.0f;
    } else {
        if (envelope.failure_confidence > 1.0f) envelope.failure_confidence = 1.0f;
        if (envelope.failure_confidence < 0.0f) envelope.failure_confidence = 0.0f;
    }

    constexpr float CONFIDENCE_FLOOR_DRIFT = 0.50f;
    constexpr float CONFIDENCE_FLOOR_SPIKE = 0.70f;

    if (envelope.failure_hint == FailureMode::DRIFT &&
        envelope.failure_confidence < CONFIDENCE_FLOOR_DRIFT)
    {
        envelope.failure_hint = FailureMode::NONE;
        envelope.failure_confidence = 0.0f;
    }

    if (envelope.failure_hint == FailureMode::SPIKE &&
        envelope.failure_confidence < CONFIDENCE_FLOOR_SPIKE)
    {
        envelope.failure_hint = FailureMode::NONE;
        envelope.failure_confidence = 0.0f;
    }

    // ── Step 8: Conditional control callback ─────────────────────────────────

    if (quality != SampleQuality::BAD)
    {
        const ProcessedSample pkg = package_sample(envelope, quality);

        const bool accepted = processed_fn_(pkg, processed_ctx_);

        if (!accepted)
        {
            sat_inc(consumer_reject_count_);
            sat_inc_32(consecutive_consumer_rejects_);
        }
        else
        {
            consecutive_consumer_rejects_ = 0u;
        }

        if (quality == SampleQuality::GOOD)
        {
            sat_inc(good_count_);
        }
        else
        {
            sat_inc(degraded_count_);
        }
    }
    else
    {
        sat_inc(bad_count_);
    }

    // ── Step 9: Return CONTINUE ───────────────────────────────────────────────

    return StageResult::CONTINUE;
}


// =============================================================================
// find_or_register_channel()
// =============================================================================

uint32_t StageS7Output::find_or_register_channel(
    const uint32_t channel_id) noexcept
{
    for (uint32_t i = 0u; i < ts_record_count_; ++i)
    {
        if (ts_records_[i].channel_id == channel_id)
        {
            return i;
        }
    }

    if (ts_record_count_ >= kMaxTrackedChannels)
    {
        return kSlotNotFound;
    }

    const uint32_t slot          = ts_record_count_;
    ts_records_[slot].channel_id = channel_id;
    ts_records_[slot].last_ts_us = 0u; 
    ++ts_record_count_;

    return slot;
}


// =============================================================================
// evaluate_drop_gate()
// =============================================================================

bool StageS7Output::evaluate_drop_gate(
    MeasurementEnvelope&      envelope,
    const bool                is_stale,
    const uint64_t            nominal_delta_t_us,
    const PerChannelTsRecord* ts_record) noexcept
{
    // ── DROP-1: Non-finite calibrated_value on a non-synthetic sample ─────────

    if (!is_stale && !std::isfinite(envelope.calibrated_value))
    {
        set_hard_invalid(envelope);
        sat_inc(drop_count_);
        return true;
    }

    // ── DROP-2: HARD_INVALID + TIMING_ANOMALY (dual corruption) ──────────────

    if (has_flag(envelope.status, SampleStatus::HARD_INVALID) &&
        has_flag(envelope.status, SampleStatus::TIMING_ANOMALY))
    {
        sat_inc(drop_count_);
        return true;
    }

    // ── DROP-3: corrected_timestamp_us == 0 (S6 not initialized) ─────────────

    if (envelope.corrected_timestamp_us == 0u)
    {
        set_hard_invalid(envelope);
        sat_inc(drop_count_);
        return true;
    }

    // ── DROP-4: corrected_delta_t_us == 0 (S6 guarantee violated) ────────────

    if (envelope.corrected_delta_t_us == 0u)
    {
        set_hard_invalid(envelope);
        sat_inc(drop_count_);
        return true;
    }

    // ── DROP-5: Timestamp monotonicity regression (S7 independent check) ─────

    if (ts_record == nullptr)
    {
        // Fail-safe: if record is missing (table exhaustion), deny monotonicity pass.
        set_hard_invalid(envelope);
        sat_inc(drop_count_);
        return true;
    }

    const uint64_t last_ts = ts_record->last_ts_us;
    const uint64_t curr_ts = envelope.corrected_timestamp_us;
    const bool last_is_saturated = (last_ts == std::numeric_limits<uint64_t>::max());

    if (!last_is_saturated && curr_ts <= last_ts)
    {
        set_hard_invalid(envelope);
        sat_inc(drop_count_);
        return true;
    }

    // ── DROP-6: Extreme dt upper bound (S6 operating envelope violated) ───────

    uint64_t hard_max_dt = 0u;
    if (!safe_mul_u64(kAbsoluteMaxDtMultiplier, nominal_delta_t_us, hard_max_dt))
    {
        set_hard_invalid(envelope);
        sat_inc(drop_count_);
        return true;
    }

    if (envelope.corrected_delta_t_us > hard_max_dt)
    {
        set_hard_invalid(envelope);
        sat_inc(drop_count_);
        return true;
    }

    // ── DROP-7: MISSING + TIMING_ANOMALY (gap + timing dual corruption) ───────

    if (has_flag(envelope.status, SampleStatus::MISSING) &&
        has_flag(envelope.status, SampleStatus::TIMING_ANOMALY))
    {
        set_hard_invalid(envelope);
        sat_inc(drop_count_);
        return true;
    }

    return false;
}


// =============================================================================
// hard_gate_pass()
// =============================================================================

HardGateResult StageS7Output::hard_gate_pass(
    const MeasurementEnvelope& env,
    const ChannelState&        cs) const noexcept
{
    // 1. Physical / Data Invalidity
    if (has_flag(env.status, SampleStatus::HARD_INVALID) ||
        !std::isfinite(env.calibrated_value) ||
        has_flag(env.status, SampleStatus::MISSING))
    {
        return {false, HardGateFailReason::INVALID_DATA};
    }

    // 2. Timing Instability
    if (env.corrected_delta_t_us == 0u ||
        env.corrected_delta_t_us > 10u * cs.nominal_delta_t_us ||
        has_flag(env.status, SampleStatus::TIMING_ANOMALY))
    {
        return {false, HardGateFailReason::TIMING_ANOMALY};
    }

    // 3. Context Instability
    if (has_flag(env.status, SampleStatus::STALE) ||
        has_flag(env.status, SampleStatus::INTERPOLATED))
    {
        return {false, HardGateFailReason::UNSTABLE_CONTEXT};
    }

    const uint64_t total = good_count_ + degraded_count_ + bad_count_ + drop_count_;
    if (total >= 100u) 
    {
        if ((drop_count_ * 100u / total) > 5u || (bad_count_ * 100u / total) > 50u) 
        {
            return {false, HardGateFailReason::UNSTABLE_CONTEXT};
        }
    }

    // 4. Warm-up / Insufficient History
    if (cs.roc_n < 30u || cs.roc_prev_sample_invalid) 
    {
        return {false, HardGateFailReason::INSUFFICIENT_HISTORY};
    }

    // 5. Numerical Safety
    const double dt_s = static_cast<double>(env.corrected_delta_t_us) * 1e-6;
    if (dt_s <= 1e-9 || 
        !std::isfinite(env.roc) || 
        !std::isfinite(env.roc_adaptive_limit)) 
    {
        return {false, HardGateFailReason::NUMERICAL_FAILURE};
    }

    return {true, HardGateFailReason::NONE};
}


// =============================================================================
// suppress_drift()
// =============================================================================

// static
void StageS7Output::suppress_drift(
    MeasurementEnvelope& env,
    ChannelState&        cs,
    const HardGateFailReason reason) noexcept
{
    // 1 & 2: Block drift scoring / anomaly output flags
    env.status = clear_flag(env.status, SampleStatus::ROC_EXCEEDED);
    env.status = clear_flag(env.status, SampleStatus::RATE_ANOMALY);

    if (env.failure_hint == FailureMode::DRIFT || env.failure_hint == FailureMode::SPIKE) 
    {
        env.failure_hint       = FailureMode::NONE;
        env.failure_confidence = 0.0f;
        env.failure_duration   = 0u;
    }

    // 3. Set output state to BAD_CONTEXT (REJECTED) or DEGRADED
    if (reason == HardGateFailReason::INVALID_DATA || 
        reason == HardGateFailReason::UNSTABLE_CONTEXT || 
        reason == HardGateFailReason::NUMERICAL_FAILURE) 
    {
        env.measurement_trust_tier = MeasurementTrustTier::REJECTED;
    } 
    else 
    {
        env.measurement_trust_tier = MeasurementTrustTier::DEGRADED;
    }

    // 4. Reset or freeze drift-related counters
    cs.roc_violation_count = 0u;

    if (cs.failure_state.latched_hint == FailureMode::DRIFT || 
        cs.failure_state.latched_hint == FailureMode::SPIKE) 
    {
        cs.failure_state.latched_hint       = FailureMode::NONE;
        cs.failure_state.latched_confidence = 0.0f;
        cs.failure_state.duration_frames    = 0u;
        cs.failure_state.enter_counter      = 0u;
        cs.failure_state.exit_counter       = 0u;
    }
}


// =============================================================================
// classify_quality()
// =============================================================================

// static
SampleQuality StageS7Output::classify_quality(
    const SampleStatus status,
    const uint64_t     corrected_delta_t_us,
    const uint64_t     nominal_delta_t_us) noexcept
{
    uint64_t large_dt_threshold = 0u;
    if (!safe_mul_u64(kLargeDtThresholdMultiplier, nominal_delta_t_us, large_dt_threshold))
    {
        large_dt_threshold = std::numeric_limits<uint64_t>::max();
    }

    const bool dt_is_large = (corrected_delta_t_us > large_dt_threshold);

    const bool stale        = has_flag(status, SampleStatus::STALE);
    const bool missing      = has_flag(status, SampleStatus::MISSING);
    const bool hard_invalid = has_flag(status, SampleStatus::HARD_INVALID);
    const bool timing       = has_flag(status, SampleStatus::TIMING_ANOMALY);
    const bool rate         = has_flag(status, SampleStatus::RATE_ANOMALY);

    // ── Priority 1: BAD ───────────────────────────────────────────────────────

    if (stale)        { return SampleQuality::BAD; }
    if (missing)      { return SampleQuality::BAD; }
    if (hard_invalid) { return SampleQuality::BAD; }
    
    if (timing && dt_is_large) { return SampleQuality::BAD; }
    if (rate && dt_is_large)   { return SampleQuality::BAD; }

    // ── Priority 2: DEGRADED ─────────────────────────────────────────────────

    if (timing || rate) { return SampleQuality::DEGRADED; }

    if (nominal_delta_t_us >= kMinDtFraction)
    {
        const uint64_t min_sane_dt = nominal_delta_t_us / kMinDtFraction;
        if (corrected_delta_t_us < min_sane_dt)
        {
            return SampleQuality::DEGRADED;
        }
    }

    // ── Priority 3: GOOD ─────────────────────────────────────────────────────

    return SampleQuality::GOOD;
}


// =============================================================================
// apply_health_check()
// =============================================================================

SampleQuality StageS7Output::apply_health_check(
    const SampleQuality quality) const noexcept
{
    if (quality != SampleQuality::GOOD)
    {
        return quality;
    }

    uint64_t total = good_count_;
    total = sat_add_u64(total, degraded_count_);
    total = sat_add_u64(total, bad_count_);
    total = sat_add_u64(total, drop_count_);

    if (total < kHealthMinSamples)
    {
        return quality;
    }

    // Division-safe evaluation over direct multiplication comparison
    const uint64_t base_100 = total / 100u;
    if (base_100 == 0u) 
    {
        return quality; // Fallback edge case, technically caught by kHealthMinSamples
    }

    const uint64_t drop_threshold = base_100 * kDropRateThresholdPct;
    if (drop_count_ > drop_threshold)
    {
        return SampleQuality::DEGRADED;
    }

    const uint64_t bad_threshold = base_100 * kBadRateThresholdPct;
    if (bad_count_ > bad_threshold)
    {
        return SampleQuality::DEGRADED;
    }

    return SampleQuality::GOOD;
}


// =============================================================================
// quality_to_trust_tier()
// =============================================================================

// static
MeasurementTrustTier StageS7Output::quality_to_trust_tier(
    const SampleQuality quality) noexcept
{
    switch (quality)
    {
        case SampleQuality::GOOD:
            return MeasurementTrustTier::TRUSTED;

        case SampleQuality::DEGRADED:
            return MeasurementTrustTier::DEGRADED;

        case SampleQuality::BAD:
            return MeasurementTrustTier::REJECTED;

        default:
            return MeasurementTrustTier::REJECTED;
    }
}


// static
bool StageS7Output::has_flag(
    const SampleStatus status,
    const SampleStatus flag) noexcept
{
    return (static_cast<uint16_t>(status) & static_cast<uint16_t>(flag)) != 0u;
}


// =============================================================================
// set_hard_invalid()
// =============================================================================

// static
void StageS7Output::set_hard_invalid(MeasurementEnvelope& envelope) noexcept
{
    if (!has_flag(envelope.status, SampleStatus::HARD_INVALID))
    {
        envelope.status = static_cast<SampleStatus>(
            static_cast<uint16_t>(envelope.status) | 
            static_cast<uint16_t>(SampleStatus::HARD_INVALID));
    }

    // Rev 2.6: Use hierarchical upgrade (INVALID always wins)
    upgrade_failure_hint(envelope, FailureMode::INVALID, 1.0f);
}


// =============================================================================
// package_sample()
// =============================================================================

// static
ProcessedSample StageS7Output::package_sample(
    const MeasurementEnvelope& envelope,
    const SampleQuality        quality) noexcept
{
    ProcessedSample pkg;

    pkg.value              = envelope.calibrated_value;
    pkg.timestamp_us       = envelope.corrected_timestamp_us;
    pkg.dt                 = static_cast<float>(
                                 static_cast<double>(envelope.corrected_delta_t_us) * 1.0e-6);
    pkg.quality            = quality;
    pkg.failure_hint       = envelope.failure_hint;
    pkg._pad_align_conf[0] = 0u;
    pkg._pad_align_conf[1] = 0u;
    pkg.failure_confidence = envelope.failure_confidence;
    pkg.failure_duration   = envelope.failure_duration;

    return pkg;
}

} // namespace signalfix