// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/stage_s6_timestamp_correction.hpp
// Spec   : SFX-M1-TDS-001  Revision 2.4
// =============================================================================
//
// Stage S6 — Timestamp Correction (PLL Timing Estimator)
//
// PIPELINE POSITION: after S5 (Rate-of-Change), before S7 (Packager).
//
// ─── REQUIRED CHANGE IN timing_correction_state.hpp ─────────────────────────
//
// Replace:
//   uint8_t  _pad[2];       // offset 30, size 2
// With:
//   int16_t  phase_frac_accum;  // offset 30, size 2 — sub-μs phase accumulator
//
// The struct remains 32 bytes. zero-init (memset) correctly seeds phase_frac_accum=0.
// This is the ONLY required change to timing_correction_state.hpp.
//
// ─── PURPOSE ────────────────────────────────────────────────────────────────
//
// S6 establishes the corrected time base consumed by all downstream systems
// (S7 Packager, Kalman filter / SR-UKF, fault detector). Its contract is:
//
//   corrected_timestamp_us — strictly monotonically non-decreasing
//   corrected_delta_t_us   — reflects true elapsed time per path (see table)
//   no NaN / Inf in any output field
//
// ─── ALGORITHM ──────────────────────────────────────────────────────────────
//
// Simplified Phase-Locked Loop (PLL) with fractional phase accumulation:
//
//   State registers (in TimingCorrectionState):
//     estimated_dt_us      — frequency register (smoothed inter-sample interval)
//     corrected_timestamp_us — phase register (corrected output timeline)
//     phase_frac_accum     — sub-μs phase accumulator, 1/256 μs units (int16_t)
//
//   Core update per sample (PLL-driven paths: NORMAL, ANOMALY, DEGRADED,
//                                              REGRESSION, STALE):
//
//     1. IIR frequency update (NORMAL/ANOMALY/DEGRADED only):
//          error        = clamp(raw_dt, min, max) − estimated_dt_us
//          estimated_dt += effective_alpha × error
//          [rate-limited: Δestimated_dt ≤ prev_estimated_dt × 10%]
//          estimated_dt  = clamp(estimated_dt, min_dt_us, max_dt_us)
//
//     2. Fractional phase advance (advance_with_frac_correction):
//          base_dt       = round(estimated_dt_us)
//          residual_256  = round((base_dt − estimated_dt_us) × 256)
//          phase_frac_accum += residual_256
//
//          if phase_frac_accum ≥ 256:            // 1 μs of over-advance accumulated
//              advance = base_dt − 1             // shed one μs
//              phase_frac_accum -= 256
//          elif phase_frac_accum ≤ -256:         // 1 μs of under-advance accumulated
//              advance = base_dt + 1             // add one μs
//              phase_frac_accum += 256
//          else:
//              advance = base_dt
//
//          corrected_ts  = prev_corrected_ts + advance  (monotonic, saturating)
//
//   Why this eliminates long-run drift:
//     Without accumulation: rounding 9999.7 → 10000 every sample drifts
//     the timeline by +0.3 μs/sample → +300 μs per 1000 samples.
//     With accumulation: residual_256 ≈ 77 per step (0.3 × 256). After
//     ceil(256/77) ≈ 4 steps the accumulator triggers a 9999 μs advance,
//     balancing the prior 10000 μs steps. Long-run mean advance converges
//     to exactly estimated_dt_us (zero systematic drift).
//
// ─── EVENT CLASSIFICATION AND HANDLING ──────────────────────────────────────
//
//   Classification priority (first match wins):
//
//   STALE      — Watchdog sample. Checked FIRST. Timestamp may be synthetic
//                (arrival_time_us is used as time source). PLL fully frozen;
//                advance via advance_with_frac_correction; last_raw_ts NOT updated.
//                Lock: LOCKED → RECOVERY.
//
//   REGRESSION — Real timestamp went backwards. TIMING_ANOMALY set.
//                PLL frozen; advance via advance_with_frac_correction;
//                last_raw_ts NOT updated.
//                Lock: LOCKED → RECOVERY.
//
//   LONG_GAP   — S4 MISSING or raw_dt > max_time_jump_us. Full PLL reset to
//                nominal. phase_frac_accum reset to 0.
//                Lock: → LOCKING.
//
//   SHORT_GAP  — raw_dt ∈ (max_dt_us, max_time_jump_us].
//                PLL frequency updated at soft alpha (anomaly_alpha_scale).
//                  Frequency input: clamp(raw_dt, min_dt, max_dt)  [protects estimator]
//                  Phase advance:   clamp(raw_dt, min_dt, max_time_jump_us)  [real dt]
//                phase_frac_accum NOT updated (advance is not PLL-derived).
//                Lock: LOCKED → RECOVERY.
//
//   DEGRADED   — S5 HARD_INVALID. alpha × hard_invalid_alpha_scale. PLL updates.
//                advance_with_frac_correction.
//                Lock: LOCKED → RECOVERY.
//
//   ANOMALY    — TIMING_ANOMALY flagged. alpha × anomaly_alpha_scale. PLL updates.
//                advance_with_frac_correction.
//                Lock: LOCKED → RECOVERY.
//
//   NORMAL     — Full-alpha PLL update. advance_with_frac_correction.
//                Lock counter incremented ONLY if dt within ±10% stability band.
//
// ─── UPSTREAM INTEGRATION ───────────────────────────────────────────────────
//
//   S4 sets MISSING       → LONG_GAP path (full PLL reset)
//   S4 sets TIMING_ANOMALY → ANOMALY path (reduced alpha)
//   S5 sets HARD_INVALID  → DEGRADED path (small alpha, NOT frozen)
//   S0/watchdog sets STALE → STALE path (frozen, advance via frac, → RECOVERY)
//
// ─── TIME SOURCE SELECTION ──────────────────────────────────────────────────
//
//   STALE:        arrival_time_us (timestamp_us is synthetic / unreliable)
//   timestamp_us > 0: timestamp_us (S2b-corrected measurement time)
//   timestamp_us == 0: arrival_time_us (S2b not configured)
//
//   For STALE, the selected time source is not used to update any PLL state
//   (last_raw_timestamp_us is not modified). The selection still matters for
//   correct regression detection: using arrival_time_us for STALE prevents
//   timestamp_us=0 from appearing as a regression against last_raw_timestamp_us.
//
// ─── OUTPUT CONTRACT ─────────────────────────────────────────────────────────
//
//   S6 writes ONLY to:
//     envelope.corrected_timestamp_us
//     envelope.corrected_delta_t_us
//
//   It DOES NOT modify:
//     envelope.timestamp_us   (raw input — preserved as audit data)
//     envelope.delta_t_us     (raw input — preserved as audit data)
//
//   All stages after S6 (S7, Kalman bridge, fault detector) MUST read
//   corrected_timestamp_us and corrected_delta_t_us.
//
// ─── corrected_delta_t_us RANGE BY PATH ─────────────────────────────────────
//
//   Path        | corrected_delta_t_us range
//   ────────────┼──────────────────────────────────────────────────────
//   NORMAL      | [min_dt_us, max_dt_us ± 1]  (PLL-driven, frac-corrected)
//   ANOMALY     | [min_dt_us, max_dt_us ± 1]  (PLL-driven, frac-corrected)
//   DEGRADED    | [min_dt_us, max_dt_us ± 1]  (PLL-driven, frac-corrected)
//   REGRESSION  | [min_dt_us, max_dt_us ± 1]  (PLL-driven, frac-corrected)
//   STALE       | [min_dt_us, max_dt_us ± 1]  (PLL-driven, frac-corrected)
//   SHORT_GAP   | (max_dt_us, max_time_jump_us]  ← exceeds max_dt_us by design
//   LONG_GAP    | [min_dt_us, max_dt_us]  (clamped nominal, no frac)
//
//   The ±1 on PLL-driven paths is the fractional correction. G2 still holds
//   since the correction brings the advance closer to estimated_dt_us, which
//   is clamped to [min_dt_us, max_dt_us]. The ±1 is bounded within this range.
//   (See advance_with_frac_correction for exact boundary handling.)
//
// ─── STAGE RESULT POLICY ─────────────────────────────────────────────────────
//
//   Always returns CONTINUE. Anomalies are communicated via status flags.
//   S7 decides on drop/pass based on flags.
//
// ─── GUARANTEES ──────────────────────────────────────────────────────────────
//
//   G1. corrected_timestamp_us[n] ≥ corrected_timestamp_us[n-1]  (all paths)
//   G2. corrected_delta_t_us ∈ [min_dt_us, max_dt_us±1] on PLL paths;
//       ∈ (max_dt_us, max_time_jump_us] on SHORT_GAP;
//       ∈ [min_dt_us, max_dt_us] on LONG_GAP.
//   G3. No NaN / Inf written to any envelope field.
//   G4. Deterministic: identical (input, state) → identical output.
//   G5. O(1) time and O(1) additional space per channel.
//   G6. No heap allocation, no exceptions, no system calls.
//   G7. uint64_t arithmetic is overflow-safe: monotone_advance() saturates at UINT64_MAX.
//   G8. Systematic phase drift eliminated via frac accumulator. Residual
//       quantization noise ≤ 1/256 μs ≈ 4 ns per step, zero-mean (random walk,
//       not cumulative drift). Accumulated error bounded to < 1 μs at all times.
//   G9. No UB from NaN: clamp_and_round_dt() guards non-finite values.
//
// =============================================================================

#pragma once

#include "signalfix/module1/stage_interface.hpp"
#include "signalfix/module1/channel_state.hpp"
#include "signalfix/module1/timestamp_correction_config.hpp"
#include "signalfix/module1/timing_correction_state.hpp"

namespace signalfix {

// ---------------------------------------------------------------------------
// StageS6TimestampCorrection
// ---------------------------------------------------------------------------
class StageS6TimestampCorrection final : public IStage
{
public:
    /// Construct with a validated config. Config is copied at construction.
    /// Invalid config values are sanitised defensively; construction never fails.
    explicit StageS6TimestampCorrection(
        const TimestampCorrectionConfig& config) noexcept;

    ~StageS6TimestampCorrection() noexcept override = default;

    // ── IStage interface ─────────────────────────────────────────────────────

    [[nodiscard]] StageResult process(
        MeasurementEnvelope& envelope,
        ChannelState&        channel_state) noexcept override;

    [[nodiscard]] const char* stage_name() const noexcept override;

    /// No stage-internal per-sample state. All timing state is in
    /// ChannelState.ts_corr, reset by init_channel_state() / reset_channel().
    void reset() noexcept override;

private:
    TimestampCorrectionConfig config_;

    // ── Private types ─────────────────────────────────────────────────────────

    /// Internal event classification — determines the PLL update path.
    /// STALE is evaluated FIRST to prevent misclassification of synthetic envelopes.
    enum class TimingEvent : uint8_t
    {
        NORMAL,      ///< Clean sample; full state-dependent alpha; frac advance.
        ANOMALY,     ///< TIMING_ANOMALY flagged; alpha × anomaly_alpha_scale; frac advance.
        DEGRADED,    ///< HARD_INVALID from S5; alpha × hard_invalid_alpha_scale; frac advance.
        REGRESSION,  ///< Raw timestamp backwards; PLL frozen; frac advance; no raw_ts update.
        SHORT_GAP,   ///< Moderate overrun; raw_dt preserved as advance; no frac update.
        LONG_GAP,    ///< True gap or S4 MISSING; full PLL + frac reset.
        STALE,       ///< Watchdog sample; PLL frozen; frac advance; → RECOVERY.
    };

    // ── Bootstrap ─────────────────────────────────────────────────────────────

    void bootstrap(
        MeasurementEnvelope& envelope,
        ChannelState&        cs) noexcept;

    // ── Steady-state update ───────────────────────────────────────────────────

    StageResult update(
        MeasurementEnvelope& envelope,
        ChannelState&        cs) noexcept;

    // ── Event classification ──────────────────────────────────────────────────

    /// Classify the current sample's timing event.
    /// STALE is checked first — synthetic watchdog envelopes must not misclassify
    /// as REGRESSION even if their timestamp appears to regress.
    [[nodiscard]] TimingEvent classify_timing_event(
        uint64_t     raw_dt_us,
        SampleStatus status,
        bool         regression) const noexcept;

    // ── Per-event handlers ────────────────────────────────────────────────────

    /// NORMAL / ANOMALY / DEGRADED: IIR update (with rate limit), then
    /// fractional phase advance via advance_with_frac_correction().
    /// Does NOT update last_raw_timestamp_us — caller handles that.
    [[nodiscard]] uint64_t handle_normal_update(
        uint64_t               raw_dt_us,
        double                 effective_alpha,
        TimingCorrectionState& ts_corr) const noexcept;

    /// REGRESSION: PLL frozen; fractional phase advance; last_raw_ts NOT updated.
    [[nodiscard]] uint64_t handle_regression(
        TimingCorrectionState& ts_corr) const noexcept;

    /// STALE: PLL fully frozen; fractional phase advance; last_raw_ts NOT updated.
    [[nodiscard]] uint64_t handle_stale(
        TimingCorrectionState& ts_corr) const noexcept;

    /// SHORT_GAP: advance by actual raw_dt (bounded at max_time_jump_us).
    ///   PLL frequency: updated at anomaly_alpha_scale × state_alpha, clamped input.
    ///   phase_frac_accum: NOT updated — advance is not PLL-derived.
    [[nodiscard]] uint64_t handle_short_gap(
        uint64_t               raw_dt_us,
        uint64_t               raw_ts,
        TimingCorrectionState& ts_corr) const noexcept;

    /// LONG_GAP: reset estimated_dt + phase_frac_accum to nominal; advance by
    /// nominal; snap last_raw_timestamp_us to raw_ts.
    [[nodiscard]] uint64_t handle_long_gap(
        uint64_t               raw_ts,
        TimingCorrectionState& ts_corr,
        uint64_t               nominal_dt_us) const noexcept;

    // ── Lock state management ─────────────────────────────────────────────────

    [[nodiscard]] double compute_state_alpha(PllLockState lock_state) const noexcept;

    /// Update lock state machine.
    ///
    /// dt_stable: true when raw_dt is within ±kDtStabilityFraction of estimated_dt_us
    ///            (pre-update). Used to gate lock acquisition counter increment
    ///            (FIX-5): prevents locking on an unconverged frequency estimate.
    ///            Pass false for all non-NORMAL events.
    static void update_lock_state(
        TimingEvent                      event,
        TimingCorrectionState&           ts_corr,
        const TimestampCorrectionConfig& config,
        bool                             dt_stable) noexcept;

    // ── Arithmetic helpers ────────────────────────────────────────────────────

    /// Select the time source for the current sample.
    ///
    ///   is_stale == true : return arrival_time_us (synthetic envelope; timestamp
    ///                      may be zero or stale). This prevents a zero-valued
    ///                      timestamp from appearing as a regression.
    ///   timestamp_us > 0 : return timestamp_us (S2b measurement-corrected time).
    ///   timestamp_us == 0: return arrival_time_us (S2b not configured; fallback).
    [[nodiscard]] static uint64_t select_time_source(
        const MeasurementEnvelope& env,
        bool                       is_stale) noexcept;

    /// Clamp estimated_dt (double) to [min_dt_us, max_dt_us] and convert to
    /// uint64_t via round-to-nearest. Guards non-finite input (NaN → min_dt_us).
    /// Deterministic on all IEEE 754 platforms (no FPU rounding mode dependency).
    [[nodiscard]] uint64_t clamp_and_round_dt(double dt_us) const noexcept;

    /// PLL-driven phase advance with fractional accumulation (FIX-1).
    ///
    ///   1. base_dt       = clamp_and_round_dt(ts_corr.estimated_dt_us)
    ///   2. residual_256  = round((base_dt − estimated_dt_us) × 256)   [1/256 μs units]
    ///   3. phase_frac_accum += residual_256
    ///   4. if |phase_frac_accum| ≥ 256: apply ±1 μs correction; adjust accum
    ///   5. monotone_advance(ts_corr, corrected_dt)
    ///
    /// Eliminates long-run systematic drift from IIR round-to-nearest truncation.
    /// Use on: NORMAL, ANOMALY, DEGRADED, REGRESSION, STALE.
    /// Do NOT use on: SHORT_GAP (real dt advance), LONG_GAP (hard reset).
    [[nodiscard]] uint64_t advance_with_frac_correction(
        TimingCorrectionState& ts_corr) const noexcept;

    /// Advance corrected_timestamp_us by delta_us, enforcing strict monotonicity
    /// and saturating at UINT64_MAX to prevent uint64_t overflow (G7).
    [[nodiscard]] uint64_t monotone_advance(
        TimingCorrectionState& ts_corr,
        uint64_t               delta_us) const noexcept;

    // ── Flag inspection ───────────────────────────────────────────────────────

    [[nodiscard]] static bool flag_set(SampleStatus status, SampleStatus flag) noexcept;
};

} // namespace signalfix
