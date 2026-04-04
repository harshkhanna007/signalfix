// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/stages/stage_s2b_clock_alignment.hpp
// Spec   : SFX-M1-TDS-001  Revision 2.1
// Stage  : S2b — Clock Alignment & Jitter Correction (PLL)
// =============================================================================
//
// StageS2bClockAlignment — corrects timestamp drift using a software PLL.
//
// Responsibilities:
//   1. If cs.gap.gap_pending == true (S2a detected a gap):
//        - Suppress PLL phase/frequency update for this sample.
//        - Assign timestamp_us = cs.last_timestamp_us + cs.nominal_delta_t_us
//          (advance by nominal only — no PLL correction during gap).
//        - Set TIMING_ANOMALY on the envelope.
//   2. If no gap (nominal path):
//        - Compute phase error: φ_err = arrival_time_us − t_estimated
//          where t_estimated = cs.last_timestamp_us + cs.nominal_delta_t_us
//        - Apply proportional correction to timestamp:
//            timestamp_us = t_estimated + K_p × φ_err
//        - Apply integral correction to frequency estimate:
//            cs.pll.freq_correction_ppm += K_f × φ_err
//        - Store accumulated phase error: cs.pll.phase_error_us = φ_err
//        - Advance lock counter: cs.pll.lock_sample_count++
//        - Mark cs.pll.locked = true once kLockThreshold samples seen
//   3. Enforce monotonicity: if computed timestamp_us ≤ cs.last_timestamp_us:
//        - Retain cs.last_timestamp_us (do not regress).
//        - Set TIMING_ANOMALY.
//   4. Compute envelope.jitter_us = arrival_time_us − timestamp_us (signed).
//      Clamp to int32_t range.
//   5. Update cs.last_timestamp_us = envelope.timestamp_us.
//
// ── PllState fields used ─────────────────────────────────────────────────────
//
// cs.pll.phase_error_us       — most recent phase error [μs]
// cs.pll.freq_correction_ppm  — accumulated frequency correction
//                               (used as an additive offset in μs, not true ppm)
// cs.pll.lock_sample_count    — samples since last reset / unlock
// cs.pll.locked               — true once kLockThreshold samples have been seen
//
// ── PLL design note ──────────────────────────────────────────────────────────
//
// The TDS specification references cs.pll_gain and cs.pll_last_error.  Those
// fields do NOT exist in the Rev 2.1 ChannelState / PllState structs.  The
// actual fields are cs.pll.phase_error_us and cs.pll.freq_correction_ppm.
//
// PLL gains are defined as compile-time constants here (K_p, K_f) rather than
// per-channel runtime parameters.  Adding per-channel gains to PllState is
// the correct long-term fix; until then these defaults follow TDS Section 3.1.2:
//   K_p = 0.02  (proportional gain — phase correction)
//   K_f = 0.001 (integral gain — frequency correction)
//
// freq_correction_ppm is repurposed here as an accumulated μs offset (not true
// parts-per-million), consistent with how S2b's integrator accumulates error
// over multiple samples.  The field name is inherited from PllState; the
// semantics for this implementation are defined above.
//
// Design constraints:
//   - No heap allocation.
//   - No exceptions (-fno-exceptions).
//   - No RTTI (-fno-rtti).
//   - O(1) execution.
//   - Deterministic output for identical inputs.
//
// Thread safety:
//   Not thread-safe.  Single-producer per IngestionPipeline contract.
//
// =============================================================================

#pragma once

#include "signalfix/module1/stage_interface.hpp"
#include "signalfix/module1/types.hpp"
#include "signalfix/module1/channel_state.hpp"

#include <cstdint>
#include <climits>   // INT32_MIN, INT32_MAX

namespace signalfix {

// ---------------------------------------------------------------------------
// StageS2bClockAlignment
// ---------------------------------------------------------------------------

class StageS2bClockAlignment final : public IStage
{
public:
    // -------------------------------------------------------------------------
    // PLL gain constants (TDS Section 3.1.2)
    //
    // K_p — proportional gain applied to phase error for timestamp correction.
    //        Small values reduce jitter but slow convergence.
    // K_f — integral gain applied to phase error for frequency correction.
    //        Accumulates slowly; provides steady-state phase error correction.
    // -------------------------------------------------------------------------
    static constexpr double kKp = 0.02;
    static constexpr double kKf = 0.001;

    // Number of consecutive non-gap samples required before the PLL is
    // considered locked.  Before lock, only K_p is applied (no integration).
    static constexpr uint32_t kLockThreshold = 10u;

    // -------------------------------------------------------------------------
    // Construction
    //
    // S2b is stateless — all persistent state lives in ChannelState.pll and
    // ChannelState.last_timestamp_us.
    // -------------------------------------------------------------------------
    StageS2bClockAlignment() noexcept = default;

    // -------------------------------------------------------------------------
    // process() — hot-path PLL execution
    //
    // Preconditions:
    //   - S2a has run and set cs.gap.gap_pending correctly.
    //   - envelope.arrival_time_us is the raw monotonic system clock [μs].
    //   - envelope.delta_t_us is positive and valid (guaranteed by S0).
    //   - cs.nominal_delta_t_us > 0.
    //
    // Postconditions (gap path):
    //   - envelope.timestamp_us  == cs.last_timestamp_us + nominal (advanced nominally)
    //   - envelope.jitter_us     is the arrival vs corrected timestamp difference
    //   - TIMING_ANOMALY set.
    //   - cs.pll state UNCHANGED (hold-last on gap).
    //   - cs.last_timestamp_us   updated.
    //
    // Postconditions (nominal path):
    //   - envelope.timestamp_us  is the PLL-corrected monotonic timestamp.
    //   - envelope.jitter_us     == clamp(arrival_time_us − timestamp_us, int32_t).
    //   - cs.pll.phase_error_us  == most recent φ_err [μs].
    //   - cs.pll.freq_correction_ppm updated (integral term).
    //   - cs.pll.lock_sample_count incremented.
    //   - cs.pll.locked          set once lock_sample_count ≥ kLockThreshold.
    //   - cs.last_timestamp_us   == envelope.timestamp_us.
    //
    // Returns:
    //   CONTINUE always.
    // -------------------------------------------------------------------------
    [[nodiscard]] StageResult process(
        MeasurementEnvelope& envelope,
        ChannelState&        channel_state) noexcept override;

    [[nodiscard]] const char* stage_name() const noexcept override
    {
        return "S2b-ClockAlignment";
    }

    // S2b carries no stage-local state; reset() is a documented no-op.
    // (ChannelState.pll is reset by init_channel_state.)
    void reset() noexcept override {}

private:
    // -------------------------------------------------------------------------
    // clamp_to_int32() — saturating cast for jitter_us
    //
    // Timestamp differences are int64_t; jitter_us is int32_t.
    // Extreme jitter (sensor restart, OS suspend) can produce differences
    // outside int32_t range.  Clamp rather than invoke UB.
    // -------------------------------------------------------------------------
    [[nodiscard]] static int32_t clamp_to_int32(int64_t v) noexcept;

    // -------------------------------------------------------------------------
    // safe_timestamp_advance() — monotonicity-safe addition
    //
    // Computes (base + delta_us) for timestamp_us, saturating at UINT64_MAX
    // to prevent wraparound.  Returns the saturated result.
    // -------------------------------------------------------------------------
    [[nodiscard]] static uint64_t safe_timestamp_advance(
        uint64_t base_us,
        uint64_t delta_us) noexcept;
};

} // namespace signalfix
