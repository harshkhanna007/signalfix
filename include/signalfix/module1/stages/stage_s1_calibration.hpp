// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/stages/stage_s1_calibration.hpp
// Spec   : SFX-M1-TDS-001  Revision 2.1
// Stage  : S1 — Calibration Transform
// =============================================================================
//
// StageS1Calibration — converts raw_value → calibrated_value.
//
// Responsibilities:
//   1. Guard against NaN / Inf in raw_value (sensor hardware fault).
//      If detected: set HARD_INVALID, leave calibrated_value = NaN, return CONTINUE.
//   2. Apply linear calibration transform:
//        calibrated_value = (raw_value × gain) + offset
//   3. Guard the calibration output against NaN / Inf (arithmetic overflow
//      or degenerate calibration coefficients).
//      If detected: set HARD_INVALID, write NaN to calibrated_value, return CONTINUE.
//   4. Update ChannelState history for downstream ROC computation (S3):
//        cs.last_calibrated_value, cs.last_calibrated_valid
//   5. Preserve all status flags set by S0. Never clear existing flags.
//
// ── Calibration coefficient design note ─────────────────────────────────────
//
// The TDS specification references per-channel calibration coefficients
// (gain and offset) stored in ChannelState.  As of Rev 2.1, ChannelState
// does NOT contain calibration_gain or calibration_offset fields — those
// fields were not added to the struct when the architecture was revised.
//
// This implementation therefore accepts gain and offset as constructor
// parameters, stored in the stage object itself.  The pipeline owner
// is responsible for constructing one StageS1Calibration per channel
// (or one shared instance with gain=1.0 / offset=0.0 for an identity
// transform) and registering it correctly.
//
// Architectural consequence: with the current ChannelState layout,
// per-channel calibration requires either:
//   (a) One StageS1Calibration instance per channel (memory: ~16 bytes each)
//   (b) A lookup table keyed by channel_id inside the stage (adds state)
// Option (a) is cleanest.  Option (b) is more flexible.
//
// Adding calibration_gain / calibration_offset to ChannelState remains the
// preferred long-term fix.  Until then, the constructor-parameter approach
// is used here.  The identity defaults (gain=1.0, offset=0.0) are safe
// for channels that are already in physical units.
//
// Design constraints:
//   - No heap allocation.
//   - No exceptions (-fno-exceptions).
//   - No RTTI (-fno-rtti).
//   - O(1) execution.
//   - Deterministic output for identical inputs.
//
// Thread safety:
//   Not thread-safe. Per IngestionPipeline contract, process() is called
//   from a single producer thread.
//
// =============================================================================

#pragma once

#include "signalfix/module1/stage_interface.hpp"
#include "signalfix/module1/types.hpp"
#include "signalfix/module1/channel_state.hpp"

#include <cmath>
#include <limits>

namespace signalfix {

// ---------------------------------------------------------------------------
// StageS1Calibration
// ---------------------------------------------------------------------------

class StageS1Calibration final : public IStage
{
public:
    // -------------------------------------------------------------------------
    // Construction
    //
    // gain   — Multiplicative calibration coefficient.
    //          Default: 1.0 (identity — raw value already in physical units).
    // offset — Additive calibration coefficient.
    //          Default: 0.0 (no zero-point correction).
    //
    // Both parameters are copied into the stage object.  No heap allocation.
    // -------------------------------------------------------------------------
    explicit StageS1Calibration(double gain   = 1.0,
                                 double offset = 0.0) noexcept;

    // -------------------------------------------------------------------------
    // process() — hot-path calibration execution
    //
    // Preconditions (enforced by preceding S0 stage):
    //   - envelope.raw_value has been set from RawSample.raw_value.
    //   - envelope.channel_id == channel_state.channel_id.
    //   - envelope.delta_t_us is a safe, positive value.
    //
    // Postconditions on StageResult::CONTINUE (HARD_INVALID path):
    //   - envelope.calibrated_value == NaN.
    //   - envelope.status has HARD_INVALID set.
    //   - cs unchanged (history not updated on invalid input).
    //
    // Postconditions on StageResult::CONTINUE (nominal path):
    //   - envelope.calibrated_value == (raw_value × gain_) + offset_.
    //   - cs.last_calibrated_value  == envelope.calibrated_value.
    //   - cs.last_calibrated_valid  == true.
    //
    // Returns:
    //   CONTINUE always — S1 does not abort the pipeline.
    //   Faults are communicated via HARD_INVALID status.
    // -------------------------------------------------------------------------
    [[nodiscard]] StageResult process(
        MeasurementEnvelope& envelope,
        ChannelState&        channel_state) noexcept override;

    [[nodiscard]] const char* stage_name() const noexcept override
    {
        return "S1-Calibration";
    }

    // S1 carries no cross-sample stage-local state beyond the immutable
    // coefficients.  reset() is a documented no-op.
    void reset() noexcept override {}

private:
    double gain_;    ///< Multiplicative calibration coefficient.
    double offset_;  ///< Additive calibration coefficient (zero-point).
};

} // namespace signalfix
