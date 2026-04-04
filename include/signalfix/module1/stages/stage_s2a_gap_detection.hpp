// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/stages/stage_s2a_gap_detection.hpp
// Spec   : SFX-M1-TDS-001  Revision 2.1
// Stage  : S2a — Gap Pre-detector
// =============================================================================
//
// StageS2aGapDetection — detects missing samples BEFORE the PLL update (S2b).
//
// Responsibilities:
//   1. Compare envelope.delta_t_us against the gap threshold:
//        gap_threshold = cs.nominal_delta_t_us × kGapMultiplier  (default 2.5)
//   2. If a gap is detected:
//        - Set cs.gap.gap_pending = true  (signals S2b to suppress PLL update)
//        - Assign envelope.gap_event_id from cs.gap.next_gap_event_id
//        - Increment cs.gap.next_gap_event_id  (advances to next available id)
//        - Increment cs.gap.consecutive_gap_count
//        - Increment cs.gap.total_gap_count
//   3. If no gap:
//        - Set cs.gap.gap_pending = false
//        - Set cs.gap.consecutive_gap_count = 0  (reset consecutive counter)
//        - Set envelope.gap_event_id = 0          (0 = no gap, per contract)
//   4. Preserve all existing status flags.
//
// ── SampleStatus::GAP_DETECTED note ─────────────────────────────────────────
//
// The TDS specification references SampleStatus::GAP_DETECTED.  That flag does
// NOT exist in the Rev 2.1 SampleStatus enum (which uses 8 defined bits, all
// already assigned).  S2a therefore does NOT set a status flag on the envelope.
// Gap presence is communicated instead via:
//   - envelope.gap_event_id != 0  (non-zero when gap detected)
//   - cs.gap.gap_pending == true  (read by S2b to suppress PLL update)
// S4 (Gap Policy Engine) is the stage that sets envelope status flags based on
// the gap policy decision (INTERPOLATED, MISSING, etc.).
//
// ── Gap multiplier ───────────────────────────────────────────────────────────
//
// kGapMultiplier = 2.5 per TDS Section 3.1.1.  Implemented as integer
// arithmetic (× 5 / 2) to avoid floating-point in the hot path.
//
// Design constraints:
//   - No heap allocation.
//   - No exceptions (-fno-exceptions).
//   - No RTTI (-fno-rtti).
//   - O(1) execution.
//   - Deterministic output for identical inputs.
//
// Thread safety:
//   Not thread-safe.  Per IngestionPipeline contract, process() is called
//   from a single producer thread.  ChannelState is exclusively owned.
//
// =============================================================================

#pragma once

#include "signalfix/module1/stage_interface.hpp"
#include "signalfix/module1/types.hpp"
#include "signalfix/module1/channel_state.hpp"

#include <cstdint>
#include <limits>

namespace signalfix {

// ---------------------------------------------------------------------------
// StageS2aGapDetection
// ---------------------------------------------------------------------------

class StageS2aGapDetection final : public IStage
{
public:
    // -------------------------------------------------------------------------
    // Construction
    //
    // S2a is stateless — all persistent state lives in ChannelState.
    // -------------------------------------------------------------------------
    StageS2aGapDetection() noexcept = default;

    // -------------------------------------------------------------------------
    // process() — hot-path gap detection
    //
    // Preconditions:
    //   - envelope.delta_t_us has been set by S0 (safe, positive, non-zero).
    //   - cs.nominal_delta_t_us > 0 (guaranteed by init_channel_state).
    //
    // Postconditions (gap detected):
    //   - envelope.gap_event_id          == cs.gap.next_gap_event_id (pre-increment)
    //   - cs.gap.next_gap_event_id        incremented by 1
    //   - cs.gap.consecutive_gap_count    incremented by 1
    //   - cs.gap.total_gap_count          incremented by 1
    //   - cs.gap.gap_pending              == true
    //
    // Postconditions (no gap):
    //   - envelope.gap_event_id          == 0
    //   - cs.gap.gap_pending             == false
    //   - cs.gap.consecutive_gap_count   == 0
    //
    // Returns:
    //   CONTINUE always.
    // -------------------------------------------------------------------------
    [[nodiscard]] StageResult process(
        MeasurementEnvelope& envelope,
        ChannelState&        channel_state) noexcept override;

    [[nodiscard]] const char* stage_name() const noexcept override
    {
        return "S2a-GapDetection";
    }

    // Stateless — reset() is a documented no-op.
    void reset() noexcept override {}

    // -------------------------------------------------------------------------
    // Gap threshold computation — exposed for testability
    //
    // gap_threshold = nominal × 5 / 2  (= nominal × 2.5, integer arithmetic)
    //
    // Overflow protection: if nominal × 5 overflows uint64_t, saturate to
    // UINT64_MAX.  At that saturation, any real delta_t_us is below threshold,
    // so no spurious gap detection occurs.
    // -------------------------------------------------------------------------
    [[nodiscard]] static uint64_t compute_gap_threshold(
        uint64_t nominal_delta_t_us) noexcept;

private:
    // kGapMultiplierNum / kGapMultiplierDen = 5/2 = 2.5 (TDS Section 3.1.1)
    static constexpr uint64_t kGapMultiplierNum = 5u;
    static constexpr uint64_t kGapMultiplierDen = 2u;
};

} // namespace signalfix
