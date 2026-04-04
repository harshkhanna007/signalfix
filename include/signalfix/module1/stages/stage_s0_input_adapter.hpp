// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/stages/stage_s0_input_adapter.hpp
// Spec   : SFX-M1-TDS-001  Revision 2.1
// Stage  : S0 — Input Adapter
// =============================================================================
//
// StageS0InputAdapter — first stage of the nine-stage pipeline.
//
// Rev 2.1-patch: six reliability fixes applied (no interface changes):
//   FIX-1  compute_delta_t_us: pure unsigned comparison replaces signed cast.
//   FIX-2  delta_t_us clamped to nominal when > kMaxDeltaMultiplier × nominal.
//   FIX-3  delta_t_us == 0 always falls back to nominal (combined with FIX-2).
//   FIX-4  Watchdog deadline: 2×nominal multiplication itself saturated before
//          the existing arrival+timeout saturation check.  Applied to both
//          process() and build_stale_envelope().
//   FIX-5  Extreme delta sanity check (same code path as FIX-2).
//   FIX-6  Debug-only assert() blocks for nominal_delta_t_us > 0 and raw_value
//          NaN detection (#ifndef NDEBUG only).  Channel-id consistency is NOT
//          asserted (the hard runtime guard below is definitive; asserting it
//          would break negative-path tests in debug builds).
//
// Responsibilities (normal path, called by IngestionPipeline::process()):
//   1. Validate that envelope.channel_id matches channel_state.channel_id.
//      Mismatch is a pipeline misconfiguration fault → ABORT_FAULT.
//   2. Assign sequence_id from cs.sequence_counter (post-increment).
//      The counter is never reset; it is monotonically increasing for the
//      lifetime of the channel registration.
//   3. Stamp calibration_version from ChannelState into the envelope.
//   4. Compute delta_t_us using pure unsigned arithmetic (FIX-1) to safely
//      handle clock step-backs.  If the computed delta is zero (FIX-3) or
//      exceeds kMaxDeltaMultiplier × nominal (FIX-2/FIX-5), fall back to
//      nominal_delta_t_us and set TIMING_ANOMALY.
//      On first sample (watchdog not yet armed) delta_t_us = nominal_delta_t_us.
//   5. Update cs.last_arrival_us.
//   6. Arm / refresh watchdog with two-level overflow saturation (FIX-4):
//        timeout = saturate(2 × nominal_delta_t_us)
//        cs.watchdog_deadline_us = saturate(arrival_time_us + timeout)
//        cs.watchdog_armed = true (cleared only by init_channel_state).
//
// The watchdog INJECTION path (STALE|MISSING synthetic envelopes) is handled
// by IngestionPipeline::process_watchdog(), which builds the envelope and
// bypasses S1–S6. S0 does not see that path; it only runs on real samples.
// A static helper build_stale_envelope() is provided for callers that wish
// to construct watchdog envelopes independently of the pipeline orchestrator.
//
// Design constraints:
//   - No heap allocation. All state is in ChannelState.
//   - No exceptions (-fno-exceptions).
//   - No RTTI (-fno-rtti).
//   - O(1) execution (no loops, no dynamic dispatch within this stage).
//   - Deterministic output for identical inputs.
//
// Thread safety:
//   Not thread-safe. Per IngestionPipeline contract, process() is called
//   from a single producer thread. ChannelState is exclusively owned.
//
// =============================================================================

#pragma once

#include "signalfix/module1/stage_interface.hpp"
#include "signalfix/module1/types.hpp"
#include "signalfix/module1/channel_state.hpp"

#include <cstdint>
#include <cmath>
#include <limits>

namespace signalfix {

// ---------------------------------------------------------------------------
// StageS0InputAdapter
// ---------------------------------------------------------------------------

class StageS0InputAdapter final : public IStage
{
public:
    // -------------------------------------------------------------------------
    // Construction
    //
    // S0 is stateless — all cross-sample persistence lives in ChannelState.
    // No constructor parameters are required.
    // -------------------------------------------------------------------------
    StageS0InputAdapter() noexcept = default;

    // -------------------------------------------------------------------------
    // process() — hot-path normal-sample execution
    //
    // Preconditions (enforced by pipeline orchestrator before calling):
    //   - envelope.arrival_time_us has been set by make_initial_envelope().
    //   - envelope.raw_value has been set by make_initial_envelope().
    //   - envelope.channel_id has been pre-stamped by make_initial_envelope()
    //     with sample.channel_id (used here for the mismatch check).
    //   - channel_state corresponds to the channel registered for this sample.
    //
    // Postconditions on StageResult::CONTINUE:
    //   - envelope.channel_id          == channel_state.channel_id
    //   - envelope.sequence_id         == previous cs.sequence_counter (post-inc)
    //   - envelope.calibration_version == channel_state.calibration_version
    //   - envelope.delta_t_us          is a safe, positive uint64_t [μs]
    //   - cs.last_arrival_us           == envelope.arrival_time_us
    //   - cs.watchdog_deadline_us      == arrival_time_us + 2×nominal_delta_t_us
    //   - cs.watchdog_armed            == true
    //   - cs.sequence_counter          == old value + 1
    //
    // Returns:
    //   CONTINUE   — normal processing complete.
    //   ABORT_FAULT — channel_id mismatch (pipeline misconfiguration).
    // -------------------------------------------------------------------------
    [[nodiscard]] StageResult process(
        MeasurementEnvelope& envelope,
        ChannelState&        channel_state) noexcept override;

    [[nodiscard]] const char* stage_name() const noexcept override
    {
        return "S0-InputAdapter";
    }

    // S0 carries no stage-local state; reset() is a documented no-op.
    void reset() noexcept override {}

    // -------------------------------------------------------------------------
    // build_stale_envelope() — static helper for watchdog injection
    //
    // Constructs a STALE|MISSING synthetic envelope from a ChannelState.
    // Used by IngestionPipeline::process_watchdog() and any orchestrator that
    // needs to synthesise a timeout notification without a real sample.
    //
    // The caller MUST:
    //   1. Pass `env` initialised via make_nominal_envelope().
    //   2. Pass `current_time_us` as a monotonic system clock timestamp.
    //   3. Route the returned envelope through the S7 output gate only
    //      (not through S1–S6).
    //
    // State mutations performed:
    //   - cs.sequence_counter  is incremented (preserves sequence monotonicity).
    //   - cs.watchdog_deadline_us is advanced by 2 × nominal_delta_t_us.
    //   - cs.nominal_streak    is reset to 0 (watchdog break ends nominal run).
    //
    // Value fields (raw_value, calibrated_value, validated_value, filtered_value,
    // roc, roc_adaptive_limit) are left as NaN (from make_nominal_envelope baseline).
    // This is deliberate: the S7 gate accepts NaN when MISSING|STALE is set.
    // -------------------------------------------------------------------------
    static void build_stale_envelope(MeasurementEnvelope& env,
                                     ChannelState&         cs,
                                     uint64_t              current_time_us) noexcept;

    // -------------------------------------------------------------------------
    // kMaxDeltaMultiplier — clamp threshold for extreme inter-sample intervals
    //
    // If the computed delta_t exceeds (kMaxDeltaMultiplier × nominal_delta_t_us),
    // S0 clamps delta_t to nominal_delta_t_us and sets TIMING_ANOMALY.
    //
    // Rationale (FM-17 / FIX-2 / FIX-5):
    //   A sensor restart or large OS scheduling stall can produce deltas that
    //   are orders of magnitude above nominal.  The watchdog should have fired
    //   already, but if the sample arrives anyway, passing the raw delta to the
    //   PLL and ROC stages would corrupt both for many subsequent samples.
    //
    // Value: 10 × nominal.  Chosen to pass normal jitter (< 2×) and even
    //   a single missed sample (< 3×) without flagging, while catching genuine
    //   sensor-absence arrivals that slipped past the watchdog.
    //
    // This constant is internal to S0.  Downstream stages must not depend on it.
    // -------------------------------------------------------------------------
    static constexpr uint64_t kMaxDeltaMultiplier = 10u;

private:
    // -------------------------------------------------------------------------
    // compute_delta_t_us — safe unsigned arithmetic for inter-sample interval
    //
    // FIX-1: Replaced the previous signed int64_t cast approach.
    //   The cast static_cast<int64_t>(uint64_t) is implementation-defined for
    //   values > INT64_MAX (C++17 [conv.integral] p3).  The unsigned comparator
    //   below is well-defined for all uint64_t inputs and produces identical
    //   semantics:
    //     • arrival <= last  → regression or identical → return 0.
    //     • arrival >  last  → safe subtraction (no underflow possible).
    //
    // Precondition: watchdog_armed == true (ensures last_arrival_us is valid).
    // Returns: positive delta [μs], or 0 on regression.
    // Caller handles 0 → TIMING_ANOMALY + nominal fallback (FM-13, FM-14).
    // -------------------------------------------------------------------------
    [[nodiscard]] static uint64_t compute_delta_t_us(
        uint64_t arrival_time_us,
        uint64_t last_arrival_us) noexcept;
};

} // namespace signalfix
