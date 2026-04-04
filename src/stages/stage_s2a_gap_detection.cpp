// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : src/module1/stages/stage_s2a_gap_detection.cpp
// Spec   : SFX-M1-TDS-001  Revision 2.1
// Stage  : S2a — Gap Pre-detector
// =============================================================================
//
// Implementation notes — failure modes addressed:
//
//  FM-S2a-01  Gap threshold overflow (nominal × 5 wraps to small value)
//             Cause   : nominal_delta_t_us near UINT64_MAX.
//             Effect  : computed threshold is smaller than nominal; every sample
//                       triggers a false gap detection.  PLL is permanently
//                       suppressed; all timestamps default to arrival_time_us.
//             Guard   : multiply nominal × 5 with overflow saturation.
//                       At saturation the threshold is UINT64_MAX/2, which no
//                       real delta_t_us can exceed (delta is capped at 10×nominal
//                       by S0 FIX-2), so no spurious gap fires.
//
//  FM-S2a-02  next_gap_event_id overflow (uint32_t wraps to 0)
//             Cause   : >4 billion gap events over the system lifetime.
//             Effect  : gap_event_id = 0 would be interpreted by downstream
//                       stages as "no gap", silently losing the event.
//             Guard   : saturate at UINT32_MAX.  A saturated id is still non-zero
//                       and distinct from 0.  At 100 Hz with 0.1% fault rate,
//                       UINT32_MAX is reached in ~13.6 years.
//
//  FM-S2a-03  total_gap_count overflow (uint64_t)
//             Cause   : overflow in ~584,542 years at 1 MHz.
//             Effect  : negligible; documented non-issue.
//             Guard   : uint64_t width is sufficient.
//
//  FM-S2a-04  consecutive_gap_count overflow (uint32_t)
//             Cause   : sensor permanently absent for >4 billion samples.
//             Effect  : counter wraps to 0, misinforming S4 policy.
//             Guard   : saturate at UINT32_MAX.
//
//  FM-S2a-05  HARD_INVALID sample reaching gap detection
//             Cause   : S1 set HARD_INVALID.  The delta_t_us is still valid
//                       (set by S0 before any calibration fault), so gap
//                       detection based on delta_t_us remains meaningful.
//             Guard   : gap detection proceeds regardless of HARD_INVALID status.
//                       S2b will suppress the PLL update on gap_pending anyway.
//                       S4 will apply policy regardless of gap detection result.
//
//  FM-S2a-06  Modification of gap_event_id when HARD_INVALID is set
//             Cause   : caller expects gap_event_id = 0 for non-gap samples
//                       and non-zero for gap samples.  Setting gap_event_id
//                       even on HARD_INVALID would corrupt the gap log.
//             Guard   : gap logic is driven solely by delta_t_us, not status flags.
//                       HARD_INVALID does not suppress gap detection.
//                       This is consistent with FM-S2a-05 above.
//
//  FM-S2a-07  gap_pending left true from a previous cycle
//             Cause   : prior sample was a gap; this sample is not.
//             Effect  : S2b incorrectly suppresses its PLL update.
//             Guard   : gap_pending is explicitly set to false on every
//                       non-gap sample.  No residue from prior cycles.
//
// =============================================================================

#include "signalfix/module1/stages/stage_s2a_gap_detection.hpp"

#include <cassert>
#include <climits>   // UINT32_MAX, UINT64_MAX
#include <cstdio>    // std::fprintf — diagnostic only

// ---------------------------------------------------------------------------
// Diagnostic macro
// ---------------------------------------------------------------------------
#ifndef SIGNALFIX_S2A_DIAG
#  define SIGNALFIX_S2A_DIAG 0
#endif

#if SIGNALFIX_S2A_DIAG
#  define S2A_LOG(fmt, ...) \
       std::fprintf(stderr, "[S2a] " fmt "\n", ##__VA_ARGS__)
#else
#  define S2A_LOG(fmt, ...) ((void)0)
#endif


namespace signalfix {

// =============================================================================
// compute_gap_threshold() — static helper, exposed for testing
// =============================================================================

uint64_t StageS2aGapDetection::compute_gap_threshold(
    uint64_t nominal_delta_t_us) noexcept
{
    // gap_threshold = nominal × 5 / 2  (= 2.5 × nominal, TDS Section 3.1.1)
    //
    // FM-S2a-01: guard the multiplication against overflow.
    // UINT64_MAX / 5 = 3,689,348,814,741,910,323.  If nominal exceeds this,
    // saturate the product at UINT64_MAX before the divide.
    static constexpr uint64_t kSafeMax = UINT64_MAX / kGapMultiplierNum;

    const uint64_t product =
        (nominal_delta_t_us <= kSafeMax)
            ? (nominal_delta_t_us * kGapMultiplierNum)
            : UINT64_MAX;

    return product / kGapMultiplierDen;
}


// =============================================================================
// StageS2aGapDetection::process() — hot path
// =============================================================================

StageResult StageS2aGapDetection::process(
    MeasurementEnvelope& envelope,
    ChannelState&        cs) noexcept
{
    // ── Debug assertions ─────────────────────────────────────────────────────
#ifndef NDEBUG
    assert(cs.nominal_delta_t_us > 0u &&
           "S2a precondition: nominal_delta_t_us must be > 0");
    assert(envelope.delta_t_us > 0u &&
           "S2a precondition: delta_t_us must be > 0 (S0 guarantees this)");
#endif

    // ── Compute gap threshold ────────────────────────────────────────────────
    //
    // FM-S2a-01: overflow-safe multiplication inside compute_gap_threshold().
    const uint64_t gap_threshold = compute_gap_threshold(cs.nominal_delta_t_us);

    S2A_LOG("channel=0x%08X seq=%llu: delta_t=%llu nominal=%llu threshold=%llu",
            envelope.channel_id,
            static_cast<unsigned long long>(envelope.sequence_id),
            static_cast<unsigned long long>(envelope.delta_t_us),
            static_cast<unsigned long long>(cs.nominal_delta_t_us),
            static_cast<unsigned long long>(gap_threshold));

    // ── Gap decision ─────────────────────────────────────────────────────────

    if (envelope.delta_t_us > gap_threshold)
    {
        // ── Gap detected ─────────────────────────────────────────────────────

        // FM-S2a-02: saturate next_gap_event_id at UINT32_MAX.
        // A saturated id is still non-zero and therefore distinguishable from
        // "no gap" (0) by downstream stages.
        if (cs.gap.next_gap_event_id == static_cast<uint32_t>(0u))
        {
            // init_channel_state sets this to 1; 0 should never appear here
            // unless state was corrupted.  Clamp to 1 defensively.
            cs.gap.next_gap_event_id = 1u;
        }

        envelope.gap_event_id = cs.gap.next_gap_event_id;

        // Advance the id, saturating at UINT32_MAX.
        if (cs.gap.next_gap_event_id < static_cast<uint32_t>(~0u))
        {
            cs.gap.next_gap_event_id++;
        }
        // (else saturated — stays at UINT32_MAX)

        // FM-S2a-04: saturate consecutive_gap_count at UINT32_MAX.
        if (cs.gap.consecutive_gap_count < static_cast<uint32_t>(~0u))
        {
            cs.gap.consecutive_gap_count++;
        }

        // FM-S2a-03: total_gap_count — uint64_t width is sufficient.
        cs.gap.total_gap_count++;

        // Signal S2b to suppress its PLL update for this sample.
        // FM-S2a-07: explicitly set (not merely left from prior state).
        cs.gap.gap_pending = true;

        // Rev 2.5: Tag as GAP
        if (envelope.failure_hint == FailureMode::NONE)
        {
            envelope.failure_hint = FailureMode::GAP;
            envelope.failure_confidence = 1.0f;
        }

        S2A_LOG("channel=0x%08X seq=%llu: GAP detected — gap_event_id=%u "
                "consecutive=%u total=%llu",
                envelope.channel_id,
                static_cast<unsigned long long>(envelope.sequence_id),
                static_cast<unsigned>(envelope.gap_event_id),
                static_cast<unsigned>(cs.gap.consecutive_gap_count),
                static_cast<unsigned long long>(cs.gap.total_gap_count));
    }
    else
    {
        // ── No gap ───────────────────────────────────────────────────────────

        // 0 is the reserved "no gap" sentinel (MeasurementEnvelope contract).
        envelope.gap_event_id = 0u;

        // FM-S2a-07: clear gap_pending so S2b applies PLL normally.
        cs.gap.gap_pending         = false;
        cs.gap.consecutive_gap_count = 0u;

        S2A_LOG("channel=0x%08X seq=%llu: no gap (delta_t=%llu <= threshold=%llu)",
                envelope.channel_id,
                static_cast<unsigned long long>(envelope.sequence_id),
                static_cast<unsigned long long>(envelope.delta_t_us),
                static_cast<unsigned long long>(gap_threshold));
    }

    return StageResult::CONTINUE;
}

} // namespace signalfix
