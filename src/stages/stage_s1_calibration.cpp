// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : src/module1/stages/stage_s1_calibration.cpp
// Spec   : SFX-M1-TDS-001  Revision 2.1
// Stage  : S1 — Calibration Transform
// =============================================================================
//
// Implementation notes — failure modes addressed:
//
//  FM-S1-01  NaN / Inf raw_value from sensor hardware or DMA corruption
//            Cause   : faulty ADC, DMA read error, or transport fault.
//            Effect  : NaN propagates silently through calibration arithmetic;
//                      calibrated_value = NaN × gain + offset = NaN. The S3
//                      plausibility checks would catch it, but only after S2b
//                      has already attempted PLL correction with a garbage value.
//            Guard   : std::isnan / std::isinf guard on raw_value BEFORE any
//                      arithmetic.  Sets HARD_INVALID and exits early.
//                      calibrated_value is left as NaN (make_nominal_envelope
//                      baseline) so the S7 gate catches any slip-through.
//
//  FM-S1-02  Calibration arithmetic produces NaN / Inf (degenerate coefficients)
//            Cause   : gain_ = Inf, gain_ = NaN (misconfiguration);
//                      raw_value near DBL_MAX × finite gain overflows to Inf.
//            Effect  : calibrated_value = Inf reaches S3, which flags
//                      HARD_INVALID, but only after the value has transited
//                      S2b where it would be read as a valid delta for PLL.
//            Guard   : post-arithmetic NaN/Inf guard on calibrated_value.
//                      Sets HARD_INVALID and writes NaN.
//                      cs history is NOT updated on this path.
//
//  FM-S1-03  HARD_INVALID already set by a prior stage
//            Cause   : S0 theoretically cannot set HARD_INVALID and still return
//                      CONTINUE (it returns ABORT_FAULT instead).  However, a
//                      future stage inserted before S1 might.
//            Effect  : S1 would attempt calibration on a value that prior logic
//                      has already deemed invalid, wasting cycles and potentially
//                      overwriting HARD_INVALID with a nominal-looking calibrated
//                      result (if raw_value happened to be finite despite the flag).
//            Guard   : early exit if HARD_INVALID is already set.
//                      calibrated_value is left as NaN.  cs not updated.
//
//  FM-S1-04  last_calibrated_valid not cleared on fault
//            Cause   : previous call succeeded (valid = true); this call faults.
//                      If valid is left as true, S3 will compute a ROC against
//                      the old calibrated value, producing a spurious spike.
//            Guard   : on any fault path (FM-S1-01, FM-S1-02, FM-S1-03),
//                      cs.last_calibrated_valid is set to false to invalidate
//                      the history.  S3 skips ROC computation on the next sample.
//
//  FM-S1-05  Heap allocation
//            Guard   : none used.  Coefficients stored as plain doubles in the
//                      stage object (stack-allocated by the pipeline owner).
//
//  FM-S1-06  Modification of prior status flags
//            Guard   : only |= is used.  No status bits are cleared.
//
// =============================================================================

#include "signalfix/module1/stages/stage_s1_calibration.hpp"

#include <cassert>
#include <cstdio>    // std::fprintf — diagnostic only
#include <cmath>     // std::isnan, std::isinf
#include <limits>    // std::numeric_limits

// ---------------------------------------------------------------------------
// Diagnostic macro — same convention as pipeline.cpp / S0
// ---------------------------------------------------------------------------
#ifndef SIGNALFIX_S1_DIAG
#  define SIGNALFIX_S1_DIAG 0
#endif

#if SIGNALFIX_S1_DIAG
#  define S1_LOG(fmt, ...) \
       std::fprintf(stderr, "[S1] " fmt "\n", ##__VA_ARGS__)
#else
#  define S1_LOG(fmt, ...) ((void)0)
#endif


namespace signalfix {

// =============================================================================
// Construction
// =============================================================================

StageS1Calibration::StageS1Calibration(double gain, double offset) noexcept
    : gain_(gain), offset_(offset)
{
    // Degenerate coefficient detection in debug builds.
    // A NaN or Inf gain/offset would silently corrupt every sample.
#ifndef NDEBUG
    assert(!std::isnan(gain_)   && "S1: gain must not be NaN");
    assert(!std::isinf(gain_)   && "S1: gain must not be Inf");
    assert(!std::isnan(offset_) && "S1: offset must not be NaN");
    assert(!std::isinf(offset_) && "S1: offset must not be Inf");
#endif
}


// =============================================================================
// StageS1Calibration::process() — hot path
// =============================================================================

StageResult StageS1Calibration::process(
    MeasurementEnvelope& envelope,
    ChannelState&        cs) noexcept
{
    // ── FIX-S1-06 debug invariant ────────────────────────────────────────────
#ifndef NDEBUG
    assert(cs.nominal_delta_t_us > 0u &&
           "S1 precondition: nominal_delta_t_us must be > 0");
#endif

    // ── FM-S1-03: early exit if a prior stage already declared this invalid ──
    //
    // S1 must not attempt calibration on a value that has already been
    // condemned.  Leave calibrated_value as NaN (make_nominal_envelope default)
    // and invalidate history so S3 does not produce a spurious ROC spike.
    if (has_flag(envelope.status, SampleStatus::HARD_INVALID))
    {
        S1_LOG("channel=0x%08X seq=%llu: HARD_INVALID already set — skip calibration",
               envelope.channel_id,
               static_cast<unsigned long long>(envelope.sequence_id));

        // Rev 2.5: Ensure failure_hint is set if not already present.
        if (envelope.failure_hint == FailureMode::NONE)
        {
            envelope.failure_hint = FailureMode::INVALID;
            envelope.failure_confidence = 1.0f;
        }

        // FM-S1-04: invalidate history so S3 skips ROC on recovery.
        cs.last_calibrated_valid = false;
        return StageResult::CONTINUE;
    }

    // ── FM-S1-01: guard raw_value before any arithmetic ──────────────────────
    //
    // NaN or Inf raw_value indicates sensor hardware or transport fault.
    // S7 gate would catch any slip-through, but the flag must be set here
    // to prevent the PLL and plausibility stages from operating on garbage.
    if (std::isnan(envelope.raw_value) || std::isinf(envelope.raw_value))
    {
        S1_LOG("channel=0x%08X seq=%llu: raw_value=%g — HARD_INVALID",
               envelope.channel_id,
               static_cast<unsigned long long>(envelope.sequence_id),
               envelope.raw_value);

        // calibrated_value remains NaN (make_nominal_envelope baseline).
        envelope.status |= SampleStatus::HARD_INVALID;

        // Rev 2.5: Tag as INVALID
        envelope.failure_hint = FailureMode::INVALID;
        envelope.failure_confidence = 1.0f;

        // FM-S1-04: invalidate history.
        cs.last_calibrated_valid = false;
        return StageResult::CONTINUE;
    }

    // ── Apply linear calibration transform ───────────────────────────────────
    //
    // calibrated_value = (raw_value × gain_) + offset_
    //
    // Both raw_value and the coefficients have been validated for NaN/Inf
    // above and in the constructor respectively.  The only remaining risk is
    // arithmetic overflow (e.g., raw_value ≈ DBL_MAX, gain_ > 1).
    const double calibrated = (envelope.raw_value * gain_) + offset_;

    // ── FM-S1-02: guard calibration output ───────────────────────────────────
    //
    // Degenerate coefficients or extreme raw values can produce Inf output
    // even with finite inputs.  Check before writing to the envelope.
    if (std::isnan(calibrated) || std::isinf(calibrated))
    {
        S1_LOG("channel=0x%08X seq=%llu: calibration produced %g "
               "(raw=%g gain=%g offset=%g) — HARD_INVALID",
               envelope.channel_id,
               static_cast<unsigned long long>(envelope.sequence_id),
               calibrated, envelope.raw_value, gain_, offset_);

        // Write NaN explicitly to make the fault visible to the S7 gate.
        envelope.calibrated_value = std::numeric_limits<double>::quiet_NaN();
        envelope.status          |= SampleStatus::HARD_INVALID;

        // Rev 2.5: Tag as INVALID
        envelope.failure_hint = FailureMode::INVALID;
        envelope.failure_confidence = 1.0f;

        // FM-S1-04: invalidate history.
        cs.last_calibrated_valid = false;
        return StageResult::CONTINUE;
    }

    // ── Nominal path ─────────────────────────────────────────────────────────

    envelope.calibrated_value = calibrated;

    S1_LOG("channel=0x%08X seq=%llu: raw=%g calibrated=%g",
           envelope.channel_id,
           static_cast<unsigned long long>(envelope.sequence_id),
           envelope.raw_value, calibrated);

    // ── Update ChannelState history for S3 ROC computation ───────────────────
    //
    // S3 reads last_calibrated_value to compute the instantaneous ROC.
    // Update only on the nominal (valid) path.
    cs.last_calibrated_value = calibrated;
    cs.last_calibrated_valid = true;

    return StageResult::CONTINUE;
}

} // namespace signalfix
