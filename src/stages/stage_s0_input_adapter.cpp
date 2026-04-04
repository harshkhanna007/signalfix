// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : src/module1/stages/stage_s0_input_adapter.cpp
// Spec   : SFX-M1-TDS-001  Revision 2.1
// Stage  : S0 — Input Adapter
// =============================================================================
//
// Implementation notes — failure modes addressed:
//
//  FM-01  Channel ID mismatch
//         Cause  : pipeline.cpp pre-stamps envelope.channel_id from RawSample;
//                  ChannelState is keyed by slot, not re-verified by default.
//         Effect  : downstream stages operate on a misrouted channel context.
//         Guard   : explicit equality check before any mutation; ABORT_FAULT.
//
//  FM-02  sequence_id non-monotonicity
//         Cause  : post-increment skipped, or counter read before increment.
//         Effect  : M2 on_sequence_gap fires spuriously; covariance inflated.
//         Guard   : single post-increment expression; counter stored in cs.
//                   Counter never resets (only init_channel_state resets it).
//
//  FM-03  sequence_counter overflow (uint64_t)
//         Cause  : at 1 MHz sample rate, wrap-around in ~585,000 years.
//         Effect  : negligible; documented non-issue.
//         Guard   : uint64_t width is sufficient for all realistic deployments.
//
//  FM-04  Unsigned timestamp underflow (Audit B3) — FIX-1 applied
//         Cause  : arrival_time_us < last_arrival_us (NTP/PTP step-back).
//                  Previous code cast to int64_t, which is
//                  implementation-defined for values > INT64_MAX (C++17
//                  [conv.integral] p3).
//         Effect  : garbage delta_t feeds PLL, ROC, and gap detection.
//         Guard   : FIX-1: pure unsigned comparison before subtraction.
//                   if (arrival <= last) return 0; else return arrival - last.
//                   Subtraction is safe because the guard guarantees arrival >
//                   last. Caller applies TIMING_ANOMALY + nominal fallback.
//
//  FM-05  First-sample delta_t
//         Cause  : last_arrival_us == 0 on first call; delta = arrival_time_us
//                  which can be millions of μs (system uptime).
//         Effect  : spurious gap detection, PLL corruption, ROC_EXCEEDED.
//         Guard   : !cs.watchdog_armed guards first-sample path;
//                   delta_t_us = nominal_delta_t_us on first sample.
//
//  FM-06  Watchdog deadline arithmetic overflow — FIX-4 applied
//         Cause  : arrival_time_us near UINT64_MAX + 2×nominal_delta_t_us
//         wraps.
//                  Also: 2 × nominal itself can overflow if nominal >
//                  UINT64_MAX/2.
//         Effect  : watchdog_deadline_us appears in the past; immediate
//         re-fire. Guard   : FIX-4: two-level saturation in both process() and
//                   build_stale_envelope():
//                   (a) timeout = 2×nominal saturated at UINT64_MAX.
//                   (b) arrival + timeout saturated at UINT64_MAX.
//
//  FM-07  nominal_delta_t_us == 0 (division-by-zero downstream)
//         Cause  : misconfigured channel registered with nominal_delta_t = 0.
//         Effect  : watchdog fires immediately; PLL and gap detection divide by
//         0. Guard   : init_channel_state clamps to 1 μs; S0 trusts this
//         invariant
//                   but documents it for reviewers.
//
//  FM-08  Raw value NaN/Inf from hardware
//         Cause  : faulty ADC or DMA corruption producing NaN in raw_value.
//         Effect  : NaN propagates through calibration to SR-UKF.
//         Guard   : S0 does NOT guard raw_value — that is S1's responsibility
//                   (per TDS Section 5.2.1, S1 adds NaN guard after
//                   calibration). S7 NaN gate provides the final backstop. S0
//                   must not duplicate S1's responsibility.
//
//  FM-09  calibration_version mismatch (stale config)
//         Cause  : config reloaded mid-flight; cs.calibration_version updated
//                  atomically, but some envelopes in flight carry old version.
//         Effect  : M2 sees inconsistent calibration history; traceability
//         lost. Guard   : S0 copies cs.calibration_version at this sample's
//         ingestion
//                   time. The copy is atomic relative to the single-producer
//                   thread model. Config reload is out of scope for this stage.
//
//  FM-10  nominal_streak not reset after watchdog
//         Cause  : process_watchdog() resets cs.nominal_streak. S0::process()
//                  is NOT called by the watchdog path. No double-reset needed.
//         Effect  : N/A — design is correct as-is.
//         Guard   : documented. S0::process() does NOT touch cs.nominal_streak;
//                   S6 (EnvelopePackager) owns that field per the architecture.
//
//  FM-11  Heap allocation in process()
//         Guard  : no `new`, no `malloc`, no std::vector, no STL containers.
//                  All state in ChannelState (pre-allocated by pipeline).
//
//  FM-12  Non-deterministic behavior
//         Guard  : process() has no side effects beyond the two parameters.
//                  Same inputs always produce same outputs. No global state.
//
//  FM-13  TIMING_ANOMALY flag not set on regression
//         Cause  : stage silently substitutes nominal delta without flagging.
//         Effect  : M2 sees a clean envelope that is quietly wrong; PLL drifts.
//         Guard   : TIMING_ANOMALY is ALWAYS set when delta is non-positive.
//
//  FM-14  TIMING_ANOMALY set but delta_t_us = 0
//         Cause  : passing 0 as delta_t_us to downstream; S3 ROC divides by it.
//         Guard   : on regression, delta_t_us = cs.nominal_delta_t_us (≥1 μs).
//                   Zero is never written to envelope.delta_t_us.
//
//  FM-15  watchdog_deadline_us not updated for first sample
//         Cause  : first-sample guard skips the deadline update.
//         Effect  : watchdog_deadline_us stays 0; watchdog fires immediately.
//         Guard   : deadline IS updated on the first-sample path (same
//         formula).
//
//  FM-16  Modification of prior stages' status flags
//         Cause  : S0 over-writes status with assignment rather than |= .
//         Effect  : flags set by a pre-S0 wrapper or debug harness are lost.
//         Guard   : S0 only ORs new flags (|=). It never clears existing flags.
//                   make_nominal_envelope() initialises status = NOMINAL
//                   (0x00), so there are no prior flags in practice; but the
//                   rule holds.
//
//  FM-17  Extreme delta_t_us from sensor restart or OS stall — FIX-2/FIX-5
//         Cause  : sensor reboots or large scheduler preemption can produce
//                  inter-arrival intervals of hundreds of nominal periods.
//                  The watchdog should have fired, but if a real sample arrives
//                  with a huge delta, the PLL interprets it as the sensor
//                  running at near-zero frequency and applies a catastrophic
//                  correction.
//         Effect  : PLL f_corr diverges; all timestamps corrupted for 100+
//         samples.
//                   ROC window accumulates a spike that inflates the adaptive
//                   threshold, masking subsequent real anomalies.
//         Guard   : FIX-2/FIX-5: if delta > 10 × nominal → clamp to nominal
//                   and set TIMING_ANOMALY. The 10× threshold is configurable
//                   via the constant; the multiplication is overflow-guarded.
//
//  FM-18  Debug-only precondition violations not surfaced early — FIX-6
//         Cause  : nominal_delta_t_us == 0, channel_id disagreement, or NaN
//                  raw_value that S1 will catch but whose origin is obscure.
//         Effect  : failures manifest far from their root cause in debug
//         sessions. Guard   : FIX-6: assert() blocks under #ifndef NDEBUG at
//         the top of
//                   process(). Zero runtime cost in release builds.
//
// =============================================================================

#include "signalfix/module1/stages/stage_s0_input_adapter.hpp"

#include <cassert> // assert — used in debug-only invariant checks (FIX-6)

// ---------------------------------------------------------------------------
// Diagnostic macro — same convention as pipeline.cpp
// ---------------------------------------------------------------------------
#ifndef SIGNALFIX_S0_DIAG
#define SIGNALFIX_S0_DIAG 0
#endif

#if SIGNALFIX_S0_DIAG
#define S0_LOG(fmt, ...) std::fprintf(stderr, "[S0] " fmt "\n", ##__VA_ARGS__)
#else
#define S0_LOG(fmt, ...) ((void)0)
#endif

namespace signalfix {

// =============================================================================
// StageS0InputAdapter::process() — normal hot path
// =============================================================================

StageResult StageS0InputAdapter::process(MeasurementEnvelope &envelope,
                                         ChannelState &cs) noexcept {
  // ── FIX-6: Debug-only invariant assertions ───────────────────────────────
  //
  // These assertions are stripped in release builds (NDEBUG defined).
  // They verify preconditions that the pipeline contract guarantees, catching
  // integration bugs early during development and testing.
  //
  // Two properties are checked:
  //
  //   (a) nominal_delta_t_us > 0
  //       init_channel_state() clamps to 1 μs, so this should always hold.
  //       A zero nominal would cause division-by-zero in S3 ROC and PLL.
  //
  //   (b) raw_value is not NaN on arrival.
  //       make_initial_envelope() initialises raw_value from
  //       RawSample.raw_value. NaN raw_value at this point indicates the sensor
  //       driver or DMA layer has delivered a corrupt value — S1 will catch it,
  //       but flagging it here surfaces the fault closer to its origin in debug
  //       builds. NOTE: raw_value NaN is NOT an ABORT_FAULT in release; S1 owns
  //       that.
  //
  // Channel-id consistency is NOT asserted here. The hard runtime check below
  // (which returns ABORT_FAULT) is the definitive guard; an assertion would
  // prevent negative-path tests (T08) from exercising that guard in debug
  // builds.
#ifndef NDEBUG
  assert(cs.nominal_delta_t_us > 0u &&
         "S0 precondition: nominal_delta_t_us must be > 0 "
         "(init_channel_state should have clamped to 1)");

  assert(!std::isnan(envelope.raw_value) &&
         "S0 precondition: raw_value is NaN — sensor driver or DMA fault; "
         "S1 will set HARD_INVALID but this assertion surfaces the origin");
#endif

  // ── Guard 1: channel_id consistency ─────────────────────────────────────
  //
  // pipeline.cpp pre-stamps envelope.channel_id from RawSample.channel_id
  // before calling us. The channel_state was looked up by that same id.
  // If they diverge, something is wrong with the pipeline's channel table.
  // This is an unrecoverable misconfiguration — ABORT_FAULT.
  //
  // FM-01: channel mismatch detection.
  if (envelope.channel_id != cs.channel_id) {
    S0_LOG("FAULT: channel_id mismatch — envelope has 0x%08X, "
           "ChannelState has 0x%08X",
           envelope.channel_id, cs.channel_id);

    envelope.status |= SampleStatus::HARD_INVALID;
    return StageResult::ABORT_FAULT;
  }

  // ── Step 1: Assign sequence_id ───────────────────────────────────────────
  //
  // cs.sequence_counter holds the NEXT id to assign.
  // Post-increment: envelope gets the current value, counter advances.
  //
  // FM-02: guaranteed monotonicity — single post-increment in one expression.
  // FM-03: uint64_t overflow at 1 MHz in 585,000 years — acceptable.
  envelope.sequence_id = cs.sequence_counter++;

  S0_LOG("channel=0x%08X seq=%llu arrival=%llu", cs.channel_id,
         static_cast<unsigned long long>(envelope.sequence_id),
         static_cast<unsigned long long>(envelope.arrival_time_us));

  // ── Step 2: Stamp calibration_version ───────────────────────────────────
  //
  // Copied from ChannelState at this sample's ingestion time.
  // Single-threaded access; no race condition within the pipeline.
  //
  // FM-09: version is captured per-sample for traceability.
  envelope.calibration_version = cs.calibration_version;

  // ── Step 3: Compute delta_t_us (safe signed arithmetic) ─────────────────
  //
  // FM-04: unsigned underflow prevention.
  // FM-05: first-sample guard — delta_t = nominal when no prior arrival.
  // FM-13: TIMING_ANOMALY set on regression.
  // FM-14: delta_t = nominal (never 0) on fallback.
  // FM-15: watchdog deadline updated on first-sample path too.

  if (!cs.watchdog_armed) {
    // First sample for this channel: no previous arrival to compute delta.
    // Use nominal_delta_t_us as the best available estimate.
    envelope.delta_t_us = cs.nominal_delta_t_us;

    S0_LOG("first-sample: delta_t set to nominal %llu μs",
           static_cast<unsigned long long>(cs.nominal_delta_t_us));
  } else {
    // Subsequent samples: compute actual inter-arrival interval.
    const uint64_t delta =
        compute_delta_t_us(envelope.arrival_time_us, cs.last_arrival_us);

    // ── FIX-2 / FIX-3 / FIX-5: clamp and zero-guard ────────────────────
    //
    // Two independent anomaly conditions both map to the same safe fallback:
    //   delta_t_us = cs.nominal_delta_t_us + TIMING_ANOMALY.
    //
    // Condition A (FIX-3): delta == 0
    //   Timestamp regression or identical arrival timestamps.
    //   Zero must never reach downstream stages that divide by delta_t_us
    //   (S3 ROC, S2b PLL). FM-13, FM-14.
    //
    // Condition B (FIX-2 / FIX-5): delta > MAX_DELTA_T (10 × nominal)
    //   Sensors that restart, large OS scheduling preemptions, or a rogue
    //   clock source can produce intervals orders of magnitude above nominal.
    //   A legitimate gap this large should have been caught by the watchdog
    //   and emitted as STALE|MISSING.  If this sample arrives anyway, a
    //   raw delta of e.g. 500 × nominal would corrupt the PLL (which
    //   interprets it as a frequency measurement) and spike the ROC window.
    //   Clamp to nominal and flag so the PLL and ROC stages know not to
    //   treat this as a representative inter-sample interval.
    //
    // MAX_DELTA_T multiplication is itself guarded against overflow:
    //   if nominal > UINT64_MAX / 10, saturate to UINT64_MAX so the
    //   comparison still works correctly (any real delta < UINT64_MAX).

    // Compute MAX_DELTA_T with overflow saturation on the multiplication.
    const uint64_t max_delta = (cs.nominal_delta_t_us <= (UINT64_MAX / 10u))
                                   ? (10u * cs.nominal_delta_t_us)
                                   : UINT64_MAX;

    if (delta == 0u) {
      // Condition A: regression or identical timestamps.
      envelope.delta_t_us = cs.nominal_delta_t_us;
      envelope.status |= SampleStatus::TIMING_ANOMALY;

      // Rev 2.5: Tag as INVALID (regression is a physical/math impossibility)
      if (envelope.failure_hint == FailureMode::NONE) {
        envelope.failure_hint = FailureMode::INVALID;
        envelope.failure_confidence = 1.0f;
      }

      S0_LOG("TIMING_ANOMALY(regression): arrival=%llu <= last=%llu; "
             "delta_t forced to nominal %llu μs",
             static_cast<unsigned long long>(envelope.arrival_time_us),
             static_cast<unsigned long long>(cs.last_arrival_us),
             static_cast<unsigned long long>(cs.nominal_delta_t_us));
    } else if (delta > max_delta) {
      // Condition B: extreme interval exceeds 10 × nominal.
      envelope.delta_t_us = cs.nominal_delta_t_us;
      envelope.status |= SampleStatus::TIMING_ANOMALY;

      // Rev 2.5: Tag as GAP (missed watchdog or ultra-slow sensor)
      if (envelope.failure_hint == FailureMode::NONE) {
        envelope.failure_hint = FailureMode::GAP;
        envelope.failure_confidence = 1.0f;
      }

      S0_LOG("TIMING_ANOMALY(clamp): delta=%llu > max=%llu; "
             "delta_t clamped to nominal %llu μs",
             static_cast<unsigned long long>(delta),
             static_cast<unsigned long long>(max_delta),
             static_cast<unsigned long long>(cs.nominal_delta_t_us));
    } else {
      // Normal case: delta is positive and within the plausible range.
      envelope.delta_t_us = delta;
    }
  }

  // ── Step 4: Update last_arrival_us ──────────────────────────────────────
  //
  // Update AFTER computing delta_t so that the delta reflects the interval
  // from the previous sample to THIS sample, not the next one.
  cs.last_arrival_us = envelope.arrival_time_us;

  // ── Step 5: Arm / refresh watchdog ──────────────────────────────────────
  //
  // Deadline = arrival_time_us + 2 × nominal_delta_t_us.
  //
  // FM-06 / FIX-4: Two overflow hazards are guarded independently:
  //   a) The multiplication  2 × nominal  can itself overflow if nominal is
  //      near UINT64_MAX/2.  Guard with a saturation check before multiplying.
  //   b) The addition  arrival + timeout  can overflow when arrival is large.
  //      Guard with the pre-existing saturation check against UINT64_MAX.
  // FM-15: first-sample path also reaches here, so deadline is always set.
  {
    // (a) Saturate the timeout multiplication.
    const uint64_t timeout = (cs.nominal_delta_t_us <= (UINT64_MAX / 2u))
                                 ? (2u * cs.nominal_delta_t_us)
                                 : UINT64_MAX;

    // (b) Saturate the addition.
    if (envelope.arrival_time_us > (UINT64_MAX - timeout)) {
      cs.watchdog_deadline_us = UINT64_MAX;
    } else {
      cs.watchdog_deadline_us = envelope.arrival_time_us + timeout;
    }

    cs.watchdog_armed = true;

    S0_LOG("watchdog armed: deadline=%llu μs",
           static_cast<unsigned long long>(cs.watchdog_deadline_us));
  }

  // ── Step 6: Re-confirm channel_id in envelope ────────────────────────────
  //
  // pipeline.cpp pre-stamps channel_id before calling us. We verified it
  // matches cs.channel_id above. Re-assert here from cs to make S0 the
  // definitive writer, consistent with the contract documentation.
  envelope.channel_id = cs.channel_id;

  // gap_event_id is populated by S4; S0 leaves it at the make_nominal_envelope
  // default of 0 (= no gap).  Do not set it here.

  // jitter_us is computed by S2b (Clock Alignment). S0 does not touch it.

  return StageResult::CONTINUE;
}

// =============================================================================
// StageS0InputAdapter::build_stale_envelope() — watchdog injection helper
// =============================================================================

void StageS0InputAdapter::build_stale_envelope(
    MeasurementEnvelope &env, ChannelState &cs,
    uint64_t current_time_us) noexcept {
  // ── Identity ─────────────────────────────────────────────────────────────
  env.channel_id = cs.channel_id;

  // Sequence_id is incremented to preserve monotonicity across the gap.
  // FM-02: post-increment in one expression.
  env.sequence_id = cs.sequence_counter++;

  env.calibration_version = cs.calibration_version;

  // ── Timing ───────────────────────────────────────────────────────────────
  // No PLL correction possible — assign raw system clock as best estimate.
  env.arrival_time_us = current_time_us;
  env.timestamp_us = current_time_us;

  // delta_t_us = 2 × nominal_delta_t_us (the watchdog timeout interval).
  // This approximates the elapsed interval since the last real sample
  // was within the watchdog deadline window.
  // FIX-4: guard the multiplication against overflow before assigning.
  env.delta_t_us = (cs.nominal_delta_t_us <= (UINT64_MAX / 2u))
                       ? (2u * cs.nominal_delta_t_us)
                       : UINT64_MAX;
  env.jitter_us = 0;

  // gap_event_id is 0: watchdog is not a gap-fill event; it signals absence.
  env.gap_event_id = 0u;

  // ── Diagnostics ──────────────────────────────────────────────────────────
  // nominal_streak_count carries the streak count that was valid immediately
  // before this timeout. S6 would normally maintain this; on the watchdog
  // path we stamp it directly from ChannelState so M2 sees the pre-stall
  // streak for diagnostic purposes.
  env.nominal_streak_count = cs.nominal_streak;
  env.roc_window_n = 0u; // No ROC derivable from a synthetic sample.

  // roc and roc_adaptive_limit remain NaN (from make_nominal_envelope
  // baseline). The S7 gate permits NaN here because MISSING | STALE is set
  // below.

  // ── Status — STALE | MISSING (canonical watchdog combination) ────────────
  // Per SFX-M1-TDS-001 Rev 2.1: STALE is always paired with MISSING.
  // validate_status_flags() must accept this combination (no INV covers it).
  env.status = SampleStatus::STALE | SampleStatus::MISSING;
  env.measurement_trust_tier = MeasurementTrustTier::REJECTED;

  // Rev 2.5: Unified failure classification
  env.failure_hint       = FailureMode::STALE;
  env.failure_confidence = 1.0f;

  // Value fields remain NaN from make_nominal_envelope — no measurement data.
  // raw_value, calibrated_value, validated_value, filtered_value = NaN.

  // Filter fields.
  env.status = clear_flag(env.status, SampleStatus::PRE_FILTER_OK);
  env.filter_window_n = 0u;

  // ── ChannelState mutations ────────────────────────────────────────────────
  // Advance watchdog deadline by another 2 × nominal period so the timer
  // does not immediately re-fire if the sensor is still absent.
  //
  // FIX-4: same two-level saturation as process():
  //   (a) 2 × nominal multiplication saturates at UINT64_MAX.
  //   (b) current_time + timeout addition saturates at UINT64_MAX.
  {
    // (a) Saturate the timeout multiplication.
    const uint64_t timeout = (cs.nominal_delta_t_us <= (UINT64_MAX / 2u))
                                 ? (2u * cs.nominal_delta_t_us)
                                 : UINT64_MAX;

    // (b) Saturate the addition.
    if (current_time_us > (UINT64_MAX - timeout)) {
      cs.watchdog_deadline_us = UINT64_MAX;
    } else {
      cs.watchdog_deadline_us = current_time_us + timeout;
    }
  }

  // FM-10: nominal_streak reset — watchdog break ends any nominal run.
  // S6 is not called on the watchdog path; reset directly here.
  cs.nominal_streak = 0u;

  S0_LOG("STALE|MISSING injected: channel=0x%08X seq=%llu time=%llu",
         cs.channel_id, static_cast<unsigned long long>(env.sequence_id),
         static_cast<unsigned long long>(current_time_us));
}

// =============================================================================
// StageS0InputAdapter::compute_delta_t_us() — private helper
// =============================================================================

uint64_t
StageS0InputAdapter::compute_delta_t_us(uint64_t arrival_time_us,
                                        uint64_t last_arrival_us) noexcept {
  // FIX-1: Pure unsigned arithmetic replaces the previous signed-cast approach.
  //
  // The signed cast  static_cast<int64_t>(uint64_t)  is implementation-defined
  // for values > INT64_MAX (C++17 [conv.integral] p3), which is reachable on
  // a system that has been running for ~292,000 years or uses a non-epoch-based
  // monotonic clock with a large base value.  The unsigned comparator below is
  // well-defined for all uint64_t inputs and produces identical semantics:
  //
  //   • arrival <= last  → regression or identical timestamp → return 0.
  //     Caller applies TIMING_ANOMALY + nominal fallback (FM-13, FM-14).
  //   • arrival >  last  → valid positive interval; subtraction cannot wrap.
  //
  // Note: a genuine clock step-back (NTP/PTP) produces arrival < last, which
  // is correctly caught by the <= guard.  Unsigned subtraction is safe because
  // the guard guarantees arrival_time_us > last_arrival_us before the subtract.
  if (arrival_time_us <= last_arrival_us) {
    return 0u;
  }

  return arrival_time_us - last_arrival_us;
}

} // namespace signalfix
