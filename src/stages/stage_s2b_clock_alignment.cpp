// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : src/module1/stages/stage_s2b_clock_alignment.cpp
// Spec   : SFX-M1-TDS-001  Revision 2.1
// Stage  : S2b — Clock Alignment & Jitter Correction (PLL)
// =============================================================================
//
// Implementation notes — failure modes addressed:
//
//  FM-S2b-01  PLL update on gap sample corrupts frequency estimate
//             Cause   : A 500 ms sensor dropout creates Δt_raw = 500 ms on a
//                       100 Hz channel.  Without suppression, the PLL
//                       interprets this as f_actual ≈ 2 Hz and applies a large
//                       frequency correction.  Overcorrection then persists for
//                       100+ samples.
//             Guard   : Check cs.gap.gap_pending before any PLL mutation.
//                       On gap: hold PLL state; advance timestamp by nominal
//                       only. (TDS Section 3.1.1 "Why suppressing the PLL
//                       update is correct.")
//
//  FM-S2b-02  Timestamp regression (non-monotonic timestamp_us)
//             Cause   : PLL correction overshoots (large negative phase error);
//                       corrected timestamp falls below previous corrected
//                       timestamp.
//             Effect  : M2 state estimator receives non-monotonic timestamps.
//                       Many Kalman implementations assume Δt > 0 in the
//                       predict step.
//             Guard   : after computing corrected_ts, if it is ≤
//             cs.last_timestamp_us,
//                       retain cs.last_timestamp_us, set TIMING_ANOMALY, and
//                       continue. Monotonicity is a hard invariant (TDS
//                       Section 3.1.3).
//
//  FM-S2b-03  Signed arithmetic for phase error
//             Cause   : arrival_time_us and t_estimated are both uint64_t.
//                       Subtracting unsigned values where arrival < t_estimated
//                       (sensor arriving early) wraps to a huge positive value,
//                       yielding a wildly incorrect correction.
//             Guard   : cast both operands to int64_t before subtraction.
//                       This is safe because both are reasonable clock values
//                       (neither is near UINT64_MAX), so the cast is
//                       well-defined. The result fits in int64_t for any
//                       realistic jitter.
//
//  FM-S2b-04  Double-precision overflow in freq_correction accumulation
//             Cause   : unbounded accumulation of K_f × φ_err over many
//             samples. Effect  : freq_correction grows to Inf; all subsequent
//             timestamps
//                       become Inf or NaN.
//             Guard   : clamp freq_correction_ppm to ±kMaxFreqCorrection_us
//                       after each update.  This limits the integrator's range.
//                       The clamp value (±10,000 μs) is generous for any
//                       realistic sensor clock drift.
//
//  FM-S2b-05  jitter_us overflow (int64_t → int32_t)
//             Cause   : extreme jitter (sensor restart, OS suspend) can produce
//                       differences that exceed INT32_MAX.
//             Guard   : clamp_to_int32() saturates at ±INT32_MAX.
//
//  FM-S2b-06  First-sample timestamp (last_timestamp_us == 0)
//             Cause   : on the first sample, cs.last_timestamp_us is 0 (from
//                       init_channel_state memset).  t_estimated = 0 + nominal.
//                       Phase error = arrival_time_us − nominal, which is the
//                       system uptime — a very large value.  This would cause
//                       the integrator to absorb a huge correction permanently.
//             Guard   : on first sample (lock_sample_count == 0 AND !locked),
//                       bootstrap by setting timestamp_us = arrival_time_us
//                       (no correction), storing last_timestamp_us =
//                       arrival_time_us, and skipping all PLL updates.  PLL
//                       starts fresh from there.
//
//  FM-S2b-07  Heap allocation
//             Guard   : none used.
//
//  FM-S2b-08  HARD_INVALID samples passing through clock alignment
//             Cause   : S1 may set HARD_INVALID.  The timestamp is still
//             meaningful
//                       even for invalid calibrated values (it records when the
//                       sensor event occurred).  The PLL should still track
//                       time for non-gap HARD_INVALID samples.
//             Guard   : S2b does not check HARD_INVALID.  It processes all
//             non-gap
//                       samples for timestamp purposes.  This is correct.
//
// =============================================================================

#include "signalfix/module1/stages/stage_s2b_clock_alignment.hpp"

#include <cassert>
#include <cstdint>

// ---------------------------------------------------------------------------
// Diagnostic macro
// ---------------------------------------------------------------------------
#ifndef SIGNALFIX_S2B_DIAG
#define SIGNALFIX_S2B_DIAG 0
#endif

#if SIGNALFIX_S2B_DIAG
#define S2B_LOG(fmt, ...) std::fprintf(stderr, "[S2b] " fmt "\n", ##__VA_ARGS__)
#else
#define S2B_LOG(fmt, ...) ((void)0)
#endif

namespace signalfix {

// ---------------------------------------------------------------------------
// FM-S2b-04: maximum integrator range [μs]
// Limits accumulated freq_correction_ppm to ±10,000 μs to prevent Inf blow-up.
// ---------------------------------------------------------------------------
static constexpr double kMaxFreqCorrection_us = 10'000.0;

// =============================================================================
// Private helpers
// =============================================================================

int32_t StageS2bClockAlignment::clamp_to_int32(int64_t v) noexcept {
  // FM-S2b-05: saturating cast from int64_t to int32_t.
  if (v > static_cast<int64_t>(INT32_MAX))
    return INT32_MAX;
  if (v < static_cast<int64_t>(INT32_MIN))
    return INT32_MIN;
  return static_cast<int32_t>(v);
}

uint64_t
StageS2bClockAlignment::safe_timestamp_advance(uint64_t base_us,
                                               uint64_t delta_us) noexcept {
  // Overflow-safe addition for timestamp fields.
  if (base_us > (UINT64_MAX - delta_us)) {
    return UINT64_MAX;
  }
  return base_us + delta_us;
}

// =============================================================================
// StageS2bClockAlignment::process() — hot path
// =============================================================================

StageResult StageS2bClockAlignment::process(MeasurementEnvelope &envelope,
                                            ChannelState &cs) noexcept {
  // ── Debug assertions ─────────────────────────────────────────────────────
#ifndef NDEBUG
  assert(cs.nominal_delta_t_us > 0u &&
         "S2b precondition: nominal_delta_t_us must be > 0");
  assert(envelope.delta_t_us > 0u &&
         "S2b precondition: delta_t_us must be > 0 (guaranteed by S0)");
#endif

  // ── FM-S2b-06: first-sample bootstrap ────────────────────────────────────
  //
  // On the very first sample (lock_sample_count == 0 and not locked),
  // last_timestamp_us is 0.  Computing t_estimated = 0 + nominal would
  // produce a huge phase error equal to system uptime.  Bootstrap instead
  // by anchoring to the raw arrival_time_us, then start PLL from there.
  if (cs.pll.lock_sample_count == 0u && !cs.pll.locked) {
    envelope.timestamp_us = envelope.arrival_time_us;
    envelope.jitter_us = 0;
    cs.last_timestamp_us = envelope.arrival_time_us;
    cs.pll.lock_sample_count = 1u;

    // freq_correction starts at 0.0 (from init_channel_state memset).
    // phase_error_us starts at 0.0.

    S2B_LOG("channel=0x%08X seq=%llu: first-sample bootstrap — "
            "timestamp anchored to arrival=%llu",
            envelope.channel_id,
            static_cast<unsigned long long>(envelope.sequence_id),
            static_cast<unsigned long long>(envelope.arrival_time_us));

    return StageResult::CONTINUE;
  }

  // ── FM-S2b-01: gap path — hold PLL, advance nominally ────────────────────
  if (cs.gap.gap_pending) {
    // Advance timestamp by one nominal period.  No phase or frequency update.
    const uint64_t gap_ts =
        safe_timestamp_advance(cs.last_timestamp_us, cs.nominal_delta_t_us);

    // FM-S2b-02: enforce monotonicity (in theory gap_ts > last, but guard).
    const uint64_t final_ts =
        (gap_ts > cs.last_timestamp_us) ? gap_ts : cs.last_timestamp_us;

    envelope.timestamp_us = final_ts;

    // jitter: difference between raw arrival and the gap-filled timestamp.
    const int64_t jitter_raw = static_cast<int64_t>(envelope.arrival_time_us) -
                               static_cast<int64_t>(final_ts);

    envelope.jitter_us = clamp_to_int32(jitter_raw); // FM-S2b-05

    // TIMING_ANOMALY: gap implies timing is unreliable for this sample.
    envelope.status |= SampleStatus::TIMING_ANOMALY;

    cs.last_timestamp_us = final_ts;

    S2B_LOG("channel=0x%08X seq=%llu: gap path — PLL held, "
            "timestamp=%llu jitter=%d",
            envelope.channel_id,
            static_cast<unsigned long long>(envelope.sequence_id),
            static_cast<unsigned long long>(final_ts),
            static_cast<int>(envelope.jitter_us));

    return StageResult::CONTINUE;
  }

  // ── Nominal PLL path ─────────────────────────────────────────────────────

  // t_estimated: where we expect this sample based on the previous
  // corrected timestamp + one nominal period.
  const uint64_t t_estimated =
      safe_timestamp_advance(cs.last_timestamp_us, cs.nominal_delta_t_us);

  // FM-S2b-03: signed phase error.
  // Both values are finite system timestamps well below INT64_MAX.
  const int64_t phi_err_i64 = static_cast<int64_t>(envelope.arrival_time_us) -
                              static_cast<int64_t>(t_estimated);

  const double phi_err = static_cast<double>(phi_err_i64);

  // ── Proportional correction (phase) ──────────────────────────────────────
  //
  // corrected_ts = t_estimated + K_p × φ_err
  //              + freq_correction  (integral term accumulated over time)
  //
  // Compute the full double correction, then convert to uint64_t carefully.
  const double correction_d =
      (kKp * phi_err) + (cs.pll.locked ? cs.pll.freq_correction_ppm : 0.0);

  // Apply correction to t_estimated.  The result must be:
  //   (a) Non-negative (timestamps cannot be negative).
  //   (b) Representable as uint64_t (no Inf or NaN from correction).
  const double corrected_ts_d = static_cast<double>(t_estimated) + correction_d;

  uint64_t corrected_ts;
  if (std::isnan(corrected_ts_d) || std::isinf(corrected_ts_d) ||
      corrected_ts_d < 0.0) {
    // Correction produced an invalid value — fall back to t_estimated.
    corrected_ts = t_estimated;
    envelope.status |= SampleStatus::TIMING_ANOMALY;

    S2B_LOG("channel=0x%08X seq=%llu: correction invalid (%g) — "
            "falling back to t_estimated",
            envelope.channel_id,
            static_cast<unsigned long long>(envelope.sequence_id),
            corrected_ts_d);
  } else if (corrected_ts_d > static_cast<double>(UINT64_MAX)) {
    corrected_ts = UINT64_MAX;
    envelope.status |= SampleStatus::TIMING_ANOMALY;
  } else {
    corrected_ts = static_cast<uint64_t>(corrected_ts_d);
  }

  // ── FM-S2b-02: monotonicity enforcement ──────────────────────────────────
  //
  // corrected_ts must be strictly greater than last_timestamp_us.
  // If the PLL overcorrects backwards, clamp and flag.
  if (corrected_ts <= cs.last_timestamp_us) {
    corrected_ts = cs.last_timestamp_us; // hold — do not regress
    envelope.status |= SampleStatus::TIMING_ANOMALY;

    S2B_LOG("channel=0x%08X seq=%llu: TIMING_ANOMALY — "
            "corrected_ts(%llu) <= last(%llu); held",
            envelope.channel_id,
            static_cast<unsigned long long>(envelope.sequence_id),
            static_cast<unsigned long long>(corrected_ts),
            static_cast<unsigned long long>(cs.last_timestamp_us));
  }

  envelope.timestamp_us = corrected_ts;

  // ── Compute jitter (signed: arrival − corrected timestamp) ───────────────
  //
  // jitter_us field contract: arrival_time_us - timestamp_us [μs] (signed).
  const int64_t jitter_raw = static_cast<int64_t>(envelope.arrival_time_us) -
                             static_cast<int64_t>(corrected_ts);

  envelope.jitter_us = clamp_to_int32(jitter_raw); // FM-S2b-05

  // ── PLL state update ─────────────────────────────────────────────────────

  // Store phase error for diagnostics.
  cs.pll.phase_error_us = phi_err;

  // Integral (frequency) update — only after lock to avoid bootstrap pollution.
  if (cs.pll.locked) {
    const double new_freq_corr = cs.pll.freq_correction_ppm + (kKf * phi_err);

    // FM-S2b-04: clamp the integrator to prevent divergence.
    if (new_freq_corr > kMaxFreqCorrection_us) {
      cs.pll.freq_correction_ppm = kMaxFreqCorrection_us;
    } else if (new_freq_corr < -kMaxFreqCorrection_us) {
      cs.pll.freq_correction_ppm = -kMaxFreqCorrection_us;
    } else {
      cs.pll.freq_correction_ppm = new_freq_corr;
    }
  }

  // Advance lock counter; mark locked once threshold reached.
  if (cs.pll.lock_sample_count < UINT32_MAX) {
    cs.pll.lock_sample_count++;
  }
  if (!cs.pll.locked && cs.pll.lock_sample_count >= kLockThreshold) {
    cs.pll.locked = true;

    S2B_LOG("channel=0x%08X: PLL LOCKED after %u samples", envelope.channel_id,
            static_cast<unsigned>(cs.pll.lock_sample_count));
  }

  // Update last corrected timestamp for the next sample's t_estimated.
  cs.last_timestamp_us = corrected_ts;

  S2B_LOG("channel=0x%08X seq=%llu: arrival=%llu t_est=%llu "
          "phi_err=%g corrected=%llu jitter=%d locked=%d",
          envelope.channel_id,
          static_cast<unsigned long long>(envelope.sequence_id),
          static_cast<unsigned long long>(envelope.arrival_time_us),
          static_cast<unsigned long long>(t_estimated), phi_err,
          static_cast<unsigned long long>(corrected_ts),
          static_cast<int>(envelope.jitter_us),
          static_cast<int>(cs.pll.locked));

  return StageResult::CONTINUE;
}

} // namespace signalfix
