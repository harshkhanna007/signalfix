// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/channel_state.hpp
// Spec   : SFX-M1-TDS-001  Revision 2.1
// =============================================================================
//
// Per-channel runtime state container.
//
// ChannelState holds all data that must persist across consecutive samples for
// a single sensor channel. Each registered channel owns exactly one
// ChannelState instance, allocated inside the pipeline's internal table — no
// heap allocation.
//
// State is grouped by the pipeline stage that owns and mutates it.
// Cross-stage access is permitted for read, but a stage should only WRITE the
// fields it is contractually responsible for (documented per-field below).
//
// Capacity budget (worst-case, kRocWindowMax = 128):
//   RocWindowState.buffer : 128 × 4 = 512 bytes
//   All other fields      :           ~120 bytes
//   sizeof(ChannelState)  :         ≈ 632 bytes
//   64 channels           :         ≈ 40 KB  (fits comfortably in L2 cache)
//
// =============================================================================

#pragma once

#include <cstdint>
#include <cstring>
#include <limits>

namespace signalfix {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Maximum ROC adaptive statistics window size (samples).
/// Balances memory footprint against statistical stability.
/// S3 may select a smaller active window per channel.
static constexpr uint16_t kRocWindowMax = 128u;

/// Sentinel value used in the channel_id lookup table to mark empty slots.
static constexpr uint32_t kChannelIdEmpty = 0xFFFF'FFFFu;

// ---------------------------------------------------------------------------
// PllState — Phase-Locked Loop state for clock alignment (S2b)
//
// Maintained by stage S2b (Clock Alignment). Provides corrected timestamps
// by tracking accumulated phase error and frequency deviation between the
// sensor's internal clock and the system monotonic clock.
//
// When locked == false, S2b falls back to using arrival_time_us directly.
// ---------------------------------------------------------------------------
struct PllState {
  double phase_error_us; ///< Accumulated phase error [μs]. Written by S2b.
  double
      freq_correction_ppm; ///< Estimated clock frequency deviation [ppm]. S2b.
  uint32_t lock_sample_count; ///< Samples since PLL acquired lock. S2b.
  bool locked;     ///< True if PLL has converged and is tracking. S2b.
  uint8_t _pad[3]; ///< Explicit padding to 24-byte aligned boundary.
};
static_assert(sizeof(PllState) == 24u, "PllState layout changed unexpectedly.");

// ---------------------------------------------------------------------------
// RocWindowState — Adaptive ROC statistics window (S3)
//
// Circular buffer of recent per-sample rate-of-change values
// [physical_units/s]. S3 uses this to compute an adaptive threshold (mean +
// k_sigma × std) and compares the current sample's ROC against it.
//
// Online statistics (running_sum, running_sum_sq) support O(1) mean and
// variance computation. When the window wraps, the oldest value is
// subtracted before the newest is added.
//
// Layout note: buffer is first to guarantee 4-byte alignment throughout.
// ---------------------------------------------------------------------------
struct RocWindowState {
  float
      buffer[kRocWindowMax]; ///< Circular buffer of ROC samples. Written by S3.
  uint32_t head;             ///< Next write index (modulo window_size). S3.
  uint32_t count;    ///< Samples currently in the window (≤ window_size). S3.
  float running_sum; ///< Sum of values in the active window. S3.
  float running_sum_sq; ///< Sum of squares in the active window. S3.
  uint16_t window_size; ///< Active window length (≤ kRocWindowMax). S3.
  uint8_t _pad[2];      ///< Padding to 4-byte boundary.
};

// ---------------------------------------------------------------------------
// GapDetectionState — Gap pre-detection and policy bookkeeping (S2a, S4)
//
// S2a detects potential gaps (missing samples) using expected delta_t.
// S4 (Gap Policy Engine) decides whether to interpolate, mark MISSING,
// or pass through, and assigns a unique gap_event_id for downstream tracing.
// ---------------------------------------------------------------------------
struct GapDetectionState {
  uint32_t next_gap_event_id; ///< Next gap_event_id to assign (1-based). S4.
  uint32_t
      consecutive_gap_count; ///< Consecutive gap events (used for policy). S4.
  uint64_t total_gap_count;  ///< Lifetime gap event counter. Diagnostic. S4.
  bool gap_pending; ///< S2a detected a gap; S4 has not yet acted. S2a/S4.
  uint8_t _pad[7];  ///< Explicit padding to 8-byte boundary.
};
static_assert(sizeof(GapDetectionState) == 24u,
              "GapDetectionState layout changed unexpectedly.");

// ---------------------------------------------------------------------------
// ChannelState — complete per-channel runtime context
//
// The pipeline owns one ChannelState per registered channel.
// Indexed by position in the pipeline's channel table; looked up by channel_id.
//
// Init: call init_channel_state() before submitting any samples for a channel.
// Reset: call init_channel_state() again to restart from a clean state.
//
// Thread safety: not thread-safe. Accessed exclusively by the owning
// IngestionPipeline instance on a single producer thread.
// ---------------------------------------------------------------------------
struct ChannelState {
  // ── Identity ─────────────────────────────────────────────────────────────
  // Set at registration; immutable during normal operation.

  uint32_t channel_id; ///< Channel identifier. Matches RawSample.channel_id.
  uint32_t calibration_version; ///< Calibration config version. Stamped onto
                                ///< envelope by S0.

  // ── Sequence tracking (S0) ───────────────────────────────────────────────
  // S0 reads and increments sequence_counter for every emitted envelope.
  // Monotonically increasing, never resets. Includes synthetic STALE envelopes.

  uint64_t sequence_counter; ///< Next sequence_id to assign. S0.

  // ── Timing state (S0, S2b) ───────────────────────────────────────────────

  uint64_t
      last_arrival_us; ///< System clock timestamp of the last accepted sample.
                       ///< Updated by S0 on every non-synthetic sample.
  uint64_t last_timestamp_us; ///< PLL-corrected timestamp of the last emitted
                              ///< envelope. Updated by S2b.
  uint64_t nominal_delta_t_us; ///< Expected inter-sample interval [μs].
                               ///< Set at registration; used by S0, S2a, S2b.

  // ── Calibrated value history (S1, S3) ────────────────────────────────────

  double last_calibrated_value; ///< Calibrated value from the previous valid
                                ///< sample. Used by S3 to compute instantaneous
                                ///< ROC.
  bool last_calibrated_valid; ///< True if last_calibrated_value holds a usable
                              ///< value.
  uint8_t _pad_cal[7];        ///< Explicit padding to 8-byte boundary.

  // ── Gap detection (S2a, S4) ──────────────────────────────────────────────

  GapDetectionState gap;

  // ── PLL clock alignment (S2b) ────────────────────────────────────────────

  PllState pll;

  // ── ROC adaptive statistics (S3) ─────────────────────────────────────────

  RocWindowState
      roc_window; ///< Circular buffer + online statistics for ROC. S3.

  // ── Nominal streak (S6) ──────────────────────────────────────────────────
  // S6 reads this to populate MeasurementEnvelope.nominal_streak_count,
  // then increments or resets it based on the current sample's status.

  uint16_t nominal_streak; ///< Consecutive NOMINAL samples preceding this one.
  uint8_t _pad_streak[6];  ///< Explicit padding to 8-byte boundary.

  // ── Watchdog (S0) ────────────────────────────────────────────────────────
  // The orchestrator or a timer thread compares system time against
  // watchdog_deadline_us to decide when to call process_watchdog().
  // S0 updates watchdog_deadline_us = last_arrival_us + 2 × nominal_delta_t_us
  // on every successfully ingested sample.

  uint64_t
      watchdog_deadline_us; ///< Expiry time for watchdog [μs]. Updated by S0.
  bool watchdog_armed;      ///< False until the first sample is received. S0.
  uint8_t _pad_wd[7];       ///< Explicit padding to 8-byte boundary.
};

// ---------------------------------------------------------------------------
// init_channel_state — safe, deterministic channel state initialisation
//
// Must be called before the first sample for any channel is submitted.
// Calling again resets all state (channel re-initialisation path).
//
// Parameters:
//   cs                  — ChannelState to initialise (written in full).
//   channel_id          — Channel identifier; must match RawSample.channel_id.
//   calibration_version — Calibration config version at init time.
//   nominal_delta_t_us  — Expected inter-sample interval [μs]. Must be > 0.
//   roc_window_size     — Active ROC window length; clamped to kRocWindowMax.
//                         0 defaults to kRocWindowMax.
// ---------------------------------------------------------------------------
inline void init_channel_state(ChannelState &cs, uint32_t channel_id,
                               uint32_t calibration_version,
                               uint64_t nominal_delta_t_us,
                               uint16_t roc_window_size) noexcept {
  // Zero-initialise the entire struct first.
  // All members are trivially constructible; memset is safe and correct.
  // This prevents partial-init footguns when new fields are added.
  std::memset(&cs, 0, sizeof(ChannelState));

  cs.channel_id = channel_id;
  cs.calibration_version = calibration_version;
  cs.sequence_counter = 0u;
  cs.last_calibrated_valid = false;
  cs.watchdog_armed = false;

  // Guard against zero nominal_delta_t to prevent watchdog division-by-zero
  // in downstream stages that compute 2 × nominal_delta_t_us.
  cs.nominal_delta_t_us = (nominal_delta_t_us > 0u) ? nominal_delta_t_us : 1u;

  // Gap policy
  cs.gap.next_gap_event_id =
      1u; // 0 is reserved as "no gap" in MeasurementEnvelope

  // ROC window: clamp requested size to valid range.
  const uint16_t safe_window =
      (roc_window_size == 0u || roc_window_size > kRocWindowMax)
          ? kRocWindowMax
          : roc_window_size;
  cs.roc_window.window_size = safe_window;

  // PLL starts unlocked — S2b will acquire lock once sufficient samples arrive.
  cs.pll.locked = false;
  cs.pll.phase_error_us = 0.0;
  cs.pll.freq_correction_ppm = 0.0;
  cs.pll.lock_sample_count = 0u;
}

} // namespace signalfix
