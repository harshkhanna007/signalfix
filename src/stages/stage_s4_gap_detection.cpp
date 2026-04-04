// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : src/module1/stage_s4_gap_detection.cpp
// =============================================================================

#include "signalfix/module1/stages/stage_s4_gap_detection.hpp"
#include <cstdint>

// Note: Assumes types.hpp defines these per the architecture audit context.
// SampleStatus::TIMING_ANOMALY, SampleStatus::MISSING,
// SampleStatus::HARD_INVALID

namespace signalfix {

StageS4GapDetection::StageS4GapDetection(
    const GapDetectionConfig &config) noexcept
    : config_(config) {}

const char *StageS4GapDetection::stage_name() const noexcept {
  return "S4-GapDetection";
}

void StageS4GapDetection::reset() noexcept {
  // Stateless stage. Channel-specific state is reset via channel_state.hpp
}

GapSeverity
StageS4GapDetection::classify_gap(uint64_t delta_t_us,
                                  uint64_t nominal_us) const noexcept {
  // Protect against division by zero; enforced by init_channel_state but
  // defensive programming guarantees safety in the hot loop.
  if (nominal_us == 0u) {
    return GapSeverity::GAP_NONE;
  }

  // O(1) integer arithmetic.
  // Safe from overflow for typical nominal_us (e.g., 1000μs) and pct (< 10000).
  const uint64_t minor_thresh =
      (nominal_us * config_.minor_gap_threshold_pct) / 100u;
  const uint64_t major_thresh =
      (nominal_us * config_.major_gap_threshold_pct) / 100u;
  const uint64_t crit_thresh =
      (nominal_us * config_.critical_gap_threshold_pct) / 100u;

  if (delta_t_us >= crit_thresh) {
    return GapSeverity::GAP_CRITICAL;
  }
  if (delta_t_us >= major_thresh) {
    return GapSeverity::GAP_MAJOR;
  }
  if (delta_t_us >= minor_thresh) {
    return GapSeverity::GAP_MINOR;
  }

  return GapSeverity::GAP_NONE;
}

StageResult StageS4GapDetection::process(MeasurementEnvelope &envelope,
                                         ChannelState &channel_state) noexcept {
  // Fetch timing parameters.
  // envelope.delta_t_us is assumed populated by S0/S2b (Rev 3.0 spec).
  const uint64_t delta_t = envelope.delta_t_us;
  const uint64_t nominal = channel_state.nominal_delta_t_us;

  // Handle initial sample sequence or synthetic watchdogs where delta might be
  // 0.
  if (delta_t == 0u) {
    channel_state.gap.consecutive_gap_count = 0u;
    channel_state.gap.gap_pending = false;
    return StageResult::CONTINUE;
  }

  const GapSeverity severity = classify_gap(delta_t, nominal);

  // Apply strict jitter budget defined in C10.
  // If it's technically GAP_NONE but exceeds strict jitter, flag as anomaly.
  if (severity == GapSeverity::GAP_NONE) {
    if (delta_t > (nominal + config_.jitter_tolerance_us)) {
      // Note: Bitwise logic relies on SampleStatus bitmasks from types.hpp
      envelope.status = static_cast<SampleStatus>(
          static_cast<uint16_t>(envelope.status) |
          static_cast<uint16_t>(SampleStatus::TIMING_ANOMALY));
    }

    // Reset state on healthy sample
    channel_state.gap.consecutive_gap_count = 0u;
    channel_state.gap.gap_pending = false;
    return StageResult::CONTINUE;
  }

  // Process Major/Critical gaps that require policy intervention
  if (severity >= GapSeverity::GAP_MAJOR) {
    // Increment and assign unique gap event ID
    envelope.gap_event_id = channel_state.gap.next_gap_event_id++;
    if (channel_state.gap.next_gap_event_id == 0u) {
      channel_state.gap.next_gap_event_id =
          1u; // Prevent wrapping to 0 (No Gap)
    }

    channel_state.gap.consecutive_gap_count++;
    channel_state.gap.total_gap_count++;
    channel_state.gap.gap_pending = false; // Claim the pending gap

    envelope.status =
        static_cast<SampleStatus>(static_cast<uint16_t>(envelope.status) |
                                  static_cast<uint16_t>(SampleStatus::MISSING));

    // Rev 2.6: Hierarchical upgrade to GAP
    upgrade_failure_hint(envelope, FailureMode::GAP, 1.0f);

    if (severity == GapSeverity::GAP_CRITICAL) {
      if (config_.drop_on_critical) {
        return StageResult::ABORT_DROP;
      } else {
        envelope.status = static_cast<SampleStatus>(
            static_cast<uint16_t>(envelope.status) |
            static_cast<uint16_t>(SampleStatus::HARD_INVALID));

        // Rev 2.6: Critical gap with HARD_INVALID is effectively INVALID
        upgrade_failure_hint(envelope, FailureMode::INVALID, 1.0f);
      }
    }
  } else if (severity == GapSeverity::GAP_MINOR) {
    // Minor gaps don't increment the event ID, but flag anomaly.
    envelope.status = static_cast<SampleStatus>(
        static_cast<uint16_t>(envelope.status) |
        static_cast<uint16_t>(SampleStatus::TIMING_ANOMALY));

    channel_state.gap.gap_pending = false;
  }

  return StageResult::CONTINUE;
}

} // namespace signalfix