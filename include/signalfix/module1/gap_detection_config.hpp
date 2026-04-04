// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/gap_detection_config.hpp
// =============================================================================

#pragma once

#include <cstdint>

namespace signalfix {

/// Gap severity classification.
enum class GapSeverity : uint8_t {
    GAP_NONE     = 0, ///< Interval within jitter tolerance.
    GAP_MINOR    = 1, ///< Small overrun; TIMING_ANOMALY set.
    GAP_MAJOR    = 2, ///< Significant overrun; MISSING set, gap_event_id assigned.
    GAP_CRITICAL = 3  ///< Extreme overrun; HARD_INVALID set or ABORT_DROP.
};

/// Configuration for the S4 Gap Policy Engine.
struct GapDetectionConfig {
    uint16_t minor_gap_threshold_pct;    ///< e.g., 150 = 1.5× nominal.
    uint16_t major_gap_threshold_pct;    ///< e.g., 500 = 5.0× nominal.
    uint16_t critical_gap_threshold_pct; ///< e.g., 1000 = 10.0× nominal.
    uint64_t jitter_tolerance_us;        ///< Static jitter budget (e.g., 200μs).
    bool     drop_on_critical;           ///< If true, GAP_CRITICAL returns ABORT_DROP.
};

} // namespace signalfix
