// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/output_types.hpp
// =============================================================================

#pragma once

#include <cstdint>
#include "types.hpp"

namespace signalfix {

/// Categorical trust classification for processed samples.
enum class SampleQuality : uint8_t {
    GOOD     = 0, ///< Fully reliable; within all statistical bounds.
    DEGRADED = 1, ///< Statistically suspect or minor timing jitter; usable with inflated covariance.
    BAD      = 2  ///< Significant anomaly or missing data; rejected by safety gates.
};

/// Final output packet for downstream consumers (e.g., Kalman Filter).
/// Guaranteed layout: 32 bytes (Rev 2.5).
struct ProcessedSample {
    double             value;              ///< Validated and filtered measurement [physical units].
    uint64_t           timestamp_us;       ///< Strictly monotonic, corrected system time [μs].
    float              dt;                 ///< Corrected inter-sample interval [s].
    SampleQuality      quality;            ///< Categorical trust classification.
    FailureMode        failure_hint;       ///< Strategic failure classification.
    uint8_t            _pad_align_conf[2];  ///< Alignment padding.
    float              failure_confidence; ///< Confidence in failure_hint [0.0 - 1.0].
    uint32_t           failure_duration;   ///< Consecutive samples in current failure (Rev 2.6).
};
static_assert(sizeof(ProcessedSample) == 32u, "ProcessedSample layout mismatch (Rev 2.5/2.6).");

/// Downstream callback for processed output.
/// Returns true if the sample was successfully consumed.
typedef bool (*ProcessedOutputFn)(const ProcessedSample&, void* context);

} // namespace signalfix
