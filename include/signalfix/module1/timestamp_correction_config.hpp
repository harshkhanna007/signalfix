// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/timestamp_correction_config.hpp
// =============================================================================

#pragma once

#include <cstdint>

namespace signalfix {

/// Configuration for the S6 Timestamp Correction (PLL) stage.
struct TimestampCorrectionConfig {
    uint64_t min_dt_us;          ///< Lower bound for frequency estimator [μs].
    uint64_t max_dt_us;          ///< Upper bound for frequency estimator [μs].
    uint64_t max_time_jump_us;   ///< Threshold for LONG_GAP reset [μs].
    
    double   alpha;              ///< Steady-state IIR alpha (LOCKED).
    double   alpha_recovery;     ///< Fast IIR alpha (RECOVERY).
    double   alpha_acquire;      ///< Extremely fast IIR alpha (LOCKING).
    
    double   anomaly_alpha_scale;      ///< Scalar for ANOMALY/SHORT_GAP events.
    double   hard_invalid_alpha_scale; ///< Scalar for DEGRADED events.
    
    uint32_t lock_acquire_count; ///< Consecutive stable samples to enter LOCKED.
    uint32_t recovery_count;     ///< Consecutive stable samples to exit RECOVERY.
};

} // namespace signalfix
