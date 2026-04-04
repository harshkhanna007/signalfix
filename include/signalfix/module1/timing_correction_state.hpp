// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/timing_correction_state.hpp
// =============================================================================

#pragma once

#include <cstdint>

namespace signalfix {

/// PLL lock state machine modes.
enum class PllLockState : uint8_t {
    LOCKING  = 0, ///< Cold start / re-acquisition; high alpha.
    LOCKED   = 1, ///< Stable tracking; nominal alpha.
    RECOVERY = 2  ///< Post-anomaly validation; intermediate alpha.
};

/// Per-channel state for S6 Timestamp Correction (PLL).
/// Guaranteed layout: 32 bytes (Rev 2.4).
struct TimingCorrectionState {
    uint64_t last_raw_timestamp_us;      ///< Last real measurement time [μs].
    uint64_t corrected_timestamp_us;     ///< Last output timeline point [μs].
    double   estimated_dt_us;            ///< Frequency register (IIR) [μs].
    uint16_t consecutive_clean_count;    ///< Stable samples since last event.
    uint16_t consecutive_anomaly_count;  ///< Consecutive anomalies detected.
    PllLockState lock_state;             ///< Current PLL FSM state (offset 28).
    bool     initialized;                ///< True once bootstrap completes (offset 29).
    int16_t  phase_frac_accum;           ///< Sub-μs residual error [1/256 μs] (offset 30).
};
static_assert(sizeof(TimingCorrectionState) == 32u, "TimingCorrectionState layout changed.");

} // namespace signalfix
