// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/stage_s4_gap_detection.hpp
// Spec   : SFX-M1-TDS-001  Revision 2.1
// =============================================================================

#pragma once

#include "signalfix/module1/stage_interface.hpp"
#include "signalfix/module1/channel_state.hpp"
#include "signalfix/module1/gap_detection_config.hpp"

namespace signalfix {

// ---------------------------------------------------------------------------
// StageS4GapDetection
//
// Responsibilities:
//   - Evaluates env.delta_t_us against channel nominal_delta_t_us.
//   - Enforces the jitter tolerance budget.
//   - Classifies gap severity (None, Minor, Major, Critical).
//   - Mutates env.status (TIMING_ANOMALY, MISSING, HARD_INVALID).
//   - Manages channel_state.gap bookkeeping (event IDs, counters).
//
// Constraints:
//   - O(1) deterministic execution.
//   - No heap allocations or exceptions.
// ---------------------------------------------------------------------------
class StageS4GapDetection final : public IStage
{
public:
    explicit StageS4GapDetection(const GapDetectionConfig& config) noexcept;
    ~StageS4GapDetection() noexcept override = default;

    [[nodiscard]] StageResult process(
        MeasurementEnvelope& envelope,
        ChannelState&        channel_state) noexcept override;

    [[nodiscard]] const char* stage_name() const noexcept override;

    void reset() noexcept override;

private:
    GapDetectionConfig config_;

    // Evaluates the raw delta against configured integer thresholds.
    [[nodiscard]] GapSeverity classify_gap(uint64_t delta_t_us, 
                                           uint64_t nominal_us) const noexcept;
};

} // namespace signalfix