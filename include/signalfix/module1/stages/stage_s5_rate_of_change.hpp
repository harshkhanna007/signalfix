// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/stages/stage_s5_rate_of_change.hpp
// =============================================================================

#pragma once

#include "signalfix/module1/stage_interface.hpp"
#include "signalfix/module1/channel_state.hpp"
#include "signalfix/module1/roc_detection_config.hpp"

namespace signalfix {

class StageS5RateOfChange final : public IStage
{
public:
    explicit StageS5RateOfChange(const RocDetectionConfig& config) noexcept;
    ~StageS5RateOfChange() noexcept override = default;

    [[nodiscard]] StageResult process(
        MeasurementEnvelope& envelope,
        ChannelState&        channel_state) noexcept override;

    [[nodiscard]] const char* stage_name() const noexcept override;

    void reset() noexcept override;

private:
    RocDetectionConfig config_;
};

} // namespace signalfix