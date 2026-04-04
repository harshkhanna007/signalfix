// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/stages/stage_s3_plausibility.hpp
// Spec   : SFX-M1-TDS-001  Revision 2.3 (Final Refactored)
// Stage  : S3 — Plausibility Validator
// =============================================================================

#pragma once

#include "signalfix/module1/stage_interface.hpp"
#include "signalfix/module1/types.hpp"
#include "signalfix/module1/channel_state.hpp"

#include <cmath>
#include <limits>

namespace signalfix {

/**
 * @brief Configuration for physical and operational sensor limits.
 */
struct PlausibilityConfig
{
    double hard_min;  ///< Hard lower bound. HARD_INVALID if below.
    double hard_max;  ///< Hard upper bound. HARD_INVALID if above.
    double soft_min;  ///< Soft lower bound. SOFT_SUSPECT if below.
    double soft_max;  ///< Soft upper bound. SOFT_SUSPECT if above.
};

/**
 * @class StageS3Plausibility
 * @brief Validates numeric integrity and physical plausibility of measurements.
 *
 * Revision 2.3 upgrades:
 *   - Hysteresis state moved to ChannelState (per-channel correctness).
 *   - Subnormal values are now allowed (only NaN/Inf rejected).
 *   - Adaptive ROC-based anomaly detection using RocWindowState.
 *   - Classification: ROC extreme + value normal → SOFT_SUSPECT (preserves spikes).
 */
class StageS3Plausibility final : public IStage
{
public:
    /**
     * @brief Constructor. Validates configuration at runtime.
     */
    explicit StageS3Plausibility(
        const PlausibilityConfig& cfg = {
            -std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max(),
            -1.0e15,
             1.0e15
        }) noexcept;

    /**
     * @brief Performs validation: Integrity, Numeric, Hard limits, ROC anomaly,
     *        and Soft limits with per-channel hysteresis.
     */
    [[nodiscard]] StageResult process(
        MeasurementEnvelope& envelope,
        ChannelState&        channel_state) noexcept override;

    /**
     * @brief Audit the current configuration for logical consistency.
     * @return true if config is safe for flight; false otherwise.
     */
    [[nodiscard]] bool is_config_valid() const noexcept;

    [[nodiscard]] const char* stage_name() const noexcept override
    {
        return "S3-PlausibilityValidator";
    }

    /**
     * @brief No-op. Hysteresis state now lives in ChannelState and is reset
     *        via init_channel_state().
     */
    void reset() noexcept override;

private:
    PlausibilityConfig cfg_;
    bool               config_ok_; ///< Cached result of is_config_valid()

    // ── Internal constants ───────────────────────────────────────────────────

    // Hysteresis margin prevents SOFT_SUSPECT chatter at the boundary.
    static constexpr double SOFT_HYSTERESIS = 1e-6;

    // ROC adaptive threshold: mean + kSigma × std_dev
    static constexpr double kSigma      = 3.0;

    // Variance floor to prevent sqrt(0) and false positives on constant signals.
    static constexpr double kRocEpsilon = 1e-12;

    // Minimum dt (seconds) to guard against divide-by-zero in ROC computation.
    static constexpr double kMinDtSec   = 1e-9;

    // ── Internal helpers ─────────────────────────────────────────────────────

    static void reject_hard(MeasurementEnvelope& env,
                            ChannelState&         cs) noexcept;
};

} // namespace signalfix