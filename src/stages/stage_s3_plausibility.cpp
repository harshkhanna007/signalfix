// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : src/module1/stages/stage_s3_plausibility.cpp
// Spec   : SFX-M1-TDS-001  Revision 2.3 (Final Refactored)
// =============================================================================

#include "signalfix/module1/stages/stage_s3_plausibility.hpp"

#include <cassert>
#include <cstdio>
#include <cmath>
#include <limits>

#ifndef SIGNALFIX_S3_DIAG
#  define SIGNALFIX_S3_DIAG 0
#endif

#if SIGNALFIX_S3_DIAG
#  define S3_LOG(...) \
       std::fprintf(stderr, "[S3] " __VA_ARGS__); \
       std::fprintf(stderr, "\n")
#else
#  define S3_LOG(...) ((void)0)
#endif

// Helper macro to verify invariants before returning
#ifndef NDEBUG
#define VERIFY_INVARIANTS(env) do { \
    bool is_nan = std::isnan((env).calibrated_value); \
    bool is_hard = signalfix::has_flag((env).status, signalfix::SampleStatus::HARD_INVALID); \
    assert((is_nan == is_hard) && "INV-1 Violation: NaN/HARD_INVALID mismatch"); \
    bool is_soft = signalfix::has_flag((env).status, signalfix::SampleStatus::SOFT_SUSPECT); \
    assert(!(is_hard && is_soft) && "INV-2 Violation: Mixed HARD/SOFT flags"); \
} while(0)
#else
#define VERIFY_INVARIANTS(env) do {} while(0)
#endif

namespace signalfix {

// =============================================================================
// Construction & State Management
// =============================================================================

StageS3Plausibility::StageS3Plausibility(const PlausibilityConfig& cfg) noexcept
    : cfg_(cfg),
      config_ok_(false)
{
    config_ok_ = is_config_valid();

    if (!config_ok_) {
        S3_LOG("CRITICAL: Stage S3 initialized with INVALID configuration.");
    }
}

bool StageS3Plausibility::is_config_valid() const noexcept
{
    bool ok = true;
    ok &= (cfg_.hard_min < cfg_.hard_max);
    ok &= (cfg_.soft_min < cfg_.soft_max);
    ok &= (cfg_.soft_min >= cfg_.hard_min);
    ok &= (cfg_.soft_max <= cfg_.hard_max);
    ok &= std::isfinite(cfg_.hard_min) && std::isfinite(cfg_.hard_max);
    ok &= std::isfinite(cfg_.soft_min) && std::isfinite(cfg_.soft_max);
    return ok;
}

void StageS3Plausibility::reset() noexcept {}

// =============================================================================
// Private Helpers
// =============================================================================

void StageS3Plausibility::reject_hard(MeasurementEnvelope& env, ChannelState& cs) noexcept
{
    env.status |= SampleStatus::HARD_INVALID;
    env.calibrated_value = std::numeric_limits<double>::quiet_NaN();

    // Rev 2.6: Hierarchical upgrade (INVALID always wins)
    upgrade_failure_hint(env, FailureMode::INVALID, 1.0f);

    cs.last_calibrated_valid = false;
}

// =============================================================================
// Process Path
// =============================================================================

StageResult StageS3Plausibility::process(MeasurementEnvelope& envelope, ChannelState& cs) noexcept
{
    const double cal = envelope.calibrated_value;

    if (!config_ok_) {
        envelope.status |= SampleStatus::HARD_INVALID;
        envelope.calibrated_value = std::numeric_limits<double>::quiet_NaN();
        cs.last_calibrated_valid = false;
        return StageResult::CONTINUE;
    }

    bool is_hard_invalid = has_flag(envelope.status, SampleStatus::HARD_INVALID);

    if (is_hard_invalid || !std::isfinite(cal)) {
        reject_hard(envelope, cs);
        VERIFY_INVARIANTS(envelope);
        return StageResult::CONTINUE;
    }

    if (cal < cfg_.hard_min || cal > cfg_.hard_max) {
        reject_hard(envelope, cs);
        VERIFY_INVARIANTS(envelope);
        return StageResult::CONTINUE;
    }

    // ── 5. ROC-Based Adaptive Plausibility (DEPRECATED -> Moved to S5) ──────
    //
    // [Architectural Repair] Legacy ROC logic removed from S3 to avoid conflict
    // with the production-grade armored adaptive model in S5. S3 now strictly
    // enforces absolute physical bounds (Hard/Soft limits).
    
    // ── 6. Soft Plausibility Limits (Per-Channel Hysteresis) ─────────────────
    // Evaluate boundaries for SOFT_SUSPECT
    const double SOFT_HYSTERESIS = 0.5; // Example constant

    if (cal < cfg_.soft_min) {
        cs.s3_is_soft_low = true;
    } else if (cal > cfg_.soft_min + SOFT_HYSTERESIS) {
        cs.s3_is_soft_low = false;
    }

    if (cal > cfg_.soft_max) {
        cs.s3_is_soft_high = true;
    } else if (cal < cfg_.soft_max - SOFT_HYSTERESIS) {
        cs.s3_is_soft_high = false;
    }

    if (cs.s3_is_soft_low || cs.s3_is_soft_high) {
        envelope.status |= SampleStatus::SOFT_SUSPECT;

        // Rev 2.6: Hierarchical upgrade for DRIFT suggestion
        double excursion = 0.0;
        if (cs.s3_is_soft_low) excursion = cfg_.soft_min - cal;
        else excursion = cal - cfg_.soft_max;

        // Confidence scales from 0.0 at bound to 1.0 at 2x hysteresis distance
        const double scale = 2.0 * SOFT_HYSTERESIS;
        float suggested_conf = static_cast<float>(std::fmin(1.0, std::fmax(0.0, excursion / scale)));
        
        upgrade_failure_hint(envelope, FailureMode::DRIFT, suggested_conf);
    }

    envelope.calibrated_value = cal;
    cs.last_calibrated_value = cal;
    cs.last_calibrated_valid = true;

    VERIFY_INVARIANTS(envelope);
    return StageResult::CONTINUE;
}

} // namespace signalfix