// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : include/signalfix/module1/roc_detection_config.hpp
// =============================================================================

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace signalfix {

/**
 * @brief Configuration for S5 Rate-of-Change (ROC) Policy Engine.
 * [Rev 2.8 0.1%% Tier — Formal Statistical Model, Certifiable Stability]
 */
struct RocDetectionConfig
{
    // --- PHYSICAL GROUNDING CONSTANTS ---
    static constexpr double PHYSICAL_MAX_ROC    = 10000000.0; // [EXPANDED for Forensic Trace]
    static constexpr double NOMINAL_SPIKE_LIMIT = 5000.0;    // [ANCHORED for 5.0 Noise @ 1kHz]

    // --- ADAPTIVE THRESHOLD PARAMETERS ---
    float    k_sigma_high{3.5f};  ///< [0.1%% Tier] Gaussian 99.9%% Boundary
    float    k_sigma_low{2.5f};
    float    sigma_min{1.0e-3f};
    double   epsilon_dt{1.0e-9};

    // --- FORMAL EMA PARAMETERS ---
    double   alpha_learning{0.05}; ///< [Learning Path] 20-sample convergence
    double   beta_up{0.005};       ///< [Inference] Rise rate (Slow)
    double   beta_down{0.05};      ///< [Inference] Fall rate (Fast-Correction)
    double   beta_baseline{0.001}; ///< [Equilibrium] Long-term anchor (1,000-sample)
    uint32_t warm_up_samples{30u};
    uint32_t drift_memory_samples{33u};  ///< [Rev 3.2] Sample memory length for drift scaling
    double   fallback_limit{NOMINAL_SPIKE_LIMIT};

    // --- STABILIZATION GOVERNOR ---
    float    sigma_min_ratio{0.5f};  ///< Minimum allowed sigma / baseline
    float    sigma_max_ratio{3.0f};  ///< Maximum allowed sigma / baseline

    // --- RECOVERY & BYPASS ---
    uint32_t baseline_reset_threshold{10u};
    uint64_t max_roc_delta_t_us{500'000u}; 

    void validate_or_die() const noexcept {
        bool ok = (k_sigma_high > k_sigma_low) &&
                  (sigma_min > 0.0f) &&
                  (warm_up_samples > 0u) &&
                  (fallback_limit >= 1.0) &&
                  (fallback_limit < PHYSICAL_MAX_ROC) &&
                  (alpha_learning > 0.0 && alpha_learning <= 0.2) &&
                  (beta_up > 0.0 && beta_up < beta_down) &&
                  (beta_down <= 0.2) &&
                  (beta_baseline > 0.0 && beta_baseline <= 0.01) &&
                  (sigma_min_ratio > 0.0f && sigma_max_ratio > sigma_min_ratio);

        if (!ok) {
            std::fprintf(stderr, "\n[FATAL] SignalFix Module 1 Config Validation FAILED (Rev 2.8.3+).\n");
            std::abort();
        }
    }

    [[nodiscard]] constexpr bool is_valid() const noexcept {
        return (k_sigma_high > k_sigma_low) && (sigma_min > 0.0f);
    }
};

} // namespace signalfix