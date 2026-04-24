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

    // --- ROC WINDOW FREQUENCY CUTOFF HELPER ---
    // Select the ROC smoothing window N based on physical frequency response:
    //
    //   N = Fs / F_noise_cutoff
    //
    // where:
    //   Fs              = sampling rate [Hz]
    //   F_noise_cutoff  = highest frequency you want to reject [Hz]
    //                     (e.g. 50 Hz for structural vibration at 1 kHz Fs)
    //
    // A small N (e.g. 1) captures every noise spike; a large N introduces
    // phase delay but smooths out high-frequency interference.
    // The result is clamped to [1, kRocWindowMaxN].
    //
    // Example: Fs=1000 Hz, F_noise_cutoff=50 Hz → N = 20 samples.
    static constexpr uint16_t kRocWindowMaxN = 128u;  ///< Hard ceiling.

    [[nodiscard]] static constexpr uint16_t
    compute_roc_window_n(float sampling_rate_hz,
                         float noise_cutoff_hz) noexcept {
        if (sampling_rate_hz <= 0.0f || noise_cutoff_hz <= 0.0f) return 1u;
        const float n_f = sampling_rate_hz / noise_cutoff_hz;
        const uint16_t n = static_cast<uint16_t>(n_f < 1.0f  ? 1u :
                           (n_f > static_cast<float>(kRocWindowMaxN))
                               ? kRocWindowMaxN
                               : static_cast<uint16_t>(n_f));
        return n;
    }

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