// =============================================================================
// SignalFix AI — Module 1: Threshold Calibration
// File   : include/signalfix/module1/threshold_calibrator.hpp
// Spec   : SFX-M1-TDS-001  Revision 1.0
// =============================================================================
//
// Offline (pre-deployment), deterministic framework for deriving all
// runtime thresholds from real baseline data.
//
// Usage pattern:
//
//   // 1. Profile multiple clean datasets.
//   BaselineProfiler p1, p2, p3;
//   for (auto r : dataset_1) p1.add_sample(r);
//   for (auto r : dataset_2) p2.add_sample(r);
//   for (auto r : dataset_3) p3.add_sample(r);
//
//   // 2. Validate and compute final ThresholdProfile.
//   ThresholdCalibrator cal;
//   ThresholdProfile profile;
//   if (!cal.calibrate({p1, p2, p3}, profile)) {
//       // Handle contaminated or inconsistent baseline data.
//   }
//
//   // 3. Inject profile into DriftStabilizerConfig at startup.
//   DriftStabilizerConfig cfg;
//   cfg.apply_threshold_profile(profile);
//
// Thread safety: none required — calibration is offline.
// =============================================================================

#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <vector>

namespace signalfix {

// =============================================================================
// ThresholdProfile — output of the calibrator; injected into runtime config.
// =============================================================================
struct ThresholdProfile {
    // ── Deadband (asymmetric) ────────────────────────────────────────────────
    float deadband_pos;           ///< 95th-percentile of positive residuals.
    float deadband_neg;           ///< 95th-percentile of negative residuals (absolute value).

    // ── CUSUM ghost-mode limits ──────────────────────────────────────────────
    float base_max_accumulation;  ///< Median of per-dataset 99.5th-pctile CUSUM peaks.

    // ── Derived state-machine thresholds ────────────────────────────────────
    float warning_threshold;      ///< base_max_accumulation * 1.5
    float alert_threshold;        ///< warning_threshold * 2.0
    float critical_threshold;     ///< alert_threshold * 1.5
    float recovery_threshold;     ///< base_max_accumulation * 0.75

    // ── Baseline Health ──────────────────────────────────────────────────────
    float baseline_intensity;     ///< Median absolute residual (noise floor reference).
    bool  is_valid;               ///< True if calibrate() succeeded.

    // ── Compute thresholds once base_max_accumulation is set ────────────────
    void derive_thresholds() noexcept {
        warning_threshold   = base_max_accumulation * 1.5f;
        alert_threshold     = warning_threshold * 2.0f;
        critical_threshold  = alert_threshold   * 1.5f;
        recovery_threshold  = base_max_accumulation * 0.75f;
    }
};


// =============================================================================
// BaselineProfiler — accumulates residuals from a single clean dataset.
//
// Residuals are the signed deviation of the RAW sensor signal from the
// sample's local running median (rolling de-trended error). If you already
// compute calibrated residuals upstream, pass those directly.
// =============================================================================
class BaselineProfiler {
public:
    static constexpr uint32_t kMaxSamples = 500'000u;  ///< Memory safety cap.

    /// Add one residual observation.
    /// Call once per tick on a known-clean dataset.
    void add_sample(double residual) noexcept {
        if (count_ >= kMaxSamples) return;
        residuals_.push_back(static_cast<float>(residual));
        count_++;

        // Accumulate absolute value for baseline intensity estimation.
        abs_sum_ += std::abs(static_cast<float>(residual));
    }

    /// Total samples added.
    [[nodiscard]] uint32_t sample_count() const noexcept { return count_; }

    /// True if enough samples exist for reliable percentile estimation.
    [[nodiscard]] bool is_sufficient() const noexcept { return count_ >= 500u; }

    /// Asymmetric 95th-percentile deadbands.
    /// Returns false if insufficient data.
    [[nodiscard]] bool compute_deadbands(float& out_pos,
                                         float& out_neg,
                                         float  percentile = 0.95f) const noexcept {
        if (!is_sufficient()) return false;

        std::vector<float> pos_vals, neg_vals;
        pos_vals.reserve(count_ / 2u + 1u);
        neg_vals.reserve(count_ / 2u + 1u);

        for (float r : residuals_) {
            if (r >= 0.0f) pos_vals.push_back(r);
            else           neg_vals.push_back(-r);  // store absolute value
        }

        if (pos_vals.empty()) { out_pos = 0.1f; }
        else {
            std::sort(pos_vals.begin(), pos_vals.end());
            auto idx = static_cast<size_t>((pos_vals.size() - 1u) * percentile);
            out_pos  = pos_vals[idx];
        }

        if (neg_vals.empty()) { out_neg = 0.1f; }
        else {
            std::sort(neg_vals.begin(), neg_vals.end());
            auto idx = static_cast<size_t>((neg_vals.size() - 1u) * percentile);
            out_neg  = neg_vals[idx];
        }
        return true;
    }

    /// MAD-based baseline intensity estimate (noise floor reference).
    [[nodiscard]] float baseline_intensity() const noexcept {
        if (count_ == 0u) return 0.0f;
        return abs_sum_ / static_cast<float>(count_);
    }

    /// Iterate every residual in insertion order (needed by GhostModeCusum).
    template<typename Fn>
    void visit_residuals(Fn&& fn) const noexcept {
        for (float r : residuals_) { fn(r); }
    }

    /// Linear slope of the dataset (units/sample). Used for macro-drift detection.
    /// If |slope| > drift_slope_threshold, reject this dataset.
    [[nodiscard]] float linear_slope() const noexcept {
        if (count_ < 2u) return 0.0f;
        // Least-squares slope on raw residual sequence.
        const double n   = static_cast<double>(count_);
        const double sx  = (n - 1.0) * n / 2.0;           // sum of indices 0..n-1
        const double sx2 = (n - 1.0) * n * (2.0 * n - 1.0) / 6.0;
        double sy = 0.0, sxy = 0.0;
        for (uint32_t i = 0u; i < count_; ++i) {
            sy  += residuals_[i];
            sxy += static_cast<double>(i) * residuals_[i];
        }
        const double denom = n * sx2 - sx * sx;
        if (std::abs(denom) < 1e-12) return 0.0f;
        return static_cast<float>((n * sxy - sx * sy) / denom);
    }

private:
    std::vector<float> residuals_;
    uint32_t           count_   = 0u;
    float              abs_sum_ = 0.0f;
};


// =============================================================================
// GhostModeCusum — runs CUSUM accumulation silently over one profiler's data
// and extracts the peak accumulation distribution.
//
// "Ghost mode" = accumulate but never trigger any output; used for calibration.
// =============================================================================
class GhostModeCusum {
public:
    /// Run ghost CUSUM over all residuals in a BaselineProfiler.
    ///
    /// @param profiler    The baseline profiler with validated clean data.
    /// @param deadband_p  Positive deadband (from BaselineProfiler::compute_deadbands).
    /// @param deadband_n  Negative deadband.
    /// @param decay_gamma Per-sample leakage factor [0, 1). Typical: 1/memory_samples.
    /// @param pctile      Percentile to extract (0.995 → 99.5th).
    /// @returns 99.5th-percentile peak CUSUM score observed, or 0 on failure.
    static float run(const BaselineProfiler& profiler,
                     float                   deadband_p,
                     float                   deadband_n,
                     float                   decay_gamma = 0.03f,
                     float                   pctile      = 0.995f) noexcept {
        if (!profiler.is_sufficient()) return 0.0f;

        // Collect all per-step CUSUM maxima.
        std::vector<float> peaks;
        peaks.reserve(profiler.sample_count());

        float cusum_pos = 0.0f;
        float cusum_neg = 0.0f;
        const float decay = 1.0f - decay_gamma;

        // Re-iterate residuals. We access them via the profiler's internal
        // compute_deadbands which sorts a copy; here we need the raw sequence.
        // BaselineProfiler exposes this through a visitor-style callback.
        profiler.visit_residuals([&](float r) {
            // Accumulate if outside deadband, otherwise decay.
            if (r > deadband_p) {
                cusum_pos = std::max(0.0f, cusum_pos * decay + (r - deadband_p));
            } else {
                cusum_pos = std::max(0.0f, cusum_pos * decay);
            }

            if (-r > deadband_n) {
                cusum_neg = std::max(0.0f, cusum_neg * decay + (-r - deadband_n));
            } else {
                cusum_neg = std::max(0.0f, cusum_neg * decay);
            }

            peaks.push_back(std::max(cusum_pos, cusum_neg));
        });

        if (peaks.empty()) return 0.0f;
        std::sort(peaks.begin(), peaks.end());
        const auto idx = static_cast<size_t>(
            static_cast<float>(peaks.size() - 1u) * pctile);
        return peaks[idx];
    }
};


// =============================================================================
// ThresholdCalibrator — orchestrates multi-dataset calibration pipeline.
// =============================================================================
class ThresholdCalibrator {
public:
    /// Configuration knobs for the calibration run.
    struct Options {
        float  deadband_percentile        = 0.95f;  ///< Asymmetric deadband coverage.
        float  ghost_cusum_pctile         = 0.995f; ///< Peak CUSUM → base_max_accum.
        float  ghost_cusum_decay          = 0.03f;  ///< Per-sample decay.
        float  max_slope_threshold        = 0.001f; ///< Macro-drift rejection limit [units/sample].
        float  max_dataset_variance_ratio = 0.20f;  ///< Cross-dataset consistency gate.
        uint32_t min_samples_per_dataset  = 500u;   ///< Minimum viable dataset size.
    };

    ThresholdCalibrator() noexcept : opts_{} {}
    explicit ThresholdCalibrator(Options opts) noexcept : opts_(opts) {}

    /// Run full calibration over 2–N profilers.
    ///
    /// @param profilers    List of pre-filled BaselineProfilers (at least 2).
    /// @param out_profile  Populated on success.
    /// @returns true on success; false if data fails validation.
    [[nodiscard]] bool calibrate(std::initializer_list<const BaselineProfiler*> profilers,
                                 ThresholdProfile& out_profile) const noexcept {
        return calibrate_impl(profilers.begin(),
                              static_cast<int>(profilers.size()),
                              out_profile);
    }

    /// Overload taking a pointer + count, suitable for C-style arrays.
    [[nodiscard]] bool calibrate(const BaselineProfiler* const* profilers,
                                 int                             count,
                                 ThresholdProfile&               out_profile) const noexcept {
        return calibrate_impl(profilers, count, out_profile);
    }

private:
    Options opts_;

    template<typename It>
    bool calibrate_impl(It begin, int count, ThresholdProfile& out) const noexcept {
        out = {};
        out.is_valid = false;
        if (count < 2) return false;

        // ── Stage 1: Macro-drift rejection ─────────────────────────────────
        for (int i = 0; i < count; ++i) {
            const BaselineProfiler* p = begin[i];
            if (!p || !p->is_sufficient()) return false;
            const float slope = std::abs(p->linear_slope());
            if (slope > opts_.max_slope_threshold) return false;
        }

        // ── Stage 2: Asymmetric deadbands (average across datasets) ─────────
        float dbp_sum = 0.0f, dbn_sum = 0.0f;
        float intensity_sum = 0.0f;
        for (int i = 0; i < count; ++i) {
            float dbp = 0.0f, dbn = 0.0f;
            if (!begin[i]->compute_deadbands(dbp, dbn, opts_.deadband_percentile))
                return false;
            dbp_sum += dbp;
            dbn_sum += dbn;
            intensity_sum += begin[i]->baseline_intensity();
        }
        const float fcount = static_cast<float>(count);
        out.deadband_pos       = dbp_sum   / fcount;
        out.deadband_neg       = dbn_sum   / fcount;
        out.baseline_intensity = intensity_sum / fcount;

        // ── Stage 3: Ghost CUSUM per dataset ────────────────────────────────
        std::vector<float> peak_scores;
        peak_scores.reserve(static_cast<size_t>(count));
        for (int i = 0; i < count; ++i) {
            const float peak = GhostModeCusum::run(
                *begin[i],
                out.deadband_pos, out.deadband_neg,
                opts_.ghost_cusum_decay,
                opts_.ghost_cusum_pctile);
            if (peak <= 0.0f) return false;
            peak_scores.push_back(peak);
        }

        // ── Stage 4: Cross-dataset variance sanity gate ──────────────────────
        float peak_min = peak_scores[0], peak_max = peak_scores[0];
        for (float s : peak_scores) {
            peak_min = std::min(peak_min, s);
            peak_max = std::max(peak_max, s);
        }
        if (peak_min > 0.0f) {
            const float variance_ratio = (peak_max - peak_min) / peak_min;
            if (variance_ratio > opts_.max_dataset_variance_ratio) return false;
        }

        // ── Stage 5: Final baseline → threshold derivation ───────────────────
        std::sort(peak_scores.begin(), peak_scores.end());
        out.base_max_accumulation = peak_scores[count / 2];  // median
        out.derive_thresholds();
        out.is_valid = true;
        return true;
    }
};

} // namespace signalfix
