// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : src/stages/stage_s5_rate_of_change.cpp
// Spec   : SFX-M1-TDS-001  Revision 2.8.5 (Hardened ROC)
// =============================================================================

#include "signalfix/module1/stages/stage_s5_rate_of_change.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <cstdio>
#include <cstdlib>

namespace signalfix {

StageS5RateOfChange::StageS5RateOfChange(const RocDetectionConfig& config) noexcept
    : config_(config) 
{
    // [Rev 2.8.4] Formal root-cause correction gate.
    config_.validate_or_die();
}

const char* StageS5RateOfChange::stage_name() const noexcept { return "S5-ROC"; }

void StageS5RateOfChange::reset() noexcept {}

StageResult StageS5RateOfChange::process(
    MeasurementEnvelope& envelope,
    ChannelState&        channel_state) noexcept
{
    // Ensure deterministic default state if we return early.
    // Downstream S7 must receive finite NaNs without triggering false flags.
    envelope.roc = std::numeric_limits<float>::quiet_NaN();
    envelope.roc_adaptive_limit = std::numeric_limits<float>::quiet_NaN();
    envelope.roc_threshold_used = std::numeric_limits<float>::quiet_NaN();

    const double dt_s = static_cast<double>(envelope.delta_t_us) / 1'000'000.0;
    
    // --- STEP 0: Continuity Guardians ---
    if (!std::isfinite(dt_s) || dt_s <= config_.epsilon_dt) {
        envelope.status |= SampleStatus::RATE_ANOMALY;
        channel_state.roc_normal_streak = 0u;
        channel_state.roc_prev_sample_invalid = true;
        return StageResult::CONTINUE;
    }
    
    const bool is_phys_invalid = has_flag(envelope.status, SampleStatus::HARD_INVALID) || 
                                 has_flag(envelope.status, SampleStatus::MISSING);
    if (is_phys_invalid || !std::isfinite(envelope.calibrated_value)) {
        channel_state.roc_prev_sample_invalid = true;
        channel_state.roc_normal_streak = 0u; // Immediate streak break.
        return StageResult::CONTINUE;
    }

    if (!channel_state.last_calibrated_valid || channel_state.roc_prev_sample_invalid) {
        channel_state.last_calibrated_value = envelope.calibrated_value;
        channel_state.last_calibrated_valid = true;
        channel_state.roc_prev_sample_invalid = false;
        
        // Warm-up samples pass through with threshold context, no ROC computable yet.
        if (channel_state.roc_n > 0) {
            envelope.roc_adaptive_limit = channel_state.roc_threshold_ema;
            envelope.roc_threshold_used = channel_state.roc_threshold_ema;
        }
        return StageResult::CONTINUE; 
    }

    // --- STEP 1: ROC Derivation & Bounding Invariants ---
    const double delta_v = std::abs(envelope.calibrated_value - channel_state.last_calibrated_value);
    
    // Defense: delta_v is suddenly non-finite despite input finity checks (extremely rare CPU math edge case).
    if (!std::isfinite(delta_v)) {
        envelope.status |= SampleStatus::RATE_ANOMALY;
        channel_state.roc_prev_sample_invalid = true;
        return StageResult::CONTINUE;
    }

    const double raw_roc = delta_v / dt_s;
    const double raw_roc_safe = std::isfinite(raw_roc) ? std::min(raw_roc, config_.PHYSICAL_MAX_ROC) : config_.PHYSICAL_MAX_ROC;
    
    // [INVARIANT: Learning Path Shield]
    const double roc_cap = config_.fallback_limit * 1.5; 
    const double bounded_roc = std::min(raw_roc_safe, roc_cap);
    
    envelope.roc = static_cast<float>(raw_roc_safe);

    // --- STEP 2: State Initialization & Warm-Up Gating ---
    if (channel_state.roc_n == 0) {
        channel_state.roc_mean = static_cast<float>(bounded_roc);
        channel_state.roc_m2   = 0.0;
        channel_state.roc_threshold_ema = static_cast<float>(config_.fallback_limit);
        channel_state.roc_sigma_baseline = 0.0f;
        channel_state.roc_sigma_accumulator = 0.0f;
        channel_state.roc_normal_streak = 0u;
        channel_state.roc_n    = 1u;
        channel_state.last_calibrated_value = envelope.calibrated_value;
        
        envelope.roc_adaptive_limit = channel_state.roc_threshold_ema;
        envelope.roc_threshold_used = channel_state.roc_threshold_ema;
        return StageResult::CONTINUE;
    }

    const double mean = static_cast<double>(channel_state.roc_mean);
    const double var  = channel_state.roc_m2;
    const double sigma = std::sqrt(std::max(var, static_cast<double>(config_.sigma_min * config_.sigma_min)));

    // =========================================================================
    // // STEP 3: [EQUILIBRIUM LAYER] — Base-Frame Frame Gating
    // =========================================================================
    if (channel_state.roc_n < config_.warm_up_samples) {
        channel_state.roc_sigma_accumulator += static_cast<float>(sigma);
        channel_state.roc_sigma_baseline = 0.0f; 
    } else if (channel_state.roc_n == config_.warm_up_samples) {
        channel_state.roc_sigma_baseline = channel_state.roc_sigma_accumulator / static_cast<float>(config_.warm_up_samples);
    }

    // =========================================================================
    // // STEP 4: [INFERENCE PATH] — Governed Adaptive Thresholding
    // =========================================================================
    double sigma_eff = sigma;
    if (channel_state.roc_sigma_baseline > 0.0f) {
        const double b = static_cast<double>(channel_state.roc_sigma_baseline);
        sigma_eff = std::fmin(b * config_.sigma_max_ratio, std::fmax(b * config_.sigma_min_ratio, sigma));
    }

    const double k = static_cast<double>(config_.k_sigma_high);
    const double t_raw = k * sigma_eff;
    const double t_min = std::max(k * static_cast<double>(config_.sigma_min), 0.01);
    const double t_max = config_.fallback_limit * 100.0;
    const double t_target = std::fmin(t_max, std::fmax(t_min, t_raw));

    const double t_old = static_cast<double>(channel_state.roc_threshold_ema);
    const double beta = (t_target > t_old) ? config_.beta_up : config_.beta_down;
    channel_state.roc_threshold_ema = static_cast<float>((1.0 - beta) * t_old + beta * t_target);
    
    // INVARIANT: Final limit must be non-zero to protect math below.
    const float final_limit = std::max(channel_state.roc_threshold_ema, 0.001f);
    envelope.roc_adaptive_limit = final_limit;
    envelope.roc_threshold_used = final_limit;

    // =========================================================================
    // // STEP 5: [CLASSIFICATION PATH] — Forensic Truth Comparison
    // =========================================================================
    const double threshold = static_cast<double>(final_limit);
    
    if (raw_roc_safe > (threshold * 2.0)) {
        envelope.status |= SampleStatus::ROC_EXCEEDED;
        channel_state.roc_violation_count++;

        const double conf_range = std::max(0.0, (raw_roc_safe / threshold - 2.0) / 8.0);
        float suggested_conf = static_cast<float>(std::fmin(1.0, std::fmax(0.6, conf_range)));
        upgrade_failure_hint(envelope, FailureMode::SPIKE, suggested_conf);
    } else if (raw_roc_safe > threshold) {
        channel_state.roc_violation_count++;
        if (channel_state.roc_violation_count >= 2u) {
            envelope.status |= SampleStatus::ROC_EXCEEDED;

            const double conf_range = std::max(0.0, (raw_roc_safe / threshold - 1.0));
            float suggested_conf = static_cast<float>(std::fmin(0.6, std::fmax(0.3, conf_range)));
            upgrade_failure_hint(envelope, FailureMode::DRIFT, suggested_conf);
        }
    } else {
        envelope.status = clear_flag(envelope.status, SampleStatus::ROC_EXCEEDED);
        channel_state.roc_violation_count = 0u;
    }

    // =========================================================================
    // // STEP 6: [LEARNING PATH] — Online State Update
    // =========================================================================
    const bool warmup_complete = (channel_state.roc_n >= config_.warm_up_samples);
    const double learning_limit = warmup_complete ? static_cast<double>(final_limit) : config_.fallback_limit;
    
    // We reject learning when values drastically diverge from the expected bounds
    const bool is_outlier = (raw_roc_safe > (learning_limit * 3.0)) || !std::isfinite(bounded_roc);
    
    if (!is_outlier) {
        const double alpha = config_.alpha_learning; 
        channel_state.roc_n++;
        const double diff_old = bounded_roc - mean;
        channel_state.roc_mean += static_cast<float>(alpha * diff_old);
        const double mean_new = static_cast<double>(channel_state.roc_mean);
        const double diff_new = bounded_roc - mean_new;
        
        double proposed_m2 = (1.0 - alpha) * var + alpha * (diff_old * diff_new);
        channel_state.roc_m2 = std::min(std::max(0.0, proposed_m2), config_.PHYSICAL_MAX_ROC * 1000.0);
        channel_state.roc_std = static_cast<float>(std::sqrt(channel_state.roc_m2));
    }

    // [Rev 2.8.5] Stability Tracking
    if (envelope.status == SampleStatus::NOMINAL) {
        channel_state.roc_normal_streak++;
    } else {
        channel_state.roc_normal_streak = 0u;
    }

    // =========================================================================
    // // STEP 7: [ROOT CORRECTION] — Gated Baseline Learning
    // =========================================================================
    const bool is_pure = (envelope.status == SampleStatus::NOMINAL);
    const bool is_stable = (channel_state.roc_normal_streak >= 5u);
    const bool is_post_warmup = (channel_state.roc_n > config_.warm_up_samples);

    if (is_pure && is_stable && is_post_warmup) {
        const double beta_b = config_.beta_baseline;
        const double b_old  = static_cast<double>(channel_state.roc_sigma_baseline);
        channel_state.roc_sigma_baseline = static_cast<float>((1.0 - beta_b) * b_old + beta_b * sigma);
    }

    // Final bookkeeping
    envelope.roc_window_n = static_cast<uint16_t>(std::min<uint32_t>(channel_state.roc_n, 0xFFFFu));
    channel_state.last_calibrated_value = envelope.calibrated_value;

    return StageResult::CONTINUE;
}

} // namespace signalfix