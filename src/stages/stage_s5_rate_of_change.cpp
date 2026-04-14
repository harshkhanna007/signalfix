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

    // Initialize envelope fault state to "no fault" (will be updated below)
    envelope.fault_state.is_active_fault = false;
    envelope.fault_state.is_recovering = false;
    envelope.fault_state.samples_since_fault_onset = 0u;
    envelope.fault_state.confidence_at_onset = 0.0f;
    envelope.fault_state.confidence_current = 0.0f;
    envelope.fault_state.recovery_confirmed = false;
    envelope.fault_state.samples_in_recovery = 0u;

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
        // Accumulate sigma only during warm-up; baseline remains unset.
        channel_state.roc_sigma_accumulator += static_cast<float>(sigma);
    } else if (channel_state.roc_n == config_.warm_up_samples && !channel_state.roc_baseline_locked) {
        // Compute baseline exactly once at warm-up completion, then freeze it.
        channel_state.roc_sigma_baseline =
            channel_state.roc_sigma_accumulator / static_cast<float>(config_.warm_up_samples);
        channel_state.roc_baseline_locked = true;
    }
    // Baseline is immutable after lock — no updates below this point.

    // =========================================================================
    // // STEP 4: [INFERENCE PATH] — Governed Adaptive Thresholding
    // =========================================================================
    
    // [Rev 3.4] Hardened Dual-Condition Learning Gate & Stable Reference
    // Gating ref is decoupled from instantaneous sigma to prevent recursive feedback.
    const double gating_ref = std::max((double)channel_state.roc_smoothed_sigma, (double)config_.sigma_min);
    const double dev_norm_gate = std::abs(physical_deviation_pre) / std::max(1e-6, gating_ref);

    const bool is_centered = (dev_norm_gate < 0.75); // Tight centrality requirement
    const bool is_healthy = (channel_state.drift_gsigma < 0.5); // Tight health requirement
    
    if (is_centered && is_healthy) {
        channel_state.roc_learning_streak++;
    } else {
        channel_state.roc_learning_streak = 0u;
    }
    
    // Only update noise model if signal has been stable and centered for 8 consecutive frames
    const bool learning_stable = (channel_state.roc_learning_streak >= 8u);

    if (learning_stable) {
        if (channel_state.roc_smoothed_sigma == 0.0f) {
            channel_state.roc_smoothed_sigma = static_cast<float>(sigma);
            channel_state.roc_mad = static_cast<float>(std::abs(physical_deviation_pre));
        } else {
            channel_state.roc_smoothed_sigma = 
                0.98f * channel_state.roc_smoothed_sigma + 0.02f * static_cast<float>(sigma);
            
            // Robust MAD update: clip contribution to 2.0 sigma to neutralize outliers
            const double contribution = std::min(std::abs(physical_deviation_pre), gating_ref * 2.0);
            channel_state.roc_mad = 0.99f * channel_state.roc_mad + 0.01f * static_cast<float>(contribution);
        }
    }


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
    
    // [STABILITY FLOOR]: Prevent indefinite threshold decay and false-positive drift
    double fallback_safe = static_cast<double>(config_.fallback_limit);
    if (!std::isfinite(fallback_safe) || fallback_safe <= 0.001) {
        fallback_safe = 0.001; // Defensive guard against invalid config
    }
    const float min_threshold = static_cast<float>(fallback_safe * 0.8);
    const float max_threshold = static_cast<float>(fallback_safe * 3.0);
    
    // Recovery from NaNs introduced by corrupted math before clamping
    if (!std::isfinite(channel_state.roc_threshold_ema)) {
        channel_state.roc_threshold_ema = max_threshold; 
    }
    channel_state.roc_threshold_ema = std::clamp(channel_state.roc_threshold_ema, min_threshold, max_threshold);

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

            // ROC detection path → SPIKE only. DRIFT is exclusively owned by g%σ accumulator.
            const double conf_range = std::max(0.0, (raw_roc_safe / threshold - 1.0));
            float suggested_conf = static_cast<float>(std::fmin(0.6, std::fmax(0.3, conf_range)));
            upgrade_failure_hint(envelope, FailureMode::SPIKE, suggested_conf);
        }
    } else {
        envelope.status = clear_flag(envelope.status, SampleStatus::ROC_EXCEEDED);
        channel_state.roc_violation_count = 0u;
    }

    // =========================================================================
    // STEP 5.5: [FAULT STATE TRACKER] — Distinguish Active vs. Recovering
    // =========================================================================

    const bool is_active_now = 
        has_flag(envelope.status, SampleStatus::ROC_EXCEEDED) ||
        has_flag(envelope.status, SampleStatus::DRIFT_EXCEEDED);
    const bool is_nominal_now = 
        !has_flag(envelope.status, SampleStatus::ROC_EXCEEDED) &&
        !has_flag(envelope.status, SampleStatus::DRIFT_EXCEEDED);
    const float current_conf = envelope.failure_confidence;

    // Case 1: NEW FAULT ONSET (transition from healthy to ROC_EXCEEDED)
    if (is_active_now && !channel_state.fault_tracker.has_active_fault) {
        // First sample of new fault
        channel_state.fault_tracker.has_active_fault = true;
        channel_state.fault_tracker.fault_onset_sequence_id = envelope.sequence_id;
        channel_state.fault_tracker.fault_onset_confidence = current_conf;
        channel_state.fault_tracker.last_sample_with_fault = envelope.sequence_id;
        channel_state.fault_tracker.recovery_fully_confirmed = false;
        channel_state.fault_tracker.recovery_grace_period = 0u;
        
        // Set envelope state
        envelope.fault_state.is_active_fault = true;
        envelope.fault_state.is_recovering = false;
        envelope.fault_state.fault_onset_sequence_id = envelope.sequence_id;
        envelope.fault_state.confidence_at_onset = current_conf;
        envelope.fault_state.confidence_current = current_conf;
        envelope.fault_state.samples_since_fault_onset = 1u;
        envelope.fault_state.recovery_confirmed = false;
    }
    // Case 2: CONTINUING ACTIVE FAULT (still in ROC_EXCEEDED phase)
    else if (is_active_now && channel_state.fault_tracker.has_active_fault) {
        const uint64_t delta_seq = 
            (envelope.sequence_id >= channel_state.fault_tracker.fault_onset_sequence_id)
            ? (envelope.sequence_id - channel_state.fault_tracker.fault_onset_sequence_id)
            : UINT32_MAX;  // saturate on wrap
        
        envelope.fault_state.is_active_fault = true;
        envelope.fault_state.is_recovering = false;
        envelope.fault_state.fault_onset_sequence_id = channel_state.fault_tracker.fault_onset_sequence_id;
        envelope.fault_state.confidence_at_onset = channel_state.fault_tracker.fault_onset_confidence;
        envelope.fault_state.confidence_current = current_conf;
        envelope.fault_state.samples_since_fault_onset = 
            std::min(static_cast<uint32_t>(delta_seq), static_cast<uint32_t>(UINT32_MAX));
        
        channel_state.fault_tracker.last_sample_with_fault = envelope.sequence_id;
    }
    // Case 3: RECOVERY PHASE (NOMINAL status but confidence > 0, within decay window)
    else if (is_nominal_now && channel_state.fault_tracker.has_active_fault && current_conf > 0.0f) {
        const uint64_t delta_seq = 
            (envelope.sequence_id >= channel_state.fault_tracker.fault_onset_sequence_id)
            ? (envelope.sequence_id - channel_state.fault_tracker.fault_onset_sequence_id)
            : UINT32_MAX;
        
        envelope.fault_state.is_active_fault = false;
        envelope.fault_state.is_recovering = true;
        envelope.fault_state.fault_onset_sequence_id = channel_state.fault_tracker.fault_onset_sequence_id;
        envelope.fault_state.confidence_at_onset = channel_state.fault_tracker.fault_onset_confidence;
        envelope.fault_state.confidence_current = current_conf;
        envelope.fault_state.samples_since_fault_onset = 
            std::min(static_cast<uint32_t>(delta_seq), static_cast<uint32_t>(UINT32_MAX));
        envelope.fault_state.recovery_confirmed = false;
        envelope.fault_state.samples_in_recovery = 0u;
    }
    // Case 4: RECOVERY COMPLETE (NOMINAL status AND confidence == 0.0)
    else if (is_nominal_now && channel_state.fault_tracker.has_active_fault && current_conf == 0.0f) {
        // Multi-sample grace period to confirm recovery (avoid noise-induced re-detection)
        channel_state.fault_tracker.recovery_grace_period++;
        
        if (channel_state.fault_tracker.recovery_grace_period >= 2u) {
            // Recovery confirmed
            channel_state.fault_tracker.has_active_fault = false;
            channel_state.fault_tracker.recovery_fully_confirmed = true;
        }
        
        envelope.fault_state.is_active_fault = false;
        envelope.fault_state.is_recovering = false;
        envelope.fault_state.fault_onset_sequence_id = 0u;
        envelope.fault_state.confidence_at_onset = 0.0f;
        envelope.fault_state.confidence_current = 0.0f;
        envelope.fault_state.samples_since_fault_onset = 0u;
        envelope.fault_state.recovery_confirmed = (channel_state.fault_tracker.recovery_grace_period >= 2u);
        envelope.fault_state.samples_in_recovery = channel_state.fault_tracker.recovery_grace_period;
    }
    // Case 5: CLEAN STATE (no fault, never had a fault)
    else if (is_nominal_now && !channel_state.fault_tracker.has_active_fault) {
        // Clean: no fault active, no recovery pending
        envelope.fault_state.is_active_fault = false;
        envelope.fault_state.is_recovering = false;
        envelope.fault_state.recovery_confirmed = false;
        envelope.fault_state.samples_since_fault_onset = 0u;
        envelope.fault_state.confidence_at_onset = 0.0f;
        envelope.fault_state.confidence_current = 0.0f;
        
        // Reset grace period
        channel_state.fault_tracker.recovery_grace_period = 0u;
    }

    // =========================================================================
    // // STEP 6: [LEARNING PATH] — Hardened Online State Update (Rev 3.1)
    // =========================================================================
    const bool warmup_complete = (channel_state.roc_n >= config_.warm_up_samples);
    const double learning_limit = warmup_complete ? static_cast<double>(final_limit) : config_.fallback_limit;
    const bool is_outlier = (raw_roc_safe > (learning_limit * 3.0)) || !std::isfinite(bounded_roc);

    // Hardened Logic Constants
    const uint32_t RECOVERY_HOLD_SAMPLES = 15u;
    const uint32_t STABILITY_STREAK_THRESHOLD = 8u;
    // Limits in Sigma Space
    const double drift_cutoff = 0.5;
    const double mom_threshold = 0.4;

    // Composite Learning Gate
    const bool learning_allowed = !warmup_complete || (
        (envelope.status == SampleStatus::NOMINAL) &&
        (channel_state.drift_gsigma < drift_cutoff) &&
        (channel_state.samples_since_drift_clear > RECOVERY_HOLD_SAMPLES)
    );

    if (!is_outlier && learning_allowed) {
        // Conservative Mode: Throttled learning post-baseline lock
        const double alpha_scale = channel_state.drift_baseline_locked ? 0.5 : 1.0;
        const double alpha = config_.alpha_learning * alpha_scale; 

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
    if (!has_flag(envelope.status, SampleStatus::ROC_EXCEEDED) &&
    !has_flag(envelope.status, SampleStatus::DRIFT_EXCEEDED)) {
    channel_state.roc_normal_streak++;
} else {
    channel_state.roc_normal_streak = 0u;
}

    // =========================================================================
    // STEP 7: [TREND-AWARE DRIFT DETECTION] — Momentum-CUSUM Architecture
    // =========================================================================
    
    // 7a. Stability-Gated Baseline Locking (Issue 3 Fix)
    const double BASELINE_WARMUP_SAMPLES = 25.0;
    if (!channel_state.drift_baseline_locked) {
        
        // Condition: Stability Gate (Relaxed from NOMINAL-only)
        const bool is_phys_valid = !has_flag(envelope.status, SampleStatus::HARD_INVALID) && 
                                   !has_flag(envelope.status, SampleStatus::MISSING);
        const bool is_stable = is_phys_valid && (channel_state.roc_normal_streak >= 5u);
        
        // Condition: Health Validation (Scale-Aware Health Gate)
        const bool is_healthy = (sigma <= static_cast<double>(final_limit) * 1.5);
        
        if (is_stable && is_healthy) {
            channel_state.drift_baseline_samples++;
            channel_state.drift_baseline += raw_roc_safe;
            
            if (channel_state.drift_baseline_samples >= static_cast<uint32_t>(BASELINE_WARMUP_SAMPLES)) {
                channel_state.drift_baseline /= BASELINE_WARMUP_SAMPLES;
                channel_state.drift_baseline_locked = true;
            }
        }
        
        // Bootstrapped Initialization: Ensure system picks up correct mean
        else if (channel_state.roc_n >= config_.warm_up_samples) {
            channel_state.drift_baseline = channel_state.roc_mean;
            channel_state.drift_baseline_locked = true;
        }
    }

    // 7b. Trend Extraction & Momentum (Issue 4 Fix: Block warmup accumulation)
    if (channel_state.drift_baseline_locked) {
        const double baseline = channel_state.drift_baseline;
        double physical_deviation = raw_roc_safe - baseline;

        // Enforce physical noise floor to prevent div by zero (Dynamic Normalization)
        const double epsilon_floor = 1e-6;
        const double nf_safe = std::max({static_cast<double>(channel_state.roc_smoothed_sigma), 
                                         static_cast<double>(config_.sigma_min), 
                                         epsilon_floor});
        
        // Convert to Normalized Sigma Space
        double dev_norm = physical_deviation / nf_safe;
        if (!std::isfinite(dev_norm)) { dev_norm = 0.0; }
        
        // [Rev 3.3] Huber Bounding (Outlier Compression)
        // Linear up to 3.0, logarithmic beyond to squash spikes non-destructively.
        if (std::abs(dev_norm) > 3.0) {
            dev_norm = 3.0 * std::copysign(1.0, dev_norm) + 
                       std::log(1.0 + std::abs(dev_norm) - 3.0) * std::copysign(1.0, dev_norm);
        }


        const double ALPHA = 0.5; // Momentum smoothing factor (more responsive)
        channel_state.drift_momentum = ALPHA * channel_state.drift_momentum + (1.0 - ALPHA) * dev_norm;

        const double MOMENTUM_LIMIT = 3.0;
        channel_state.drift_momentum = std::clamp(channel_state.drift_momentum, -MOMENTUM_LIMIT, MOMENTUM_LIMIT);

        // 7c. Noise-Aware Adaptive Slack (Normalized space)
        // [Rev 3.4] Rate-Limited Adaptive Slack (MAD-derived)
        const double mad_ratio = static_cast<double>(channel_state.roc_mad) / nf_safe;
        const double target_slack = std::clamp(mad_ratio + 0.05, 0.75, 0.95);
        
        // Prevent slack from jumping more than 0.001 per frame to maintain CUSUM stability
        const double delta_slack = std::clamp(target_slack - channel_state.drift_smoothed_slack, -0.001, 0.001);
        channel_state.drift_smoothed_slack += static_cast<float>(delta_slack);
        const double k_slack = channel_state.drift_smoothed_slack;



        // 7e. CUSUM Accumulation (Leaky Integrator)
        const double gamma = 1.0 / std::max(1.0, static_cast<double>(config_.drift_memory_samples));
        channel_state.drift_cusum_pos = std::max(0.0, channel_state.drift_cusum_pos * (1.0 - gamma) + channel_state.drift_momentum - k_slack);
        channel_state.drift_cusum_neg = std::max(0.0, channel_state.drift_cusum_neg * (1.0 - gamma) - channel_state.drift_momentum - k_slack);

        // Limit values to prevent explosion
        const double MAX_CUSUM = 50.0;
        channel_state.drift_cusum_pos = std::min(channel_state.drift_cusum_pos, MAX_CUSUM);
        channel_state.drift_cusum_neg = std::min(channel_state.drift_cusum_neg, MAX_CUSUM);

        // 7f. Signal Export & Trigger (Latched Hysteresis)
        channel_state.drift_gsigma = std::max(channel_state.drift_cusum_pos, channel_state.drift_cusum_neg);

        // [Rev 3.4] Generalized Mathematical Thresholds & Guarding
        const double SENSITIVITY_MIN = 0.20;
        const double SENSITIVITY_MAX = 0.60;
        const double drift_sensitivity_sigma = std::clamp(0.35, SENSITIVITY_MIN, SENSITIVITY_MAX);
        
        const double DRIFT_ON_THRES = std::max(5.0, drift_sensitivity_sigma / gamma);
        const double DRIFT_OFF_THRES = DRIFT_ON_THRES * 0.4;
        
        if (channel_state.drift_gsigma >= DRIFT_ON_THRES) {
            channel_state.drift_trigger_persistence += 1.0f;
        } else {
            // [Rev 3.4] Balanced Persistence Decay: Soft linear decay (max 0.25 at zero)
            // This prevents evidence 'erasure' deep in nominal while responding to gaps.
            const double ratio = channel_state.drift_gsigma / DRIFT_ON_THRES;
            const float decay = static_cast<float>((1.0 - ratio) * 0.25);
            channel_state.drift_trigger_persistence = std::max(0.0f, channel_state.drift_trigger_persistence - decay);
        }


        if (!channel_state.drift_active) {
            if (channel_state.drift_trigger_persistence >= 3.0f) {
                channel_state.drift_active = true;
            }
        } else {
            if (channel_state.drift_gsigma < DRIFT_OFF_THRES) {
                channel_state.drift_active = false;
                channel_state.drift_trigger_persistence = 0.0f;
            }
        }

        if (channel_state.drift_active) {
            upgrade_failure_hint(envelope, FailureMode::DRIFT, 0.9f);
            envelope.status |= SampleStatus::DRIFT_EXCEEDED;
        } else {
            envelope.status = clear_flag(envelope.status, SampleStatus::DRIFT_EXCEEDED);
        }

    }

    // --- Update Recovery Guard ---
    if (channel_state.drift_gsigma < 0.2) { // Practical recovery threshold
        channel_state.samples_since_drift_clear++;
    } else {
        channel_state.samples_since_drift_clear = 0u;
    }

    // Final bookkeeping
    envelope.roc_window_n = static_cast<uint16_t>(std::min<uint32_t>(channel_state.roc_n, 0xFFFFu));
    channel_state.last_calibrated_value = envelope.calibrated_value;

    // Export gSigma so the interpretation layer can map severity without
    // needing access to ChannelState. S5 is the sole writer of this value.
    envelope.drift_gsigma = static_cast<float>(channel_state.drift_gsigma);

    return StageResult::CONTINUE;
}

} // namespace signalfix