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
    // // STEP 6: [LEARNING PATH] — Online State Update
    // =========================================================================
    const bool warmup_complete = (channel_state.roc_n >= config_.warm_up_samples);
    const double learning_limit = warmup_complete ? static_cast<double>(final_limit) : config_.fallback_limit;
    
    // We reject learning when values drastically diverge from the expected bounds
    const bool is_outlier = (raw_roc_safe > (learning_limit * 3.0)) || !std::isfinite(bounded_roc);
    
    if (!is_outlier && envelope.status == SampleStatus::NOMINAL) {
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
    if (!has_flag(envelope.status, SampleStatus::ROC_EXCEEDED) &&
    !has_flag(envelope.status, SampleStatus::DRIFT_EXCEEDED)) {
    channel_state.roc_normal_streak++;
} else {
    channel_state.roc_normal_streak = 0u;
}

    // =========================================================================
    // // STEP 7: [g%σ DRIFT DETECTION] — Slow Degradation Accumulator
    // =========================================================================
    if (channel_state.roc_baseline_locked) {
        // Numerically safe: baseline is clamped to sigma_min (> 0) to prevent div/0.
        const double baseline = std::max(
            static_cast<double>(channel_state.roc_sigma_baseline),
            static_cast<double>(config_.sigma_min)
        );

        // g%σ step: fractional deviation of current sigma from frozen baseline.
        const double g_step = std::abs(sigma - baseline) / baseline;

        // Guard: only accumulate finite values (protects against NaN propagation).
        if (std::isfinite(g_step)) {
            channel_state.drift_gsigma += g_step;
        }

        // Drift detection trigger: sustained fractional deviation exceeds threshold.
        const double DRIFT_THRESHOLD = 3.0 * static_cast<double>(config_.warm_up_samples);
        if (channel_state.drift_gsigma > DRIFT_THRESHOLD) {
            upgrade_failure_hint(envelope, FailureMode::DRIFT, 0.9f);
            envelope.status |= SampleStatus::DRIFT_EXCEEDED;
        }

        // Controlled decay: bleed off accumulator when signal has been stable for 20+ samples.
        if (channel_state.roc_normal_streak > 20) {
            if (!has_flag(envelope.status, SampleStatus::DRIFT_EXCEEDED)) {
                channel_state.drift_gsigma *= 0.98;
            }
        }
    }

    // Final bookkeeping
    envelope.roc_window_n = static_cast<uint16_t>(std::min<uint32_t>(channel_state.roc_n, 0xFFFFu));
    channel_state.last_calibrated_value = envelope.calibrated_value;

    return StageResult::CONTINUE;
}

} // namespace signalfix