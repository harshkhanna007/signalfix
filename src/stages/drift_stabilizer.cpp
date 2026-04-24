// =============================================================================
// SignalFix AI — Module 1: Drift Stabilizer
// File   : src/stages/drift_stabilizer.cpp
// Spec   : SFX-M1-TDS-001  Revision 4.0 (Orthogonal Decision Architecture)
// =============================================================================
//
// Implementation of DriftStabilizer.
// Two-phase logic per frame:
//
//   Phase 1 — Severity Score (space domain, O(1) per frame):
//     - Confidence hard gate: if confidence < CONFIDENCE_MIN, score = 0.
//     - Dead-band floor: subtract MIN_GSIGMA from drift_gsigma.
//     - Momentum amplifier: multiply by (1 + K_MOMENTUM * clamp(momentum, 0, MAX_MOMENTUM)).
//     - Final score bounded to [0, severity_max].
//
//   Phase 2 — State Machine (time domain, O(1) per frame):
//     NOMINAL state:
//       - severity >= thresh_trigger: persist_timer++
//         When persist_timer >= required_persist: -> DRIFT_CONFIRMED, emit DRIFT_STARTED.
//       - severity <  thresh_trigger: persist_timer -= persist_decay (clamp to 0)
//         This asymmetric decay defeats oscillating/noisy signals conclusively.
//
//     DRIFT_CONFIRMED state:
//       - Recovery condition = (severity < thresh_recover) AND (momentum <= 0.0f)
//         Both must hold simultaneously every frame, or recover_timer resets to 0.
//         When recover_timer >= required_recover: -> NOMINAL, emit DRIFT_RESOLVED.
//       - Escalation: if severity >= escalation_threshold AND cooldown_timer == 0:
//         emit DRIFT_ESCALATED, reset cooldown_timer.
//
//   Alert Cooldown:
//     cooldown_timer is decremented each frame. Any alert emission sets it to
//     cooldown_frames. No alert fires while cooldown_timer > 0.
//
// =============================================================================

#include "drift_stabilizer.hpp"
#include <algorithm>  // std::clamp, std::max
#include <cstdio>     // std::printf

namespace signalfix {

// =============================================================================
// String helpers
// =============================================================================

const char* drift_state_to_string(DriftState s) noexcept {
    switch (s) {
        case DriftState::NOMINAL:         return "NOMINAL";
        case DriftState::DRIFT_CONFIRMED: return "DRIFT_CONFIRMED";
        default:                          return "UNKNOWN";
    }
}

const char* drift_output_to_string(DriftOutput o) noexcept {
    switch (o) {
        case DriftOutput::NOMINAL:         return "NOMINAL";
        case DriftOutput::DRIFT_CONFIRMED: return "DRIFT_CONFIRMED";
        default:                           return "UNKNOWN";
    }
}

const char* drift_event_to_string(DriftEvent e) noexcept {
    switch (e) {
        case DriftEvent::NONE:            return "NONE";
        case DriftEvent::DRIFT_STARTED:   return "DRIFT_STARTED";
        case DriftEvent::DRIFT_ESCALATED: return "DRIFT_ESCALATED";
        case DriftEvent::DRIFT_RESOLVED:  return "DRIFT_RESOLVED";
        default:                          return "UNKNOWN";
    }
}

// =============================================================================
// reset()
// =============================================================================

void DriftStabilizer::reset() noexcept {
    state_         = DriftState::NOMINAL;
    persist_timer_ = 0;
    recover_timer_ = 0;
    cooldown_timer_= 0;
    last_severity_ = 0.0f;

    // Clear dynamic-deadband rolling window.
    std::memset(intensity_window_, 0, sizeof(intensity_window_));
    intensity_head_  = 0;
    intensity_count_ = 0;
    intensity_sum_   = 0.0f;
    active_deadband_pos_ = cfg_.deadband_base_pos;
    active_deadband_neg_ = cfg_.deadband_base_neg;

    // Gain schedule
    pv_filtered_ = 0.0f;
    k_schedule_  = 1.0f;
}

// =============================================================================
// update_dynamic_deadband() — Phase 0: Rolling intensity + adaptive expansion
// =============================================================================

void DriftStabilizer::update_dynamic_deadband(float residual_abs) noexcept {
    if (!cfg_.dynamic_deadband_enabled) {
        // Fall back to static base deadbands.
        active_deadband_pos_ = cfg_.deadband_base_pos;
        active_deadband_neg_ = cfg_.deadband_base_neg;
        return;
    }

    // ── Update O(1) circular rolling window ────────────────────────────────
    if (intensity_count_ >= kDeadbandIntensityWindow) {
        // Evict oldest sample.
        intensity_sum_ -= intensity_window_[intensity_head_];
    } else {
        intensity_count_++;
    }
    intensity_window_[intensity_head_] = residual_abs;
    intensity_sum_ += residual_abs;
    intensity_head_ = (intensity_head_ + 1) % kDeadbandIntensityWindow;

    // ── Local noise intensity vs calibrated baseline ───────────────────────
    const float local_intensity =
        (intensity_count_ > 0) ? (intensity_sum_ / static_cast<float>(intensity_count_))
                               : residual_abs;

    const float baseline = std::max(cfg_.baseline_intensity, 1e-6f);
    const float intensity_ratio = local_intensity / baseline;

    // ── Linear expansion between trigger and max ───────────────────────────
    //   ratio < trigger  → multiplier = 1.0  (no expansion)
    //   ratio = 2×trigger → multiplier = deadband_expand_max
    const float trigger = cfg_.deadband_expand_trigger;
    float multiplier = 1.0f;
    if (intensity_ratio > trigger) {
        const float span = std::max(1e-6f, 2.0f * trigger - trigger); // expansion range
        const float t    = std::min(1.0f, (intensity_ratio - trigger) / span);
        multiplier = 1.0f + t * (cfg_.deadband_expand_max - 1.0f);
    }
    multiplier = std::min(multiplier, cfg_.deadband_expand_max);

    active_deadband_pos_ = cfg_.deadband_base_pos * multiplier;
    active_deadband_neg_ = cfg_.deadband_base_neg * multiplier;
}

// =============================================================================
// compute_gain_schedule() — Interpolate threshold multiplier from PV
// =============================================================================

float DriftStabilizer::compute_gain_schedule(float pv_raw) noexcept {
    if (cfg_.gain_schedule_count <= 0) return 1.0f;

    // Low-pass filter the process variable to suppress jitter.
    pv_filtered_ = (1.0f - cfg_.schedule_lpf_alpha) * pv_filtered_
                 + cfg_.schedule_lpf_alpha * pv_raw;

    const float pv = pv_filtered_;
    const int   n  = cfg_.gain_schedule_count;

    // Clamp to table range.
    if (pv <= cfg_.gain_schedule[0].pv_value)
        return std::clamp(cfg_.gain_schedule[0].multiplier,
                          cfg_.k_schedule_min, cfg_.k_schedule_max);
    if (pv >= cfg_.gain_schedule[n - 1].pv_value)
        return std::clamp(cfg_.gain_schedule[n - 1].multiplier,
                          cfg_.k_schedule_min, cfg_.k_schedule_max);

    // Linear interpolation between adjacent breakpoints.
    for (int i = 1; i < n; ++i) {
        const float pv_lo = cfg_.gain_schedule[i - 1].pv_value;
        const float pv_hi = cfg_.gain_schedule[i].pv_value;
        if (pv <= pv_hi) {
            const float span = pv_hi - pv_lo;
            if (span < 1e-6f) {
                return cfg_.gain_schedule[i].multiplier;
            }
            const float t = (pv - pv_lo) / span;
            const float m = cfg_.gain_schedule[i - 1].multiplier * (1.0f - t)
                          + cfg_.gain_schedule[i].multiplier * t;
            return std::clamp(m, cfg_.k_schedule_min, cfg_.k_schedule_max);
        }
    }
    return 1.0f;
}


float DriftStabilizer::compute_severity(float drift_gsigma,
                                         float confidence,
                                         float momentum) const noexcept {
    // ── Hard gate: confidence ──────────────────────────────────────────────────
    // If the evidence quality is insufficient, refuse to generate any score.
    // Low confidence means the signal cannot be trusted at all — not just less.
    if (confidence < cfg_.confidence_min) {
        return 0.0f;
    }

    // ── Dead-band floor: eliminate nominal noise wander ────────────────────────
    // Subtract the minimum expected gsigma baseline. Any value below the floor
    // returns exactly 0.0, preventing weak accumulation from contributing.
    const float active_gsigma = std::max(0.0f, drift_gsigma - cfg_.min_gsigma);
    if (active_gsigma == 0.0f) {
        return 0.0f;  // Trivially below floor. No severity.
    }

    // ── Confidence: already validated, clamp to [0, 1] for safety ─────────────
    const float safe_conf = std::clamp(confidence, 0.0f, 1.0f);

    // ── Momentum amplifier ────────────────────────────────────────────────────
    // Momentum accelerates detection of *worsening* drift only.
    // Negative momentum (recovery direction) does not amplify — it contributes 0.
    // Upper-bounded to prevent a single-frame derivative explosion.
    const float safe_momentum = std::clamp(momentum, 0.0f, cfg_.max_momentum);
    const float m_factor      = 1.0f + (cfg_.k_momentum * safe_momentum);

    // ── Bounded score ─────────────────────────────────────────────────────────
    const float raw_score = active_gsigma * safe_conf * m_factor;
    return std::min(raw_score, cfg_.severity_max);
}

// =============================================================================
// update() — Phase 2: Time Domain State Machine
// =============================================================================

StabilizerResult DriftStabilizer::update(float drift_gsigma,
                                          float confidence,
                                          float momentum) noexcept {
    // ── Phase 0a: Dynamic deadband update (uses gsigma as intensity proxy) ──
    // drift_gsigma is non-negative; use it as a residual magnitude estimate
    // when no explicit residual signal is available from the pipeline.
    update_dynamic_deadband(drift_gsigma);

    // ── Phase 0b: Gain scheduling ──────────────────────────────────────────
    // In the default (no-schedule) case, compute_gain_schedule() returns 1.0
    // and has no cost. When a gain schedule is configured, the caller must
    // call update() with pv injected via the process_variable parameter.
    // For now we use a per-instance stored PV (updated externally via
    // set_process_variable()); here we just apply the last computed K.
    // (No recomputation needed per-frame unless PV changes.)

    // Apply K_schedule to effective thresholds (non-destructive — base config is immutable).
    const float k     = k_schedule_;
    const float eff_trigger     = cfg_.thresh_trigger     * k;
    const float eff_recover     = cfg_.thresh_recover     * k;
    const float eff_escalation  = cfg_.escalation_threshold * k;

    // ── Phase 1: Compute instantaneous severity ─────────────────────────────
    const float severity = compute_severity(drift_gsigma, confidence, momentum);
    last_severity_       = severity;

    // ── Drain cooldown timer ───────────────────────────────────────────────────
    if (cooldown_timer_ > 0) {
        cooldown_timer_--;
    }

    DriftEvent event = DriftEvent::NONE;

    // ── Phase 2: State Machine ─────────────────────────────────────────────────
    switch (state_) {

        // ──────────────────────────────────────────────────────────────────────
        // NOMINAL: accumulate evidence; resist noise via asymmetric decay.
        // ──────────────────────────────────────────────────────────────────────
        case DriftState::NOMINAL: {
            if (severity >= eff_trigger) {
                // Above trigger: accumulate persistence evidence.
                persist_timer_++;

                if (persist_timer_ >= cfg_.required_persist) {
                    // ── TRANSITION: NOMINAL -> DRIFT_CONFIRMED ─────────────────
                    state_         = DriftState::DRIFT_CONFIRMED;
                    recover_timer_ = 0;

                    if (cooldown_timer_ == 0) {
                        event          = DriftEvent::DRIFT_STARTED;
                        cooldown_timer_= cfg_.cooldown_frames;
                    }
                    // If still in cooldown from a recent reset, we silently enter
                    // DRIFT_CONFIRMED. This is an intentional design choice: accurate
                    // state takes precedence over alert spam prevention.
                }
            } else {
                // Below trigger: asymmetric decay.
                // Subtracts persist_decay per frame below threshold.
                // Effect: an oscillating signal spending 50% of time above and 50%
                // below will drain at net -(persist_decay - 1) per cycle and never
                // latch. The signal MUST be sustained.
                persist_timer_ -= cfg_.persist_decay;
                if (persist_timer_ < 0) {
                    persist_timer_ = 0;
                }
            }
            break;
        }

        // ──────────────────────────────────────────────────────────────────────
        // DRIFT_CONFIRMED: gate on recovery; check escalation.
        // ──────────────────────────────────────────────────────────────────────
        case DriftState::DRIFT_CONFIRMED: {

            // ── Escalation check (runs every frame; gated by cooldown) ─────────
            if (severity >= eff_escalation && cooldown_timer_ == 0) {
                event          = DriftEvent::DRIFT_ESCALATED;
                cooldown_timer_= cfg_.cooldown_frames;
            }

            // ── Recovery logic ────────────────────────────────────────────────
            // Recovery requires THREE simultaneous conditions:
            //   1. Spatial:    severity < thresh_recover  (energy is truly low)
            //   2. Rate:       momentum <= 0.0f           (not actively worsening)
            //   3. Temporal:   recover_timer >= required_recover (sustained)
            //
            // If either condition (1) or (2) breaks, recover_timer RESETS INSTANTLY.
            // This prevents a recovering system that spikes once from being
            // declared healthy prematurely.
            const bool spatial_ok  = (severity < eff_recover);
            const bool rate_ok     = (momentum  <= 0.0f);

            if (spatial_ok && rate_ok) {
                recover_timer_++;

                if (recover_timer_ >= cfg_.required_recover) {
                    // ── TRANSITION: DRIFT_CONFIRMED -> NOMINAL ─────────────────
                    state_         = DriftState::NOMINAL;
                    persist_timer_ = 0;
                    recover_timer_ = 0;

                    // Resolution alerts are not gated by cooldown.
                    // An operator MUST know when a confirmed drift resolves.
                    event          = DriftEvent::DRIFT_RESOLVED;
                    cooldown_timer_= cfg_.cooldown_frames;
                }
            } else {
                // At least one recovery condition failed.
                // Instantly zero the recovery timer — there is no grace period.
                // If momentum went positive, the engine is worsening. Do not recover.
                recover_timer_ = 0;
            }
            break;
        }

        // Defensive default: should never be reached. Reset to safe state.
        default: {
            state_         = DriftState::NOMINAL;
            persist_timer_ = 0;
            recover_timer_ = 0;
            cooldown_timer_= 0;
            break;
        }
    }

    // ── Assemble result ────────────────────────────────────────────────────────
    return StabilizerResult{
        /* output        = */ (state_ == DriftState::DRIFT_CONFIRMED)
                                  ? DriftOutput::DRIFT_CONFIRMED
                                  : DriftOutput::NOMINAL,
        /* event         = */ event,
        /* severity      = */ severity,
        /* persist_timer = */ persist_timer_,
        /* recover_timer = */ recover_timer_,
    };
}

} // namespace signalfix
