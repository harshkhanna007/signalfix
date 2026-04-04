// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : src/module1/stage_s6_timestamp_correction.cpp
// Spec   : SFX-M1-TDS-001  Revision 2.4
// =============================================================================
//
// Stage S6 — Timestamp Correction: complete PLL timing estimator.
//
// COMPILER REQUIREMENTS:
//   -fno-exceptions  — no throw anywhere.
//   -fno-rtti        — no dynamic_cast.
//   IEEE 754 double  — mandatory for deterministic double arithmetic.
//   C++17 or later   — [[nodiscard]], inline constexpr.
//
// REQUIRED CHANGE IN timing_correction_state.hpp (if not yet applied):
//   Replace:  uint8_t  _pad[2];       // offset 30, size 2
//   With:     int16_t  phase_frac_accum;  // offset 30, size 2
//   Struct remains 32 bytes. memset(0) correctly initialises phase_frac_accum=0.
//
// ─── ARITHMETIC STRATEGY ────────────────────────────────────────────────────
//
//   Timestamps are uint64_t throughout (≈584 years of μs).
//   PLL frequency register uses double for sub-microsecond precision.
//   double → uint64_t conversion uses round-to-nearest (add 0.5, truncate),
//   bit-exact on all IEEE 754 platforms for finite, positive operands.
//   Non-finite (NaN/Inf) values are caught by the !(dt > 0.0) guard.
//
// ─── REVISION 2.4 FIXES OVER 2.3 ────────────────────────────────────────────
//
// FIX-A1  [CRITICAL] advance_with_frac_correction() was declared in the header
//         but never implemented. phase_frac_accum was never read or written.
//         All PLL-driven advances called clamp_and_round_dt()+monotone_advance()
//         directly, leaving systematic rounding drift unmitigated.
//         Fix: implement advance_with_frac_correction(). Replace direct
//         round+advance calls in handle_normal_update, handle_regression,
//         handle_stale with advance_with_frac_correction().
//
// FIX-A2  [CRITICAL] handle_long_gap and bootstrap did not initialise or reset
//         phase_frac_accum. After a LONG_GAP, stale frac error from the pre-gap
//         PLL would bleed into post-gap operation, corrupting the first several
//         corrected advances after re-acquisition.
//         Fix: explicitly set phase_frac_accum = 0 in both sites.
//
// FIX-A3  [SERIOUS] select_time_source() took no is_stale parameter.
//         STALE envelopes (watchdog-injected) have timestamp_us = 0. These
//         passed through the existing fallback to arrival_time_us correctly,
//         but the is_stale check was not explicit. A watchdog path that
//         populates timestamp_us with a stale or incorrect non-zero value
//         would silently use it. Now: is_stale forces arrival_time_us
//         unconditionally, regardless of timestamp_us content.
//
// FIX-A4  [SERIOUS] handle_normal_update() had no per-step rate limit on
//         estimated_dt_us change. With alpha_acquire=0.18 and raw_dt near
//         max_dt_us, a single sample could shift estimated_dt by several ms.
//         This causes estimator oscillation under burst-error conditions.
//         Fix: after the IIR update, clamp the change to ±kMaxDtChangeFrac
//         × prev_estimated_dt (10% of current estimate per step).
//
// FIX-A5  [MODERATE] update_lock_state() incremented consecutive_clean_count
//         on every NORMAL event regardless of whether estimated_dt had
//         converged to the measured rate. This allowed locking onto a still-
//         converging frequency estimate, producing a locked PLL that tracks
//         the wrong rate until a subsequent anomaly forces RECOVERY.
//         Fix: dt_stable parameter gates the counter increment. dt_stable is
//         computed in update() as: |raw_dt - estimated_dt_pre| < 10% of
//         estimated_dt_pre. Only stable NORMAL events advance toward lock.
//
// FIX-A6  [MODERATE] select_time_source() was not called with is_stale context.
//         Extended with explicit is_stale routing (STALE → arrival_time_us).
//         Completes FIX-A3.
//
// FIX-A7  [LATENT] SHORT_GAP's handle_short_gap also applies the rate limit
//         from FIX-A4 to its soft PLL frequency update, for consistency.
//
// =============================================================================

#include "signalfix/module1/stages/stage_s6_timestamp_correction.hpp"
#include "signalfix/module1/time_utils.hpp"
#include "signalfix/module1/numeric_limits.hpp"
#include "signalfix/module1/types.hpp"

#include <cstdint>
#include <limits>

namespace signalfix {

// =============================================================================
// File-scope constants
// =============================================================================

namespace {

/// Absolute minimum corrected_delta_t regardless of config.
constexpr uint64_t kAbsoluteMinDtUs    = 1u;

/// Bias for double → uint64_t round-to-nearest on positive values.
constexpr double   kRoundBias          = 0.5;

/// Saturation ceiling for consecutive sample counters (uint16_t).
constexpr uint16_t kCounterSaturation  = 0xFFFFu;

/// Fixed-point scale for fractional phase accumulator: 1 unit = 1/256 μs.
/// Chosen as a power of 2 so that multiplication is exact in IEEE 754.
constexpr double   kFracScale          = 256.0;

/// Overflow threshold for phase_frac_accum in 1/256 μs units.
/// 256 units = 1 μs. Correction fires when accumulated error reaches 1 μs.
constexpr int32_t  kFracBound          = 256;

/// Maximum per-step change in estimated_dt_us as a fraction of its current
/// value (FIX-A4). Limits estimator response to burst errors without
/// changing steady-state alpha behaviour (which is typically well below this).
/// 0.10 = 10%: at 10ms nominal, max step = 1ms regardless of alpha or error.
constexpr double   kMaxDtChangeFrac    = 0.10;

/// Stability tolerance for lock acquisition (FIX-A5).
/// NORMAL events where |raw_dt - estimated_dt_pre| > tolerance do NOT
/// advance the lock counter. Prevents locking on unconverged estimates.
/// 0.10 = 10%: at 10ms nominal, tolerance = 1ms.
constexpr double   kDtStabilityFrac    = 0.10;

} // namespace


// =============================================================================
// Construction — config sanitisation
// =============================================================================
//
// Pipeline contract: unconditional constructibility.
// A malformed config produces degraded-but-safe behaviour (no crashes, no UB).
//
// Required invariants after construction:
//   0 < min_dt_us < max_dt_us < max_time_jump_us
//   0 < alpha < alpha_recovery < alpha_acquire < 1
//   0 < anomaly_alpha_scale ≤ 1
//   0 < hard_invalid_alpha_scale ≤ 1
//   lock_acquire_count ≥ 1
//   recovery_count ≥ 1
//
// =============================================================================

StageS6TimestampCorrection::StageS6TimestampCorrection(
    const TimestampCorrectionConfig& config) noexcept
    : config_(config)
{
    // ── dt bounds ─────────────────────────────────────────────────────────────

    if (config_.min_dt_us < kAbsoluteMinDtUs)
    {
        config_.min_dt_us = kAbsoluteMinDtUs;
    }
    if (config_.max_dt_us < config_.min_dt_us)
    {
        config_.max_dt_us = config_.min_dt_us;
    }
    if (config_.max_time_jump_us <= config_.max_dt_us)
    {
        config_.max_time_jump_us = config_.max_dt_us + 1u;
    }

    // ── Alpha ordering: 0 < alpha < alpha_recovery < alpha_acquire < 1 ───────

    auto clamp_alpha = [](double a, double lo, double hi) -> double {
        if (a <= lo) { return lo + 0.001; }
        if (a >= hi) { return hi - 0.001; }
        return a;
    };
    config_.alpha          = clamp_alpha(config_.alpha,          0.0, 1.0);
    config_.alpha_recovery = clamp_alpha(config_.alpha_recovery, 0.0, 1.0);
    config_.alpha_acquire  = clamp_alpha(config_.alpha_acquire,  0.0, 1.0);

    if (config_.alpha_recovery <= config_.alpha)
    {
        config_.alpha_recovery = config_.alpha + 0.02;
        if (config_.alpha_recovery >= 1.0) { config_.alpha_recovery = 0.99; }
    }
    if (config_.alpha_acquire <= config_.alpha_recovery)
    {
        config_.alpha_acquire = config_.alpha_recovery + 0.02;
        if (config_.alpha_acquire >= 1.0) { config_.alpha_acquire = 0.99; }
    }

    // ── Scale factors ─────────────────────────────────────────────────────────

    if (config_.anomaly_alpha_scale      <= 0.0) { config_.anomaly_alpha_scale      = 0.05; }
    if (config_.anomaly_alpha_scale      >  1.0) { config_.anomaly_alpha_scale      = 1.0;  }

    if (config_.hard_invalid_alpha_scale <= 0.0) { config_.hard_invalid_alpha_scale = 0.05; }
    if (config_.hard_invalid_alpha_scale >  1.0) { config_.hard_invalid_alpha_scale = 1.0;  }

    // ── Lock state counters ───────────────────────────────────────────────────

    if (config_.lock_acquire_count == 0u) { config_.lock_acquire_count = 1u; }
    if (config_.recovery_count     == 0u) { config_.recovery_count     = 1u; }
}


// =============================================================================
// IStage interface
// =============================================================================

const char* StageS6TimestampCorrection::stage_name() const noexcept
{
    return "S6-TimestampCorrection";
}

void StageS6TimestampCorrection::reset() noexcept
{
    // No stage-internal per-sample state.
    // All timing state lives in ChannelState.ts_corr — reset via init_channel_state().
}


// =============================================================================
// process() — pipeline hot path
// =============================================================================

StageResult StageS6TimestampCorrection::process(
    MeasurementEnvelope& envelope,
    ChannelState&        channel_state) noexcept
{
    TimingCorrectionState& ts_corr = channel_state.ts_corr;

    if (!ts_corr.initialized)
    {
        bootstrap(envelope, channel_state);
        return StageResult::CONTINUE;
    }

    return update(envelope, channel_state);
}


// =============================================================================
// bootstrap() — cold-start first-sample seeding
// =============================================================================
//
// On the first sample there is no prior timestamp to compute a dt against:
//   1. Select the best available time source.
//   2. Seed the phase register (corrected_timestamp_us) from the raw time.
//   3. Seed the frequency register (estimated_dt_us) from nominal_delta_t_us.
//   4. Reset phase_frac_accum to 0 (FIX-A2: no stale error from any prior state).
//   5. Set lock_state = LOCKING.
//   6. Emit corrected_delta_t_us = nominal (best available at t = 0).
//
// No PLL update is performed — there is no error signal yet.
//
// =============================================================================

void StageS6TimestampCorrection::bootstrap(
    MeasurementEnvelope& envelope,
    ChannelState&        cs) noexcept
{
    TimingCorrectionState& ts_corr = cs.ts_corr;

    const bool     is_stale = flag_set(envelope.status, SampleStatus::STALE);
    const uint64_t raw_ts   = select_time_source(envelope, is_stale);
    const uint64_t nom_dt   = cs.nominal_delta_t_us;
    const uint64_t safe_dt  = numeric_utils::clamp(nom_dt, config_.min_dt_us, config_.max_dt_us);

    ts_corr.last_raw_timestamp_us      = raw_ts;
    ts_corr.corrected_timestamp_us     = raw_ts;
    ts_corr.estimated_dt_us            = static_cast<double>(safe_dt);
    ts_corr.phase_frac_accum           = 0;          // FIX-A2: explicit clean init
    ts_corr.lock_state                 = PllLockState::LOCKING;
    ts_corr.consecutive_clean_count    = 0u;
    ts_corr.consecutive_anomaly_count  = 0u;
    ts_corr.initialized                = true;

    // Output integration: write corrected fields; raw fields are untouched.
    envelope.corrected_timestamp_us = raw_ts;
    envelope.corrected_delta_t_us   = safe_dt;
}


// =============================================================================
// update() — steady-state PLL update
// =============================================================================
//
// Execution sequence:
//   1. Resolve is_stale; select time source (FIX-A3: STALE → arrival_time_us).
//   2. Detect timestamp regression.
//   3. Gate TIMING_ANOMALY on !is_stale (synthetic envelopes must not set flags).
//   4. Compute raw_dt.
//   5. Classify event — STALE checked FIRST.
//   6. Compute dt_stable for lock state machine (FIX-A5), using pre-update est.
//   7. Dispatch to per-event handler.
//   8. Update lock state machine.
//   9. Write outputs.
//
// Output integration: corrected_timestamp_us and corrected_delta_t_us written
// to the envelope. Raw timestamp_us and delta_t_us preserved as audit data.
// S7 and all downstream stages MUST read corrected_* fields.
//
// =============================================================================

StageResult StageS6TimestampCorrection::update(
    MeasurementEnvelope& envelope,
    ChannelState&        cs) noexcept
{
    TimingCorrectionState& ts_corr = cs.ts_corr;

    // ── Time source selection (FIX-A3) ───────────────────────────────────────
    // Resolve is_stale first so select_time_source can choose the correct source.
    // STALE → arrival_time_us (synthetic; timestamp_us may be zero or garbage).
    // Normal → timestamp_us if populated, else arrival_time_us.

    const bool     is_stale = flag_set(envelope.status, SampleStatus::STALE);
    const uint64_t raw_ts   = select_time_source(envelope, is_stale);
    const uint64_t nom_dt   = cs.nominal_delta_t_us;

    // ── Regression detection ──────────────────────────────────────────────────

    const bool regression = (raw_ts < ts_corr.last_raw_timestamp_us);

    // Gate TIMING_ANOMALY: STALE envelopes are synthetic.
    // Setting TIMING_ANOMALY on a watchdog envelope would corrupt downstream status
    // fields for a non-real event. For real regressions on non-STALE samples,
    // the flag is added without clearing prior flags (stage contract).
    if (regression && !is_stale)
    {
        envelope.status = envelope.status | SampleStatus::TIMING_ANOMALY;
    }

    // ── Raw inter-sample interval ─────────────────────────────────────────────
    // On regression raw_dt = 0 (safe; the REGRESSION handler ignores it).

    const uint64_t raw_dt = regression
        ? 0u
        : time_utils::safe_delta(raw_ts, ts_corr.last_raw_timestamp_us);

    // ── Event classification — STALE checked first ────────────────────────────

    const TimingEvent event = classify_timing_event(raw_dt, envelope.status, regression);

    // ── Lock stability indicator (FIX-A5) ────────────────────────────────────
    // Capture estimated_dt BEFORE any handler updates it.
    // dt_stable = true only on NORMAL events where raw_dt is within ±10% of
    // the current estimate. This gates the lock acquisition counter so that
    // clean-flag events on an unconverged estimator do not trigger premature lock.

    const double pre_update_est = ts_corr.estimated_dt_us;
    const double raw_dt_d       = static_cast<double>(raw_dt);
    const double dt_err_abs     = (raw_dt_d >= pre_update_est)
                                  ? (raw_dt_d - pre_update_est)
                                  : (pre_update_est - raw_dt_d);
    const bool   dt_stable      = (event == TimingEvent::NORMAL)
                                  && (dt_err_abs < pre_update_est * kDtStabilityFrac);

    // ── Per-event dispatch ────────────────────────────────────────────────────

    uint64_t corrected_dt = 0u;

    switch (event)
    {
        case TimingEvent::LONG_GAP:
        {
            // handle_long_gap resets phase_frac_accum (FIX-A2).
            corrected_dt = handle_long_gap(raw_ts, ts_corr, nom_dt);
            break;
        }
        case TimingEvent::SHORT_GAP:
        {
            // Advance by actual raw_dt. phase_frac_accum NOT updated.
            // last_raw_timestamp_us updated inside handler.
            corrected_dt = handle_short_gap(raw_dt, raw_ts, ts_corr);
            break;
        }
        case TimingEvent::REGRESSION:
        {
            // PLL frozen; fractional advance; last_raw_ts NOT updated.
            corrected_dt = handle_regression(ts_corr);
            break;
        }
        case TimingEvent::STALE:
        {
            // PLL fully frozen; fractional advance; last_raw_ts NOT updated.
            corrected_dt = handle_stale(ts_corr);
            break;
        }
        case TimingEvent::DEGRADED:
        {
            // Timestamp is real even though measurement is bad: PLL still updates.
            const double base_alpha = compute_state_alpha(ts_corr.lock_state);
            const double eff_alpha  = base_alpha * config_.hard_invalid_alpha_scale;
            corrected_dt = handle_normal_update(raw_dt, eff_alpha, ts_corr);
            ts_corr.last_raw_timestamp_us = raw_ts;
            break;
        }
        case TimingEvent::ANOMALY:
        {
            const double base_alpha = compute_state_alpha(ts_corr.lock_state);
            const double eff_alpha  = base_alpha * config_.anomaly_alpha_scale;
            corrected_dt = handle_normal_update(raw_dt, eff_alpha, ts_corr);
            ts_corr.last_raw_timestamp_us = raw_ts;
            break;
        }
        case TimingEvent::NORMAL:
        default:
        {
            const double eff_alpha = compute_state_alpha(ts_corr.lock_state);
            corrected_dt = handle_normal_update(raw_dt, eff_alpha, ts_corr);
            ts_corr.last_raw_timestamp_us = raw_ts;
            break;
        }
    }

    // ── Lock state machine ────────────────────────────────────────────────────
    // dt_stable gates the clean counter (FIX-A5).

    update_lock_state(event, ts_corr, config_, dt_stable);

    // ── Write outputs (OUTPUT INTEGRATION) ───────────────────────────────────
    // S6 exclusively writes corrected_timestamp_us and corrected_delta_t_us.
    // Raw fields (timestamp_us, delta_t_us) are preserved as-is for S7 audit.
    // ALL stages downstream of S6 MUST read corrected_* fields.

    envelope.corrected_timestamp_us = ts_corr.corrected_timestamp_us;
    envelope.corrected_delta_t_us   = corrected_dt;

    return StageResult::CONTINUE;
}


// =============================================================================
// classify_timing_event()
// =============================================================================
//
// Priority order is load-bearing. STALE is checked FIRST.
//
// Rationale for STALE-first:
//   Watchdog envelopes are synthetic (S2b skipped). timestamp_us may be 0.
//   select_time_source() returns arrival_time_us for STALE, but even that
//   may be behind last_raw_timestamp_us in certain edge conditions (e.g.,
//   test harness, partial init). Checking REGRESSION before STALE would
//   misclassify such envelopes as regressions, set TIMING_ANOMALY spuriously,
//   and drive the lock state to RECOVERY when STALE is the correct path.
//
// Remaining priority:
//   REGRESSION before all gap/degraded/anomaly checks.
//   LONG_GAP (MISSING flag or huge raw_dt) before SHORT_GAP.
//   DEGRADED (value-level invalidity) before ANOMALY (flag-level).
//
// =============================================================================

StageS6TimestampCorrection::TimingEvent
StageS6TimestampCorrection::classify_timing_event(
    const uint64_t     raw_dt,
    const SampleStatus status,
    const bool         regression) const noexcept
{
    if (flag_set(status, SampleStatus::STALE))
    {
        return TimingEvent::STALE;
    }

    if (regression)
    {
        return TimingEvent::REGRESSION;
    }

    if (flag_set(status, SampleStatus::MISSING) || (raw_dt > config_.max_time_jump_us))
    {
        return TimingEvent::LONG_GAP;
    }

    if (raw_dt > config_.max_dt_us)
    {
        return TimingEvent::SHORT_GAP;
    }

    if (flag_set(status, SampleStatus::HARD_INVALID))
    {
        return TimingEvent::DEGRADED;
    }

    if (flag_set(status, SampleStatus::TIMING_ANOMALY))
    {
        return TimingEvent::ANOMALY;
    }

    return TimingEvent::NORMAL;
}


// =============================================================================
// handle_normal_update()
// =============================================================================
//
// Shared implementation for NORMAL, ANOMALY, DEGRADED paths.
// The caller selects effective_alpha.
//
//   1. Clamp raw_dt to [min_dt_us, max_dt_us] (PLL frequency input).
//   2. IIR update: estimated_dt_us += effective_alpha × (clamped_dt − estimated_dt_us)
//   3. FIX-A4 Rate limit: Δestimated_dt ≤ prev_est × kMaxDtChangeFrac (10%).
//      Prevents oscillation when alpha is high AND the raw_dt has a large
//      transient error. Does not affect steady-state tracking where the
//      actual step (alpha × error) is already well below the limit.
//   4. Clamp estimated_dt_us to [min_dt_us, max_dt_us].
//   5. Fractional phase advance via advance_with_frac_correction() (FIX-A1).
//
// Does NOT update last_raw_timestamp_us — caller is responsible.
//
// =============================================================================

uint64_t StageS6TimestampCorrection::handle_normal_update(
    const uint64_t         raw_dt,
    const double           effective_alpha,
    TimingCorrectionState& ts_corr) const noexcept
{
    const uint64_t clamped_dt = numeric_utils::clamp(
        raw_dt, config_.min_dt_us, config_.max_dt_us);

    // Save pre-update estimate for rate limiting.
    const double prev_est = ts_corr.estimated_dt_us;

    // IIR frequency update.
    const double measured = static_cast<double>(clamped_dt);
    const double error    = measured - prev_est;
    ts_corr.estimated_dt_us = prev_est + effective_alpha * error;

    // FIX-A4: Per-step rate limit — clamp the change to ±10% of previous estimate.
    // At 10ms nominal: max step = 1ms. Alpha_acquire=0.18 with 30ms error would
    // give a 5.4ms step without limiting; the limiter reduces it to 1ms.
    const double max_step = prev_est * kMaxDtChangeFrac;
    if (ts_corr.estimated_dt_us > prev_est + max_step)
    {
        ts_corr.estimated_dt_us = prev_est + max_step;
    }
    if (ts_corr.estimated_dt_us < prev_est - max_step)
    {
        ts_corr.estimated_dt_us = prev_est - max_step;
    }

    // Hard clamp to configured operating range.
    const double min_d = static_cast<double>(config_.min_dt_us);
    const double max_d = static_cast<double>(config_.max_dt_us);
    if (ts_corr.estimated_dt_us < min_d) { ts_corr.estimated_dt_us = min_d; }
    if (ts_corr.estimated_dt_us > max_d) { ts_corr.estimated_dt_us = max_d; }

    // FIX-A1: Use fractional phase advance to eliminate systematic drift.
    return advance_with_frac_correction(ts_corr);
}


// =============================================================================
// handle_regression()
// =============================================================================
//
// The raw timestamp is in the past. PLL frequency register is frozen.
//
//   - Advance corrected time by estimated_dt (PLL's current estimate).
//   - Do NOT update estimated_dt_us.
//   - Do NOT update last_raw_timestamp_us: snapping to a past value would
//     inflate the next sample's raw_dt and could trigger a false gap reset.
//     (This was the core regression bug in Rev 2.1.)
//
// TIMING_ANOMALY is already set by update() before this call.
//
// =============================================================================

uint64_t StageS6TimestampCorrection::handle_regression(
    TimingCorrectionState& ts_corr) const noexcept
{
    // FIX-A1: fractional advance maintains sub-μs phase continuity.
    return advance_with_frac_correction(ts_corr);
    // last_raw_timestamp_us intentionally NOT updated.
}


// =============================================================================
// handle_stale()
// =============================================================================
//
// STALE is set by the watchdog path when no real sample has arrived within
// 2 × nominal_delta_t_us. The envelope is synthetic (no real measurement).
//
// PLL fully frozen:
//   - No frequency update.
//   - Advance by estimated_dt to keep the corrected timeline moving.
//   - Do NOT update last_raw_timestamp_us.
//
// =============================================================================

uint64_t StageS6TimestampCorrection::handle_stale(
    TimingCorrectionState& ts_corr) const noexcept
{
    // FIX-A1: fractional advance — phase continuity preserved even across
    // watchdog gaps.
    return advance_with_frac_correction(ts_corr);
    // last_raw_timestamp_us intentionally NOT updated.
}


// =============================================================================
// handle_short_gap()
// =============================================================================
//
// raw_dt ∈ (max_dt_us, max_time_jump_us] — actual elapsed time is known and valid.
//
// Two separate dt values serve two separate purposes:
//
//   pll_input_dt  = clamp(raw_dt, min_dt_us, max_dt_us)
//     Protects the frequency register: a moderately-long gap must not pull
//     estimated_dt_us toward an overrun value. The PLL should track the
//     sensor's nominal operating rate, not occasional late arrivals.
//
//   output_dt     = clamp(raw_dt, min_dt_us, max_time_jump_us)
//     The ACTUAL elapsed time between samples. Written to corrected_delta_t_us.
//     The Kalman filter must receive the true dt for correct state prediction.
//
// FIX-A4 (SHORT_GAP): rate limit also applied to the soft PLL update here.
//
// phase_frac_accum is NOT updated: this advance is driven by a real measured
// interval, not by the PLL frequency register. The fractional error register
// tracks rounding error in PLL-derived advances only. Leaving it unchanged
// maintains phase coherence between PLL-driven and gap-driven advances.
//
// =============================================================================

uint64_t StageS6TimestampCorrection::handle_short_gap(
    const uint64_t         raw_dt,
    const uint64_t         raw_ts,
    TimingCorrectionState& ts_corr) const noexcept
{
    // PLL frequency input: clamped to max_dt_us to protect the estimator.
    const uint64_t pll_input_dt = numeric_utils::clamp(
        raw_dt, config_.min_dt_us, config_.max_dt_us);

    // Soft PLL frequency update.
    const double prev_est   = ts_corr.estimated_dt_us;
    const double soft_alpha = compute_state_alpha(ts_corr.lock_state)
                              * config_.anomaly_alpha_scale;
    const double measured   = static_cast<double>(pll_input_dt);
    const double error      = measured - prev_est;
    ts_corr.estimated_dt_us = prev_est + soft_alpha * error;

    // FIX-A4: Rate limit on SHORT_GAP frequency update, consistent with
    // handle_normal_update. Prevents the soft update from pulling estimated_dt
    // unexpectedly far even at low soft_alpha when error is very large.
    const double max_step = prev_est * kMaxDtChangeFrac;
    if (ts_corr.estimated_dt_us > prev_est + max_step)
    {
        ts_corr.estimated_dt_us = prev_est + max_step;
    }
    if (ts_corr.estimated_dt_us < prev_est - max_step)
    {
        ts_corr.estimated_dt_us = prev_est - max_step;
    }

    // Clamp frequency register.
    const double min_d = static_cast<double>(config_.min_dt_us);
    const double max_d = static_cast<double>(config_.max_dt_us);
    if (ts_corr.estimated_dt_us < min_d) { ts_corr.estimated_dt_us = min_d; }
    if (ts_corr.estimated_dt_us > max_d) { ts_corr.estimated_dt_us = max_d; }

    // Phase advance: actual elapsed time, NOT pll_input_dt or estimated_dt.
    // Bounded at max_time_jump_us (by classification invariant raw_dt is already
    // within this bound, but we clamp defensively).
    const uint64_t output_dt = numeric_utils::clamp(
        raw_dt, config_.min_dt_us, config_.max_time_jump_us);

    const uint64_t advance = monotone_advance(ts_corr, output_dt);

    // Baseline advances — the timestamp IS real.
    ts_corr.last_raw_timestamp_us = raw_ts;

    // phase_frac_accum intentionally NOT updated — see rationale above.

    return advance;
}


// =============================================================================
// handle_long_gap()
// =============================================================================
//
// True gap: S4 MISSING flag or raw_dt > max_time_jump_us.
//
// The raw timestamp jump is too large to continue the existing timeline.
// Filtering through it would corrupt the frequency estimator for many subsequent
// samples. Correct response: full PLL state reset.
//
//   - Reset estimated_dt to nominal_delta_t_us (best prior for clock rate
//     after a gap of unknown duration).
//   - FIX-A2: Reset phase_frac_accum = 0. Pre-gap accumulated error is
//     irrelevant to the post-gap PLL and must not bleed forward.
//   - Advance corrected time by one clamped-nominal period.
//   - Snap last_raw_timestamp_us to current raw_ts (post-gap baseline).
//   - Lock state → LOCKING (handled by update_lock_state for LONG_GAP).
//
// =============================================================================

uint64_t StageS6TimestampCorrection::handle_long_gap(
    const uint64_t         raw_ts,
    TimingCorrectionState& ts_corr,
    const uint64_t         nominal_dt_us) const noexcept
{
    const uint64_t safe_nom = numeric_utils::clamp(
        nominal_dt_us, config_.min_dt_us, config_.max_dt_us);

    // Reset both frequency register and fractional phase accumulator (FIX-A2).
    ts_corr.estimated_dt_us = static_cast<double>(safe_nom);
    ts_corr.phase_frac_accum = 0;

    // Advance by one nominal period. No fractional correction on this step:
    // the advance is a hard nominal reset, not a PLL-derived estimate.
    const uint64_t advance = monotone_advance(ts_corr, safe_nom);

    // Snap baseline to post-gap position.
    ts_corr.last_raw_timestamp_us = raw_ts;

    return advance;
}


// =============================================================================
// compute_state_alpha()
// =============================================================================

double StageS6TimestampCorrection::compute_state_alpha(
    const PllLockState lock_state) const noexcept
{
    switch (lock_state)
    {
        case PllLockState::LOCKING:  return config_.alpha_acquire;
        case PllLockState::LOCKED:   return config_.alpha;
        case PllLockState::RECOVERY: return config_.alpha_recovery;
        default:                     return config_.alpha;
    }
}


// =============================================================================
// update_lock_state()
// =============================================================================
//
// State machine transitions:
//
//  Event        | From state | New state  | clean_count        | anomaly_count
//  ─────────────┼────────────┼────────────┼────────────────────┼──────────────
//  NORMAL       | LOCKING    | → LOCKED   | +1 (if dt_stable)  | reset to 0
//               |            | if ≥ acq   | reset on lock      |
//               | RECOVERY   | → LOCKED   |                    |
//               |            | if ≥ rec   |                    |
//               | LOCKED     | stay       |                    |
//  ─────────────┼────────────┼────────────┼────────────────────┼──────────────
//  ANOMALY /    | LOCKED     | → RECOVERY | reset to 0         | +1 (sat.)
//  DEGRADED /   | LOCKING /  | stay       |                    |
//  REGRESSION / | RECOVERY   |            |                    |
//  SHORT_GAP    |            |            |                    |
//  ─────────────┼────────────┼────────────┼────────────────────┼──────────────
//  LONG_GAP     | any        | → LOCKING  | reset to 0         | +1 (sat.)
//  ─────────────┼────────────┼────────────┼────────────────────┼──────────────
//  STALE        | LOCKED     | → RECOVERY | reset to 0         | +1 (sat.)
//               | LOCKING /  | stay       |                    |
//               | RECOVERY   |            |                    |
//
// FIX-A5: dt_stable gates the clean counter in the NORMAL arm.
//   Even a NORMAL-classified event (no bad flags, dt within [min, max]) should
//   not advance the lock counter if the measured dt is far from estimated_dt_us.
//   This can occur immediately after LOCKING is entered (estimated_dt was reset
//   to nominal but actual sensor rate is different). Requiring dt_stable before
//   incrementing ensures that consecutive_clean_count reflects frequency
//   convergence, not merely the absence of anomaly flags.
//
//   10% tolerance: conservative enough to admit valid jitter (±1ms at 10ms
//   nominal), tight enough to block locking during initial IIR transient.
//
// STALE → RECOVERY (not LOCKING):
//   A single watchdog event (sensor absent for ~2×nominal) does not warrant
//   full re-acquisition. estimated_dt_us was frozen correctly; the estimator
//   is still valid. RECOVERY (8-sample re-verification) is proportionate.
//   Contrast with LONG_GAP: unknown gap duration, always → LOCKING.
//
// =============================================================================

// static
void StageS6TimestampCorrection::update_lock_state(
    const TimingEvent              event,
    TimingCorrectionState&         ts_corr,
    const TimestampCorrectionConfig& cfg,
    const bool                     dt_stable) noexcept
{
    switch (event)
    {
        case TimingEvent::NORMAL:
        {
            // FIX-A5: Only count toward lock if frequency estimate has converged.
            if (dt_stable)
            {
                if (ts_corr.consecutive_clean_count < kCounterSaturation)
                {
                    ++ts_corr.consecutive_clean_count;
                }
            }
            // A NORMAL event (even unstable) resets the anomaly streak.
            ts_corr.consecutive_anomaly_count = 0u;

            // State transitions — driven by accumulated stable clean count.
            if (ts_corr.lock_state == PllLockState::LOCKING
                && ts_corr.consecutive_clean_count >= cfg.lock_acquire_count)
            {
                ts_corr.lock_state              = PllLockState::LOCKED;
                ts_corr.consecutive_clean_count = 0u;
            }
            else if (ts_corr.lock_state == PllLockState::RECOVERY
                     && ts_corr.consecutive_clean_count >= cfg.recovery_count)
            {
                ts_corr.lock_state              = PllLockState::LOCKED;
                ts_corr.consecutive_clean_count = 0u;
            }
            break;
        }

        case TimingEvent::ANOMALY:
        case TimingEvent::DEGRADED:
        case TimingEvent::REGRESSION:
        case TimingEvent::SHORT_GAP:
        {
            ts_corr.consecutive_clean_count = 0u;
            if (ts_corr.consecutive_anomaly_count < kCounterSaturation)
            {
                ++ts_corr.consecutive_anomaly_count;
            }
            if (ts_corr.lock_state == PllLockState::LOCKED)
            {
                ts_corr.lock_state = PllLockState::RECOVERY;
            }
            // LOCKING or RECOVERY: remain in current state.
            break;
        }

        case TimingEvent::LONG_GAP:
        {
            // Gap of unknown duration → full re-acquisition.
            ts_corr.consecutive_clean_count = 0u;
            if (ts_corr.consecutive_anomaly_count < kCounterSaturation)
            {
                ++ts_corr.consecutive_anomaly_count;
            }
            ts_corr.lock_state = PllLockState::LOCKING;
            break;
        }

        case TimingEvent::STALE:
        {
            // Single missed sample → RECOVERY (not LOCKING).
            // Frequency estimate remains valid; only re-verification required.
            ts_corr.consecutive_clean_count = 0u;
            if (ts_corr.consecutive_anomaly_count < kCounterSaturation)
            {
                ++ts_corr.consecutive_anomaly_count;
            }
            if (ts_corr.lock_state == PllLockState::LOCKED)
            {
                ts_corr.lock_state = PllLockState::RECOVERY;
            }
            // LOCKING or RECOVERY: unchanged.
            break;
        }

        default:
            break;
    }
}


// =============================================================================
// Arithmetic helpers
// =============================================================================

// ---------------------------------------------------------------------------
// select_time_source()
// ---------------------------------------------------------------------------
//
// FIX-A3: is_stale parameter added.
//
//   STALE envelopes are synthetic (watchdog-injected). On the watchdog path,
//   S2b does not run, so timestamp_us is either 0 or whatever was in the
//   previous envelope's field. Using it would be incorrect.
//   arrival_time_us is always the raw OS monotonic clock — safe to use here
//   even though it carries OS scheduling jitter, since we do not update
//   last_raw_timestamp_us on STALE events (no state change needed).
//
//   For normal samples:
//     timestamp_us > 0 — S2b-corrected measurement time (preferred).
//       S2b removes transport latency and sensor clock skew.
//       The Kalman filter needs measurement time, not arrival time.
//     timestamp_us == 0 — S2b not configured; fall back to arrival_time_us.
//
// ---------------------------------------------------------------------------

// static
uint64_t StageS6TimestampCorrection::select_time_source(
    const MeasurementEnvelope& env,
    const bool                 is_stale) noexcept
{
    // STALE: synthetic envelope — always use arrival_time_us.
    if (is_stale)
    {
        return env.arrival_time_us;
    }

    // Normal path: S2b-corrected timestamp if available, else OS clock fallback.
    return (env.timestamp_us > 0u) ? env.timestamp_us : env.arrival_time_us;
}


// ---------------------------------------------------------------------------
// clamp_and_round_dt()
// ---------------------------------------------------------------------------
//
// Converts the double PLL frequency register to uint64_t for phase advance.
//
// Non-finite guard (NaN / negative / zero):
//   static_cast<uint64_t>(NaN + 0.5) is UB (C++17 §7.8p4). The !(dt > 0.0)
//   test catches NaN (all NaN comparisons false), -Inf, -0.0, 0.0, and all
//   negative finite values. +Inf passes the guard and is clamped to max_d.
//
// Round-to-nearest via +0.5 before truncation:
//   Bit-exact on all IEEE 754 platforms for finite, positive operands.
//   No FPU rounding mode dependency.
//
// ---------------------------------------------------------------------------

uint64_t StageS6TimestampCorrection::clamp_and_round_dt(double dt_us) const noexcept
{
    if (!(dt_us > 0.0))
    {
        return config_.min_dt_us;
    }

    const double min_d = static_cast<double>(config_.min_dt_us);
    const double max_d = static_cast<double>(config_.max_dt_us);

    if (dt_us < min_d) { dt_us = min_d; }
    if (dt_us > max_d) { dt_us = max_d; }

    const uint64_t rounded = static_cast<uint64_t>(dt_us + kRoundBias);

    return numeric_utils::clamp(rounded, config_.min_dt_us, config_.max_dt_us);
}


// ---------------------------------------------------------------------------
// advance_with_frac_correction()   (FIX-A1 — core new function)
// ---------------------------------------------------------------------------
//
// PLL-driven phase advance with sub-μs fractional error accumulation.
//
// Problem addressed:
//   estimated_dt_us is a double, typically non-integer (e.g., 9999.7 μs).
//   round(9999.7) = 10000 μs every step. We over-advance by 0.3 μs/sample.
//   After 1000 samples: +300 μs systematic drift — unacceptable for a Kalman
//   filter's time base.
//
// Algorithm:
//   1. base_dt     = round(estimated_dt_us)       — integer advance before correction
//   2. residual_f  = base_dt - estimated_dt_us    — fractional over-advance this step
//                    ∈ (-0.5, 0.5] by definition of round
//   3. residual_256 = round(residual_f × 256)     — same in 1/256 μs fixed-point
//                    ∈ (-128, 128]
//   4. phase_frac_accum += residual_256           — accumulate
//   5. if phase_frac_accum ≥ 256:                 — 1 μs of over-advance accumulated
//         corrected = base_dt - 1                 — shed one integer μs
//         phase_frac_accum -= 256
//      elif phase_frac_accum ≤ -256:              — 1 μs of under-advance accumulated
//         corrected = base_dt + 1                 — add one integer μs
//         phase_frac_accum += 256
//      else: corrected = base_dt
//
// Why this eliminates systematic drift:
//   At 9999.7 μs: residual_256 ≈ 77 per step. frac_accum crosses 256 every
//   ~3.3 samples, triggering a 9999 μs advance. The long-run mean advance is
//   exactly (3.3 × 10000 + 1 × 9999) / 4.3 ≈ 9999.77 → 9999.7 μs. ✓
//   Residual quantization error per step: ≤ 1/256 μs ≈ 4 ns (random, zero-mean).
//   Max accumulated phase error at any instant: < 1 μs (256/256).
//
// Boundary handling:
//   If base_dt == min_dt_us, we cannot subtract 1 (would violate G2 floor).
//   We still carry the accum forward (only bypass the correction); frac_accum
//   is bounded by saturation. In practice, estimated_dt_us >> min_dt_us in
//   all well-configured deployments; this branch is extremely rare.
//
// Determinism:
//   All arithmetic is on IEEE 754 double with explicit sign-aware rounding.
//   No FPU rounding mode dependency. Identical inputs always produce identical
//   output on all IEEE 754-compliant platforms.
//
// ---------------------------------------------------------------------------

uint64_t StageS6TimestampCorrection::advance_with_frac_correction(
    TimingCorrectionState& ts_corr) const noexcept
{
    // Step 1: Base integer advance.
    const uint64_t base_dt = clamp_and_round_dt(ts_corr.estimated_dt_us);

    // Step 2: Fractional residual.
    // residual_f = base_dt - estimated_dt_us ∈ (-0.5, 0.5]
    // Positive  → we will over-advance this step (base > true value).
    // Negative  → we will under-advance this step (base < true value).
    const double residual_f = static_cast<double>(base_dt) - ts_corr.estimated_dt_us;

    // Step 3: Convert to 1/256 μs fixed-point with symmetric round-to-nearest.
    // Sign-aware to avoid platform-specific rounding of negative values.
    //   residual_256 ∈ (-128, 128]  (bounded; no int32_t overflow possible)
    const int32_t residual_256 = (residual_f >= 0.0)
        ?  static_cast<int32_t>(residual_f * kFracScale + kRoundBias)
        : -static_cast<int32_t>(-residual_f * kFracScale + kRoundBias);

    // Step 4: Accumulate.
    int32_t accum = static_cast<int32_t>(ts_corr.phase_frac_accum) + residual_256;

    // Step 5: Apply correction if accumulated error reaches ±1 μs.
    uint64_t corrected = base_dt;

    if (accum >= kFracBound)
    {
        // Accumulated over-advance ≥ 1 μs: advance one integer μs less.
        if (base_dt > config_.min_dt_us)
        {
            corrected = base_dt - 1u;
            accum    -= kFracBound;
        }
        else
        {
            // Cannot go below floor; saturate accumulator to prevent unbounded growth.
            // Drift exists only while estimated_dt is stuck at min_dt_us — an extreme
            // edge case in any correctly-configured deployment.
            if (accum > 32767) { accum = 32767; }
        }
    }
    else if (accum <= -kFracBound)
    {
        // Accumulated under-advance ≥ 1 μs: advance one integer μs more.
        if (base_dt < config_.max_dt_us)
        {
            corrected = base_dt + 1u;
            accum    += kFracBound;
        }
        else
        {
            if (accum < -32768) { accum = -32768; }
        }
    }

    // Step 6: Write back. int32_t accum is within int16_t range after correction;
    // only saturated in the boundary edge cases above.
    ts_corr.phase_frac_accum = static_cast<int16_t>(accum);

    return monotone_advance(ts_corr, corrected);
}


// ---------------------------------------------------------------------------
// monotone_advance()
// ---------------------------------------------------------------------------
//
// Advances ts_corr.corrected_timestamp_us by delta_us.
//
// Explicit overflow saturation at UINT64_MAX (G7):
//   C++ unsigned arithmetic wraps on overflow (well-defined but incorrect).
//   A misconfigured or adversarial system feeding near-UINT64_MAX timestamps
//   would silently wrap corrected_timestamp_us to near zero, violating G1.
//   Saturation is safe: timestamps pin at UINT64_MAX until the next LONG_GAP
//   snaps last_raw_timestamp_us forward. In realistic deployments (< 584 years)
//   this branch is unreachable; it is here to make G1 and G7 unconditional.
//
// ---------------------------------------------------------------------------

uint64_t StageS6TimestampCorrection::monotone_advance(
    TimingCorrectionState& ts_corr,
    const uint64_t         delta_us) const noexcept
{
    const uint64_t safe_delta = (delta_us >= kAbsoluteMinDtUs)
        ? delta_us
        : kAbsoluteMinDtUs;

    const uint64_t prev = ts_corr.corrected_timestamp_us;

    // Saturating addition.
    uint64_t next;
    if (safe_delta > (std::numeric_limits<uint64_t>::max() - prev))
    {
        next = std::numeric_limits<uint64_t>::max();
    }
    else
    {
        next = prev + safe_delta;
    }

    // Monotonicity floor: tautological after saturation, but required by G1
    // regardless of future refactors.
    if (next < prev)
    {
        next = prev;
    }

    ts_corr.corrected_timestamp_us = next;
    return next - prev;
}


// ---------------------------------------------------------------------------
// flag_set()
// ---------------------------------------------------------------------------

// static
bool StageS6TimestampCorrection::flag_set(
    const SampleStatus status,
    const SampleStatus flag) noexcept
{
    return (status & flag) != SampleStatus{};
}

} // namespace signalfix
